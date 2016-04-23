#include "MPU6050_6Axis_MotionApps20.h"
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <ctime>
#include <RF24/RF24.h>
#include "PID.h"
#include "motor.hpp"

using namespace std;

MPU6050 mpu;


uint16_t packet_size;
uint16_t fifo_count;     /* Count of all bytes currently in FIFO  */
uint8_t fifo_buffer[64]; /* FIFO storage buffer */

Quaternion q;           /* [w, x, y, z]         quaternion container */
VectorFloat gravity;    /* [x, y, z]            gravity vector */
double actual_ypr[3];   /* [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector */

PID *pids_ypr[3];
double desired_ypr[3] = {0, 0, 0};
double pids_output_ypr[3];
double pid_tunings[3][3] = {
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0}
};

float throttle = 0;

RF24 radio(49, 0);
const uint8_t pipes[][6] = {"1Node","2Node"};
char radio_msg[32];

Motor *motors[4];
const BlackLib::pwnName motor_pins[4] = {
	BlackLib::EHRPWM1A,
	BlackLib::EHRPWM1B,
	BlackLib::EHRPWM2A,
	BlackLib::EHRPWM2B
};

void setup()
{
	uint8_t dev_status;

	mpu.initialize();
	printf(mpu.testConnection() ?
	       "MPU6050 connection successful\n" :
	       "MPU6050 connection failed\n");
	printf("Initializing DMP...\n");
	dev_status = mpu.dmpInitialize();

	if (dev_status == 0) {
		printf("Enabling DMP...\n");
		mpu.setDMPEnabled(true);
		printf("DMP ready!\n");
		packet_size = mpu.dmpGetFIFOPacketSize();
	} else {
		printf("DMP Initialization failed (code %d)\n", dev_status);
		exit(0);
	}

	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(50);
	radio.setRetries(15,15);
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);
	radio.startListening();

        /* Initialize PID controllers */
        for(int i = 0; i < 3; i++) {
		pids_ypr[i] = new PID(&actual_ypr[i],
				      &pids_output_ypr[i],
				      &desired_ypr[i],
				      pid_tunings[i][0],
				      pid_tunings[i][1],
				      pid_tunings[i][2],
				      DIRECT);
        }

	for(int i = 0; i < 4; i++)
		motors[i] = new Motor(motor_pins[i]);
}

void loop() {
	static float yaw_target = 0;

	uint16_t channels[4];
	float motor[4];

	fifo_count = mpu.getFIFOCount();

	if ((fifo_count >= packet_size * 3 && fifo_count % packet_size == 0)
	    || fifo_count == 1024) {
		mpu.resetFIFO();
		printf("FIFO Overflow");
		fifo_count = mpu.getFIFOCount();
	}

	while (fifo_count < packet_size)
		fifo_count = mpu.getFIFOCount();

	while ((fifo_count/packet_size) > 0) {
		mpu.getFIFOBytes(fifo_buffer, packet_size);
		mpu.dmpGetQuaternion(&q, fifo_buffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll((float *) actual_ypr, &q, &gravity);

		cout << std::time(0) << " " << fifo_count << " ";
		for (int i = 0; i < 3; i++) {
			actual_ypr[i] *= 180 / M_PI;
			cout << actual_ypr[i] << " ";
		}
		fifo_count -= packet_size;
		cout << endl;

	}

	/* If radio has data, read the damn data */
	if (radio.available()) {
		while (radio.available()) {
			radio.read(radio_msg, 32);
		}
		cout << radio_msg << endl;
	}

        /* Call PID::Compute(). This should be called as often as
         * possible. It is up to Compute() to decide when to update
         * the output */
	bool update_flag = false;
        for (int i = 0; i < 3; i++) {
		if (pids_ypr[i]->Compute())
			update_flag = true;
	}

	if (update_flag) {
		/* Ensure that update_flag is set when throttle is changed */
		if (throttle < 2) {
			for (int i = 0; i < 4; i++)
				motors[i].set_power(0);
		} else {
			/* The output of the PID must be positive in
			 * the direction of positive rotation
			 * (right-hand rule. Also given on MPU)
			 *
			 * For yaw, motors 0 & 2 move anticlockwise from above.
			 */
			motors[0]->set_power(throttle - pids_ypr[0] + pids_ypr[1] - pids_ypr[2]);
			motors[1]->set_power(throttle + pids_ypr[0] - pids_ypr[1] - pids_ypr[2]);
			motors[2]->set_power(throttle - pids_ypr[0] - pids_ypr[1] + pids_ypr[2]);
			motors[3]->set_power(throttle + pids_ypr[0] + pids_ypr[1] + pids_ypr[2]);
		}
	}

}

int main(int argc, char *argv[])
{
	setup();

	while (1) {
		loop();
	}
	return 0;
}
