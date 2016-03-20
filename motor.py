import Adafruit_BBIO.PWM as PWM
import logging

class motor:
    MAX_POWER = 100
    MIN_POWER = 0

    def __init__(self, pin):
        self.pin = pin
        self.power = self.MIN_POWER
        # Initialize motor without sending a pulse.
        PWM.start(self.pin, 0)

        # setting frequency in the initializer causes weird errors as
        # explained here:
        # https://github.com/adafruit/adafruit-beaglebone-io-python/issues/66
        # Would be good to find a fix and patch the upstream repository
        PWM.set_frequency(self.pin, 50)

        # calibrate throttle with full power
        PWM.set_duty_cycle(self.pin, 10)
        time.sleep(1)
        PWM.set_duty_cycle(self.pin, 5)
        # start motor with power = 0 (5 on the scale of [5, 10])

    def setPower(self, power):
        if power > self.MAX_POWER:
            self.power = self.MAX_POWER
        elif power < self.MIN_POWER:
            self.power = self.MIN_POWER
        else:
            self.power = power

        # map values from [0,100] to [5,10].
        # A frequency of 50Hz translates to a period of 20ms
        # pulse of 1ms is no power, pulse of 2ms is full power.
        power = 5 + self.MAX_POWER / 20.0
        PWM.set_duty_cycle(self.pin, power)

    def getPower(self):
        return self.power
