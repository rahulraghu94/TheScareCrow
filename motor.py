import Adafruit_BBIO.PWM as PWM
from scipy.interpolate import interp1d
import logging

class motor:
    power = 0.0

    def __init__(self, pin):
        self.pin = pin
        if type(pin) is not str:
            logging.info("Invalid pin given: %s", pin)
            exit()
        # start motor with power = 0 (5 on the scale of [5, 10])
        PWM.start(self.pin, 5, 50)

    def setPower(self, power):
        if power > 100 or power < 0 :
            logging.info("Value entered %d is not valid", power)
            return
        self.power = power
        # map values from [0,100] to [5,10].
        # A frequency of 50Hz translates to a period of 20ms
        # pulse of 1ms is no power, pulse of 2ms is full power.
        power = interp1d([0,100],[5.0,10.0])(power)
        PWM.set_duty_cycle(self.pin, power)

    def getPower(self):
        return self.power

    def __del__(self):
        # stop motor with power = 0 (5 on the scale of [5, 10])
        PWM.set_duty_cycle(self.pin, 5)
        PWM.stop(self.pin)
