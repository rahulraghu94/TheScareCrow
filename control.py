import pid as PID
import math
import mpu6050
import logging
import select
import sys
from bbio import *
from bbio.libraries.Servo import *

motors = []
for i in [PWM1A, PWM1B, PWM2A, PWM2B]:
    motors.append(Servo(i))

pid = {'y' : PID.PID(), 'p' : PID.PID(), 'r' : PID.PID()}

mpu = mpu6050.MPU6050()
mpu.dmpInitialize()
mpu.setDMPEnabled(True)
packetSize = mpu.dmpGetFIFOPacketSize()

cmds = { 'p' : 0, 'r' : 0, 'y' : 0, # desired pitch, roll and yaw
         'pp' : 0, 'pi' : 0, 'pd' : 0, # Pitch PID
         'rp' : 0, 'ri' : 0, 'rd' : 0, # Roll PID
         'yp' : 0, 'yi' : 0, 'yd' : 0, # Yaw PID
         't' : 0 }                     # Current throttle value

def parseInput(data):
    try:
        cmd, value = data.split(' ') # command is in the format <string>.split(<delimiter>, [<max-split>])
        cmds[cmd] = float(value)
    except ValueError:
        print "Invalid input!"
        return

    if cmd == 't':
        # value already set and is used directly from the dictionary
        pass
    elif cmd == 'p' or cmd == 'r' or cmd == 'y':
        # sets the set points for the p, r and y PIDs
        pid[cmd].SetPoint = float(value)
    else:
        # cmd[0] gives the PID (pitch, roll or yaw PID)
        # cmd[1] gives the constant to be set
        if cmd[1] == 'p':
            pid[cmd[0]].setKp(float(value))
        elif cmd[1] == 'i':
            pid[cmd[0]].setKi(float(value))
        elif cmd[1] == 'd':
            pid[cmd[0]].setKd(float(value))

def getMpuAngles():
    mpuIntStatus = mpu.getIntStatus()
    if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently)
        # get current FIFO count
        fifoCount = mpu.getFIFOCount()
        # check for overflow (this should never happen unless our code
        # is too inefficient)
        # The buffer size is actually 1024, but it starts to give bad
        # values close to 1024. Also, a smaller buffer means fresher
        # values.
        # A packet, by default, is 42 bytes in size. So 10 packets
        # should be 420 bytes
        if fifoCount == packetSize * 10:
        # reset so we can continue cleanly
            mpu.resetFIFO()
            print 'FIFO overflow!'
            fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()

        # fifoCount will be greater than 0 the first time
        while (fifoCount / packetSize) > 0:
            result = mpu.getFIFOBytes(packetSize)
            # track FIFO count here in case there is > 1 packet available
            # (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize

        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        mpuVal = mpu.dmpGetYawPitchRoll(q, g)

        for i in mpuVal:
            mpuVal[i] = mpuVal[i] * 180 / math.pi

        # track FIFO count here in case there is > 1 packet available
        # (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize

        return mpuVal
    else:
        return None

def updateMotors(mpuAngles):
    op = {}
    for i in mpuAngles:
        pid[i[0]].update(mpuAngles[i])
        op[i] = pid[i[0]].output

    motors[0].write(cmds['t'] + op['pitch'] - op['roll'] - op['yaw'])
    motors[1].write(cmds['t'] - op['pitch'] - op['roll'] + op['yaw'])
    motors[2].write(cmds['t'] - op['pitch'] + op['roll'] - op['yaw'])
    motors[3].write(cmds['t'] + op['pitch'] + op['roll'] + op['yaw'])

def loop():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            parseInput(sys.stdin.readline())

    mpuAngles = getMpuAngles()
    if (mpuAngles):
        updateMotors(mpuAngles)
        # debug info
        print "^",              # beginning delimiter
        for i in [
                mpuAngles['pitch'], mpuAngles['roll'], mpuAngles['yaw'],
                cmds['p'], cmds['r'], cmds['y'],
                cmds['pp'], cmds['pi'], cmds['pd'],
                cmds['rp'], cmds['ri'], cmds['rd'],
                cmds['yp'], cmds['yi'], cmds['yd'],
                motors[0].read(), motors[1].read(),
                motors[2].read(), motors[3].read()
        ]:
            print "%.2f" % (i),
        print "$"               # ending delimiter

try:

    while(True):
        loop()

except:
    for i in motors:
        i.detach()
    print "Cleaned up. Exiting..."
    raise                       # reraise exception
