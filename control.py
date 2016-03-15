import motor as Motor
import pid as PID
import math
import mpu6050
import logging
import select
import sys

motors = []
for i in ["P9_14", "P9_16", "P9_21", "P9_22"]:
    motors.append(Motor.motor(i))

pid = {'y' : PID.PID(), 'p' : PID.PID(), 'r' : PID.PID()}

mpu = mpu6050.MPU6050()
mpu.dmpInitialize()
mpu.setDMPEnabled(True)
packetSize = mpu.dmpGetFIFOPacketSize()

cmds = {'p' : 0, 'r' : 0, 'y' : 0,
        'pp' : 0, 'pi' : 0, 'pd' : 0,
        'rp' : 0, 'ri' : 0, 'rd' : 0,
        'yp' : 0, 'yi' : 0, 'yd' : 0,
        't' : 0}

def parseInput(data):
    cmd, value = str.split(' ', data)
    cmds[cmd] = value

    if cmd == 't':
        # value already set and is used directly from the dict
        pass
    elif cmd == 'p' or cmd == 'r' or cmd == 'y':
        # sets the set points for the p, r and y PIDs
        pid[cmd].setPoint(value)
    else:
        # cmd[0] gives the PID (pitch, roll or yaw PID)
        # cmd[1] gives the constant to be set
        if cmd[1] == 'p':
            pid[cmd[0]].setKp(value)
        elif cmd[1] == 'i':
            pid[cmd[0]].setKi(value)
        elif cmd[1] == 'd':
            pid[cmd[0]].setKd(value)

def loop():

    mpuIntStatus = mpu.getIntStatus()
    if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently)
        # get current FIFO count
        fifoCount = mpu.getFIFOCount()
        # check for overflow (this should never happen unless our code is too inefficient)
        if fifoCount == 1024:
        # reset so we can continue cleanly
            mpu.resetFIFO()
            logging.debug('FIFO overflow!')
        fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()

        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        mpuVal = mpu.dmpGetYawPitchRoll(q, g)

        for i in mpuVal:
            mpuVal[i] = mpuVal[i] * 180 / math.pi

        # track FIFO count here in case there is > 1 packet available
        # (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            parseInput(sys.stdin.readline())

        op = {}
        for i in mpuVal:
            op[i] = pid[i[0]].update(mpuVal[i])

        motors[0].setPower(cmds['t'] + op['pitch'] - op['roll'] - op['yaw'])
        motors[1].setPower(cmds['t'] - op['pitch'] - op['roll'] + op['yaw'])
        motors[2].setPower(cmds['t'] - op['pitch'] + op['roll'] - op['yaw'])
        motors[3].setPower(cmds['t'] + op['pitch'] + op['roll'] + op['yaw'])

        # debug info
        print "%f %f %f " % (mpuVal['pitch'], mpuVal['roll'], mpuVal['yaw']),
        print "%f %f %f " % (cmds['p'], cmds['r'], cmds['y']),
        print "%f %f %f " % (cmds['pp'], cmds['pi'], cmds['pd']),
        print "%f %f %f " % (cmds['rp'], cmds['ri'], cmds['rd']),
        print "%f %f %f " % (cmds['yp'], cmds['yi'], cmds['yd']),
        for i in motors:
            print "%f " % (i.getPower()),
        print ""

while(True):
    loop()

