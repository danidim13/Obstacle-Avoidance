#! /usr/bin/python

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import sys
import time
import msvcrt as m

def wait():
    m.getch()

print "Hello"

#time.sleep(50)

#while True:
#    pass



def main(argc, argv):
    portNb = 0
    leftMotorHandle = None
    rightMotorHandle = None
    sensorHandle = None

    if argc >= 5:
        portNb = int(argv[1])
        leftMotorHandle = int(argv[2])
        rightMotorHandle = int(argv[3])
        sensorHandle = int(argv[4])
    else:
        print "Arguments error"
        time.sleep(5)
        return 0

    clientID = vrep.simxStart("127.0.0.1",portNb,True,True,2000,5)

    if clientID != -1:
    #{
        # Succes!
        driveBackStartTime = -99000
        motorSpeeds = [0., 0.]

        while (vrep.simxGetConnectionId(clientID) != -1):
        ##{
            returnCode, sensorTrigger = vrep.simxReadProximitySensor(clientID,sensorHandle,vrep.simx_opmode_streaming)[0:2]
            if (returnCode == vrep.simx_return_ok):
            ##{ We succeeded at reading the proximity sensor
                simulationTime = vrep.simxGetLastCmdTime(clientID)
                if (simulationTime-driveBackStartTime < 3000):
                ##{ // driving backwards while slightly turning:
                    motorSpeeds[0] = -3.1415*0.5
                    motorSpeeds[1] = -3.1415*0.25
                ##}
                else:
                ##{ // going forward:
                    motorSpeeds[0] = 3.1415
                    motorSpeeds[1] = 3.1415
                    if (sensorTrigger):
                        driveBackStartTime = simulationTime # // We detected something, and start the backward mode
                ##}
                vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, motorSpeeds[0], vrep.simx_opmode_oneshot)
                vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, motorSpeeds[1], vrep.simx_opmode_oneshot)
            ##}
            time.sleep(0.005)
        ##}
        vrep.simxFinish(clientID)
    ##}
    return 0

try:
    main(len(sys.argv), sys.argv)
except Exception as inst:
    print type(inst)
    print inst.args
    print inst
    print "press a key"
    wait()

print "End of exectution"
wait()

