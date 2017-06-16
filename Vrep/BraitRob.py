#! /usr/bin/python
# python

import sys
import time
import math

windows_path = 'C:\\Program Files (x86)\\V-REP3\\V-REP_PRO_EDU\\Obstacle-Avoidance\\Py'
linux_path = '/home/daniel/Documents/UCR/XI Semestre/Proyecto/Codigo/Obstacle-Avoidance/Py'
#sys.path.insert(0, '/home/daniel/Documents/UCR/XI Semestre/Proyecto/Codigo/Obstacle-Avoidance/Py')
#print sys.path
#sys.path.insert(0, windows_path)
sys.path.insert(0, linux_path)
#print sys.path
import Braitenberg as brait

#print "Redirecting stdout and stderr to logfile BubbleRob.log"
#sys.stdout.flush()
#logfile = open('BubbleRob.log', 'w')
#sys.stdout = logfile
print "LOG START"


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


try:
    import msvcrt as m
    def wait():
        sys.stdout.flush()
        m.getch()

except Exception as e:
    print e
    print "Defaulting wait function"
    def wait():
        sys.stdout.flush()
        raw_input("")


def runSim(argc, argv):

    #####################################
    ### First start up the conection  ###
    ### and parse arguments           ###

    portNb = 0
    leftMotorHandle = None
    rightMotorHandle = None
    leftSensorHandle = None
    rightSensorHandle = None

    print "Initializing values"
    if argc == 6:
        portNb = int(argv[1])
        leftMotorHandle = int(argv[2])
        rightMotorHandle = int(argv[3])
        leftSensorHandle = int(argv[4])
        rightSensorHandle = int(argv[5])
    else:
        print "Arguments error"
        time.sleep(5)
        return 0

    clientID = vrep.simxStart("127.0.0.1",portNb,True,True,2000,5)

    ###                               ###
    #####################################

    if clientID != -1:

        ### Succesfull Conection!
        print "%s Conected to V-Rep" % argv[0]
    
        ### Global var definition
        #driveBackStartTime = -99000
        motorSpeeds = [0., 0.]
        sensorReadings = [0., 0.]

        #############################
        ### Main Simulation Loop ###
        while (vrep.simxGetConnectionId(clientID) != -1):

            print "Reading sensors..."
            ## Read Sensors
            leftReturnCode, leftSensorTrigger, leftData, leftDist = vrep.simxReadProximitySensor(clientID,leftSensorHandle,vrep.simx_opmode_streaming)[0:4]
            rightReturnCode, rightSensorTrigger, rightData, rightDist = vrep.simxReadProximitySensor(clientID,rightSensorHandle,vrep.simx_opmode_streaming)[0:4]

            if (leftReturnCode == vrep.simx_return_ok and rightReturnCode == vrep.simx_return_ok):
            ## We succeeded at reading the proximity sensor
                dleft = brait.D_MAX
                dright = brait.D_MAX

                if (leftSensorTrigger):
                    print "Detected something on the left!"
                    print leftData
                    print leftDist
                    dleft = math.sqrt(sum([x**2 for x in leftData]))
                if (rightSensorTrigger):
                    print "Detected something on the right!"
                    print rightData
                    print rightDist
                    dright = math.sqrt(sum([x**2 for x in rightData]))
                vleft, vright = brait.Braitenberg2b(dleft, dright)

                motorSpeeds[0] = vleft
                motorSpeeds[1] = vright


            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, motorSpeeds[0], vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, motorSpeeds[1], vrep.simx_opmode_oneshot)
            time.sleep(0.005)

        ### End of Simulation loop ###
        ##############################

        vrep.simxFinish(clientID)

    return 0

if __name__ == "__main__":
    try:
        runSim(len(sys.argv), sys.argv)
    except Exception as inst:
        print type(inst)
        print inst.args
        print inst
        print "press a key"
        wait()

    print "End of exectution"
    wait()

#logfile.close()
