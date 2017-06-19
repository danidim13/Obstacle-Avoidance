#! python
# /usr/bin/python

import sys
import time
import math
import numpy as np

windows_path = 'C:\\Program Files (x86)\\V-REP3\\V-REP_PRO_EDU\\Obstacle-Avoidance\\Py'
linux_path = '/home/daniel/Documents/UCR/XI Semestre/Proyecto/Codigo/Obstacle-Avoidance/Py'
#sys.path.insert(0, '/home/daniel/Documents/UCR/XI Semestre/Proyecto/Codigo/Obstacle-Avoidance/Py')
#print sys.path
sys.path.insert(0, windows_path)
#sys.path.insert(0, linux_path)
#print sys.path
import Braitenberg as brait

logfile_name = 'HokuyoRob.log'
print "Redirecting stdout and stderr to logfile %s" % logfile_name
sys.stdout.flush()
logfile = open(logfile_name, 'w')
sys.stdout = logfile
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
    if argc == 9:
        portNb = int(argv[1])
        leftMotorHandle = int(argv[2])
        rightMotorHandle = int(argv[3])
        leftSensorHandle = int(argv[4])
        rightSensorHandle = int(argv[5])
        laserSignalName = str(argv[6])
        posSignalName = str(argv[7])
        oriSignalName = str(argv[8])
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
    
        #############################
        ### Global var definition ###
        ### and initialization    ###
        
        motorSpeeds = [0., 0.]
        sensorReadings = [0., 0.]

        # Laser Signal
        laserReturnCode, laserSignal = vrep.simxGetStringSignal(clientID, laserSignalName, vrep.simx_opmode_streaming)
        if laserReturnCode == vrep.simx_return_ok:
            print "Laser Signal returned ok on first call, this is unexpeted"
        elif laserReturnCode == vrep.simx_return_novalue_flag:
            print "Laser Signal stream opened succesfully!"
        else:
            print "ERROR: failed to open the Laser Signal stream"
            raise Exception("Error while opening %s StringSignal, call returned %d" % (laserSignal, laserReturnCode))

        # Position Signal
        posReturnCode, posSignal = vrep.simxGetStringSignal(clientID, posSignalName, vrep.simx_opmode_streaming)
        if posReturnCode == vrep.simx_return_ok:
            print "Pos Signal returned ok on first call, this is unexpeted"
        elif posReturnCode == vrep.simx_return_novalue_flag:
            print "Pos Signal stream opened succesfully!"
        else:
            print "ERROR: failed to open the Pos Signal stream"
            raise Exception("Error while opening %s StringSignal, call returned %d" % (posSignal, posReturnCode))

        # Orientation Signal
        oriReturnCode, oriSignal = vrep.simxGetStringSignal(clientID, oriSignalName, vrep.simx_opmode_streaming)
        if oriReturnCode == vrep.simx_return_ok:
            print "Ori Signal returned ok on first call, this is unexpeted"
        elif oriReturnCode == vrep.simx_return_novalue_flag:
            print "Ori Signal stream opened succesfully!"
        else:
            print "ERROR: failed to open the Ori Signal stream"
            raise Exception("Error while opening %s StringSignal, call returned %d" % (oriSignal, oriReturnCode))

        prevLaserSignal = ""
        ### End of initialization ###
        #############################
        

        ############################
        ### Main Simulation Loop ###
        while (vrep.simxGetConnectionId(clientID) != -1):


            ### Read Sensors ###

            #print "Reading sensors..."
            leftReturnCode, leftSensorTrigger, leftData, leftDist = vrep.simxReadProximitySensor(clientID,leftSensorHandle,vrep.simx_opmode_streaming)[0:4]
            rightReturnCode, rightSensorTrigger, rightData, rightDist = vrep.simxReadProximitySensor(clientID,rightSensorHandle,vrep.simx_opmode_streaming)[0:4]

            posReturnCode, posSignal = vrep.simxGetStringSignal(clientID, posSignalName, vrep.simx_opmode_buffer)
            oriReturnCode, oriSignal = vrep.simxGetStringSignal(clientID, oriSignalName, vrep.simx_opmode_buffer)
            laserReturnCode, laserSignal = vrep.simxGetStringSignal(clientID, laserSignalName, vrep.simx_opmode_buffer)

            # Process position
            if posReturnCode == vrep.simx_return_ok:
                print "Pos Signal read"
                posData = vrep.simxUnpackFloats(posSignal)
                print posData

            # Process orientation
            if oriReturnCode == vrep.simx_return_ok:
                print "Ori Signal read"
                oriData = vrep.simxUnpackFloats(oriSignal)
                print oriData

            ## Process Laser Sensor ##
            if laserReturnCode == vrep.simx_return_ok:
                print "Laser Signal read"

                if laserSignal == prevLaserSignal:
                    print "No new data in Laser Signal, skipping"

                else:
                    prevLaserSignal = laserSignal
                    laserData = vrep.simxUnpackFloats(laserSignal)
                    if len(laserData)%3 != 0:
                        print "ERROR: unexpected number of laser data floats"

                    total_points = len(laserData)/3
                    laserPoints = np.float_(laserData).reshape((total_points,3))

                    print "Total %d floats, %d points" % (len(laserData), total_points)
                    print laserPoints
                    for i in xrange(total_points):
                        if not np.array_equal(laserPoints[i, :], [0, 0, 0]):
                            print laserPoints[i, :]
                    radians = np.arctan2(laserPoints[:,1],laserPoints[:,0])
                    dist = np.sqrt(np.square(laserPoints[:,0]) + np.square(laserPoints[:,1]))
                    new_data = np.vstack((dist,radians)).T


            elif laserReturnCode == vrep.simx_return_novalue_flag:
                print "Laser Signal didn't have a value ready"
            else:
                print "ERROR: failed to read Laser Signal"


            ## Process Proximity sensors ##
            if (leftReturnCode == vrep.simx_return_ok and rightReturnCode == vrep.simx_return_ok):
                # We succeeded at reading the proximity sensor
                dleft = brait.D_MAX
                dright = brait.D_MAX

                if (leftSensorTrigger):
                    #print "Detected something on the left!"
                    #print leftData
                    #print leftDist
                    dleft = math.sqrt(sum([x**2 for x in leftData]))
                if (rightSensorTrigger):
                    #print "Detected something on the right!"
                    #print rightData
                    #print rightDist
                    dright = math.sqrt(sum([x**2 for x in rightData]))
                vleft, vright = brait.Braitenberg2b(dleft, dright)

                motorSpeeds[0] = vleft
                motorSpeeds[1] = vright


            ## Set motor speeds
            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, motorSpeeds[0], vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, motorSpeeds[1], vrep.simx_opmode_oneshot)
            time.sleep(0.05)

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
        #print "press a key"
        #wait()

    print "End of exectution"
    #wait()

logfile.close()
