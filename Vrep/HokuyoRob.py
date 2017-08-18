#! python
# /usr/bin/python

import sys
import time
import math
import numpy as np
import matplotlib.pyplot as plt

logfile_name = 'HokuyoRob.log'
print "Redirecting stdout and stderr to logfile %s" % logfile_name
sys.stdout.flush()
logfile = open(logfile_name, 'w')
sys.stdout = logfile
print "LOG START"

windows_path = 'C:\\Program Files (x86)\\V-REP3\\V-REP_PRO_EDU\\Obstacle-Avoidance\\Py'
linux_path = '/home/daniel/Documents/UCR/XI Semestre/Proyecto/Codigo/Obstacle-Avoidance/Py'

sys.path.insert(0, windows_path)
#sys.path.insert(0, linux_path)

import Braitenberg as brait
import DiffRobot as DR
import VFH as vfh



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
        
        #robot = vfh.VFHModel()
        print "Iniciando el robot!"

        #modo = DR.M_VFH
        modo = DR.M_BRAIT

        data_filename = "sim_data_{:d}.csv".format(modo)
        data_dump = open(data_filename, 'w')


        robot = DR.DiffRobot(c_type=modo)

        # Pista 1
        #robot.set_target(2.5,3.0) 

        # Pista 2
        #robot.set_target(2.6,3.1) 

        # Pista 3
        robot.set_target(2.75,3.5) 


        X_TRAS = 2.5
        Y_TRAS = 2.5
        G_TRAS = 0.0
        robot.set_initial_pos(X_TRAS, Y_TRAS,0)

        # Sin objetivo
        #target = None
        # Pista 1
        #target = np.array([2.5-X_TRAS,3.0-Y_TRAS])
        # Pista 2
        #target = np.array([2.6-X_TRAS,3.1-Y_TRAS])
        # Pista 3
        target = np.array([2.75-X_TRAS,3.5-Y_TRAS])

        simTime = 0.0

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

            laserReturnCode, laserSignal = vrep.simxGetStringSignal(clientID, laserSignalName, vrep.simx_opmode_buffer)
            posReturnCode, posSignal = vrep.simxGetStringSignal(clientID, posSignalName, vrep.simx_opmode_buffer)
            oriReturnCode, oriSignal = vrep.simxGetStringSignal(clientID, oriSignalName, vrep.simx_opmode_buffer)

            now = vrep.simxGetLastCmdTime(clientID)
            if (now == simTime):
                #print "Simulation has not advanced, skipping..."
                continue
            print "\nSIMTIME ", now
            delta_t = (now - simTime)/100.0
            simTime = now

            ######################
            ## Process position ##
            if posReturnCode == vrep.simx_return_ok and oriReturnCode == vrep.simx_return_ok:
                print "Pos Signal read"
                print "Ori Signal read"
                x, y, z = vrep.simxUnpackFloats(posSignal)
                alpha, beta, gamma = vrep.simxUnpackFloats(oriSignal)
                robot.update_pos(x + X_TRAS, y + Y_TRAS, gamma, delta_t)
                #print posData
                #print oriData
                print "current pos: %f, %f, %f" % (x, y, gamma)

                #print "Acording to robot: %f, %f, %f" % (robot.model.x_0, robot.model.y_0, robot.model.cita)
                #print "Acording to robot: %f, %f, %f" % (robot.model.x, robot.model.y, robot.model.gamma)
            elif posReturnCode == vrep.simx_return_novalue_flag or oriReturnCode == vrep.simx_return_novalue_flag:
                print "Pos Signal didn't have a value ready"
            else:
                print "ERROR: failed to read Pos Signal"
            ##                  ##
            ######################


            ##########################
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

                    radians = np.arctan2(laserPoints[:,1],laserPoints[:,0])
                    dist = np.sqrt(np.square(laserPoints[:,0]) + np.square(laserPoints[:,1]))
                    new_data = np.vstack((dist,radians)).T
                    robot.update_readings(new_data)

            elif laserReturnCode == vrep.simx_return_novalue_flag:
                print "Laser Signal didn't have a value ready"
            else:
                print "ERROR: failed to read Laser Signal"
            ##                      ##
            ##########################


            ################################
            ## Main control logic for VFH ##
            if posReturnCode == vrep.simx_return_ok and \
                    oriReturnCode == vrep.simx_return_ok and \
                    laserReturnCode == vrep.simx_return_ok:
                robot.update_target()

            ## Set motor speeds
            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, robot.left_motor, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, robot.right_motor, vrep.simx_opmode_oneshot)
            ##                            ##
            ################################

            #######################
            ## Dump data to file ##
            if posReturnCode == vrep.simx_return_ok and oriReturnCode == vrep.simx_return_ok:
                
                if target is None:
                    pass
                    csv_line = "{:.5f},{:.5f},{:.5f},{:.5f}\n".format(now,x,y,gamma)
                    data_dump.write(csv_line)
                    if now >= 5000:
                        print "Robot has run for 10 seconds!"
                        vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, vrep.simx_opmode_oneshot)
                        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, vrep.simx_opmode_oneshot)
                        break
                else:
                    d = np.sqrt(np.square(target[0] - x) + np.square(target[1] - y))
                    csv_line = "{:.5f},{:.5f},{:.5f},{:.5f},{:.5f}\n".format(now,x,y,gamma,d)
                    data_dump.write(csv_line)
                    if d < 0.010:
                        print "Robot has reached its target!"
                        vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, vrep.simx_opmode_oneshot)
                        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, vrep.simx_opmode_oneshot)
                        break
            ##                   ##
            #######################
            

            time.sleep(0.05)

        ### End of Simulation loop ###
        ##############################
        vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0.0, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0.0, vrep.simx_opmode_oneshot)
        time.sleep(0.5)

        vrep.simxFinish(clientID)

        print "\nEnd of Simulation"
        data_dump.close()

        if robot.c_type == DR.M_VFH:
            print "\nRobot position and orientation"
            print robot.model.x_0, robot.model.y_0, robot.model.cita

            print "\nRobot i, j, k"
            print robot.model.i_0, robot.model.j_0, robot.model.k_0

            print "\nRobot active grid"
            print robot.model._active_grid()

            print "\nRobot polar histogram"
            print robot.model.polar_hist

            print "\nRobot filtered histogram"
            print robot.model.filt_polar_hist

            print "\nValleys"
            print robot.model.valleys
            sys.stdout.flush()

            # Figuras y graficos
            plt.figure(1)
            x = [vfh.ALPHA*x for x in range(len(robot.model.filt_polar_hist))]
            i = [a for a in range(len(robot.model.filt_polar_hist))]
            plt.bar(x, robot.model.polar_hist, 4.0, 0, color='r')
            plt.title("Histograma polar")

            plt.figure(2)
            plt.bar(x, robot.model.filt_polar_hist, 4.0, 0, color='b')
            plt.title("Histograma polar filtrado")

            plt.figure(3)
            plt.pcolor(robot.model._active_grid().T, alpha=0.75, edgecolors='k',vmin=0,vmax=20)
            plt.xlabel("X")
            plt.ylabel("Y", rotation='horizontal')
            plt.show()

        if robot.c_type == DR.M_BRAIT:
            print "BraitRob end!"

    return 0

if __name__ == "__main__":
    np.set_printoptions(threshold=np.inf)
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
