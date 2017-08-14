import sys
import numpy as np
import PID
import VFH
import VFHP
import Braitenberg as bra

TARGET_X = 2.5
TARGET_Y = 3.5

M_VFH = 0
M_BRAIT = 1
M_VFHP = 2

# Robot differencial
class DiffRobot(object):

    def __init__(self, r=0.02, b=0.05, wm_max=6.28, c_type = 0):
        # r = radio de las ruedas [m]
        self.r = r

        # b = distancia entre las ruedas [m]
        self.b = b

        # wm_max = maxima velocidad angular de rotacion de las ruedas
        # omega max = maxima velocidad angular de rotacion del robot
        self.wm_max = wm_max
        self.omega_max = 0.9*r*wm_max/b
        self.v_max = 0.9*r*wm_max

        self.model = None
        self.c_type = c_type
        if c_type == M_VFH:
            print "Using VFH algorithm"
            VFH.OMEGA_MAX = self.omega_max
            VFH.V_MAX = self.v_max
            VFH.V_MIN = self.v_max*0.2
            self.model = VFH.VFHModel()
        elif c_type == M_BRAIT:
            print "Using Braitenberg algorithm"
            self.model = bra.BraitModel(bra.SMODE_FULL, 0.05, 0.3, -self.v_max*0.1, self.v_max, self.omega_max)
        elif c_type == M_VFHP:
            print "Using VFH+ algorithm"
            self.model = VFHP.VFHPModel()


        # Valores deseados de velocidad, velocidad angular y
        # orientacion
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.w_prev = 0.0

        self.cita = 0.0
        self.cita_prev = 0.0
        self.cita_ref = 0.0

        # Velocidades de los motores
        self.left_motor = 0.0
        self.right_motor = 0.0

        # Controlador PI para la orientacion del robot
        # Kp = 0.808, Ti = 3.5 (sintonizacion kogestad)
        self.angle_controller = PID.PID(1.2, 10.0, 0.0, self.omega_max, -self.omega_max)
        self.angle_controller.begin(0.0)

    # Todas las mediciones se dan en el sistema metrico [m, rad]
    def set_initial_pos(self, x, y, cita):
        
        if self.c_type == M_VFH:
            self.set_initial_pos_VFH(x,y,cita)
        elif self.c_type == M_BRAIT:
            self.set_initial_pos_Brait(x,y,cita)
        elif self.c_type == M_VFHP:
            self.set_initial_pos_VFHP(x,y,cita)
        else:
            print "ERROR: no control type defined!"


    def set_initial_pos_VFH(self, x, y, cita):
        self.cita = cita
        self.cita_prev = cita
        self.w_prev = 0.0
        self.model.update_position(x, y, np.degrees(cita))
        self.angle_controller.setInput(cita)
        self.angle_controller.setRef(cita)

    def set_initial_pos_Brait(self, x, y, cita):
        self.model.UpdatePos(x, y, cita)
        
    def set_initial_pos_VFHP(self, x, y, cita):
        pass

    def set_target(self, x, y):
        if self.c_type == M_VFH:
            self.model.set_target(x,y)
        elif self.c_type == M_BRAIT:
            self.model.SetTarget(x,y)
        elif self.c_type == M_VFHP:
            self.model.set_target(x,y)
        else:
            print "ERROR: no control type defined!"

    def unset_target(self):
        if self.c_type == M_VFH:
            self.model.set_target()
        elif self.c_type == M_BRAIT:
            self.model.SetTarget(0,0,True)
        elif self.c_type == M_VFHP:
            self.model.set_target()
        else:
            print "ERROR: no control type defined!"

    def update_pos(self, x, y, cita, delta_t):
        
        if self.c_type == M_VFH:
            self.update_pos_VFH(x,y,cita,delta_t)
        elif self.c_type == M_BRAIT:
            self.update_pos_Brait(x,y,cita,delta_t)
        elif self.c_type == M_VFHP:
            self.update_pos_VFHP(x,y,cita,delta_t)
        else:
            print "ERROR: no control type defined!"

    def update_pos_VFH(self, x, y, cita, delta_t):

        self.cita_prev = self.cita
        self.cita = cita
        self.w_prev = 0.7*PID.angleDiff(self.cita, self.cita_prev)/delta_t + 0.3*self.w_prev

        # El controlador VFH requiere angulos en grados
        self.model.update_position(x, y, np.degrees(cita))
        self.angle_controller.setInput(cita)
        self.w_ref = self.angle_controller.timestep(delta_t)
        print "PID readings: effort %f, error %f, acumulated_error %f " % (self.w_ref , self.angle_controller.error, self.angle_controller._integral)

    def update_pos_Brait(self, x, y, cita, delta_t):
        self.model.UpdatePos(x, y, cita)

    def update_pos_VFHP(self, x, y, cita, delta_t):
        pass

    def update_readings(self, data):
        if self.c_type == M_VFH:
            self.update_readings_VFH(data)
        elif self.c_type == M_BRAIT:
            self.update_readings_Brait(data)
        elif self.c_type == M_VFHP:
            self.update_readings_VFHP(data)
        else:
            print "ERROR: no control type defined!"

    def update_readings_VFH(self, data):
        try:
            self.model.update_obstacle_density(data)
        except Exception as e:
            print "Exception caught during sensor update:"
            print e

    def update_readings_Brait(self, data):
        self.model.UpdateSensors(data)

    def update_readings_VFHP(self, data):
        pass

    def update_target(self):
        if self.c_type == M_VFH:
            self.update_target_VFH()
        elif self.c_type == M_BRAIT:
            self.update_target_Brait()
        elif self.c_type == M_VFHP:
            self.update_target_VFHP()
        else:
            print "ERROR: no control type defined!"

    def update_target_VFH(self):
        self.model.update_active_window()
        self.model.update_polar_histogram()
        self.model.update_filtered_polar_histogram()

        if self.model.find_valleys() != -1:
            try:
                self.cita_ref = np.radians(self.model.calculate_steering_dir())
            except Exception as e:
                print "No valleys found, keeping current dir"
                self.cita_ref = np.radians(self.model.cita)

            self.v_ref = self.model.calculate_speed(self.w_prev)
            print "Obstacle detected! target dir is %f" % self.cita_ref
        else:
            print "No nearby obstacles keeping current dir"
            self.cita_ref = np.radians(self.model.cita)
            print "Setting target dir at %f" % self.cita_ref
            #self.w_ref = 0.0
            self.v_ref = self.v_max

        # Se actualiza el valor deseado de orientacion
        # en el controlador
        self.angle_controller.setRef(self.cita_ref)
        self.setMotorSpeed()

    def update_target_Brait(self):
        #v, w = self.model.Evade2b()
        v, w = self.model.Mixed2b3a()
        self.v_ref = v
        self.w_ref = w
        self.setMotorSpeed()

    def setMotorSpeed(self):
        self.left_motor = (self.v_ref - self.b*self.w_ref)/self.r
        self.right_motor = (self.v_ref + self.b*self.w_ref)/self.r

    def __str__(self):
        info = "(x=%f, y=%f, g=%f)\n" % (self.model.x_0, self.model.y_0, self.cita)
        ref = "(g_ref=%f, w_ref=%f, v_ref=%f)" % (self.cita_ref, self.w_ref, self.v_ref)
        struct = "Radio de las ruedas: %f\nDistancia entre las ruedas: %f\n" %(self.r, self.b)
        actu = "Vel Motores: [%f , %f]\n" % (self.left_motor, self.right_motor)
        return "%s%s%s%s" % (struct, info, ref, actu)

    def __print__(self):
        print str(self)



def main():
    # simulamos 50 ms 
    dt = 0.5

    robot = DiffRobot()
    robot.set_initial_pos(2.5, 2.5, 0)
    robot.update_pos(2.5,2.5,0,dt)

    pseudo_readings = np.float_([[0.25, np.radians(x)] for x in range(0,220,2)])
    robot.update_readings(pseudo_readings)
    robot.update_target()

    robot.update_pos(2.5,2.5,0,dt)
    robot.update_target()

    print robot.model._active_grid()

    print robot

if __name__ == "__main__":
    main()

