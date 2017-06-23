import sys
import numpy as np
import PID
import VFH

# Robot differencial
class DiffRobot(object):

    def __init__(self, r=0.02, b=0.05, w_max=0.261):
        # r = radio de las ruedas [m]
        self.r = r

        # b = distancia entre las ruedas [m]
        self.b = b

        # wm = maxima velocidad de rotacion del robot [rad/s]
        self.w_max = w_max

        self.model = VFH.VFHModel()

        # Valores deseados de velocidad, velocidad angular y
        # orientacion
        self.v_ref = 0.0
        self.w_ref = 0.0

        self.cita = 0.0
        self.cita_ref = 0.0

        # Velocidades de los motores
        self.left_motor = 0.0
        self.right_motor = 0.0

        # Controlador PI para la orientacion del robot
        # Kp = 0.808, Ti = 3.5 (sintonizacion kogestad)
        self.angle_controller = PID.PID(0.808, 3.5, 0, w_max, -w_max)
        self.angle_controller.begin(0.0)

    # Todas las mediciones se dan en el sistema metrico [m, rad]
    def set_initial_pos(self, x, y, cita):
        self.cita = cita
        self.model.update_position(x, y, np.degrees(cita))
        self.angle_controller.setInput(cita)
        self.angle_controller.setRef(cita)

    def update_pos(self, x, y, cita, delta_t):
        self.cita = cita
        # El controlador VFH requiere angulos en grados
        self.model.update_position(x, y, np.degrees(cita))
        self.angle_controller.setInput(cita)
        self.w_ref = self.angle_controller.timestep(delta_t)

    def update_readings(self, data):
        try:
            self.model.update_obstacle_density(data)
        except Exception as e:
            print "Exception caught during sensor update:"
            print e

    def update_target(self):
        self.model.update_active_window()
        self.model.update_polar_histogram()
        self.model.update_filtered_polar_histogram()

        if self.model.find_valleys() != -1:
            print "Obstacle detected!"
            self.cita_ref = np.radians(self.model.calculate_steering_dir())
            self.v_ref = self.model.calculate_speed()
        else:
            print "No nearby obstacles"
            self.cita_ref = self.cita
            self.v_ref = self.r*3.14

        # Se actualiza el valor deseado de orientacion
        # en el controlador
        self.angle_controller.setRef(self.cita_ref)
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

