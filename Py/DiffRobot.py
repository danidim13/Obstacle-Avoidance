
# coding=utf8

import sys
import numpy as np
import PID
import VFH
import VFHP
import Braitenberg as bra

r"""
.. ::module DiffRobot

Este módulo define un modelo para el robot diferencial, que sirve de
interfaz entre los controladores de evasión y los motores.

"""

TARGET_X = 2.5
TARGET_Y = 3.5

M_VFH = 0
"""int: Modo de operación con VFH.
"""

M_BRAIT = 1
"""int: Modo de operación con vehículo de Braitenberg.
"""

M_VFHP = 2
"""int: Modo de operación con VFH+.
"""

# Robot differencial
class DiffModel(object):
    r"""Clase para el modelo del robot diferencial.

    Define un objeto que representa el robot diferencial, y permite
    abstraer la cinemática (comandos para los motores) a comandos
    de alto nivel.

    Parameters
    ----------
    r : float, opcional
        Radio de las ruedas (m).
    b : float, opcional
        Distancia entre las ruedas (m).
    wm_max : float, opcional
        Máxima velocidad angular de los motores (rad/s).
    c_type : {:const:`M_VFH`, :const:`M_BRAIT`, :const:`M_VFHP`}, opcional
        Tipo de controlador para la evasión de obstáculos.

    Attributes
    ----------
    r : float
        Radio de las ruedas (m).
    b : float
        Distancia entre las ruedas (m).
    wm_max : float
        Máxima velocidad angular del motor (rad/s).
    omega_max : float
        Máxima velocidad angular del robot (velocidad de giro, en rad/s).
    v_max : float
        Máxima velocidad lineal del robot (m/s).
    model : Controlador
        Una instacia de un controlador para evasión, se escoge a partir de ``c_type``.

    v_ref : float
        Valor de velocidad lineal de referencia para el movimiento del robot, definido por
        el controlador :attr:`model`.
    w_ref : float
        Valor de velocidad angular de referencia para el movimiento del robot (rad/s). Definido por
        el controlador :attr:`model`.
    w_prev : float
        Velocidad angular anterior (rad/s). Usado por el controlador VFH para calcular
        la velocidad angular actual.

    cita : float
        Dirección actual del robot (rad).
    cita_prev : float
        Dirección anterior del robot (rad), usado por el controlador VFH para calcular la
        velocidad angular actual.
    cita_ref : float
        Dirección de referencia para el movimiento del robot (rad). Se usa en el caso de los
        controladores VFH y VFH+, luego un PID determina la velocidad angular requerida
        :attr:`w_ref` para alcanzar esta dirección.

    left_motor : float
        Velocidad angular del motor izquierdo (rad/s).
    right_motor : float
        Velocidad angular del motor derecho (rad/s).

    angle_controller : :class:`PID.PID`
        Un controlador PID que se usa con los algoritmos VFH y VFH+ para determinar la
        velocidad angular de referencia :attr:`w_ref` a partir de la dirección de referencia
        :attr:`cita_ref`.
    """

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
            VFHP.OMEGA_MAX = self.omega_max
            VFHP.V_MAX = self.v_max
            VFHP.V_MIN = self.v_max*0.2
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
        r"""Define la postura inicial del robot.

        Indica las condiciones iniciales :math:`(x,\; y, \; \theta)` del robot.

        Parameters
        ----------
        x : float
            Posición absoluta del robot sobre el eje :math:`x`
            en metros.
        y : float
            Posición absoluta del robot sobre el eje :math:`y`
            en metros.
        cita : float
            Orientación del robot resepecto el eje :math:`z`
            en radianes.
        
        Notes
        -----
        Modifica el estado interno del controlador :attr:`model`.
        
        """
        
        if self.c_type == M_VFH:
            self._set_initial_pos_VFH(x,y,cita)
        elif self.c_type == M_BRAIT:
            self._set_initial_pos_Brait(x,y,cita)
        elif self.c_type == M_VFHP:
            self._set_initial_pos_VFHP(x,y,cita)
        else:
            print "ERROR: no control type defined!"


    def _set_initial_pos_VFH(self, x, y, cita):
        self.cita = cita
        self.cita_prev = cita
        self.w_prev = 0.0
        self.model.update_position(x, y, np.degrees(cita))
        self.angle_controller.setInput(cita)
        self.angle_controller.setRef(cita)

    def _set_initial_pos_Brait(self, x, y, cita):
        self.model.UpdatePos(x, y, cita)
        
    def _set_initial_pos_VFHP(self, x, y, cita):
        self.cita = cita
        #self.cita_prev = cita
        #self.w_prev = 0.0
        self.model.update_position(x, y, np.degrees(cita))
        self.angle_controller.setInput(cita)
        self.angle_controller.setRef(cita)

    def set_target(self, x, y):
        r"""Indica un punto objetivo.

        Habilita el seguimiento de trayectorias e indica
        al robot el punto al cual debe dirigirse.

        Parameters
        ----------
        x : float
            Posición absoluta del objetivo sobre el eje :math:`x`.
        y : float
            Posición absoluta del objetivo sobre el eje :math:`y`.

        Notes
        -----
        Modifica el estado interno del controlador :attr:`model`.
        """

        if self.c_type == M_VFH:
            self.model.set_target(x,y)
        elif self.c_type == M_BRAIT:
            self.model.SetTarget(x,y)
        elif self.c_type == M_VFHP:
            self.model.set_target(x,y)
        else:
            print "ERROR: no control type defined!"

    def unset_target(self):
        r"""Dehabilita el seguimiento de trayectorias.


        Notes
        -----
        Modifica el estado interno del controlador :attr:`model`.
        """
        if self.c_type == M_VFH:
            self.model.set_target()
        elif self.c_type == M_BRAIT:
            self.model.SetTarget(0,0,True)
        elif self.c_type == M_VFHP:
            self.model.set_target()
        else:
            print "ERROR: no control type defined!"

    def update_pos(self, x, y, cita, delta_t):
        r"""Actualiza la posición del robot.

        Parameters
        ----------
        x : float
            Posición absoluta del robot sobre el eje :math:`x`.
        y : float
            Posición absoluta del robot sobre el eje :math:`y`.
        cita : float
            Orientación del robot resepecto el eje :math:`z`
            en radianes.
        delta_t : float
            El tiempo transcurrido desde la última actualización.
            
        Notes
        -----
        Modifica el estado interno del controlador :attr:`model`,
        además del atributo :attr:`cita`. En el caso de VFH y VFH+
        realiza un paso de integración del PID.
        """
        
        if self.c_type == M_VFH:
            self._update_pos_VFH(x,y,cita,delta_t)
        elif self.c_type == M_BRAIT:
            self._update_pos_Brait(x,y,cita,delta_t)
        elif self.c_type == M_VFHP:
            self._update_pos_VFHP(x,y,cita,delta_t)
        else:
            print "ERROR: no control type defined!"

    def _update_pos_VFH(self, x, y, cita, delta_t):

        self.cita_prev = self.cita
        self.cita = cita
        self.w_prev = 0.7*PID.angleDiff(self.cita, self.cita_prev)/delta_t + 0.3*self.w_prev

        # El controlador VFH requiere angulos en grados
        self.model.update_position(x, y, np.degrees(cita))
        self.angle_controller.setInput(cita)
        self.w_ref = self.angle_controller.timestep(delta_t)
        print "PID readings: effort %f, error %f, acumulated_error %f " % (self.w_ref , self.angle_controller.error, self.angle_controller._integral)

    def _update_pos_Brait(self, x, y, cita, delta_t):
        self.cita = cita
        self.model.UpdatePos(x, y, cita)

    def _update_pos_VFHP(self, x, y, cita, delta_t):

        self.cita = cita
        self.model.update_position(x, y, np.degrees(cita))

        self.angle_controller.setInput(cita)
        self.w_ref = self.angle_controller.timestep(delta_t)
        print "PID readings: effort %f, error %f, acumulated_error %f " % (self.w_ref , self.angle_controller.error, self.angle_controller._integral)

    def update_readings(self, data):
        """Procesa las lecturas de un sensor.

        Actualiza el estado del controlador :attr:`model`
        con las lecturas de un sensor de distancia.

        Parameters
        ----------
        data : ndarray
            Una estructura de datos tipo ``numpy.ndarray`` que
            contiene las lecturas de un sensor de distancias.
            Debe ser un arreglo de dimensiones
            :math:`(n\times 2)` para :math:`n` puntos o
            lecturas, donde cada par representa una coordenada
            polar :math:`(r,\theta)` respecto al marco de
            referencia del robot, dada en metros y radianes.

        """
        if self.c_type == M_VFH:
            self._update_readings_VFH(data)
        elif self.c_type == M_BRAIT:
            self._update_readings_Brait(data)
        elif self.c_type == M_VFHP:
            self._update_readings_VFHP(data)
        else:
            print "ERROR: no control type defined!"

    def _update_readings_VFH(self, data):
        try:
            self.model.update_obstacle_density(data)
        except Exception as e:
            print "Exception caught during sensor update:"
            print e

    def _update_readings_Brait(self, data):
        self.model.UpdateSensors(data)

    def _update_readings_VFHP(self, data):
        try:
            self.model.update_obstacle_density(data)
        except Exception as e:
            print "Exception caught during sensor update:"
            print e
        pass

    def update_target(self):
        r"""Aplica la acción de control para evasión de obstáculos.

        Actualiza el estado interno de :attr:`model` para obtener
        la acción de control :math:`(v_{ref}, w_{ref})`. Luego,
        la convierte en acciones individuales sobre cada motor
        mediante el método :meth:`setMotorSpeed`.

        Notes
        -----
        Modifica los siguientes atributos de clase:

        .. hlist::

            * :attr:`v_ref`
            * :attr:`cita_ref`
            * :attr:`w_ref`
            * :attr:`left_motor`
            * :attr:`right_motor`
        """
        if self.c_type == M_VFH:
            self._update_target_VFH()
        elif self.c_type == M_BRAIT:
            self._update_target_Brait()
        elif self.c_type == M_VFHP:
            self._update_target_VFHP()
        else:
            print "ERROR: no control type defined!"

    def _update_target_VFH(self):
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

    def _update_target_Brait(self):
        if self.model.target == None:
            v, w = self.model.Evade2b()
        else:
            v, w = self.model.Mixed2b3a()
        self.v_ref = v
        self.w_ref = w
        self.setMotorSpeed()

    def _update_target_VFHP(self):
        self.model.update_active_window()
        self.model.update_polar_histogram()
        self.model.update_bin_polar_histogram()
        self.model.update_masked_polar_hist(self.b,self.b)

        try:
            result = self.model.find_valleys()
            cita, v = self.model.calculate_steering_dir(result)
        except Exception as e:
            print "Exception caught on VHF+ control loop"
            print e
            cita = self.cita_ref
            v = self.v_ref

        self.v_ref = v
        self.cita_ref = np.radians(cita)
        self.angle_controller.setRef(self.cita_ref)
        self.setMotorSpeed()

    def setMotorSpeed(self):
        r"""Determina la velocidad de los motores.

        Determina la velocidad individual que debe tener cada
        motor (en rad/s), según el modelo cinemático inverso, para
        que el robot alcance la velocidad lineal y angular de
        referencia :attr:`v_ref` y :attr:`w_ref`. 

        Notes
        -----
        Modifica los siguientes atributos de clase:

        .. hlist::

            * :attr:`left_motor`
            * :attr:`right_motor`
        """
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

    robot = DiffModel(c_type=M_VFHP)
    robot.set_initial_pos(2.5, 2.5, 0)
    robot.update_pos(2.5,2.5,0,dt)

    pseudo_readings = np.float_([[0.25, np.radians(x)] for x in range(0,220,1)])
    robot.update_readings(pseudo_readings)
    robot.update_target()

    robot.update_pos(2.5,2.5,0,dt)
    robot.update_target()

    print robot.model._active_grid()

    print robot

if __name__ == "__main__":
    main()

