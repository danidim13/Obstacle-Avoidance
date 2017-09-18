#! /usr/bin/python
# coding=utf8

r"""

Este módulo define el controlador para evasión mediante el
algoritmo VFH, y constantes asociadas.

"""

import numpy as np
import matplotlib.pyplot as plt
import copy


########################################
###            Constantes           ####
########################################
###                                 ####
###                                 ####

# Size of the full Grid
GRID_SIZE = 125
r"""int: Tamaño de la cuadrícula de certeza.
"""

# Resolution of each cell (in m)
RESOLUTION = np.float_(0.04)
r"""float: Resolución de cada celda (en m).
"""

# Size of the active window that
# travels with the robot
WINDOW_SIZE = 15
r"""int: Tamaño de la ventana activa.

.. warning::
    La ventana activa debe tener un número impar de celdas.
"""
assert WINDOW_SIZE%2 == 1, "Window should have an odd number of cells for better results"

WINDOW_CENTER = WINDOW_SIZE/2
r"""int: Índice de la celda central en la ventana activa.

Se define automáticamente a partir de ``WINDOW_SIZE``.
"""

# Size of each polar sector
# in the polar histogram (in degrees)
ALPHA = 5
r"""int: Tamaño de cada sector del histograma polar (en grados).

.. warning::
    ``ALPHA`` debe ser un divisor de 360.
"""
if np.mod(360, ALPHA) != 0:
    raise ValueError("Alpha must define an int amount of sectors")

HIST_SIZE = 360/ALPHA
r"""int: Cantidad de sectores en el histograma polar.
Se define automáticamente a partir de ``ALPHA``.
"""

# Valley/Peak threshold
THRESH = 20000.0
r"""float: Valor de umbral para valles y picos.
"""

WIDE_V = HIST_SIZE/4
r"""int: Tamaño límite de un valle ancho y angosto.
"""
V_MAX = 0.0628
r"""float: Velocidad máxima del robot (m/s).
"""

V_MIN = 0.00628
r"""float: Velocidad mínima del robot (m/s).
"""

OMEGA_MAX = 1.256
r"""float: Velocidad angular máxima del robot (rad/s)
"""

# Constants for virtual vector magnitude calculations
# D_max^2 = 2*(ws-1/2)^2
# A - B*D_max^2 = 1
D_max2 = np.square(WINDOW_SIZE-1)/2.0
B = np.float_(1.0)
r"""float: Constante :math:`b` de la ecuación de la magnitud del
vector de obstáculos.
"""

A = np.float_(1+B*D_max2)
r"""float: Constante :math:`a` de la ecuación de la magnitud del
vector de obstáculos. Se define a partir de ``B`` de modo que
se cumpla :math:`a - b\cdot d_{max}^2 = 1`.
"""

# Active window array indexes
MAG = 0
BETA = 1
DIST2 = 2

###                                 ####
###                                 ####
########################################

class VFHModel:
    r"""Clase que define el controlador para evasión mediante
    el algoritmo VFH.

    Attributes
    ----------
    obstacle_grid : ndarray of short
        La cuadrícula de certeza. Cada celda tiene un valor entre
        0 y 20 que indica que tan probable es que haya un
        obstáculo ocupando la celda.

    active_window : ndarray of float
        La ventana activa que se mueve con el robot. Cada celda
        contiene tres valores: la magnitud del vector de
        obstaculos :math:`m_{i,j}`, la dirección del vector
        :math:`\beta_{i,j}`, y la distancia al robot
        :math:`d_{i,j}`.

    polar_hist : ndarray of float
        El histograma polar. Indica la densidad de obstáculos
        en cada sector alrededor de la vecindad del robot.

    filt_polar_hist : ndarray of float
        El histograma polar filtrado. Se construye a aplicando
        un filtro al histograma polar.

    valleys : list of 2-tuples
        Contiene una lista de los valles en el histograma polar.
        Cada es un par :math:`(s_1,s_2)`, donde :math:`s_1` es
        el sector donde inicia y :math:`s_2` donde termina, en
        sentido antihorario.

    x_0 : float
        Posición del robot sobre el eje :math:`x`.
    y_0 : float
        Posición del robot sobre el eje :math:`y`.
    cita : float
        Orientación del robot resepecto el eje :math:`z`.

    i_0 : float
        Índice de la celda que ocupa el robot sobre la cuadrícula
        de certeza en el eje :math:`x`.
    j_0 : float
        Índice de la celda que ocupa el robot sobre la cuadrícula
        de certeza en el eje :math:`y`.
    k_0 : float
        Sector en el histrograma polar de la dirección del robot
        resepecto el eje :math:`z`.

    target : tuple
        Punto :math:`\left(x,\:y\right)` objetivo o ``None`` si
        no se está siguiendo una trayectoria.

    Notes
    -----
        Aquí vendrán algunas notas de uso.

    Examples
    --------
        Aquí vendrán algunos ejemplos de uso.

    """

    def __init__(self):

        # The Obstacle Grid keeps count of the certainty values
        # for each cell in the grid
        grid_dim = (GRID_SIZE, GRID_SIZE)
        self.obstacle_grid = np.zeros( grid_dim, dtype = np.int8 )

        # The Active Window has information (magnitude,
        # direction, distance to robot) of an obstacle vector
        # for each active cell in the grid [mij, citaij, dij]
        window_dim = (WINDOW_SIZE, WINDOW_SIZE, 3)
        self.active_window = np.zeros( window_dim, dtype = np.float_ )

        # Initialize angles and distance (these don't change)

        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):
                if j == WINDOW_CENTER and i == WINDOW_CENTER:
                    continue
                beta_p = np.degrees(np.arctan2(j-WINDOW_CENTER, i-WINDOW_CENTER))
                self.active_window[i,j,BETA] = beta_p + 360 if beta_p < 0 else beta_p
                # The distance is measured in terms of cells
                # (independent of scale/resolution of the grid)
                self.active_window[i,j,DIST2] = np.float_(np.square(i-WINDOW_CENTER) + np.square(j-WINDOW_CENTER))


        # The Polar Histogram maps each cell in the active window
        # to an angular sector
        hist_dim = HIST_SIZE
        self.polar_hist = np.zeros( hist_dim, dtype = np.float_ )

        # The Filtered Polar Histogram holds the actual data to
        # be analized
        self.filt_polar_hist = np.zeros( hist_dim, dtype = np.float_ )

        # The valleys are stored as start-end pairs of sectors
        # in the filtered polar histogram
        self.valleys = []

        # Real position of the robot
        self.x_0 = 0.0
        self.y_0 = 0.0
        self.cita = 0.0

        # Cell of the robot
        self.i_0 = 0
        self.j_0 = 0
        # Sector of the robot
        self.k_0 = 0

        # (x,y) pair representing the target, or
        # None if there is no target
        self.target = None

    def update_position(self, x, y, cita):
        r"""Actualiza la posición del robot.

        Parameters
        ----------
        x : float
            Posición absoluta del robot sobre el eje :math:`x`.
        y : float
            Posición absoluta del robot sobre el eje :math:`y`.
        cita : float
            Orientación del robot resepecto el eje :math:`z`
            en grados.

        Notes
        -----

        Modifica los siguientes atributos de clase:
         * :attr:`x_0`
         * :attr:`y_0`
         * :attr:`cita`
         * :attr:`i_0`
         * :attr:`j_0`
         * :attr:`k_0`

        """

        self.x_0 = x
        self.y_0 = y
        self.cita = cita if cita >= 0 else cita + 360

        self.i_0 = int(x / RESOLUTION)
        self.j_0 = int(y / RESOLUTION)
        self.k_0 = int(self.cita / ALPHA)%HIST_SIZE

    def set_target(self, x=None, y=None):
        r"""Define el punto objetivo o deshabilita el seguimiento
        de trayectorias.

        Define un punto objetivo :math:`(x,y)` para la evasión
        con seguimiento de trayectorias, o deshabilita el 
        seguimiento si el método es invocado sin parámetros.

        Parameters
        ----------
        x : float, opcional
            Posición absoluta del robot sobre el eje :math:`x`.
        y : float, opcional
            Posición absoluta del robot sobre el eje :math:`y`.

        Returns
        -------
        int
            Retorna 1 si se estableció un punto objetivo o 0
            si inhabilitó el seguimiento.

        Raises
        ------
        ValueError
            Si el objetivo dado se sale de la cuadrícula de
            certeza.

        Notes
        -----

        Modifica los siguientes atributos de clase:
         * :attr:`target`

        """
        if x != None and y != None:
            if x >= 0 and y >= 0 and \
                    x < GRID_SIZE*RESOLUTION and \
                    y < GRID_SIZE*RESOLUTION:
                self.target = (x,y)
                return 1
            else:
                raise ValueError("Tried to set the target ({:d},{:d}) outside the obstacle grid".format(x,y))
        else:
            self.target = None
            return 0

    def update_obstacle_density(self, sensor_readings):
        r"""Actualiza la cuadrícula de certeza a partir de las
        lecturas de un sensor.

        Para cada lectura aumenta el valor de una única celda
        en 1, hasta un máximo de 20.

        Parameters
        ----------
        sensor_readings : ndarray
            Una estructura de datos tipo ``numpy.ndarray`` que
            contiene las lecturas de un sensor de distancias.
            Debe ser un arreglo de dimensiones
            :math:`(n\times 2)` para :math:`n` puntos o
            lecturas, donde cada par representa una coordenada
            polar :math:`(r,\theta)` respecto al marco de
            referencia del robot.

        Raises
        ------
        TypeError
            Si ``sensor_readings`` no es de tipo
            ``numpy.ndarray``.
        ValueError
            Si ``sensor_readings`` no tiene las dimensiones
            correctas.

        Notes
        -----
        Las coordenadas deben ser dadas en metros y radianes.

        Modifica los siguientes atributos de clase:
         * :attr:`obstacle_grid`

        """
        # Receives a numpy array of (r, theta) data points #
        # r in meters, theta in radians
        # 0,0 means no reading

        if (type(sensor_readings) != np.ndarray):
            raise TypeError("Expected numpy ndarray, received %s" % type(sensor_readings))
        elif sensor_readings.ndim != 2 or sensor_readings.shape[1] != 2:
            raise ValueError("Expected (n, 2) array, received %s" % str(sensor_readings.shape))

        for x in xrange(sensor_readings.shape[0]):
            r, theta = sensor_readings[x,:]
            if r == 0 and theta == 0:
                continue
            i = int( (self.x_0 + r*np.cos(theta + np.radians(self.cita)))/RESOLUTION )
            j = int( (self.y_0 + r*np.sin(theta + np.radians(self.cita)))/RESOLUTION )
            if self.obstacle_grid[i,j] < 20: self.obstacle_grid[i,j] += 1

    def _active_grid(self):

        i_window = max(self.i_0 - (WINDOW_CENTER), 0)
        j_window = max(self.j_0 - (WINDOW_CENTER), 0)
        i_max = min(i_window + WINDOW_SIZE, GRID_SIZE)
        j_max = min(j_window + WINDOW_SIZE, GRID_SIZE)

        return self.obstacle_grid[i_window:i_max, j_window:j_max]

    def update_active_window(self):
        r"""Calcula la magnitud de los vectores de obstáculo para
        las celdas de la ventana activa.

        El cálculo se hace a partir de los datos guardados en la
        :attr:`cuadrícula de certeza<obstacle_grid>`.

        Notes
        -----
        Modifica los siguientes atributos de clase:
         * :attr:`active_window`

        """

        i_window = self.i_0 - (WINDOW_CENTER)
        j_window = self.j_0 - (WINDOW_CENTER)
        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):
                if j == WINDOW_CENTER and i == WINDOW_CENTER:
                    continue

                grid_i = i_window + i
                grid_j = j_window + j

                if grid_i < 0 or grid_i > GRID_SIZE or grid_j < 0 or grid_j > GRID_SIZE:
                    self.active_window[i,j,MAG] = 0.0
                else :
                    cij = np.float_(self.obstacle_grid[grid_i, grid_j])
                    mij = np.square(cij)*(A - B*self.active_window[i, j, DIST2])
                    self.active_window[i,j,MAG] = mij

        #return self.active_window

    def update_polar_histogram(self):
        r"""Actualiza el histograma polar de obstáculos.

        El cálculo se hace a partir de los vectores de obstáculo
        guardados en la :attr:`ventana activa<active_window>`.

        Notes
        -----
        Modifica los siguientes atributos de clase:
         * :attr:`polar_hist`

        """

        for i in xrange(HIST_SIZE):
            self.polar_hist[i] = 0

        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):
                if j == WINDOW_CENTER and i == WINDOW_CENTER:
                    continue
                k = int(self.active_window[i, j, BETA]/ALPHA) % HIST_SIZE
                assert k < HIST_SIZE and k >= 0, "Error for polar histogram index: %d on i = %d, j = %d" % (k, i, j)
                self.polar_hist[k] += self.active_window[i, j, MAG]

        #return self.polar_hist

    def update_filtered_polar_histogram(self):
        r"""Calcula el histograma polar filtrado a partir de el
        :attr:`histograma polar<polar_hist>`.

        Notes
        -----
        Modifica los siguientes atributos de clase:
         * :attr:`filt_polar_hist`
        """

        ## Amount of adyacent sectors to filter
        L = 5
        assert (2*L + 1) < HIST_SIZE
        for i in xrange(HIST_SIZE):
            coef = [[L - abs(L-j) + 1, (i + (j-L))%HIST_SIZE] for j in xrange(2*L+1)]
            self.filt_polar_hist[i] = np.sum([c*self.polar_hist[k] for c, k in coef])/(2*L+1)

    def find_valleys(self):
        r"""Analiza el histograma polar filtrado y determina los
        valles candidatos.

        Los valles se obtienen a partir del
        :attr:`histograma polar filtrado<filt_polar_hist>` y 
        el :const:`valor de umbral<THRESH>`.

        Returns
        -------
        int
             * ``-1`` si no se encuentra ningún sector con un valor
               mayor a :const:`THRESH` en el hisotgrama filtrado.
             * ``0`` de lo contrario.

        """

        start = None
        for x in xrange(HIST_SIZE):
            if self.filt_polar_hist[x] > THRESH:
                start = copy.copy(x)
                #print "Found start at {:d}".format(x)
                break

        # If no value was found over the threshold no action
        # needs to be taken since there are no nearby obstacles
        if start == None:
            return -1

        # Else, look for valleys after 'start'
        #print "Looking for valleys after k={:d}, c={:.1f}".format(start, start*ALPHA)
        self.valleys = []
        valley_found = False
        for i in xrange(HIST_SIZE+1):
            index = (start + i)%HIST_SIZE
            #print "Sector {:d}, h={:.2f}".format(index, self.filt_polar_hist[index])
            if not valley_found:
                if self.filt_polar_hist[index] < THRESH:
                    v_start = index
                    valley_found = True

            else:
                if self.filt_polar_hist[index] > THRESH:
                    self.valleys.append(tuple([v_start, index-1]))
                    valley_found = False
        return 0

    def calculate_steering_dir(self):
        r"""Calcula la dirección de movimiento para la evasión.

        El cálculo de la dirección para el robot se hace a
        partir de los valles candidato, la orientación actual
        y el objetivo. Si no hay objetivo, se toma la dirección
        actual de movimiento como objetivo.

        Returns
        -------
        new_dir : float
            La dirección del movimiento, dada en grados en el
            rango :math:`[0°,\:360°[` .

        Raises
        ------
        Exception
            Si :attr:`valleys` está vacío.
        
        """

        # Set the target sector. If there is a target point
        # calculate the sector from that, else it's the 
        # current orientation
        k_t = None
	dir_t = None
        if self.target == None:
            k_t = self.k_0
	    dir_t = self.cita
        else:
            dx = self.target[0] - self.x_0
            dy = self.target[1] - self.y_0
            angle = np.degrees(np.arctan2(dy, dx))
            angle = angle + 360 if angle < 0 else angle
            print "target dir is %.1f" % angle
            k_t = int(angle / ALPHA)%HIST_SIZE
	    dir_t = angle

        # First we determine which valley is closest to the
        # target or current robot orientation
        closest_dist = None
        closest_valley = None
        closest_sect = None


        if len(self.valleys) == 0:
            raise Exception("No candidate valleys found to calculate a new direction")

        for v in self.valleys:

            d1, d2 = [self._dist(self.filt_polar_hist, k_t, sector) for sector in v]

            if closest_dist != None:
                min_dist = min(d1, d2)
                if min_dist < closest_dist:
                    closest_dist = min_dist
                    closest_valley = v
                    if d1 < d2:
                        closest_sect = 0
                    else:
                        closest_sect = 1

            else:
                closest_dist = min(d1, d2)
                closest_valley = v
                if d1 < d2:
                    closest_sect = 0
                else:
                    closest_sect = 1

        print "Closest valley is %s" % str(closest_valley)
        s1, s2 = closest_valley
        v_size = (s2 - s1) if s2 >= s1 else HIST_SIZE - (s1-s2)

        if v_size < WIDE_V :
            # For narrow valleys move in the direction of the middle of
            # the valley.
            print "Crossing a narrow valley"
            new_dir = ALPHA * (s1 + v_size/2.0)
        else:
            print "Crossing a wide valley"

            # For wide valleys move in the direction of travel if
            # the closest distance is bigger than WIDE_V/2 and its
            # inside the valley
            if closest_dist > WIDE_V/2.0:
                k_inside = False
                if s1 < s2:
                    k_inside = s1 < k_t and k_t < s2
                else:
                    k_inside = not (s2 < k_t and k_t < s1)

                if k_inside:
                    print "Maintining current direction"
                    new_dir = dir_t
                else:
                    print "Current direction is blocked!"

            # If the target is closer to the edge then travel near
            # the closer edge of the valley
            elif closest_sect == 0:
                print "Staying near right"
                new_dir = ALPHA * (s1 + WIDE_V/2.0)
            else:
                print "Staying near left"
                new_dir = ALPHA * (s2 - WIDE_V/2.0)

        if new_dir >= 360:
            new_dir = new_dir - 360
        elif new_dir < 0:
            new_dir = new_dir + 360

        print "Setting course to target at %.2f" % new_dir
        return new_dir

    def calculate_speed(self, omega=0):
        r"""Calcula la velocidad del robot.

        Calcula la velocidad a partir de la densidad de 
        obstáculos en la dirección actual del movimiento.
        El robot se mueve más despacio entre mayor sea la 
        densidad.

        Opcionalmente recibe la velocidad angular del robot como
        parámetro. En este caso, se hace una reducción adicional
        de la velocidad dependiendo del valor de ``omega``
        en comparación a :const:`OMEGA_MAX`.

        Parameters
        ----------
        omega : float, opcional
            Velocidad angular del robot (rad/s).

        Returns
        -------
        V : float
            Velocidad lineal deseada del robot (m/s).
        """

        # Omega in [rad/s]
        # Obstacle density in the current direction of travel
        H_M = THRESH*1.8
        h_cp = self.filt_polar_hist[self.k_0]
        h_cpp = min(h_cp, H_M)

        V_prime = V_MAX*(1 - h_cpp/H_M)

        V = V_prime*(1 - omega/OMEGA_MAX) + V_MIN

        return V

    @staticmethod
    def _dist(array,i,j):
        n_dist = abs(i-j)
        return min(n_dist, len(array) - n_dist)


def main():
    np.set_printoptions(precision=2)
    robot = VFHModel()
    print "Obstacle grid"
    print robot.obstacle_grid, "\n"
    print "Active Window angles"
    print robot.active_window[:,:,BETA], "\n"
    print "Active Window squared distances"
    print robot.active_window[:,:,DIST2], "\n"
    print "Max distance squared: %f" % D_max2

    
    print("Updating the obstacle grid and robot position")

    robot.update_position(1.5,1.5,270.0)

#    robot.obstacle_grid[1,6] = 1
#    robot.obstacle_grid[1,5] = 2
#    robot.obstacle_grid[1,4] = 2
#    robot.obstacle_grid[1,3] = 5
#    robot.obstacle_grid[1,2] = 13
#    robot.obstacle_grid[2,2] = 3
#    robot.obstacle_grid[3,2] = 3
#    robot.obstacle_grid[4,2] = 3
#
#    robot.obstacle_grid[9,2] = 4
#    robot.obstacle_grid[9,3] = 5
#    robot.obstacle_grid[9,4] = 6
#    robot.obstacle_grid[9,5] = 5
#    robot.obstacle_grid[9,6] = 4

    print robot.i_0, robot.j_0
    print robot._active_grid(), "\n"

    print "Simulating a set of sensor readings"
    pseudo_readings = np.float_([[0.3, np.radians(x)] for x in range(0,90,1)])
    robot.update_obstacle_density(pseudo_readings)
    robot.update_obstacle_density(pseudo_readings)
    #print int(1.2/RESOLUTION), int(1.3/RESOLUTION)
    robot.obstacle_grid[23, 26] = 20
    robot.obstacle_grid[23, 27] = 20
    robot.obstacle_grid[24, 26] = 20
    robot.obstacle_grid[24, 27] = 20
    robot.obstacle_grid[24, 25] = 17
    robot.obstacle_grid[23, 25] = 16
    robot.obstacle_grid[25, 24] = 10
    robot.obstacle_grid[24, 24] = 8

    #robot.obstacle_grid[30, 30] = -20

    print "Updating the active window"
    robot.update_active_window()
    print robot.active_window[:, :, MAG], "\n"

    print "Updating polar histogram"
    robot.update_polar_histogram()
    print robot.polar_hist, "\n"

    print "Updating filtered histogram"
    robot.update_filtered_polar_histogram()
    print robot.filt_polar_hist, "\n"

    print "Looking for valleys"
    robot.find_valleys()
    print robot.valleys, "\n"

    try:
        print "Setting steer direction"
        cita = robot.calculate_steering_dir()
        print cita, "\n"
    except:
        pass

    ### Figuras y graficos ###
    plt.figure(1)
    x = [ALPHA*x for x in range(len(robot.filt_polar_hist))]
    i = [a for a in range(len(robot.filt_polar_hist))]
    plt.bar(x, robot.polar_hist, 4.0, 0, color='r')
    plt.title("Histograma polar")

    plt.figure(2)
    plt.bar(x, robot.filt_polar_hist, 4.0 ) #, 0.1, 0, color='b')
    plt.title("Histograma polar filtrado")

    plt.figure(3)
    plt.pcolor(robot._active_grid().T, alpha=0.75, edgecolors='k',vmin=0,vmax=20)
    plt.xlabel("X")
    plt.ylabel("Y", rotation='horizontal')
    plt.title("Ventana activa")
    
    plt.show()

if __name__ == "__main__":
    main()
