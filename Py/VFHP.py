#! /usr/bin/python

import numpy as np
import matplotlib.pyplot as plt



########################################
###            Constantes           ####
########################################
###                                 ####
###                                 ####

# Size of the full Grid
GRID_SIZE = 100
# Maximum certantinty value
C_MAX = 20

# Resolution of each cell (in m)
RESOLUTION = np.float_(0.05)

# Size of the active window that
# travels with the robot
WINDOW_SIZE = 25
assert WINDOW_SIZE%2 == 1, "Window should have an odd number of cells for better results"
WINDOW_CENTER = WINDOW_SIZE/2

# Size of each polar sector
# in the polar histogram (in degrees)
ALPHA = 5
assert np.mod(360, ALPHA) == 0, "Alpha must define an int amount of sectors"
HIST_SIZE = 360/ALPHA

# Constants for virtual vector magnitude calculations
# D_max^2 = 2*(ws-1/2)^2*R^2
# A - B*D_max^2 = 1
D_max2 = np.square((WINDOW_SIZE-1)*RESOLUTION)/2.0
B = np.float_(10.0)
A = np.float_(1+B*D_max2)

# Constants for obstacle enlargement
# Robot radius
R_ROB = 0.0375
# Minimum obstacle distance
D_S   = 0.08
R_RS  = R_ROB + D_S

# Valley/Peak threshold
T_LO = 2000.0
T_HI = 8000.0
WIDE_V = HIST_SIZE/6
V_MAX = 0.0628
V_MIN = 0.0 #0.00628
OMEGA_MAX = 1.256

# Cost function constants
# Recommended mu1 > m2 + m3
# mu1 = target following cost
# mu2 = sharp steering cost
# mu3 = direction commiting cost
mu1 = 5.0
mu2 = 2.0
mu3 = 2.0
MAX_COST = 180.0*(mu1+mu2+mu3)


# Active window array indexes
MAG = 0
BETA = 1
DIST2 = 2
ABDIST = 3
GAMA = 4

###                                 ####
###                                 ####
########################################

class VFHPModel:

    def __init__(self):

        # The Obstacle Grid keeps count of the certainty values
        # for each cell in the grid
        grid_dim = (GRID_SIZE, GRID_SIZE)
        self.obstacle_grid = np.zeros( grid_dim, dtype = np.int8 )

        # The Active Window has information (magnitude,
        # direction, distance to robot) of an obstacle vector
        # for each active cell in the grid [mij, betaij, d^2, a-bd^2_ij, gij]
        window_dim = (WINDOW_SIZE, WINDOW_SIZE, 5)
        self.active_window = np.zeros( window_dim, dtype = np.float_ )

        # Initialize angles and distance (these don't change)

        print "INITIALIZING"
        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):
                if j == WINDOW_CENTER and i == WINDOW_CENTER:
                    continue
                beta_p = np.degrees(np.arctan2(j-WINDOW_CENTER, i-WINDOW_CENTER))
                self.active_window[i,j,BETA] = beta_p + 360 if beta_p < 0 else beta_p
                # The distance is measured in terms of cells
                # (independent of scale/resolution of the grid)
                dist2 = np.square(RESOLUTION)*np.float_(np.square(i-WINDOW_CENTER) + np.square(j-WINDOW_CENTER))
                self.active_window[i,j,DIST2] = dist2
                self.active_window[i,j,ABDIST] = A - B*dist2
                self.active_window[i,j,GAMA] = np.degrees(np.arcsin(np.float_(R_RS)/np.sqrt(dist2)))
                if np.isnan(self.active_window[i,j,GAMA]):
                    print "cell ({:d},{:d}) gamma = ".format(i,j), self.active_window[i,j,GAMA]
                    print "setting to 90.0"
                    #print dist2, type(dist2)
                    #print R_RS, type(R_RS)
                    #print np.sqrt(dist2)
                    #print np.float_(R_RS)/np.sqrt(dist2)
                    #print "arcoseno ", np.arcsin(np.float_(R_RS)/np.sqrt(dist2))
                    self.active_window[i,j,GAMA] = np.float_(90.0)

        # The Polar Histogram maps each cell in the active window
        # to one or more angular sector
        hist_dim = HIST_SIZE
        self.polar_hist = np.zeros( hist_dim, dtype = np.float_ )

        # The Binary Polar Histogram defines free and blocked sectors
        self.bin_polar_hist = np.zeros( hist_dim, dtype = np.bool_ )

        # The Masked Polar Histogram further restricts the free sectors
        # depending on vehicle dynamics
        self.masked_polar_hist = np.zeros( hist_dim, dtype = np.bool_ )

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

        self.target = None
        self.t_dir = 0
        self.prev_dir = 0
        self.prev_cost = 0

    def set_target(self, x=None, y=None):
        if x != None and y != None:
            if x >= 0 and y >= 0 and \
                    x < GRID_SIZE*RESOLUTION and \
                    y < GRID_SIZE*RESOLUTION:
                self.target = (x,y)
                return 0
            else:
                raise ValueError("Tried to set the target ({:d},{:d}) outside the obstacle grid".format(x,y))
        else:
            self.target = None
            return -1

    def update_position(self, x, y, cita):
        self.x_0 = x
        self.y_0 = y
        self.cita = cita if cita >= 0 else cita + 360

        self.i_0 = int(x / RESOLUTION)
        self.j_0 = int(y / RESOLUTION)
        self.k_0 = int(self.cita / ALPHA)%HIST_SIZE


    def update_obstacle_density(self, sensor_readings):
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
            if self.obstacle_grid[i,j] < C_MAX: self.obstacle_grid[i,j] += 1

    def _active_grid(self):

        i_window = max(self.i_0 - (WINDOW_CENTER), 0)
        j_window = max(self.j_0 - (WINDOW_CENTER), 0)
        i_max = min(i_window + WINDOW_SIZE, GRID_SIZE)
        j_max = min(j_window + WINDOW_SIZE, GRID_SIZE)

        return self.obstacle_grid[i_window:i_max, j_window:j_max]

    def update_active_window(self):

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
                    self.active_window[i,j,MAG] = np.square(cij)*self.active_window[i,j,ABDIST]

        #return self.active_window

    def update_polar_histogram(self):

        for i in xrange(HIST_SIZE):
            self.polar_hist[i] = 0

        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):
                if j == WINDOW_CENTER and i == WINDOW_CENTER:
                    continue
                
                #print "Determine ranges for cell ({:d}, {:d}".format(i,j)
                beta = self.active_window[i,j,BETA]
                gama = self.active_window[i,j,GAMA]
                alfa = ALPHA
                #print beta, type(beta), gama, type(gama), alfa, type(alfa)

                # Determine the range of histogram sectors that needs to be updated
                low = int(np.ceil((self.active_window[i,j,BETA] - self.active_window[i,j,GAMA])/ALPHA))
                #print low
                high = int(np.floor((self.active_window[i,j,BETA] + self.active_window[i,j,GAMA])/ALPHA))
                #print high
                #print "Updating sectors [{:d} {:d}] for cell ({:d}, {:d})".format(low, high, i, j)
                k_range = [x%HIST_SIZE for x in np.linspace(low, high, high-low+1, True, dtype = int)]
                #print k_range
                for k in k_range:
                    assert k < HIST_SIZE and k >= 0, "Error for polar histogram index: %d on i = %d, j = %d" % (k, i, j)
                    self.polar_hist[k] += self.active_window[i, j, MAG]

        #return self.polar_hist

    def update_bin_polar_histogram(self):

        for k in xrange(HIST_SIZE):
            if self.polar_hist[k] > T_HI:
                blocked = True
            elif self.polar_hist[k] < T_LO:
                blocked = False
            else:
                blocked = self.bin_polar_hist[k]

            self.bin_polar_hist[k] = blocked

    def update_masked_polar_hist(self,steer_l,steer_r):

        phi_back = self.cita + 180.0
        phi_back = phi_back - 360.0 if phi_back > 360.0 else phi_back

        phi_left = phi_back
        phi_right = phi_back

        X_r = WINDOW_SIZE*RESOLUTION/2.0
        Y_r = WINDOW_SIZE*RESOLUTION/2.0

        print "INFO: Current dir is {:.3f}".format(self.cita)
        print "INFO: phi_back is {:.3f}".format(phi_back)
        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):

                if self.active_window[i,j,MAG] <= 2*(C_MAX**2) or (i == WINDOW_CENTER and j == WINDOW_CENTER):
                    #print "Skipped cel ({},{}) = {:.3f}".format(i,j,self.polar_hist[k])
                    continue

                if self.active_window[i,j,BETA] == self.cita:
                    #print "Entering cell ({:d},{:d}) dead ahead".format(i,j)

                    # position of the cell
                    cij_x = (i+0.5)*RESOLUTION
                    cij_y = (j+0.5)*RESOLUTION

                    #### Left steering radius
                    r_robl_x = steer_l*np.cos(np.radians(self.cita+90.0))
                    r_robl_y = steer_l*np.sin(np.radians(self.cita+90.0))
                    # center of the steering radius in the active window
                    r_steer_x = X_r + r_robl_x
                    r_steer_y = Y_r + r_robl_y

                    c_dist2 = np.square(cij_x - r_steer_x) + np.square(cij_y - r_steer_y)
                    #print "Left dist {:.3f}".format(c_dist2)
                    if c_dist2 < np.square(R_RS + steer_l):
                        phi_left = self.active_window[i,j,BETA] + 0.1
                        print "Setting left limit angle to {:.1f} on cell ({:d},{:d}) = {:.1f}".format(phi_left, i, j, self.active_window[i,j,MAG])

                    #### Right steering radius
                    r_robr_x = steer_r*np.cos(np.radians(self.cita-90.0))
                    r_robr_y = steer_r*np.sin(np.radians(self.cita-90.0))
                    # center of the steering radius in the active window
                    r_steer_x = X_r + r_robr_x
                    r_steer_y = Y_r + r_robr_y

                    c_dist2 = np.square(cij_x - r_steer_x) + np.square(cij_y - r_steer_y)
                    #print "Right dist {:.3f}, limit {:.3f}".format(c_dist2,np.square(R_RS + steer_r))
                    if c_dist2 < np.square(R_RS + steer_r):
                        phi_right = self.active_window[i,j,BETA] - 0.1
                        print "Setting right limit angle to {:.1f} on cell ({:d},{:d})".format(phi_right, i, j)


                elif self._isInRange(self.cita, phi_left, self.active_window[i,j,BETA]):
                    #left
                    #print "Entering cell ({:d},{:d}) in left range".format(i,j)
                    r_robl_x = steer_l*np.cos(np.radians(self.cita+90.0))
                    r_robl_y = steer_l*np.sin(np.radians(self.cita+90.0))

                    # center of the steering radius in the active window
                    r_steer_x = X_r + r_robl_x
                    r_steer_y = Y_r + r_robl_y

                    # position of the cell
                    cij_x = (i+0.5)*RESOLUTION
                    cij_y = (j+0.5)*RESOLUTION

                    # distance^2 from the cell to the steering center
                    c_dist2 = np.square(cij_x - r_steer_x) + np.square(cij_y - r_steer_y)

                    if c_dist2 < np.square(R_RS + steer_l):
                        phi_left = self.active_window[i,j,BETA]
                        print "Setting left limit angle to {:.1f} on cell ({:d},{:d}) = {:.1f}".format(phi_left, i, j, self.active_window[i,j,MAG])

                elif self._isInRange(phi_right,self.cita,self.active_window[i,j,BETA]):
                    #right
                    #print "Entering cell ({:d},{:d}) in right range".format(i,j)
                    r_robr_x = steer_r*np.cos(np.radians(self.cita-90.0))
                    r_robr_y = steer_r*np.sin(np.radians(self.cita-90.0))

                    # center of the steering radius in the active window
                    r_steer_x = X_r + r_robr_x
                    r_steer_y = Y_r + r_robr_y

                    # position of the cell
                    cij_x = (i+0.5)*RESOLUTION
                    cij_y = (j+0.5)*RESOLUTION

                    # distance^2 from the cell to the steering center
                    c_dist2 = np.square(cij_x - r_steer_x) + np.square(cij_y - r_steer_y)

                    if c_dist2 < np.square(R_RS + steer_r):
                        phi_right = self.active_window[i,j,BETA]
                        print "Setting right limit angle to {:.1f} on cell ({:d},{:d})".format(phi_right, i, j)


        print "Limit angles:"
        print "Left:", phi_left
        print "Right:", phi_right
        print
        for k in xrange(HIST_SIZE):
            if self.bin_polar_hist[k] == False and self._isInRange(phi_right,phi_left,k*ALPHA):
                self.masked_polar_hist[k] = False
            else:
                self.masked_polar_hist[k] = True

    # This function determines if an angle in the range
    # [0, 360[ is inside the sector given, from start to
    # end (counter-clockwise), also angles in the range [0, 360[
    @staticmethod
    def _isInRange(start,end,angle):
        if start < end:
            return (start <= angle and angle <= end)
        else:
            return (start <= angle and angle <= 360) or (0 <= angle and angle <= end)


    def find_valleys(self):
        start = None
        for x in xrange(HIST_SIZE):
            if self.masked_polar_hist[x]:
                start = x
                break

        # If no value was found over the threshold no action
        # needs to be taken since there are no nearby obstacles
        if start == None:
            return -1

        # Else, look for valleys after 'start'
        # True means blocked in the masked histogram
        # False means free
        self.valleys = []
        valley_found = False
        for i in xrange(HIST_SIZE+1):
            index = (start + i)%HIST_SIZE
            if not valley_found:
                if not self.masked_polar_hist[index]:
                    v_start = index
                    valley_found = True

            else:
                if self.masked_polar_hist[index]:
                    self.valleys.append(tuple([v_start, index-1]))
                    valley_found = False
        return 0

    def calculate_steering_dir(self):

        if len(self.valleys) == 0:
            raise Exception("No candidate valleys found to calculate a new direction")

        t_dir = None
        if self.target != None:
            # If there is a target for the robot we calculate the direction
            t_dir = np.degrees(np.arctan2(self.target[1]-self.y_0, self.target[0]-self.x_0))
            t_dir = t_dir if t_dir >= 0 else t_dir + 360
        else:
            # Else set the current direction as the target
            t_dir = self.cita

        candidate_dirs = []
        for v in self.valleys:
            s1, s2 = v
            v_size = (s2 - s1) if s2 >= s1 else HIST_SIZE - (s1-s2)

            if v_size < WIDE_V:
                # Narrow valley
                # The only target dir is the middle of 
                # the oppening
                print "narrow valley"
                c_center = ALPHA*(s1 + v_size/2.0)
                c_center = c_center - 360.0 if c_center >= 360.0 else c_center
                candidate_dirs.append(c_center)

            else:
                # Wide valley
                # Target dirs are the left and right
                # borders,
                print "wide valley"
                c_right = ALPHA*(s1 + WIDE_V/2.0)
                c_right = c_right - 360.0 if c_right >= 360.0 else c_right

                c_left = ALPHA*(s2 - WIDE_V/2.0)
                c_left = c_left + 360.0 if c_left < 0.0 else c_left

                candidate_dirs.append(c_left)
                candidate_dirs.append(c_right)

                if c_right != c_left and self._isInRange(c_right,c_left,t_dir):
                    candidate_dirs.append(t_dir)

        print candidate_dirs
        # Once all we know all possible candidate dirs
        # choose the one with the lowest cost
        new_dir = None
        best_cost = None
        for c in candidate_dirs:
            cost = mu1*self._abs_angle_diff(c,t_dir) + \
                    mu2*self._abs_angle_diff(c,self.cita) + \
                    mu3*self._abs_angle_diff(c,self.prev_dir)
            print "For candidate dir {:.1f}: {:.3f} cost".format(c,cost)

            if best_cost == None:
                new_dir = c
                best_cost = cost
            elif cost < best_cost:
                new_dir = c
                best_cost = cost

        self.prev_dir = new_dir
        self.prev_cost = best_cost

        V = V_MAX*(1 - best_cost/MAX_COST) + V_MIN

        return new_dir, V

    @staticmethod
    def _abs_angle_diff(a1, a2):
        return min(360.0 - abs(a1 - a2), abs(a1 - a2))

    def calculate_speed(self):

        V = V_MAX*(1 - self.prev_cost/MAX_COST) + V_MIN
        return V

    @staticmethod
    def _dist(array,i,j):
        n_dist = abs(i-j)
        return min(n_dist, len(array) - n_dist)


def main():
    np.set_printoptions(precision=2)
    robot = VFHPModel()
    print "Obstacle grid"
    print robot.obstacle_grid, "\n"
    print "Active Window angles"
    print robot.active_window[:,:,BETA], "\n"
    print "Active Window squared distances"
    print robot.active_window[:,:,DIST2], "\n"
    print "Active Window a-bd^2 constants"
    print robot.active_window[:,:,ABDIST], "\n"
    print "Max distance squared: %f" % D_max2

    
    print("Updating the obstacle grid and robot position")

    robot.update_position(1.5,1.5,90.0)

    #robot.obstacle_grid[1,6] = 1
    #robot.obstacle_grid[1,5] = 2
    #robot.obstacle_grid[1,4] = 2
    #robot.obstacle_grid[1,3] = 5
    #robot.obstacle_grid[2,2] = 3
    #robot.obstacle_grid[3,2] = 3
    #robot.obstacle_grid[4,2] = 3
#
#    robot.obstacle_grid[9,2] = 4
#    robot.obstacle_grid[9,3] = 5
#    robot.obstacle_grid[9,4] = 6
#    robot.obstacle_grid[9,5] = 5
#    robot.obstacle_grid[9,6] = 4

    robot.obstacle_grid[27,30] = 20
    robot.obstacle_grid[27,31] = 20
    robot.obstacle_grid[27,29] = 20
    robot.obstacle_grid[28,30] = 20
    robot.obstacle_grid[26,30] = 20

    robot.obstacle_grid[30,37] = 20
    robot.obstacle_grid[31,37] = 20
    robot.obstacle_grid[29,37] = 20
    robot.obstacle_grid[30,38] = 20
    robot.obstacle_grid[30,36] = 20

    robot.obstacle_grid[41,30] = 20
    robot.obstacle_grid[40,30] = 20
    robot.obstacle_grid[42,30] = 20
    robot.obstacle_grid[41,31] = 20
    robot.obstacle_grid[41,29] = 20

    print "i , j , k "
    print robot.i_0, robot.j_0, robot.k_0
    print robot._active_grid(), "\n"

    print "Simulating a set of sensor readings"
    pseudo_readings = np.float_([[0.2, np.radians(x)] for x in range(0,90,2)])
    #pseudo_readings = np.float_([[0.3,
    robot.update_obstacle_density(pseudo_readings)


    print "Updating the active window"
    robot.update_active_window()
    print robot.active_window[:, :, MAG], "\n"

    print "Updating polar histogram"
    robot.update_polar_histogram()
    print robot.polar_hist, "\n"

    print "Updating binary histogram"
    robot.update_bin_polar_histogram()
    print robot.bin_polar_hist, "\n"

    print "Updating masked polar histogram"
    robot.update_masked_polar_hist(R_ROB*1,R_ROB*1)
    print robot.masked_polar_hist, "\n"

    print "Looking for valleys"
    robot.find_valleys()
    print robot.valleys, "\n"

    print "Select new direction"
    robot.prev_dir = 90.0
    print robot.calculate_steering_dir(), "\n"

    print "Setting speed to (MAX = {:.2f})".format(V_MAX)
    print robot.calculate_speed(), "\n"

    
#    print "Updating filtered histogram"
#    robot.update_filtered_polar_histogram()
#    print robot.filt_polar_hist, "\n"
#
#    print "Looking for valleys"
#    robot.find_valleys()
#    print robot.valleys, "\n"
#
#    try:
#        print "Setting steer direction"
#        cita = robot.calculate_steering_dir()
#        print cita, "\n"
#    except:
#        pass

    ### Figuras y graficos ###
#
#    plt.figure(2)
#    plt.plot(i, robot.filt_polar_hist ) #, 0.1, 0, color='b')
#    plt.title("Histograma polar filtrado")

    plt.figure(1)
    plt.pcolor(robot._active_grid().T, alpha=0.75, edgecolors='k',vmin=0,vmax=20)
    plt.xlabel("X")
    plt.ylabel("Y", rotation='horizontal')
    
    plt.figure(2)
    x = [ALPHA*x for x in range(len(robot.polar_hist))]
    i = [a for a in range(len(robot.polar_hist))]
    plt.bar(x, robot.polar_hist, 3.0, 0, color='r')
    plt.title("Histograma polar")

    plt.figure(3)
    x = [ALPHA*x for x in range(len(robot.polar_hist))]
    i = [a for a in range(len(robot.polar_hist))]
    plt.bar(x, robot.bin_polar_hist, 3.0, 0, color='b')
    plt.title("Histograma polar binario")
    
    plt.figure(4)
    x = [ALPHA*x for x in range(len(robot.polar_hist))]
    i = [a for a in range(len(robot.polar_hist))]
    plt.bar(x, robot.masked_polar_hist, 3.0, 0, color='g')
    plt.title("Histograma polar mascarado")

    plt.show()

if __name__ == "__main__":
    main()
