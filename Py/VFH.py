#! /usr/bin/python

import numpy as np

# Size of the full Grid
GRID_SIZE = 100

# Resolution of each cell (in cm)
RESOLUTION = np.float_(10.0)

# Size of the active window that
# travels with the robot
WINDOW_SIZE = 9
assert WINDOW_SIZE%2 == 1, "Window should have an odd number of cells for better results"
WINDOW_CENTER = WINDOW_SIZE/2

# Size of each polar sector
# in the polar histogram (in degrees)
ALPHA = 10
if np.mod(360.0, ALPHA) != 0:
    raise ValueError("Alpha must define an int amount of sectors")
HIST_SIZE = 360/ALPHA

# Constants for virtual vector magnitude calculations
# D_max^2 = 2*(ws-1/2)^2
# A - B*D_max^2 = 1
D_max2 = np.square(WINDOW_SIZE-1)/2.0
B = np.float_(1.0)
A = np.float_(1+B*D_max2)

# Active window array indexes
MAG = 0
BETA = 1
DIST2 = 2

class VFHModel:

    def __init__(self):

        #if not callable(certainty_func):
        #    raise TypeError("Variable certainty_func must define a function")

        grid_dim = (GRID_SIZE, GRID_SIZE)
        # [mij, citaij, dij]
        window_dim = (WINDOW_SIZE, WINDOW_SIZE, 3)
        hist_dim = HIST_SIZE

        # The Obstacle Grid keeps count of the certainty values
        # for each cell in the grid
        self.obstacle_grid = np.zeros( grid_dim, dtype = np.int16 )

        # The Active Window has information (magnitude and direction)
        # of an obstacle vector for each active cell in the grid
        self.active_window = np.zeros( window_dim, dtype = np.float_ )

        # Initialize angles (these don't change)

        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):
                if j == WINDOW_CENTER and i == WINDOW_CENTER:
                    continue
                beta_p = np.degrees(np.arctan2(j-WINDOW_CENTER, i-WINDOW_CENTER))
                self.active_window[i,j,BETA] = beta_p + 360 if beta_p < 0 else beta_p
                # The distance is measured in terms of cells (independent of scale/resolution of the grid)
                self.active_window[i,j,DIST2] = np.float_(np.square(i-WINDOW_CENTER) + np.square(j-WINDOW_CENTER))


        # The Polar Histogram maps each cell in the active window
        # to an angular sector
        self.polar_hist = np.zeros( hist_dim, dtype = np.float_ )

        # The Filtered Polar Histogram holds the actual data to be analized
        self.filt_polar_hist = np.zeros( hist_dim, dtype = np.float_ )

        # Real position of the robot
        self.x_0 = 0.0
        self.y_0 = 0.0
        self.cita = 0.0

        # Cell of the robot
        self.i_0 = 0
        self.j_0 = 0

    def _init_params(self):
        pass

    def update_position(self, x, y, cita):
        self.x_0 = x
        self.y_0 = y
        self.cita = cita

        self.i_0 = int(x / RESOLUTION)
        self.j_0 = int(y / RESOLUTION)

    def update_obstacle_densitiy(self, sensor_readings):
        pass

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
                    cij = self.obstacle_grid[grid_i, grid_j]
                    mij = np.square(cij)*(A - B*self.active_window[i, j, DIST2])
                    self.active_window[i,j,MAG] = mij

        #return self.active_window

    def update_polar_histogram(self):

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

        ## Amount of adyacent sectors to filter
        L = 3
        assert (2*L + 1) < HIST_SIZE
        for i in xrange(HIST_SIZE):
            coef = [[L - abs(L-j) + 1, (i + (j-L))%HIST_SIZE] for j in xrange(2*L+1)]
            self.filt_polar_hist[i] = np.sum([c*self.polar_hist[k] for c, k in coef])/(2*L+1)
        
    def analyze_valleys():
        pass
    def calculate_steering_dir(self):
        pass






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

    robot.update_position(52.1,50.7,0.0)

    robot.obstacle_grid[1,6] = 1
    robot.obstacle_grid[1,5] = 2
    robot.obstacle_grid[1,4] = 2
    robot.obstacle_grid[1,3] = 5
    robot.obstacle_grid[1,2] = 13
    robot.obstacle_grid[2,2] = 3
    robot.obstacle_grid[3,2] = 3
    robot.obstacle_grid[4,2] = 3

    robot.obstacle_grid[9,2] = 4
    robot.obstacle_grid[9,3] = 5
    robot.obstacle_grid[9,4] = 6
    robot.obstacle_grid[9,5] = 5
    robot.obstacle_grid[9,6] = 4

    print robot.i_0, robot.j_0
    print robot._active_grid(), "\n"

    print "Updating the active window"
    robot.update_active_window()
    print robot.active_window[:, :, MAG], "\n"

    print "Updating polar histogram"
    robot.update_polar_histogram()
    print robot.polar_hist, "\n"

    print "Updating filtered histogram"
    robot.update_filtered_polar_histogram()
    print robot.filt_polar_hist, "\n"

if __name__ == "__main__":
    main()
