#! /usr/bin/python

import numpy as np

# Size of the full Grid
GRID_SIZE = 100

# Resolution of each cell (in cm)
RESOLUTION = np.float_(10.0)

# Size of the active window that
# travels with the robot
WINDOW_SIZE = 9
WINDOW_CENTER = WINDOW_SIZE/2

# Size of each polar sector
# in the polar histogram (in degrees)
ALPHA = 5
if np.mod(360.0/alpha) != 0:
    raise ValueError("Alpha must define an int amount of sectors")
HIST_SIZE = 360/ALPHA

# Constants for virtual vector magnitude calculations
# A - B*D_max = 0
D_max = (WINDOW_SIZE-1)*np.sqrt(2)*RESOLUTION/2.0
B = np.float_(1.0)
A = np.float_(B*D_max)

# Active window array indexes
MAG = 0
BETA = 1
DIST = 2

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
                self.active_window[i,j,BETA] = np.degrees(np.arctan2(j-WINDOW_CENTER, i-WINDOW_CENTER))
                self.active_window[i,j,DIST] = RESOLUTION*np.sqrt(np.square(i-WINDOW_CENTER) + np.square(j-WINDOW_CENTER))


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
                    mij = np.square(cij)*(A - (B*self.active_window[i, j, DIST]))
                    self.active_window[i,j,MAG] = mij

        #return self.active_window

    def update_polar_histogram(self):

        for i in xrange(HIST_SIZE):
            self.polar_hist[i] = 0

        for i in xrange(WINDOW_SIZE):
            for j in xrange(WINDOW_SIZE):
                if j == WINDOW_CENTER and i == WINDOW_CENTER:
                    continue
                k = int((self.active_window[i, j, BETA] + 180)/ALPHA)
                self.polar_hist[k] += self.active_window[i, j, MAG]

        #return self.polar_hist

    def update_filtered_polar_histogram(self):
        pass
        
    def calculate_steering_dir(self):
        pass






def main():
    np.set_printoptions(precision=2)
    H = VFHModel()
    print(H.obstacle_grid)
    print(H.active_window[:,:,BETA])
    print(H.active_window[:,:,DIST])
    print D_max
    

if __name__ == "__main__":
    main()
