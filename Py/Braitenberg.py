#! python
## /usr/bin/python
import math
import numpy as np

ALPHA = np.pi/6.0

class BraitModel(object): 
    def __init__(self, d_min=0.01, d_max=0.1, v_min=-math.pi, v_max=math.pi):
        
        self.x = 0.
        self.y = 0.
        self.cita = 0.

        # Distancia minima y maxima de vision
        # del algoritmo
        self.D_MIN = d_min
        self.D_MAX = d_max

        self.V_MIN = v_min
        self.V_MAX = v_max

        self.sensor_l = 0.0
        self.sensor_r = 0.0

    def Evade2b(self, d_left, d_right):
        v_left = MapStimulus(d_right, self.D_MIN, self.D_MAX, self.V_MIN, self.V_MAX)
        v_right = MapStimulus(d_left, self.D_MIN, self.D_MAX, self.V_MIN, self.V_MAX)
        return v_left, v_right
            
    def Evade3a(self, d_left, d_right):
        v_left = MapStimulus(d_left, self.D_MIN, self.D_MAX, self.V_MAX, self.V_MIN)
        v_right = MapStimulus(d_right, self.D_MIN, self.D_MAX, self.V_MAX, self.V_MIN)
        return v_left, v_right

    def update_sensors(self, sensor_readings):
        # Receives a numpy array of (r, theta) data points #
        # r in meters, theta in radians
        # 0,0 means no reading

        if (type(sensor_readings) != np.ndarray):
            raise TypeError("Expected numpy ndarray, received %s" % type(sensor_readings))
        elif sensor_readings.ndim != 2 or sensor_readings.shape[1] != 2:
            raise ValueError("Expected (n, 2) array, received %s" % str(sensor_readings.shape))

        left = []
        right = []
        for x in xrange(sensor_readings.shape[0]):
            r, theta = sensor_readings[x,:]
            if r == 0 and theta == 0:
                continue
            if -ALPHA < theta && theta < 0:
                # left
                pass

            elif 0 < theta < ALPHA:
                #right
                pass

def MapStimulus(s, s_min, s_max, r_min, r_max):
    if s >= s_max:
        return r_max
    elif s > s_min:
        #y = mx + b
        m = (r_max - r_min)/(s_max - s_min)
        b = r_min - (m*s_min)
        return (m*s) + b
    else:
        return r_min
		
def main():
    pass

if __name__ == "__main__":
    main()
