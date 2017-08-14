#! python
## /usr/bin/python
import math
import numpy as np


SMODE_MIN = 0
SMODE_AVG = 1
SMODE_FULL = 2

class BraitModel(object): 
    def __init__(self, s_mode=0, d_min=0.01, d_max=0.25, v_min=-0.3, v_max=0.3, w_max=0.628, alpha = np.pi/6.0):
        
        self.x = 0.
        self.y = 0.
        self.gamma = 0.
        self.radius = 0.02

        self.alpha = alpha
        # Distancia minima y maxima de vision
        # del algoritmo
        self.D_MIN = d_min
        self.D_MAX = d_max

        self.V_MIN = v_min
        self.V_MAX = v_max
        self.W_MAX = w_max

        self.sensor_l = d_max
        self.sensor_r = d_max
        self.s_mode = s_mode

        self.target = None

    def Evade2b(self, d_left=None, d_right=None):

        # Defaults to the object instance's sensor stimuli
        # this behaviour can be overriden by providing the
        # distances values as parameters.
        if d_left == None:
            d_left = self.sensor_l
        if d_right == None:
            d_right = self.sensor_r
            
        v_left = MapStimulus(d_right, self.D_MIN, self.D_MAX, 0.0, 1.0)
        v_right = MapStimulus(d_left, self.D_MIN, self.D_MAX, 0.0, 1.0)

        #print "v_left = %.3f" % v_left
        #print "v_right = %.3f" % v_right

        v_norm = (v_right + v_left)/2.0
        w_norm = (v_right - v_left)/2.0

        #print "v_norm = %.3f" % v_norm
        #print "w_norm = %.3f" % w_norm

        v_rob = MapStimulus(v_norm, 0.0, 1.0, self.V_MIN, self.V_MAX)
        w_rob = MapStimulus(w_norm, -0.5, 0.5, -self.W_MAX, self.W_MAX)

        #return v_left, v_right
        return v_rob, w_rob
            
    def Evade3a(self, d_left=None, d_right=None):

        # Defaults to the object instance's sensor stimuli
        # this behaviour can be overriden by providing the
        # distances values as parameters.
        if d_left == None:
            d_left = self.sensor_l
        if d_right == None:
            d_right = self.sensor_r

        v_left = MapStimulus(d_left, self.D_MIN, self.D_MAX, 1.0, 0.0)
        v_right = MapStimulus(d_right, self.D_MIN, self.D_MAX, 1.0, 0.0)

        v_norm = (v_right + v_left)/2.0
        w_norm = (v_right - v_left)/2.0

        v_rob = MapStimulus(v_norm, 0.0, 1.0, self.V_MIN, self.V_MAX)
        w_rob = MapStimulus(w_norm, -0.5, 0.5, -self.W_MAX, self.W_MAX)

        #return v_left, v_right
        return v_rob, w_rob

    def Mixed2b3a(self, d_left=None, d_right=None):
        
        ### Target following logic

        left_x  = self.x + np.cos(self.gamma+np.pi/2)*self.radius
        left_y  = self.y + np.sin(self.gamma+np.pi/2)*self.radius
        right_x = self.x + np.cos(self.gamma-np.pi/2)*self.radius
        right_y = self.y + np.sin(self.gamma-np.pi/2)*self.radius

        print "left is (%.3f, %.3f)" % (left_x, left_y)
        print "right is (%.3f, %.3f)" % (right_x, right_y)
        
        left_d2  = np.sqrt(np.square(self.target[0]-left_x)  + np.square(self.target[1]-left_y))
        right_d2 = np.sqrt(np.square(self.target[0]-right_x) + np.square(self.target[1]-right_y))
        print "left = %.2f, right = %2f" % (left_d2, right_d2)

        thresh = (self.radius*2)
        if left_d2 > thresh  or right_d2 > thresh:
            d_extra = max(left_d2,right_d2) - thresh

            print "left = %.2f, right = %2f, t=%.2f" % (left_d2, right_d2, thresh)
            left_d2 = left_d2 - d_extra
            right_d2 = right_d2 - d_extra
            print "Triming distance by %.2f" % d_extra

        left_s  = (self.radius)/left_d2
        right_s = (self.radius)/right_d2

        close_min = 0.5
        close_max = 1.0

        v_left1 = MapStimulus(left_s, close_min, close_max, 1.0, 0.0)
        v_right1 = MapStimulus(right_s, close_min, close_max, 1.0, 0.0)
        print "v_left1 = %.3f" % v_left1
        print "v_right1 = %.3f" % v_right1

        ### Obstacle avoidance logic

        if d_left == None:
            d_left = self.sensor_l
        if d_right == None:
            d_right = self.sensor_r
            
        v_left2 = MapStimulus(d_right, self.D_MIN, self.D_MAX, 0.0, 1.0)
        v_right2 = MapStimulus(d_left, self.D_MIN, self.D_MAX, 0.0, 1.0)

        ratio = 0.2
        v_left = (v_left1*ratio + v_left2*(1-ratio))/2
        v_right = (v_right1*ratio + v_right2*(1-ratio))/2
        print "v_left = %.3f" % v_left
        print "v_right = %.3f" % v_right

        v_norm = (v_right + v_left)/2.0
        w_norm = (v_right - v_left)/2.0

        #print "v_norm = %.3f" % v_norm
        #print "w_norm = %.3f" % w_norm

        v_rob = MapStimulus(v_norm, 0.0, 1.0, self.V_MIN, self.V_MAX)
        w_rob = MapStimulus(w_norm, -0.5, 0.5, -self.W_MAX, self.W_MAX)

        #return v_left, v_right
        return v_rob, w_rob
        
        pass

    def Mixed3a2b(self, d_left, d_right):
        pass

    def UpdatePos(self, x, y, gamma):
        self.x = x
        self.y = y
        self.gamma = gamma

    def SetTarget(self, x, y, unset = False):
        if unset:
            self.target = None
        else:
            self.target = (x,y)

    def UpdateSensors(self, sensor_readings):
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
            if -self.alpha < theta and theta < 0:
                # right
                right.append(r)
            elif 0 < theta and theta < self.alpha:
                left.append(r)
                #left

        if self.s_mode == SMODE_MIN:
            if len(left) > 0:
                self.sensor_l = min(left)
            else:
                self.sensor_l = self.D_MAX

            if len(right) > 0:
                self.sensor_r = min(right)
            else:
                self.sensor_r = self.D_MAX

        elif self.s_mode == SMODE_AVG:
            if len(left) == 0:
                self.sensor_l = self.D_MAX
            else:
                self.sensor_l = sum(left)/float(len(left))

            if len(right) == 0:
                self.sensor_r = self.D_MAX
            else:
                self.sensor_r = sum(right)/float(len(right))

        elif self.s_mode == SMODE_FULL:
            num = 15

            sum_l = sum(left)
            if len(left) < num:
                sum_l += (num-len(left))*self.D_MAX
                self.sensor_l = sum_l/num
            else:
                self.sensor_l = sum_l/len(left)

            sum_r = sum(right)
            if len(right) < num:
                sum_r += (num-len(right))*self.D_MAX
                self.sensor_r = sum_r/num
            else:
                self.sensor_r = sum_r/len(right)


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

    robot = BraitModel(s_mode=SMODE_FULL)
    pseudo_readings = np.float_([[0.41, np.radians(x)] for x in range(-5,90,1)])
    robot.UpdateSensors(pseudo_readings)
    robot.UpdatePos(0.0,0.0,np.pi/2)
    robot.SetTarget(0.0000, 0.20)
    print "Left sensor reading %.2f" % robot.sensor_l
    print "Right sensor reading %.2f" % robot.sensor_r

    print
    print "Speed range: [%.2f, %.2f]" % (robot.V_MIN, robot.V_MAX)
    print "Rotation range: [%.2f, %.2f]" % (-robot.W_MAX, robot.W_MAX)
    v, w = robot.Mixed2b3a()
    dire = "izq" if w > 0 else "der"
    print "Result: v=%.2f , w=%.2f (%s)" % (v, w, dire)

    return 0


if __name__ == "__main__":
    main()
