import numpy as np
import time

class PID:

    def __init__(self, Kp, Ti, Td, MAX, MIN):
        self.Kp = np.float_(Kp)
        self.Ti = np.float_(Ti)
        self.Td = np.float_(Td)
        self.max = np.float_(MAX)
        self.min = np.float_(MIN)

        self.ref = np.float_(0)

        self.error = np.float_(0)
        self._integral = np.float_(0)
        self._derivada = np.float_(0)
        self.effort = np.float(0)
        
        self.past_error = np.float_(0)
        self.past_input = np.dtype(np.float_)

    def setRef(self, ref):
        self.ref = np.float_(ref)

    def setInput(self, inp):
        self.input = np.float_(inp)
    
    def begin(self,entrada):
        self.past_input = np.float_(entrada)
        
    def timestep(self, delta_t):

        # Luego se determina el error, el termino integral y el derivativo
        self.error = angleDiff(self.ref, self.input)
        self._derivada = -angleDiff(self.input, self.past_input)/delta_t
        self._integral = self._integral + (self.error + self.past_error)*delta_t/2

        
        # Se calcula el esfuerzo del controlador
        self.effort = self.Kp*(self.error + self._integral/self.Ti + self._derivada*self.Td)
        if self.effort > self.max:
            self.effort = self.max
        elif self.effort < self.min:
            self.effort = self.min

        # Se actualizan los valores pasados
        self.past_error = self.error
        self.past_input = self.input
        return self.effort

# Calcula a1 - a2 [radianes]
def angleDiff(a1, a2):
    return np.arctan2(np.sin(a1-a2), np.cos(a1-a2))
