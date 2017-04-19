
D_MIN = 5.0
D_MAX = 30.0
V_MIN = -0.5
V_MAX = 5.0

def Braitenberg2b(d_left, d_right):
	v_left = MapStimulus(d_right, D_MIN, D_MAX, V_MIN, V_MAX)
	v_right = MapStimulus(d_left, D_MIN, D_MAX, V_MIN, V_MAX)
	return v_left, v_right
	
def Braitenberg3a(d_left, d_right):
	v_left = MapStimulus(d_left, D_MIN, D_MAX, V_MAX, V_MIN)
	v_right = MapStimulus(d_right, D_MIN, D_MAX, V_MAX, V_MIN)
	return v_left, v_right

def MapStimulus(s, s_min, s_max, r_min, r_max):
	if s >= s_max:
		return r_max:
	elif s > s_min:
		#y = mx + b
		m = (r_max - r_min)/(s_max - s_min)
		b = r_min - (m*s_min)
		return (m*s) + b
	else:
		return r_min;
		