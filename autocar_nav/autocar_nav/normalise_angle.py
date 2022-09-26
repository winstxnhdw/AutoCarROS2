import math as m

def normalise_angle(angle):
    
    angle = m.atan2(m.sin(angle), m.cos(angle))

    return angle