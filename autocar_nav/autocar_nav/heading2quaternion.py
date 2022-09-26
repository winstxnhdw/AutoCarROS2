import numpy as np

from geometry_msgs.msg import Quaternion

def heading_to_quaternion(heading):
    ''' 
    Converts yaw heading to quaternion coordinates.
    '''
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = np.sin(heading / 2)
    quaternion.w = np.cos(heading / 2)

    return quaternion