from math import cos, sin

from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(heading):
    ''' 
    Converts yaw heading to quaternion coordinates.
    '''
    theta = 0.5 * heading
    quaternion = Quaternion()
    quaternion.z = sin(theta)
    quaternion.w = cos(theta)

    return quaternion