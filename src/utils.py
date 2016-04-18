import math

# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from tf import transformations as tft

def quaternion_to_heading(quaternion):
    """
    Converts a quaternion to equivalent Euler yaw/heading
    input: nav_msgs.msg.Quaternion
    output: euler heading in radians
    """
    try:
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    except AttributeError:
        quat = quaternion
    yaw = tft.euler_from_quaternion(quat)[2]
    return yaw

def heading_to_quaternion(heading):
    """
    Converts a Euler yaw/heading angle to equivalent quaternion
    input: euler heading in radians
    output: nav_msgs.msg.Quaternion
    """

    quat = tft.quaternion_from_euler(0, 0, heading)

    quaternion = Quaternion()
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]
    return quaternion

def dot_product(vec1, vec2):
    """
    calcuates the dot product of two tuples/vectors a, b
    input: 2 three-tuples a, vec2
    output: double: dot product
    """
    return vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2]

def cross_product(vec1, vec2):
    """
    calcuates the cross product of two tuples/vectors a, b
    input: 2 three-tuples vec1, vec2
    output: three-tuple (cross product of a, vec2)

     i    j    k
    a[0] a[1] a[2]
    b[0] b[1] b[2]
    """
    i = vec1[1]*vec2[2]-vec1[2]*vec2[1]
    j = vec1[0]*vec2[2]-vec1[2]*vec2[0]
    k = vec1[0]*vec2[1]-vec1[1]*vec2[0]
    return (i, j, k,)

def scale(vector, magnitude):
    """
    scales the given vector by the magnitude (scalar multiplication)
    input: three-tuple vector, double magnitude
    output: three-tuple scaled vector
    """
    return (vector[0]*magnitude, vector[1]*magnitude, vector[2]*magnitude)

def unit(ector):
    """
    returns the unit vector in the same direction as the given vector
    """
    length = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+
        vector[2]*vector[2])
    if under_minimum(vector):
        raise ZeroDivisionError('vector len 0 can;t be scaled to unit vector')
    return scale(vector, 1.0/length)

def under_minimum(vector):
    # TODO(buckbaskin): implement for real
    return False

def addv(vec1, vec2):
    return (vec1[0]+vec2[0], vec1[1]+vec2[1], vec1[2]+vec2[2],)
