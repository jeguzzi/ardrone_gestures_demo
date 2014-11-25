import math
import rospy

from ardrone_autonomy.srv import LedAnim
from ardrone_autonomy.srv import FlightAnim

from drone_info import DroneCommands
from tag import VisionData

def msg_to_quaternion(msg):
    return [msg.x, msg.y, msg.z, msg.w]

def rad_to_deg(alpha):
    return (alpha*180)/math.pi

def deg_to_rad(alpha):
    return (alpha*math.pi)/180

def normalize_angle(alpha):
    while alpha > 180: alpha -= 360
    while alpha < -180: alpha += 360
    return alpha

def adjust_command(roll, pitch, yaw, gaz):
    if roll > 1: roll = 1
    elif roll < -1: roll = -1
    
    if pitch > 1: pitch = 1
    elif pitch < -1: pitch = -1
    
    if yaw > 1: yaw = 1
    elif yaw < -1: yaw = -1
    
    if gaz > 0: gaz = gaz*1.3
    if gaz < 0: gaz = gaz/1.8
    if gaz > 1: gaz = 1
    elif gaz < -1: gaz = -1
    
    return [roll, pitch, yaw, gaz]

def led_animation(id_anim):
    rospy.wait_for_service('/ardrone/setledanimation')
    try:
        animation = rospy.ServiceProxy('/ardrone/setledanimation', LedAnim)
        res = animation(id_anim,4,3)
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e
        
def flight_animation(id_anim):
    rospy.wait_for_service('/ardrone/setflightanimation')
    try:
        animation = rospy.ServiceProxy('/ardrone/setflightanimation', FlightAnim)
        res = animation(id_anim,0)
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e        
        
class RingBuffer:
    def __init__(self, size):
        self.data = [None for i in xrange(size)]

    def append(self, x):
        self.data.pop(0)
        self.data.append(x)

    def get(self):
        return self.data
    
    def is_empty(self):
        empty = True
        for elem in self.data:
            if elem is not None:
                empty = False
        return empty 
