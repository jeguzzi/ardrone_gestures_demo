import rospy
from std_msgs.msg import String
from ardrone_autonomy.srv import FlightAnim,LedAnim 

led_anim=None
flight_anim=None

def call_flight_anim(i):
    if flight_anim is None:
        return 
    try:
      response = flight_anim(id=i,duration=0)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

def call_led_anim(i):
    if led_anim is None:
        return 
    try:
      response = led_anim(id=i,freq=4,duration=3)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))
    
def callback(msg):
    word=msg.data
    
    if(word=='FAIL'):
        call_flight_anim(4)
    elif(word=='END'):
        call_led_anim(4)
    else:
        call_led_anim(5)
    

if __name__ == '__main__':
    rospy.init_node("ardrone_feedback")
    rospy.Subscriber("words", String, callback)
    rospy.wait_for_service('setledanimation')
    rospy.wait_for_service('setflightanimation')
    led_anim = rospy.ServiceProxy('setledanimation', LedAnim)
    flight_anim = rospy.ServiceProxy('setflightanimation', FlightAnim)
    
    