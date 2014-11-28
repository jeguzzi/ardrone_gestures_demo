#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_gesture_demo')
import rospy

from std_msgs.msg import Int8, Int16, Empty

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

PATH = '/home/juan/catkin_ws/src/ardrone_gestures_demo/drone_gesture_demo/sounds/'
COMMAND_FOR_ME = PATH + 'command_for_me.wav' #you gesture
END_SENTENCE = PATH + 'end_sentence.wav' #before land, right e left
ERROR = PATH + 'error.wav'
FOLLOW = PATH + 'follow.wav' #follow gesture
FOLLOW_ALE = PATH + 'follow_ale.wav' #after switch marker (before, ok word)
FOLLOW_GIANNI = PATH + 'follow_gianni.wav'
FOLLOW_JACK = PATH + 'follow_jack.wav'
FOLLOW_JAWAD = PATH + 'follow_jerome.wav'
FOLLOW_JEROME = PATH + 'follow_jerome.wav'
FOLLOW_LUCA = PATH + 'follow_luca.wav'
FOLLOW_UNKNOWN = PATH + 'follow_unknown.wav'
LANDING = PATH + 'landing.wav' #land gesture
NOBODY_FOUND = PATH + 'nobody_found.wav' #ad-hoc message
OK_WORD = PATH + 'ok_word.wav' 
SEARCH_LEFT = PATH + 'search_left.wav' #left gesture
SEARCH_RIGHT = PATH + 'search_right.wav' #right gesture
SENTENCE_UNFINISHED = PATH + 'sentence_unfinished.wav' #ad-hoc message
TAKEOFF = PATH + 'take_off.wav' #takeoff gesture

COMMAND_FOR_ME_2 = PATH + 'command_for_me_2.wav' #you gesture
FOLLOW_2 = PATH + 'follow_2.wav' #follow gesture
FOLLOW_ALE_2 = PATH + 'follow_ale_2.wav' #after switch marker (before, ok word)
FOLLOW_GIANNI_2 = PATH + 'follow_gianni_2.wav'
FOLLOW_JACK_2 = PATH + 'follow_jack_2.wav'
FOLLOW_JAWAD_2 = PATH + 'follow_jerome_2.wav'
FOLLOW_JEROME_2 = PATH + 'follow_jerome_2.wav'
FOLLOW_LUCA_2 = PATH + 'follow_luca_2.wav'
FOLLOW_UNKNOWN_2 = PATH + 'follow_unknown_2.wav'
LANDING_2 = PATH + 'landing_2.wav' #land gesture
NOBODY_FOUND_2 = PATH + 'nobody_found_2.wav' #ad-hoc message 
SEARCH_LEFT_2 = PATH + 'search_left_2.wav' #left gesture
SEARCH_RIGHT_2 = PATH + 'search_right_2.wav' #right gesture
SENTENCE_UNFINISHED_2 = PATH + 'sentence_unfinished_2.wav' #ad-hoc message
TAKEOFF_2 = PATH + 'take_off_2.wav' #takeoff gesture


class Sounds(object):
    def __init__(self):
        self.soundhandle = SoundClient()
        self.subGesture = rospy.Subscriber('/ardrone/gest', Int8, self.receive_gesture)
        self.subMarker = rospy.Subscriber('/ardrone/switch_marker', Int16, self.receive_marker)
        self.subNobodyFound = rospy.Subscriber('/ardrone/nobody_found', Empty, self.receive_nobody)
        self.subTakeOff = rospy.Subscriber('/ardrone/takeoff', Empty, self.receive_takeoff)
        self.last_marker_1 = None
        
        self.subGesture = rospy.Subscriber('/ardrone_2/gest', Int8, self.receive_gesture_2)
        self.subMarker = rospy.Subscriber('/ardrone_2/switch_marker', Int16, self.receive_marker_2)
        self.subNobodyFound = rospy.Subscriber('/ardrone_2/nobody_found', Empty, self.receive_nobody_2)
        self.subTakeOff = rospy.Subscriber('/ardrone_2/takeoff', Empty, self.receive_takeoff_2)
        self.last_marker_2 = None
        
        
    def receive_takeoff(self, msg):
        print "received takeoff"
        self.last_marker_1 = None
        self.soundhandle.playWave(END_SENTENCE)
        rospy.sleep(1)
        self.soundhandle.playWave(TAKEOFF)
        
    
    def receive_nobody(self, msg):
       self.soundhandle.playWave(ERROR)
       rospy.sleep(1.0)
       self.soundhandle.playWave(NOBODY_FOUND)
       rospy.sleep(3.0)
       if self.last_marker_1 == 7:
          self.soundhandle.playWave(FOLLOW_JACK)
       
       elif self.last_marker_1 == 16:
           self.soundhandle.playWave(FOLLOW_UNKNOWN)
      
       elif self.last_marker_1 == 0:
           self.soundhandle.playWave(FOLLOW_ALE)
       
       elif self.last_marker_1 == 1:
           self.soundhandle.playWave(FOLLOW_JEROME)
       
       elif self.last_marker_1 == 2:
           self.soundhandle.playWave(FOLLOW_GIANNI)
       
       elif self.last_marker_1 == 3:
           self.soundhandle.playWave(FOLLOW_LUCA)
       
       elif self.last_marker_1 == 4:
           self.soundhandle.playWave(FOLLOW_JAWAD)                          
    
    def receive_marker(self, msg):
        if msg.data == 7:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_JACK)
        
        elif msg.data == 16:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_UNKNOWN)
        
        elif msg.data == 0:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_ALE)
                
        elif msg.data == 1:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_JEROME)
                             
        elif msg.data == 2:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_GIANNI)
       
        elif msg.data == 3:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_LUCA)
                         
        elif msg.data == 4:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_JAWAD)                 
        self.last_marker_1 = msg.data        
    
    def receive_gesture(self, msg):
        if msg.data == 1:
            self.soundhandle.playWave(OK_WORD)
            rospy.sleep(1.0)
            self.soundhandle.playWave(COMMAND_FOR_ME)
        
        elif msg.data == 2:
            self.soundhandle.playWave(OK_WORD)
            rospy.sleep(1.0)
            self.soundhandle.playWave(FOLLOW)
        
        elif msg.data == 3:
             self.soundhandle.playWave(END_SENTENCE)
             rospy.sleep(1)
             self.soundhandle.playWave(LANDING)      
        
        elif msg.data == 4:
             self.soundhandle.playWave(END_SENTENCE)
             rospy.sleep(1)
             self.soundhandle.playWave(SEARCH_LEFT)
             
        elif msg.data == 5:
             self.soundhandle.playWave(END_SENTENCE)
             rospy.sleep(1)
             self.soundhandle.playWave(SEARCH_RIGHT)
        
        elif msg.data == 10:
             self.soundhandle.playWave(ERROR)
             rospy.sleep(1)
             self.soundhandle.playWave(SENTENCE_UNFINISHED)
    
    def receive_takeoff_2(self, msg):
        self.last_marker_2 = None
        self.soundhandle.playWave(END_SENTENCE)
        rospy.sleep(1)
        self.soundhandle.playWave(TAKEOFF_2)
        
    
    def receive_nobody_2(self, msg):
       self.soundhandle.playWave(ERROR)
       rospy.sleep(1.0)
       self.soundhandle.playWave(NOBODY_FOUND_2)
       rospy.sleep(3.0)
       if self.last_marker_1 == 7:
          self.soundhandle.playWave(FOLLOW_JACK_2)
       
       elif self.last_marker_1 == 16:
           self.soundhandle.playWave(FOLLOW_UNKNOWN_2)
      
       elif self.last_marker_1 == 0:
           self.soundhandle.playWave(FOLLOW_ALE_2)
       
       elif self.last_marker_1 == 1:
           self.soundhandle.playWave(FOLLOW_JEROME_2)
       
       elif self.last_marker_1 == 2:
           self.soundhandle.playWave(FOLLOW_GIANNI_2)
       
       elif self.last_marker_1 == 3:
           self.soundhandle.playWave(FOLLOW_LUCA_2)
       
       elif self.last_marker_1 == 4:
           self.soundhandle.playWave(FOLLOW_JAWAD_2)                          
    
    def receive_marker_2(self, msg):
        if msg.data == 7:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_JACK_2)
        
        elif msg.data == 16:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_UNKNOWN_2)
        
        elif msg.data == 0:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_ALE_2)
                
        elif msg.data == 1:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_JEROME_2)
                             
        elif msg.data == 2:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_GIANNI_2)
       
        elif msg.data == 3:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_LUCA_2)
                         
        elif msg.data == 4:
            if self.last_marker_1 is not None and msg.data == self.last_marker_1:
                self.last_marker_1 = None
            
            else:    
                self.soundhandle.playWave(OK_WORD)
                rospy.sleep(1.0)
                self.soundhandle.playWave(FOLLOW_JAWAD_2)                 
        self.last_marker_1 = msg.data        
    
    def receive_gesture_2(self, msg):
        if msg.data == 1:
            self.soundhandle.playWave(OK_WORD)
            rospy.sleep(1.0)
            self.soundhandle.playWave(COMMAND_FOR_ME_2)
        
        elif msg.data == 2:
            self.soundhandle.playWave(OK_WORD)
            rospy.sleep(1.0)
            self.soundhandle.playWave(FOLLOW_2)
        
        elif msg.data == 3:
             self.soundhandle.playWave(END_SENTENCE)
             rospy.sleep(1)
             self.soundhandle.playWave(LANDING_2)      
        
        elif msg.data == 4:
             self.soundhandle.playWave(END_SENTENCE)
             rospy.sleep(1)
             self.soundhandle.playWave(SEARCH_LEFT_2)
             
        elif msg.data == 5:
             self.soundhandle.playWave(END_SENTENCE)
             rospy.sleep(1)
             self.soundhandle.playWave(SEARCH_RIGHT_2)
        
        elif msg.data == 10:
             self.soundhandle.playWave(ERROR)
             rospy.sleep(1)
             self.soundhandle.playWave(SENTENCE_UNFINISHED_2)               
 
def main():
    rospy.init_node('sounds')
    s = Sounds()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Keyboard interrupted"
        
if __name__ == '__main__':
    main() 
