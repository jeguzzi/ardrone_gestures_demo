import rospy
from std_msgs.msg import String
    
class Word(object):
    def __init__(self,value,action=None):
        self.value=value
        self.action=action
        self.next={}
    
    def next_word(self,string):
        if self.next.has_key(string):
            word=self.next[string]
            word_pub.publish(word.value)
            return word
        else:
            return self
     
    @property        
    def is_end_of_sentence(self):
        return len(self.next)==0
            
class Parser(object):
    
    TIME_CL=1.5
    TIME_FEEDBACK=2.5
  
    begin=Word('BEGIN')
    end=Word('END')
    fail=Word('FAIL')
    you=Word('you')
    land=Word('land')
    take_off=Word('take off')
    follow=Word('follow')
    follow_left=Word('left')
    follow_right=Word('right')
    begin.next['you']=begin.next['top_blob']=begin.next['top_two_blobs']=you
    begin.next['take off']=begin.next['big_blob']=take_off
    you.next['follow']=you.next['top_two_blobs']=follow
    you.next['land']=you.next['top_blob']=land
    follow.next['left']=follow.next['left_blob']=follow_left
    follow.next['right']=follow.next['right_blob']=follow_right
     
    def __init__(self):    
        self.gesture_sub=rospy.Subscriber("/ardrone/gestures",String,hasReceivedNewGesture);
        self.word_pub = rospy.Publisher("/ardrone/words", queue_size=1);
        self.word=self.begin
        self.gesture=None
        

    def hasReceivedNewGesture(self,msg):
        self.gesture=msg.data;

    def update(self):
        
        new_word=self.word.next_word(self.gesture)
        
        if (self.word is self.begin) and (new_word is None):
            return
            
        if new_word:
            self.word=new_word
            self.word_pub.publish(self.word.value)
            if self.word.is_end_of_sentence:
                self.word_pub.publish('END')
                self.word=begin
        else:
            self.word_pub.publish('FAIL')
            self.word=begin

        rospy.sleep(TIME_FEEDBACK+TIME_CL)
        

if __name__ == '__main__':
    rospy.init_node("grammar_parser")
    parser=Parser()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        parser.update()
        r.sleep()

