#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_swarm')
import rospy
import math

from std_msgs.msg import String
from ar_track_alvar.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from ardrone_autonomy.msg import Navdata 
from ardrone_swarm.msg import Position

import utils
from drone_info import DroneStatus
from tag import Tag, TagAR, TagIds, RoomTagCoor, get_average_tag_info, get_alpha_by_vision_info, get_ref_coor

class StateEstimation(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.alpha = 0.0
        
        self.last_altd = None
        self.last_rotZ = None
        self.start_time = rospy.Time.now()
        self.prev_time = rospy.Time.now()
        
        self.tags_buffer_room = utils.RingBuffer(10)
        self.using_odometry = True
        
        self.subARMarkerRoom = rospy.Subscriber('/ar_pose_marker_room', AlvarMarkers, self.receive_marker)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.receive_navdata)
        self.subResetPosition = rospy.Subscriber('/ardrone/reset_position', String, self.receive_reset)
        
        self.tag_located = False
        #put tag_located flag to true when a room tag is first seen
        self.pubPosition = rospy.Publisher('/ardrone/position', Position)
    
    #if I see some room marker, use them to update the position. if not, update position using odometry.
    def receive_marker(self,data):
        tag_seen = False
        for marker in data.markers:
            if marker.id == TagIds.RoomLeftTag or marker.id == TagIds.RoomFrontTag:
                tag_seen = True        
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                self.tags_buffer_room.append(TagAR(marker.id,marker.pose.pose.position.x,
                                              marker.pose.pose.position.y,
                                              marker.pose.pose.position.z,
                                              roll,pitch,yaw))
        if not tag_seen:
            self.tags_buffer_room.append(None)
            
        if self.tags_buffer_room.is_empty():
            self.using_odometry = True
        
        else: 
            self.using_odometry = False
            if not self.tag_located:
                self.tag_located = True
            #get the tag tbelt will be used for estimating the position + drone orientation
            self.alpha,ref_tag = get_alpha_by_vision_info(self.tags_buffer_room, 0.0)
            #get informations relative to tbelt tag
            x_tag,y_tag,dist_tag,pitch_tag,count_tag = get_average_tag_info(self.tags_buffer_room, ref_tag)
            #compute the horizontal displacement
            displacement_x = utils.rad_to_deg(math.atan2(x_tag, dist_tag))
            
            x_ref,y_ref,z_ref = get_ref_coor(ref_tag)
            
            self.x = x_ref - math.cos(utils.deg_to_rad(self.alpha - displacement_x))*(dist_tag + 0.15)
            self.y = y_ref - math.sin(utils.deg_to_rad(self.alpha - displacement_x))*(dist_tag + 0.15)
            self.z = z_ref + y_tag
            self.pubPosition.publish(Position(self.alpha, self.x, self.y, self.z, self.tag_located, displacement_x, ref_tag))
                                 
    #use this when estimating position with markers
    def update_time(self):
        self.prev_time = rospy.Time.now()
        
    def receive_reset(self, msg):
        if msg.data == 'RESET':
            self.x = 0
            self.y = 0
            self.z = 0
            self.alpha = 0
        
    def receive_navdata(self, navdata):
        if navdata.state == DroneStatus.TakingOff:
            self.start_time = rospy.Time.now()
            
        if self.last_altd is None or self.last_rotZ is None:
            self.update_time()
            self.last_altd = navdata.altd/1e3
            self.last_rotZ = navdata.rotZ
         
        else:
            if self.using_odometry: 
                
                measures_corrupted = False
                
                alpha_variation = navdata.rotZ - self.last_rotZ
                while alpha_variation > 180: alpha_variation -= 360
                while alpha_variation < -180: alpha_variation += 360
                
                z_variation = navdata.altd/1e3 - self.last_altd
                
                if (rospy.Time.now() - self.start_time).to_sec() < 5: 
                    z_tol = 0.7 
                else: z_tol = 0.12
                
                if abs(alpha_variation) > 15 or abs(z_variation) > z_tol:
                    measures_corrupted = True
                    rospy.loginfo("Measurements corrupted!")
       
                if not measures_corrupted:
                    
                    self.alpha = self.alpha + alpha_variation
                    while self.alpha > 180: self.alpha -= 360
                    while self.alpha < -180: self.alpha += 360
                                       
                    self.z = self.z + z_variation                    

                    now = rospy.Time.now()
                    delta_t = (now - self.prev_time).to_sec()
                    self.prev_time = now
                    
                    alpha_rad = utils.deg_to_rad(self.alpha)
                    vx = navdata.vx/1e3
                    vy = navdata.vy/1e3
                    
                    self.x = self.x + delta_t*(vx*math.cos(alpha_rad) - vy*math.sin(alpha_rad)) 
                    self.y = self.y + delta_t*(vx*math.sin(alpha_rad) + vy*math.cos(alpha_rad))
                    
                    self.pubPosition.publish(Position(self.alpha, self.x, self.y, self.z, self.tag_located, None, -1))
                    
                    self.last_rotZ = navdata.rotZ
                    self.last_altd = navdata.altd/1e3
                
                else:
                    self.update_time() 
                    self.last_rotZ = navdata.rotZ
                    self.last_altd = navdata.altd/1e3
            
            #event if I'm not using odometry, keep up to date useful datas
            else:
                    self.update_time()
                    self.last_rotZ = navdata.rotZ
                    self.last_altd = navdata.altd/1e3        
 
def main():
    rospy.init_node( 'state_estimation' )
    se = StateEstimation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Keyboard interrupted"
        
if __name__ == '__main__':
    main()              