#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_gesture_demo')
import rospy
import sys
import math

from geometry_msgs.msg import Point   
from std_msgs.msg import Empty, String, Int8, Int16           
from std_srvs.srv import Empty as EmptySrv
from ardrone_autonomy.msg import Navdata 
#from ardrone_swarm.msg import Position, Drone_info, led
from gesture_messages.msg import Position, Drone_info, led
#from ar_track_alvar.msg import AlvarMarkers
from ar_track_alvar_msgs_idsia.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

import pid
import utils
from drone_info import DroneStatus, DroneCommands, DroneTask, DroneInfo, Gestures, Faces
from controllers_misc import PidWrapper, CurrNavdata, rotate, get_roll_follow_belt
from tag import TagAR, get_average_tag_info, get_angles_from_vision, TagIds, get_box_ang_pos, get_belt_ang_pos, VisionData

TIME = 0.1
TOL_MARKER = 8
K_ANG_Z = 0.02
POWER_YAW = 0.15
TOL_DIST = 0.5
ALT = 0.8
ALT_HIGH = 1.2
ALT_HIGH_2 = 1.5
VEL_TAN = 0.35 #0.18
VEL_BACK = -1
N_M = 5

class DroneController(object):
    def __init__(self, id):        

        #from individual tag topic
        self.led = True
        self.tags_buffer_follow = utils.RingBuffer(15)
        self.tags_buffer_box_sides = utils.RingBuffer(10)
        self.tags_buffer_belt_sides = utils.RingBuffer(10)
        
        #from bundle tag topic
        self.tags_buffer_box = utils.RingBuffer(10)
        self.tags_buffer_belt = utils.RingBuffer(10)
        
        self.navdata = CurrNavdata()
    
        #a bunch of controllers
        self.controller_pos_x = PidWrapper(0.5, 0.0, 0.75, 0.0) 
        self.controller_pos_y = PidWrapper(0.5, 0.0, 0.75, 0.0)
        self.controller_pos_z = PidWrapper(0.6, 0.0, 0.2, ALT)  #0.6
        self.controller_speed_y = PidWrapper(0.5, 0.0, 0.5, 0.0)   #0.5 0.5
        self.controller_speed_x = PidWrapper(0.5, 0.0, 0.5, 0.0)
        
        self.drone_info = DroneInfo(id)
        self.init = True
        
        #position control
        self.target_x, self.target_y, self.target_z, self.target_yaw = [0.0, 0.0, 0.0, 0.0]
        
        #sync orbit!
        self.drone_2_info = DroneInfo()
        self.drone_3_info = DroneInfo()
        self.drone_4_info = DroneInfo()
        
        self.subARMarkerBundle = rospy.Subscriber('/ar_pose_marker_bundle', AlvarMarkers, self.receive_marker_bundle)
        self.subARMarkerIndividual = rospy.Subscriber('/ar_pose_marker_individual', AlvarMarkers, self.receive_marker_individual)
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.receive_navdata) 
        self.subPosition = rospy.Subscriber('/ardrone/position', Position, self.receive_position)
        self.subDrone2Info = rospy.Subscriber('/ardrone/info_in_1', Drone_info, self.receive_drone_2_info)
        self.subDrone3Info = rospy.Subscriber('/ardrone/info_in_2', Drone_info, self.receive_drone_3_info)
        self.subDrone4Info = rospy.Subscriber('/ardrone/info_in_3', Drone_info, self.receive_drone_4_info)
        self.subGesture = rospy.Subscriber('/ardrone/gest', Int8, self.receive_gesture)
        self.subFace = rospy.Subscriber('/ardrone/face', Int8, self.receive_face)
        self.pubDroneInfo = rospy.Publisher('/ardrone/out_info', Drone_info)
        self.pubLand    = rospy.Publisher('/ardrone/land', Empty)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        self.pubReset   = rospy.Publisher('/ardrone/reset', Empty)
        self.pubBoxPosition = rospy.Publisher('/ardrone/box_position', Point)
        self.pubResetPosition = rospy.Publisher('/ardrone/reset_position', String)
        self.pubSwitchMarker = rospy.Publisher('/ardrone/switch_marker', Int16)
        self.pubLanded = rospy.Publisher('/ardrone/landed', Empty)
        
        self.pubLed = rospy.Publisher('/drone1/led', led)
        
        self.cur_marker = None
        self.marker_count = 0
        
        self.after_move = None
        self.time_takeoff = None 

    def receive_marker_bundle(self, data):
        box_tag_seen = False
        belt_tag_seen = False
        for marker in data.markers:
            #just to be sure
            if marker.id == TagIds.MasterTag:
                box_tag_seen = True
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                self.tags_buffer_box.append(TagAR(marker.id,marker.pose.pose.position.x,
                                                  marker.pose.pose.position.y,
                                                  marker.pose.pose.position.z,
                                                  roll, pitch, yaw))
            
            elif marker.id == TagIds.BeltFakeTag:
                belt_tag_seen = True
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                self.tags_buffer_belt.append(TagAR(marker.id,marker.pose.pose.position.x,
                                                  marker.pose.pose.position.y,
                                                  marker.pose.pose.position.z,
                                                  roll, pitch, yaw))
                        
        if not box_tag_seen: self.tags_buffer_box.append(None)
        if not belt_tag_seen: self.tags_buffer_belt.append(None)     
     
    def receive_marker_individual(self,data):
        box_sides_tag_seen = False
        belt_sides_tag_seen = False
        follow_tag_seen = False 
        for marker in data.markers:
            if (self.drone_info.task == DroneTask.DemoGesture and self.drone_info.tag_to_follow is None
                and (self.time_takeoff is not None and (rospy.Time.now() - self.time_takeoff).to_sec() > 5) and 
                (self.drone_info.command == DroneCommands.StartFollow or self.drone_info.command == DroneCommands.Rotate)):
                if self.drone_info.last_tag_to_follow is None:
                    if marker.id in TagIds.AllFollowTags:
                        if self.cur_marker is None:
                            self.cur_marker = marker.id
                            self.marker_count += 1
                        
                        else:
                            if marker.id == self.cur_marker:
                                self.marker_count += 1
                                if self.marker_count == N_M:
                                    self.marker_count = 0
                                    self.cur_marker = None
                                    self.drone_info.tag_to_follow = marker.id
                                    self.pubSwitchMarker.publish(Int16(marker.id))
                                    print "New tag to follow: %d" %marker.id
                            else:
                                self.cur_marker = marker.id
                                self.marker_count = 1        
                
                elif marker.id in filter(lambda x: x != self.drone_info.last_tag_to_follow, TagIds.AllFollowTags):
                    if self.cur_marker is None:
                            self.cur_marker = marker.id
                            self.marker_count += 1
                        
                    else:
                        if marker.id == self.cur_marker:
                            self.marker_count += 1
                            if self.marker_count == N_M:
                                self.marker_count = 0
                                self.cur_marker = None
                                self.drone_info.tag_to_follow = marker.id
                                self.drone_info.last_tag_to_follow = None
                                self.pubSwitchMarker.publish(Int16(marker.id))
                                print "New tag to follow: %d" %marker.id
                        else:
                            self.cur_marker = marker.id
                            self.marker_count = 1 
                            
            if (self.drone_info.tag_to_follow is not None and
               ((marker.id == TagIds.FollowTag and self.drone_info.tag_to_follow == TagIds.FollowTag)or 
                (marker.id == TagIds.FollowTag2 and self.drone_info.tag_to_follow == TagIds.FollowTag2)or
                (marker.id == TagIds.FollowTag3 and self.drone_info.tag_to_follow == TagIds.FollowTag3)or
                (marker.id == TagIds.FollowTag4 and self.drone_info.tag_to_follow == TagIds.FollowTag4)or
                (marker.id == TagIds.FollowTag5 and self.drone_info.tag_to_follow == TagIds.FollowTag5)or
                (marker.id == TagIds.FollowTag6 and self.drone_info.tag_to_follow == TagIds.FollowTag6)or
                (marker.id == TagIds.FollowTag7 and self.drone_info.tag_to_follow == TagIds.FollowTag7))):
                follow_tag_seen = True
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                self.tags_buffer_follow.append(TagAR(marker.id,marker.pose.pose.position.x,
                                                     marker.pose.pose.position.y,
                                                     marker.pose.pose.position.z,
                                                     roll, pitch, yaw))          

            elif self.drone_info.box_master_tag == TagIds.MasterTag and (marker.id == TagIds.FrontTag
                 or marker.id == TagIds.BackTag or marker.id == TagIds.LeftTag or marker.id == TagIds.RightTag):
                box_sides_tag_seen = True
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                self.tags_buffer_box_sides.append(TagAR(marker.id,marker.pose.pose.position.x,
                                                     marker.pose.pose.position.y,
                                                     marker.pose.pose.position.z,
                                                     roll, pitch, yaw))
            
            elif marker.id == TagIds.BeltFrontTag or marker.id == TagIds.BeltBackTag or marker.id == TagIds.BeltLeftTag or marker.id == TagIds.BeltRightTag:
                belt_sides_tag_seen = True
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                self.tags_buffer_belt_sides.append(TagAR(marker.id,marker.pose.pose.position.x,
                                                     marker.pose.pose.position.y,
                                                     marker.pose.pose.position.z,
                                                     roll, pitch, yaw))
                
        if not follow_tag_seen: self.tags_buffer_follow.append(None)
        if not box_sides_tag_seen: self.tags_buffer_box_sides.append(None)
        if not belt_sides_tag_seen: self.tags_buffer_belt_sides.append(None)
        
        if self.tags_buffer_box_sides.is_empty(): self.drone_info.box_in_sight = False
        else: 
            self.drone_info.box_in_sight = True
            self.drone_info.alpha_box = get_box_ang_pos(self.tags_buffer_box_sides, 0.0)
            
        if self.tags_buffer_belt_sides.is_empty(): self.drone_info.belt_in_sight = False
        else: 
            self.drone_info.belt_in_sight = True
            self.drone_info.alpha_belt = get_belt_ang_pos(self.tags_buffer_belt_sides, 0.0)
                                               
    def receive_navdata(self, navdata):
        self.navdata.set_navdata(navdata.state, navdata.altd/1000, navdata.rotZ, navdata.vx/1000, navdata.vy/1000, navdata.vz/1000, 
                                 navdata.ax/1000, navdata.ay/1000, navdata.az/1000)
        
        self.drone_info.status = navdata.state
        
        if self.init:
            self.init = False
            self.drone_info.init() 

    def receive_position(self, position):
        self.drone_info.alpha, self.drone_info.x, self.drone_info.y, self.drone_info.z, self.drone_info.marker_displacement, self.drone_info.room_marker_tracked, self.drone_info.room_located = [position.alpha, position.x, position.y, position.z, position.marker_displacement, position.room_marker_tracked, position.tag_located]
            
        
    def receive_drone_2_info(self, info):
        self.drone_2_info.set_drone_info(info.status, info.task, info.command, info.alpha, info.x, info.y, info.z, info.alpha_box, info.box_in_sight, info.alpha_belt, info.belt_in_sight)
    
    def receive_drone_3_info(self, info):
        self.drone_3_info.set_drone_info(info.status, info.task, info.command, info.alpha, info.x, info.y, info.z, info.alpha_box, info.box_in_sight, info.alpha_belt, info.belt_in_sight)
        
    def receive_drone_4_info(self, info):
        self.drone_4_info.set_drone_info(info.status, info.task, info.command, info.alpha, info.x, info.y, info.z, info.alpha_box, info.box_in_sight, info.alpha_belt, info.belt_in_sight)
    
    #this function will be modified to handle the actual signal received
    def receive_gesture(self, msg):
        print "task in receive gesture = %d" %self.drone_info.task
        if self.drone_info.task == DroneTask.DemoGesture:
            rospy.loginfo("GESTURE RECEIVED %d" %(msg.data))
            self.drone_info.gesture = msg.data
            
            if self.drone_info.gesture == Gestures.You and self.drone_info.command == DroneCommands.StartFollow:
                if not self.led:
                    #self.drone_info.time_go_up = rospy.Time.now()
                    pass
                else:
                   self.drone_info.time_go_up = rospy.Time.now()     
                   self.pubLed.publish(led("off", 1, 0.3))
            elif self.drone_info.gesture == Gestures.Follow and self.drone_info.command == DroneCommands.StartFollow:
                if self.led:
                    self.pubLed.publish(led("off", 1, 0.3))
                
            elif self.drone_info.gesture == Gestures.Takeoff and not self.drone_info.command == DroneCommands.StartFollow:
                self.send_follow_tag()
                if self.led:
                    self.pubLed.publish(led("off", 2, 0.3))
                print "Taking off and sending follow tag.\n"
        
            elif self.drone_info.gesture == Gestures.Land:
                self.send_land()
                if self.led:
                    self.pubLed.publish(led("off", 2, 0.3))
                print "Sending land.\n"
            
            else:
                if ((self.drone_info.gesture == Gestures.Left or self.drone_info.gesture == Gestures.Right) 
                    and not (self.drone_info.command == DroneCommands.Rotate or self.drone_info.command == DroneCommands.GoAtPoint)):
                    if self.drone_info.gesture == Gestures.Left:
                        self.drone_info.search_yaw = -0.3
                    else:
                        self.drone_info.search_yaw = 0.3   
                    
                    print "Rotating and searching for another marker.\n"
                    print "Setting change demo to true.\n"
                    self.drone_info.last_tag_to_follow = self.drone_info.tag_to_follow
                    self.drone_info.tag_to_follow = None
                    self.drone_info.change_demo = True
                    if self.led:     
                        self.pubLed.publish(led("off", 2, 0.3))
             
    
    def receive_face(self, msg):
        rospy.loginfo("FACE RECEIVED %d" %(msg.data))
        self.drone_info.face_dir = msg.data
            
    def send_takeoff(self):
        if(self.navdata.status == DroneStatus.Landed):
            self.time_takeoff = rospy.Time.now()
            self.pubTakeoff.publish(Empty())
            #pass
            
            
    def send_land(self):
        self.time_takeoff = None
        self.pubLand.publish(Empty())
        self.drone_info.command = DroneCommands.NoCommand
        self.init = True

    def send_emergency(self):
       # Send an emergency (or reset) message to the ardrone driver
        rospy.loginfo(rospy.get_name() + ": Emergency sent")
        self.pubReset.publish(Empty())
        
    def reset(self):
        rospy.loginfo(rospy.get_name() + ": Reset Everything")
        self.init = True
        self.pubResetPosition.publish(String("RESET"))

    def send_flat_trim(self):
        rospy.wait_for_service('/ardrone/flattrim')
        try:
            rospy.ServiceProxy('/ardrone/flattrim', EmptySrv)
            rospy.loginfo(rospy.get_name() + ": Flat trim sent")
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e 
    #setup pid function is used only at the beginning of a task. Variations to setpoints are handled separately
    #during the task, if needed.
    def setup_pid(self, pos_x, pos_y, pos_z, speed_x, speed_y):
        if pos_x is not None:
            self.controller_pos_x.setpoint = pos_x
        if pos_y is not None:
            self.controller_pos_y.setpoint = pos_y
        if pos_z is not None:
            self.controller_pos_z.setpoint = pos_z    
        if speed_x is not None:
            self.controller_speed_x.setpoint = speed_x
        if speed_y is not None:
            self.controller_speed_y.setpoint = speed_y
            
    def send_follow_tag(self):
        self.send_takeoff()
        self.drone_info.angle_to_mantain = 0.0
        self.setup_pid(1.8, 0.0, 0.0, 0.0, 0.0) #1.7
        if not self.drone_info.task == DroneTask.DemoGesture:
            self.drone_info.tag_to_follow = TagIds.FollowTag  
        self.drone_info.command = DroneCommands.StartFollow
    
    def send_follow_face(self):
        self.send_takeoff()
        print "sending follow face"
        self.setup_pid(None, None, ALT, 0.0, 0.0)
        self.drone_info.command = DroneCommands.FollowFace
    
    def send_follow_belt(self):
        self.send_takeoff()
        self.setup_pid(2.0, 0.0, 0.1, 0.0, 0.0)
        self.drone_info.command = DroneCommands.FollowBelt
        rospy.loginfo("Follow belt sent")
        
    def send_orbit(self):
        self.send_takeoff()
        self.setup_pid(1.6, None, 0.1, 0.0, -VEL_TAN)
        self.drone_info.command = DroneCommands.Orbit
    
    def send_orbit_sync(self):
        self.send_takeoff()
        self.setup_pid(1.8, None, 0.1, 0.0, -VEL_TAN)
        self.drone_info.command = DroneCommands.OrbitSync        
    
    def send_follow_me(self):
        self.send_takeoff()
        self.setup_pid(None, None, 0.1, VEL_BACK, 0.0)
        self.drone_info.command = DroneCommands.FollowMe
    
    def fly_horizontally(self):
        self.send_takeoff()
        self.drone_info.flyght_stages.compute_points(DroneTask.FlyH, self.drone_info.flyght_tag)
        self.drone_info.task = DroneTask.FlyH
    
    def demo_gesture(self):
        print "in gesture demo"
        self.drone_info.task = DroneTask.DemoGesture
        print "task after button pushed = %d" %self.drone_info.task   
    
    def fly_back_and_forth(self):
        self.send_takeoff()
        self.drone_info.flyght_stages.compute_points(DroneTask.FlyBAF, self.drone_info.flyght_tag)
        self.drone_info.task = DroneTask.FlyBAF
    
    def avoid_walls(self):
        if not self.drone_info.avoid_walls: self.drone_info.avoid_walls = True
        else: self.drone_info.avoid_walls = False
    
    def avoid_drones(self):
        if not self.drone_info.avoid_drones: self.drone_info.avoid_drones = True
        else: self.drone_info.avoid_drones = False
            
    def track_room_marker(self):
        if not self.drone_info.track_room_marker: self.drone_info.track_room_marker = True
        else: self.drone_info.track_room_marker = False
        
    def search_optimal_marker(self):
        if not self.drone_info.search_marker: self.drone_info.search_marker = True
        else: self.drone_info.search_marker = False       
    
    #called only internally
    def send_rot_exploration(self):
        #this function should be called when the drone is already flying!
        self.setup_pid(None, None, min([self.drone_info.z, ALT]), 0.0, 0.0)
        self.drone_info.command = DroneCommands.Rotate
        
    def go_at_point_test(self):
        self.go_at_point(1.5, 1, 0.9, 0)  

    #the targets are in my reference frame!    
    def go_at_point(self, target_x, target_y, target_z, target_yaw):
        self.send_takeoff()
        self.setup_pid(0.0, 0.0, None, None, 0.0)
        self.drone_info.move_accomplished = False
        self.target_x, self.target_y, self.target_z, self.target_yaw = [target_x, target_y, target_z, target_yaw]    
        self.drone_info.command = DroneCommands.GoAtPoint                    
        
    def receive_input(self,new_input):
        self.drone_info.command = new_input                    
    
    def follow_tag(self, angle_to_mantain, tag_to_follow):
        roll, pitch, yaw, gaz = [0 for i in range(4)]  
            
        x_med, y_med, dist_med, pitch_med, count = get_average_tag_info(self.tags_buffer_follow, tag_to_follow)
                 
        if tag_to_follow is not None and count is not 0:
            if self.controller_pos_z.prev_setpoint is not None:
                self.controller_pos_z.setpoint = self.controller_pos_z.prev_setpoint
            
            elif not self.controller_pos_z.setpoint == 0.0:
                self.controller_pos_z.setpoint = 0.0   
                    
            if pitch_med > 0: pitch_pos = True
            else: pitch_pos = False
                    
            displacement_x = utils.rad_to_deg(math.asin(x_med/dist_med))
            if displacement_x > 0: tag_on_left = False
            else: tag_on_left = True
                    
            yaw = - K_ANG_Z*displacement_x

            if displacement_x < 0:
                self.drone_info.last_seen = 1
            else: self.drone_info.last_seen = -1
                    
            #keep the distance
            pitch = - self.controller_pos_x.update(dist_med, -self.navdata.vx) #receive the dist_med 
            if pitch < 0: pitch *= 1.5
                      
            gaz = self.controller_pos_z.update(y_med, self.navdata.vz)
                            
            #follow the orientation
            if tag_on_left and not(pitch_pos):
                [beta,omega,alpha,gamma] = get_angles_from_vision(displacement_x, pitch_med, VisionData.TAG_ON_LEFT, VisionData.PITCH_NEG)
                displacement_y = dist_med*math.sin(alpha + angle_to_mantain)/math.sin(gamma)
            elif not(tag_on_left) and pitch_pos:
                [beta,omega,alpha,gamma] = get_angles_from_vision(displacement_x, pitch_med, VisionData.TAG_ON_RIGHT, VisionData.PITCH_POS)
                displacement_y = -dist_med*math.sin(alpha - angle_to_mantain)/math.sin(gamma)
            elif tag_on_left and pitch_pos:
                [beta,omega,alpha,gamma] = get_angles_from_vision(displacement_x, pitch_med, VisionData.TAG_ON_LEFT, VisionData.PITCH_POS)
                displacement_y = -dist_med*math.sin(alpha - angle_to_mantain)/math.sin(gamma)
            else: #tag_on_right and not pitch pos
                [beta,omega,alpha,gamma] = get_angles_from_vision(displacement_x, pitch_med, VisionData.TAG_ON_RIGHT, VisionData.PITCH_NEG)
                displacement_y = dist_med*math.sin(alpha + angle_to_mantain)/math.sin(gamma)
                     
            roll = self.controller_pos_y.update(displacement_y, self.navdata.vy)   
            if self.drone_info.angle_to_mantain is not 0.0: roll = roll / 2
            self.drone_info.last_z = self.drone_info.z                             
        else: #search starting rotating from where the tag was last seen
            if self.drone_info.last_seen == 1:
                yaw = 0#POWER_YAW
            elif self.drone_info.last_seen == -1:
                yaw = 0#-POWER_YAW
            #and try not to go around too much
            pitch = self.controller_speed_x.update(self.navdata.vx, 0.0)
            roll = self.controller_speed_y.update(self.navdata.vy, 0.0)
            
            #only the belt following changes the setpoint
            self.controller_pos_z.setpoint = ALT 
            gaz = self.controller_pos_z.update(self.drone_info.z, self.navdata.vz)
            #moreover, signal tbelt no tag was detected with a led animation
            utils.led_animation(5)
                
        return utils.adjust_command(roll, pitch, yaw, gaz)

    def orbit(self, box_tag):
        roll, pitch, yaw, gaz = [0  for i in range(4)]
        
        x_med, y_med, dist_med, pitch_med, count = get_average_tag_info(self.tags_buffer_box, box_tag)        
                
        if count is not 0:
            #if values were changed after lost of visual tracking
            if not self.controller_pos_x.setpoint == 1.6:
                self.controller_pos_x.setpoint = 1.6
        
            if not self.controller_pos_z.setpoint == 0.1:
                self.controller_pos_z.setpoint = 0.1
            #the alpha angle computed with side markers is surely related to the correct bundle!
            displacement_x = utils.rad_to_deg(math.atan2(x_med, dist_med))
            
            if not self.drone_info.box_located:
                self.drone_info.box_located = True
            
            #update box position (if only odometry is used, it will move!)    
            self.drone_info.x_box = self.drone_info.x + math.cos(utils.deg_to_rad(self.drone_info.alpha - displacement_x))*dist_med
            self.drone_info.y_box = self.drone_info.y + math.sin(utils.deg_to_rad(self.drone_info.alpha - displacement_x))*dist_med
            self.pubBoxPosition.publish(Point(self.drone_info.x_box, self.drone_info.y_box, 0.0))
                       
            yaw = -K_ANG_Z*displacement_x

            if self.drone_info.command is DroneCommands.OrbitSync:
                distances = []
                if (self.drone_2_info.box_in_sight and self.drone_2_info.command is DroneCommands.OrbitSync and 
                    self.drone_2_info.status is DroneStatus.Flying): 
                    
                    dist = self.drone_2_info.alpha_box - self.drone_info.alpha_box
                    dist = utils.normalize_angle(dist)
                    distances.append(dist)
                
                if (self.drone_3_info.box_in_sight and self.drone_3_info.command is DroneCommands.OrbitSync and 
                    self.drone_3_info.status is DroneStatus.Flying): 
                    
                    dist = self.drone_3_info.alpha_box - self.drone_info.alpha_box
                    dist = utils.normalize_angle(dist)
                    distances.append(dist)    
                #...fill with other distances#
                if not distances == []:
                    distances_pos = filter(lambda x: x >= 0, distances)
                    distances_neg = filter(lambda x: x < 0, distances)
                    
                    if not distances_pos == []:
                        dd = min(distances_pos)
                    else: dd = 360 + min(distances_neg)
                    
                    if not distances_neg == []:
                        ds = -max(distances_neg)
                    else: ds = 360 - max(distances_pos)
                    
                    self.controller_speed_y.setpoint = -(VEL_TAN + (dd - ds)/float(1000))
                    
                else: self.controller_speed_y.setpoint = -VEL_TAN        
            
            rospy.loginfo("setpoint 1: %f" %self.controller_speed_y.setpoint)           
            #keep the distance
            pitch = - self.controller_pos_x.update(dist_med, -self.navdata.vx)/float(2) #receive the dist_med   
            roll = self.controller_speed_y.update(self.navdata.vy, 0.0)
            gaz = self.controller_pos_z.update(y_med, self.navdata.vz)
            self.drone_info.last_z = self.drone_info.z
                 
        else: 
            #if visual contact is lost, try to restore it using odometry informations
            if self.drone_info.box_located:
                self.target_x = self.drone_info.x_box
                self.target_y = self.drone_info.y_box
                self.target_z = self.drone_info.last_z
                self.target_yaw = utils.rad_to_deg(math.atan((self.target_y - self.drone_info.y)/(self.target_x - self.drone_info.x)))
                if ((self.target_y > self.drone_info.y and self.target_x < self.drone_info.x) or 
                    (self.target_y < self.drone_info.y and self.target_x < self.drone_info.x)):
                    self.target_yaw += 180
                self.target_yaw = utils.normalize_angle(self.target_yaw)
                self.setup_pid(0.0, None, None, None, 0.0)
                roll, pitch, yaw, gaz = self.position_control(1.5)
                pitch = pitch / float(2) 
            else:
                #should not happen unless at the beginning
                self.setup_pid(None, None, ALT, None, 0.0)
                roll, pitch, yaw, gaz = self.rot_exploration()
            
            utils.led_animation(5)
                    
        return utils.adjust_command(roll, pitch, yaw, gaz) 
    
    def position_control(self, des_distance = 0):
        roll, pitch, yaw, gaz = [0 for i in range(4)]
        
        self.controller_pos_z.setpoint = self.target_z
        
        if self.target_yaw is not None:
            yaw,dr = rotate(self.target_yaw, self.drone_info.alpha, K_ANG_Z)
        #convert target wrt drone reference frame and change sign
        if des_distance == 0:
            alpha_rad = utils.deg_to_rad(self.drone_info.alpha)
            x_drone_ref = -math.cos(alpha_rad)*(self.target_x - self.drone_info.x) - math.sin(alpha_rad)*(self.target_y - self.drone_info.y)
            y_drone_ref = math.sin(alpha_rad)*(self.target_x - self.drone_info.x) - math.cos(alpha_rad)*(self.target_y - self.drone_info.y)
            
            target_reached = False
            if self.target_yaw is None:
                if abs(x_drone_ref) < TOL_DIST and abs(y_drone_ref) < TOL_DIST and abs(self.drone_info.z - self.target_z) < TOL_DIST:
                    target_reached = True                  
            else:
                if abs(x_drone_ref) < TOL_DIST and abs(y_drone_ref) < TOL_DIST and abs(self.drone_info.z - self.target_z) < TOL_DIST and abs(dr) < 7:
                    target_reached = True
                    
            if target_reached:
                if self.drone_info.time_keep_pos is None:
                    self.drone_info.time_keep_pos = rospy.Time.now()
                
                elif (rospy.Time.now() - self.drone_info.time_keep_pos).to_sec() > TIME:    
                    self.drone_info.move_accomplished = True
                    #hover
                    rospy.loginfo("Move accomplished!")
                    self.drone_info.time_keep_pos = None
                    roll,pitch,yaw,gaz = [0,0,0,0]
            else:
                self.drone_info.time_keep_pos = None    
                roll = self.controller_pos_y.update(y_drone_ref, self.navdata.vy)
                pitch = self.controller_pos_x.update(x_drone_ref, self.navdata.vx)
                gaz = self.controller_pos_z.update(self.drone_info.z, self.navdata.vz)
        else:
            if abs(dr) < 30: #start moving only if the target is not behind!
                distance = -(math.sqrt((self.target_x - self.drone_info.x)**2 + (self.target_y - self.drone_info.y)**2) - des_distance)
                if abs(distance) < TOL_DIST and abs(self.drone_info.z - self.target_z) < TOL_DIST and abs(dr) < 7:
                    self.drone_info.move_accomplished = True
                    #hover
                    rospy.loginfo("Move accomplished!")
                    roll,pitch,yaw,gaz = [0,0,0,0]    
                else:
                    pitch = self.controller_pos_x.update(distance, self.navdata.vx) 
                    roll = self.controller_speed_y.update(self.navdata.vy, 0.0)
                    gaz = self.controller_pos_z.update(self.drone_info.z, self.navdata.vz)     
        
        return utils.adjust_command(roll, pitch, yaw, gaz)                         
                       
    def rot_exploration(self):
        roll = self.controller_speed_y.update(self.navdata.vy,0.0)
        pitch = self.controller_speed_x.update(self.navdata.vx, 0.0)
        
        if self.drone_info.task == DroneTask.DemoGesture:
            yaw = self.drone_info.search_yaw
        else:
            yaw = POWER_YAW    
                     
        gaz = self.controller_pos_z.update(self.drone_info.z, self.navdata.vz)
            
        utils.led_animation(5)
        
        return utils.adjust_command(roll, pitch, yaw, gaz)
    
    def follow_belt(self):
        roll, pitch, yaw, gaz = [0 for i in range(4)]    
        x_med, y_med, dist_med, pitch_med, count = get_average_tag_info(self.tags_buffer_belt, TagIds.BeltFakeTag)        
                
        if count is not 0:
            if not self.controller_pos_x.setpoint == 2.0:
                self.controller_pos_x.setpoint = 2.0
        
            if not self.controller_pos_z.setpoint == 0.1:
                self.controller_pos_z.setpoint = 0.1

            if not self.drone_info.belt_located:
                self.drone_info.belt_located = True
                
            displacement_x = utils.rad_to_deg(math.atan2(x_med, dist_med))           
            yaw = -K_ANG_Z*displacement_x
            
            other_drones = filter(lambda drone: drone.command == DroneCommands.FollowBelt and drone.belt_in_sight and
                                  drone.status is DroneStatus.Flying, [self.drone_2_info, self.drone_3_info, self.drone_4_info])
            roll = get_roll_follow_belt(self.drone_info, other_drones,
                                        self.controller_pos_y, self.controller_speed_y, self.navdata)                                                     
            #keep the distance
            pitch = - self.controller_pos_x.update(dist_med, -self.navdata.vx)
            if pitch < 0: pitch = pitch*2  
            gaz = self.controller_pos_z.update(y_med, self.navdata.vz)
            
            #for later in case
            if  displacement_x < 0: 
                #for knowing from where to start rotating
                self.drone_info.last_yaw = POWER_YAW
            else:
                self.drone_info.last_yaw = -POWER_YAW
                     
            self.drone_info.last_z = self.drone_info.z
            self.drone_info.alpha_start_expl = self.drone_info.alpha      
             
        else:
            if self.drone_info.z < ALT:
                #should not happen unless it's the beginning
                self.setup_pid(None, None, ALT + 0.1, None, 0.0)
                roll = self.controller_speed_y.update(self.navdata.vy,0.0)
                pitch = self.controller_speed_x.update(self.navdata.vx, 0.0)
                gaz = self.controller_pos_z.update(self.drone_info.z, self.navdata.vz)
                 
            elif self.drone_info.belt_located:
                self.setup_pid(None, None, self.drone_info.last_z, 0.0, 0.0)
                roll = self.controller_speed_y.update(self.navdata.vy,0.0)
                pitch = self.controller_speed_x.update(self.navdata.vx, 0.0)
                
                diff = self.drone_info.alpha - self.drone_info.alpha_start_expl 
                diff = utils.normalize_angle(diff)
                if diff < -20:
                    yaw = POWER_YAW
                    self.drone_info.last_yaw = yaw
                elif diff > 20:
                    yaw = -POWER_YAW
                    self.drone_info.last_yaw = yaw
                else: yaw = self.drone_info.last_yaw
                          
                gaz = self.controller_pos_z.update(self.drone_info.z, self.navdata.vz)    
            
            utils.led_animation(5)
                    
        return utils.adjust_command(roll, pitch, yaw, gaz)
    
    def follow_me(self):
        roll, pitch, yaw, gaz = [0,0,0,0]
            
        x_med, y_med, dist_med, pitch_med, count = get_average_tag_info(self.tags_buffer_belt, TagIds.BeltFakeTag)        
                
        if count is not 0: 
            if not self.controller_pos_z.setpoint == 0.1:
                self.controller_pos_z.setpoint = 0.1
            
            if not self.controller_speed_x.setpoint == VEL_BACK:
                self.controller_speed_x.setpoint = VEL_BACK
                   
            if not self.drone_info.belt_located:
                self.drone_info.belt_located = True           
            
            if dist_med > 2.0:
                if self.drone_info.time_anim is None:
                    utils.flight_animation(4)
                    self.drone_info.time_anim = rospy.Time.now()
                
                elif (rospy.Time.now() - self.drone_info.time_anim).to_sec()> 6:
                    self.drone_info.time_anim = None    
            
            else:
                if self.drone_info.time_anim is not None:
                    self.drone_info.time_anim = None
                     
                displacement_x = utils.rad_to_deg(math.atan2(x_med, dist_med))  
                pitch = self.controller_speed_x.update(self.navdata.vx, 0.0)
                yaw = -K_ANG_Z*displacement_x   
                roll = self.controller_speed_y.update(self.navdata.vy, 0.0)
                gaz = self.controller_pos_z.update(y_med, self.navdata.vz)
            
            utils.led_animation(1)    
        
        else: #count is 0
            if self.drone_info.z < ALT or self.drone_info.belt_located:
                #should not happen unless it's the beginning
                self.setup_pid(None, None, ALT + 0.1, 0.0, None)
                roll = self.controller_speed_y.update(self.navdata.vy,0.0)
                pitch = self.controller_speed_x.update(self.navdata.vx, 0.0)
                gaz = self.controller_pos_z.update(self.drone_info.z, self.navdata.vz)
 
            utils.led_animation(5)
            pass
                    
        return utils.adjust_command(roll, pitch, yaw, gaz)
