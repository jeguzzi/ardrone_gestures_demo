#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_gestures_demo')
import rospy
import sys
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Int16
#from ardrone_swarm.msg import Position, Drone_info
from gesture_messages.msg import Position, Drone_info

import utils
from drone_info import DroneStatus, DroneCommands, DroneTask, Gestures
from controllers_misc import  avoid_wall, avoid_drones, track_room_marker, track_generic_marker
from controller import DroneController
from tag import TagIds

COMMAND_PERIOD = 50 #ms
TASK_PERIOD = 100
INFO_PERIOD = 100 

class ControllerMain(object):
    def __init__(self, id):
        
        self.low = DroneController(id)       
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist)
        self.pubNobodyFound = rospy.Publisher('/ardrone/nobody_found', Empty)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0), self.controller)
        self.taskTimer = rospy.Timer(rospy.Duration(TASK_PERIOD/1000.0), self.task_controller)
        self.infoTimer = rospy.Timer(rospy.Duration(INFO_PERIOD/1000.0), self.emit_info)


    def controller(self,event):

        if self.low.navdata.status == DroneStatus.Flying or self.low.navdata.status == DroneStatus.GotoHover or self.low.navdata.status == DroneStatus.Hovering:
            #if flying, mantain hovering if not specified anything else
            self.command.linear.x, self.command.linear.y, self.command.linear.z, self.command.angular.z = [0,0,0,0]
            
            if self.low.drone_info.command == DroneCommands.NoCommand:
                pass
            #keyboard controls
            elif self.low.drone_info.command == DroneCommands.YawLeft:
                self.command.angular.z = 0.7
            elif self.low.drone_info.command == DroneCommands.YawRight:
                self.command.angular.z = -0.7      
            elif self.low.drone_info.command == DroneCommands.PitchForward:
                self.command.linear.x = 1
            elif self.low.drone_info.command == DroneCommands.PitchBackward:
                self.command.linear.x = -1
            elif self.low.drone_info.command == DroneCommands.RollLeft:
                self.command.linear.y = 1   
            elif self.low.drone_info.command == DroneCommands.RollRight:
                self.command.linear.y = -1
            elif self.low.drone_info.command == DroneCommands.IncreaseAlt:
                self.command.linear.z = 1
            elif self.low.drone_info.command == DroneCommands.DecreaseAlt:
                self.command.linear.z = -1
            #########################################################################################
            #tag detection
            elif self.low.drone_info.command == DroneCommands.StartFollow:
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.follow_tag(self.low.drone_info.angle_to_mantain, self.low.drone_info.tag_to_follow)
            #########################################################################################
            #face following
            elif self.low.drone_info.command == DroneCommands.FollowFace:
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.follow_face()
            #follow belt
            elif self.low.drone_info.command == DroneCommands.FollowBelt:
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.follow_belt() #there is only one belt                            
            #########################################################################################
            #position control
            elif self.low.drone_info.command == DroneCommands.GoAtPoint:  
                    if not self.low.drone_info.move_accomplished: 
                        self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.position_control(0.0)
                    else: 
                        rospy.loginfo("Move accomplished!")
                        if self.low.after_move is not None:
                            if self.low.after_move == 1:
                                self.low.send_land()
                            
                            elif self.low.after_move == 2:    
                                self.low.go_at_point(self.low.drone_info.x, self.low.drone_info.y, 
                                                 self.low.drone_info.z, self.low.drone_info.alpha + 135)
                            
                            elif self.low.after_move == 3:
                                self.low.go_at_point(self.low.drone_info.x, self.low.drone_info.y, 
                                                 self.low.drone_info.z, self.low.drone_info.alpha - 135)
                            
                            elif self.low.after_move == 4:
                                self.low.go_at_point(self.low.drone_info.x + 2.5*math.cos(utils.deg_to_rad(self.low.drone_info.alpha - 90)), 
                                         self.low.drone_info.y + 2.5*math.sin(utils.deg_to_rad(self.low.drone_info.alpha - 90)), 
                                         self.low.drone_info.z, self.low.drone_info.alpha - 90)
                            
                            self.low.after_move = None               
            #########################################################################################
            #orbit
            elif self.low.drone_info.command == DroneCommands.Orbit:
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.orbit(TagIds.MasterTag)                      
            #########################################################################################
            #orbit sync
            elif self.low.drone_info.command == DroneCommands.OrbitSync: 
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.orbit(TagIds.MasterTag)
            #########################################################################################
            #explore by continously spinning on current place
            elif self.low.drone_info.command == DroneCommands.Rotate:
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.rot_exploration()
            #########################################################################################
            #this behaviour supposes the perfect alignment rescurer-box
            elif self.low.drone_info.command == DroneCommands.FollowMe:
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = self.low.follow_me()
            #finally the command is published with eventual modifications
            
            if self.low.drone_info.avoid_walls:
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = avoid_wall(self.command, self.low.drone_info, self.low.navdata)
            
            #it may overwrite some commands of avoid_walls, but tbelt's to be safer
            if self.low.drone_info.avoid_drones: #only with 2 drones (for now)
                self.command.linear.y, self.command.linear.x, self.command.angular.z, self.command.linear.z = avoid_drones(self.command, self.low.drone_info, self.low.drone_2_info, self.low.navdata) 
            
            '''if not self.low.drone_info.task == DroneTask.FlyBAF or self.low.drone_info.task == DroneTask.FlyH:    
                if self.low.drone_info.track_room_marker:
                    if self.command.angular.z == 0 and self.low.drone_info.room_located: #None is considered 0.0 after being sent! 
                        #self.command.angular.z = - K_ANG_Z*self.low.drone_info.marker_displacement
                        self.command.angular.z = track_room_marker(self.low.drone_info)
                        #this can be done in the task control function
                        if self.command.linear.z == 0:
                            self.low.controller_pos_z.setpoint = 1.2
                            self.command.linear.z = self.low.controller_pos_z.update(self.low.drone_info.z, self.low.navdata.vz)
            
            else:
                self.command.angular.z = track_generic_marker(self.low.drone_info, self.low.drone_info.flyght_tag)'''
            
            if self.low.drone_info.time_go_up is not None and (rospy.Time.now() - self.low.drone_info.time_go_up).to_sec() < 0.25: #for gesture demo
                self.command.linear.z = 1 #overwrite
            
            elif (self.low.drone_info.time_go_up is not None and self.low.drone_info.task == DroneTask.DemoGesture and 
                 (rospy.Time.now() - self.low.drone_info.time_go_up).to_sec() > 0.3):
                self.low.drone_info.time_go_up = None
                self.low.drone_info.time_go_down = rospy.Time.now() #move up and down after a word   
            
            if self.low.drone_info.time_go_down is not None and (rospy.Time.now() - self.low.drone_info.time_go_down).to_sec() < 0.2: #for gesture demo
                self.command.linear.z = -1 #overwrite
            
            '''if self.low.drone_info.task == DroneTask.DemoGesture:
                if self.low.drone_info.time_go_back is not None and (rospy.Time.now() - self.low.drone_info.time_go_back).to_sec() < 7: #for gesture demo
                    self.low.controller_pos_x.setpoint = 2.2 #overwrite
                    
                    if (rospy.Time.now() - self.low.drone_info.time_go_back).to_sec() < 1:
                        self.command.linear.x = -0.5
                else:
                    self.low.controller_pos_x.setpoint = 1.7'''
                
                    
            self.pubCommand.publish(self.command)  
    
    def task_controller(self, event):
            
        if not (self.low.navdata.status == DroneStatus.Landed or self.low.navdata.status == DroneStatus.Emergency):
            if self.low.drone_info.task == DroneTask.DemoGesture:
                if self.low.drone_info.change_demo:
                    print "change demo was true\n."
                    self.low.drone_info.change_demo = False
                    print "Sending rot exploration\n."
                    self.low.send_rot_exploration()
                    self.low.drone_info.last_yaw = self.low.drone_info.alpha
                    '''if self.low.drone_info.gesture == Gestures.Left:
                        self.low.go_at_point(self.low.drone_info.x + math.cos(utils.deg_to_rad(self.low.drone_info.alpha - 120)), 
                                         self.low.drone_info.y + math.sin(utils.deg_to_rad(self.low.drone_info.alpha - 120)), 
                                         self.low.drone_info.z, self.low.drone_info.alpha - 120)
                    
                    elif self.low.drone_info.gesture == Gestures.Right:
                        self.low.go_at_point(self.low.drone_info.x + math.cos(utils.deg_to_rad(self.low.drone_info.alpha + 120)), 
                                         self.low.drone_info.y + math.sin(utils.deg_to_rad(self.low.drone_info.alpha + 120)), 
                                         self.low.drone_info.z, self.low.drone_info.alpha + 120)'''
                
                elif ((self.low.drone_info.command == DroneCommands.Rotate or self.low.drone_info.command == DroneCommands.GoAtPoint) and 
                        self.low.drone_info.tag_to_follow is not None):
                        print "Sending follow tag"
                        print self.low.drone_info.tag_to_follow
                        self.low.send_follow_tag()
                
                elif (self.low.drone_info.command == DroneCommands.Rotate and self.low.drone_info.last_yaw is not None and
                      abs(utils.normalize_angle(self.low.drone_info.alpha - self.low.drone_info.last_yaw)) > 135):
                    print "Exceeded rotation allowed. Going back."
                    self.pubNobodyFound.publish(Empty())
                    self.low.drone_info.last_yaw = None
                    utils.led_animation(5)
                    self.low.drone_info.tag_to_follow = self.low.drone_info.last_tag_to_follow
                    self.low.drone_info.last_tag_to_follow = None
                    self.low.pubSwitchMarker.publish(Int16(self.low.drone_info.tag_to_follow))
                    if self.low.drone_info.search_yaw > 0:
                        self.low.drone_info.last_seen = -1
                    else: self.low.drone_info.last_seen = 1    
                    self.low.send_follow_tag()
                                
                                                                   
    def emit_info(self, event):
        self.low.pubDroneInfo.publish(Drone_info(self.low.drone_info.status, self.low.drone_info.task, self.low.drone_info.command, self.low.drone_info.alpha, self.low.drone_info.x, self.low.drone_info.y, self.low.drone_info.z,
                                             self.low.drone_info.alpha_box, self.low.drone_info.box_in_sight, self.low.drone_info.alpha_belt,
                                             self.low.drone_info.belt_in_sight))
    
