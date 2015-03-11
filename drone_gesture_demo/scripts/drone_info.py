#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_gestures_demo')
import rospy
import sys
import math

from tag import TagIds

#this class is used to maintain informations relative to other drones
class DroneInfo(object):
    def __init__(self, id = 0):
        self.id = id
        self.status = -1
        self.task = DroneTask.Waiting
        self.command = DroneCommands.NoCommand
        self.alpha = 0
        self.x = 0
        self.y = 0
        self.z = 0
        
        self.alpha_box = 0.0
        self.box_in_sight = False
        
        self.alpha_belt = 0.0
        self.belt_in_sight = False
        
        ####these are not to send to other drones###
        self.tag_to_follow = None
        self.last_tag_to_follow = None
        self.gesture = Gestures.NoGesture
        self.face_dir = Faces.NoFace
        self.role = 0 #0=follower, 1 = explorer
        self.time_anim = None
        
        self.flyght_tag = TagIds.RoomFrontTag
        
        self.time_keep_pos = None
        
        self.time_go_back = None
        
        self.room_located = False
        
        self.box_master_tag = TagIds.MasterTag
        self.x_box = None
        self.y_box = None
        self.box_located = False
        
        '''self.x_belt = 0.0
        self.y_belt = 0.0'''
        self.belt_located = False
        
        self.alpha_start_expl = None
        self.last_yaw = None
        self.last_z = None
        
        self.move_accomplished = False
        self.time_gesture = rospy.Time.now()
        self.last_seen = 0
        self.angle_to_mantain = 0.0
        self.marker_displacement = None
        self.room_marker_tracked = -1
        self.avoid_walls = False
        self.avoid_drones = False
        self.track_room_marker = False
        self.search_marker = False
        
        self.search_yaw = None
        self.change_demo = False
        
        self.time_go_up = None
        self.time_go_down = None 
    
    def set_drone_info(self, status, task, command, alpha, x, y, z, alpha_box, box_in_sight, alpha_belt, belt_in_sight):
        self.status = status
        self.task = task
        self.command = command
        self.alpha = alpha
        self.x = x
        self.y = y
        self.z = z
        self.alpha_box = alpha_box
        self.box_in_sight = box_in_sight
        self.alpha_belt = alpha_belt
        self.belt_in_sight = belt_in_sight   
    
    def init(self):
        self.command = DroneCommands.NoCommand
        self.time_keep_pos = None
        self.time_anim = None
        self.belt_located = False
        self.move_accomplished = False
        self.last_seen = 0.0
        self.search_yaw = None
        self.change_demo = False
        self.time_go_up = None
        self.time_go_down = None
        self.last_yaw = None
        self.tag_to_follow = None
        self.last_tag_to_follow = None 
        if not self.task == DroneTask.DemoGesture:
            self.task = DroneTask.Waiting
            #self.tag_to_follow = TagIds.FollowTag  

class DroneStatus(object):
    Emergency = 0
    Inited    = 1
    Landed    = 2
    Flying    = 3
    Hovering  = 4
    Test      = 5
    TakingOff = 6
    GotoHover = 7
    Landing   = 8
    Looping   = 9 
        
class DroneCommands(object):
    NoCommand     = -1
    YawLeft       = 0
    YawRight      = 1
    PitchForward  = 2
    PitchBackward = 3
    RollLeft      = 4
    RollRight     = 5
    IncreaseAlt   = 6
    DecreaseAlt   = 7
    StartFollow   = 8
    StartTag      = 9
    GoAtSpeed     = 10
    Rotate        = 11
    GoAtPoint     = 12
    Orbit         = 13
    OrbitSync     = 14
    FollowBelt    = 15
    FollowMe      = 16
    FollowFace    = 17 

#this class is used to communicate the current task between drones.
class DroneTask(object):
    Waiting = 0
    DemoGesture = 7
    
class Gestures(object):
    NoGesture = 0
    You = 1
    Follow = 2
    Takeoff = 6
    Land = 3
    Left = 4
    Right = 5

class Faces(object):
    NoFace = 0
    Center = 1
    Right = 2
    Left = 3               
