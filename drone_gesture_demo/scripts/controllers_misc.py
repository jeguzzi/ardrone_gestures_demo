#!/usr/bin/env python
import pid
import rospy
import utils
import math
from tag import RoomTagCoor, TagIds
from drone_info import DroneCommands, DroneStatus

TOL_WALL = 1.5
TOL_DRONE = 1.8
K_ANG_Z = 0.05
K_ANG_Z_EXPL = 0.015
Y_SP = 0.65
#this class is a wrapper for a PID controller
class PidWrapper:
    def __init__( self, gain_p = 0.0, gain_i = 0.0, gain_d = 0.0, setpoint = 0.0, 
                  time_constant = 0.0, limit = -1.0 ):
        self.pid = pid.Pid2(gain_p, gain_i, gain_d)
        self.prev_time = None
        self.setpoint = setpoint
        self.prev_setpoint = None
        
        
    def update(self,curr,diff):
        output = 0
        
        if self.prev_time == None:
            self.prev_time = rospy.Time.now()
            delta_t = 0.0
        else:
            now = rospy.Time.now()
            delta_t = (now - self.prev_time).to_sec()
            self.prev_time = now
            
        output = self.pid.update(self.setpoint, curr, diff, delta_t)

        return output

    def reset( self ):
        self.prev_time = None
        self.pid.reset()
         
class CurrNavdata(object):
    def __init__(self):
        self.status = -1
        self.altd = 0.0
        self.rotZ = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        
    def set_navdata(self, status, altd, rotZ, vx, vy, vz, ax, ay, az):
        self.status = status
        self.altd = altd
        self.rotZ = rotZ
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.ax = ax
        self.ay = ay
        self.az = az    


def rotate(target_yaw, alpha, k_speed):
    dr = target_yaw - alpha
    while dr > 180: dr -= 360
    while dr < -180: dr += 360
    yaw = 0
        
    if(abs(dr)<5):  # deadband
        pass
    else:
        yaw = k_speed*dr
        
    if yaw < -1: yaw = -1
    elif yaw > 1: yaw = 1
        
    return [yaw,dr]

def get_roll_follow_belt(drone_info, other_drones, controller_speed_y, controller_pos_y, navdata):
    distances = []
    dist_front = []
    dist_front_nat = []
    drone_2_info = None
    drone_3_info = None
    drone_4_info = None
    
    for drone in other_drones:
        if drone_2_info is None: drone_2_info = drone
        elif drone_3_info is None: drone_3_info = drone
        elif drone_4_info is None: drone_4_info = drone
            
    if drone_2_info is not None:             
        dist = drone_2_info.alpha_belt - drone_info.alpha_belt
        dist = utils.normalize_angle(dist)
        distances.append(dist)
        dist_front.append(abs(drone_2_info.alpha_belt))
        dist_front_nat.append(drone_2_info.alpha_belt)
                
    if drone_3_info is not None:              
        dist = drone_3_info.alpha_belt - drone_info.alpha_belt
        dist = utils.normalize_angle(dist)
        distances.append(dist)
        dist_front.append(abs(drone_3_info.alpha_belt))
        dist_front_nat.append(drone_3_info.alpha_belt)
        
    if drone_4_info is not None:            
        dist = drone_4_info.alpha_belt - drone_info.alpha_belt
        dist = utils.normalize_angle(dist)
        distances.append(dist)
        dist_front.append(abs(drone_4_info.alpha_belt))
        dist_front_nat.append(drone_4_info.alpha_belt)        
                
    if distances == [] or abs(drone_info.alpha_belt) < min(dist_front): # I am alone or the "master"
        drone_info.role = 0
        if abs(drone_info.alpha_belt) < 35:
            roll = controller_pos_y.update(dist_med*math.tan(utils.deg_to_rad(drone_info.alpha_belt)), -navdata.vy)
        else: 
            if drone_info.alpha_belt > 0: controller_speed_y.setpoint = Y_SP
            else: controller_speed_y.setpoint = -Y_SP
            roll = controller_speed_y.update(navdata.vy, 0.0)    
    else:
        drone_info.role = 1        
        if len(distances) == 1:
            if drone_info.alpha_belt < -145 or drone_info.alpha_belt > 145:
                if drone_info.alpha_belt > 0:
                    angle_ref = drone_info.alpha_belt - 180
                else: angle_ref = drone_info.alpha_belt + 180
                roll = -controller_pos_y.update(dist_med*math.tan(utils.deg_to_rad(angle_ref)), -navdata.vy)
            else: 
                if drone_info.alpha_belt > 0: controller_speed_y.setpoint = -Y_SP
                else: controller_speed_y.setpoint = +Y_SP
                roll = controller_speed_y.update(navdata.vy, 0.0)    
        
        elif len(distances) > 1:
            front = dist_front.index(min(dist_front)) + 2 #this is the drone I don't have to consider
            dist_front_nat.pop(front - 2)
            go_90 = False
            go_minus_90 = False
                
            #choose pal angle    
            if len(distances) == 2:
                if front == 2:
                    pal_angle = drone_3_info.alpha_belt
                else: #should be 3    
                    pal_angle = drone_2_info.alpha_belt
                    
                if drone_info.alpha_belt > 0 and pal_angle > 0:
                    if drone_info.alpha_belt > pal_angle: #I have to go to the other side
                        go_minus_90 = True
                    else:
                        go_90 = True
                               
                elif drone_info.alpha_belt < 0 and pal_angle < 0:
                    if drone_info.alpha_belt < pal_angle:
                        go_90 = True
                    else:
                        go_minus_90 = True
                
                else: 
                    if drone_info.alpha_belt > 0:
                        go_90 = True
                    else: go_minus_90 = True
                
            elif len(distances) == 3:
                all_same_side = False
                go_behind = False
                dist_pos = filter(lambda x: x > 0, dist_front_nat)
                dist_neg = filter(lambda x: x < 0, dist_front_nat)                
                
                if ((dist_neg == [] and drone_info.alpha_belt > 0) or
                    (dist_pos == [] and drone_info.alpha_belt < 0)):
                    all_same_side = True
                   
                if ((all_same_side and drone_info.alpha_belt > 0 and drone_info.alpha_belt > min(dist_pos) 
                    and drone_info.alpha_belt < max(dist_pos)) or
                    (all_same_side and drone_info.alpha_belt < 0 and
                    drone_info.alpha_belt < max(dist_neg) and drone_info.alpha_belt > min(dist_neg))): 
                    go_behind = True
                    
                else: #not all on the same side or on the same side but I don't have to go behind
                    
                    if all_same_side:
                        if drone_info.alpha_belt > 0:
                            if drone_info.alpha_belt < min(dist_pos):
                                go_90 = True
                            elif drone_info.alpha_belt > max(dist_pos):
                                go_minus_90 = True
                        
                        else:
                             if drone_info.alpha_belt > max(dist_neg):
                                go_minus_90 = True
                             elif drone_info.alpha_belt < min(dist_neg):
                                go_90 = True           
                    
                    else:    
                        if drone_info.alpha_belt > 0:
                            if not dist_pos == []:
                                pal_pos = dist_pos[0]
                                if drone_info.alpha_belt < pal_pos:
                                    go_90 = True
                                else: go_behind = True
                            
                            else: go_90 = True    
                            
                        else:
                            if not dist_neg == []: 
                                pal_neg = dist_neg[0]
                                if drone_info.alpha_belt > pal_neg:
                                    go_minus_90 = True
                                else: go_behind = True
                            
                            else: go_minus_90 = True    
                        
                if go_behind:
                    if drone_info.alpha_belt < -145 or drone_info.alpha_belt > 145:
                        if drone_info.alpha_belt > 0:
                            angle_ref = drone_info.alpha_belt - 180
                        else: angle_ref = drone_info.alpha_belt + 180
                        roll = -controller_pos_y.update(dist_med*math.tan(utils.deg_to_rad(angle_ref)), -navdata.vy)
                    else: 
                        if drone_info.alpha_belt > 0: controller_speed_y.setpoint = -Y_SP
                        else: controller_speed_y.setpoint = +Y_SP
                        roll = controller_speed_y.update(navdata.vy, 0.0)        

            if go_90:
                if drone_info.alpha_belt > 0:
                    if drone_info.alpha_belt > 55 or drone_info.alpha_belt < 125:
                        angle_ref = drone_info.alpha_belt - 90
                        roll = -controller_pos_y.update(dist_med*math.tan(utils.deg_to_rad(angle_ref)), -navdata.vy)
                    else:
                        if drone_info.alpha_belt < 90: controller_speed_y.setpoint = -Y_SP
                        else: controller_speed_y.setpoint = +Y_SP
                        roll = controller_speed_y.update(navdata.vy, 0.0)
                        
                else: 
                    controller_speed_y.setpoint = +Y_SP
                    roll = controller_speed_y.update(navdata.vy, 0.0)        
                
            elif go_minus_90:
                if drone_info.alpha_belt < 0:
                    if drone_info.alpha_belt < -55 or drone_info.alpha_belt > -125:
                        angle_ref = drone_info.alpha_belt + 90
                        roll = -controller_pos_y.update(dist_med*math.tan(utils.deg_to_rad(angle_ref)), -navdata.vy)
                    else:
                        if drone_info.alpha_belt < -90: controller_speed_y.setpoint = -Y_SP
                        else: controller_speed_y.setpoint = +Y_SP
                        roll = controller_speed_y.update(navdata.vy, 0.0)
                
                else: 
                    controller_speed_y.setpoint = -Y_SP
                    roll = controller_speed_y.update(navdata.vy, 0.0)                                                                          
        
        '''else:
            distances_pos = filter(lambda x: x >= 0, distances)
            distances_neg = filter(lambda x: x < 0, distances)
                    
            if not distances_pos == []:
                dd = min(distances_pos)
            else: dd = 360 + min(distances_neg)
                    
            if not distances_neg == []:
                ds = -max(distances_neg)
            else: ds = 360 - max(distances_pos)
                    
            controller_speed_y.setpoint = -(dd - ds)/float(150)
 
            if controller_speed_y.setpoint > Y_SP: controller_speed_y.setpoint = Y_SP
            elif controller_speed_y.setpoint < -Y_SP: controller_speed_y.setpoint = -Y_SP           
            roll = controller_speed_y.update(navdata.vy, 0.0)'''
    
    return roll 
   
def avoid_wall(command, drone_info, navdata):
    roll, pitch, yaw, gaz = [command.linear.y, command.linear.x, command.angular.z, command.linear.z]
    wall_x = 0 #-1 left 1 right
    wall_y = 0 #-1 back 1 front
        
    forbid_pitch_front = False
    forbid_pitch_back = False
    forbid_roll_left = False
    forbid_roll_right = False
        
    if drone_info.x < TOL_WALL: wall_x = -1
    elif drone_info.x > RoomTagCoor.DIM_X - TOL_WALL: wall_x = 1
        
    if drone_info.y < TOL_WALL: wall_y = -1
    elif drone_info.y > RoomTagCoor.DIM_Y - TOL_WALL: wall_y = 1
        
    if wall_y == 1: #front
        if drone_info.alpha > 0 and drone_info.alpha < 180:
            forbid_pitch_front = True
        else: forbid_pitch_back = True
            
        if drone_info.alpha > -90 and drone_info.alpha < 90:
            forbid_roll_left = True
        else: forbid_roll_right  = True        
        
    elif wall_y == -1: #back
        if drone_info.alpha > 0 and drone_info.alpha < 180:
            forbid_pitch_back = True
        else: forbid_pitch_front = True
           
        if drone_info.alpha > -90 and drone_info.alpha < 90:
            forbid_roll_right = True
        else: forbid_roll_left = True
           
    if wall_x == -1: #left
        if drone_info.alpha > -90 and drone_info.alpha < 90:
            forbid_pitch_back = True
        else: forbid_pitch_front = True
           
        if drone_info.alpha > 0 and drone_info.alpha < 180:
            forbid_roll_left = True
        else: forbid_roll_right = True
       
    elif wall_x == 1: #right
        if drone_info.alpha > -90 and drone_info.alpha < 90:
            forbid_pitch_front = True
        else: forbid_pitch_back = True
           
        if drone_info.alpha > 0 and drone_info.alpha < 180:
            forbid_roll_right = True
        else: forbid_roll_left = True                                    
        
    if forbid_pitch_front:
        if navdata.vx > 0.2 and pitch > 0: pitch = -1  #if speed is too high, hovering is not sufficient to stop the drone! 
        else:
            if drone_info.y < TOL_WALL or drone_info.y > RoomTagCoor.DIM_Y - TOL_WALL: pitch = -0.15
            else: pitch = 0
            
    elif forbid_pitch_back:
        if navdata.vx < -0.2 and pitch < 0: pitch = 1
        else:
            if drone_info.y < TOL_WALL or drone_info.y > RoomTagCoor.DIM_Y - TOL_WALL: pitch = 0.15
            else: pitch = 0 
            
    if forbid_roll_left:
        if navdata.vy > 0.2 and roll > 0: roll = -1
        else:
            if drone_info.x < TOL_WALL or drone_info.x > RoomTagCoor.DIM_X - TOL_WALL: roll = -0.15
            else: roll = 0
        
    elif forbid_roll_right:
        if navdata.vy < -0.2 and roll < 0: roll = 1
        else:
            if drone_info.x < TOL_WALL or drone_info.x > RoomTagCoor.DIM_X - TOL_WALL: roll = 0.15
            else: roll = 0
    
    rospy.loginfo("forbid pitch front: %d" %forbid_pitch_front)
    rospy.loginfo("forbid pitch back: %d" %forbid_pitch_back)
    rospy.loginfo("forbid pitch left: %d" %forbid_roll_left)
    rospy.loginfo("forbid pitch right: %d" %forbid_roll_right)
        
    return [roll, pitch, yaw, gaz]
    
def avoid_drones(command, drone_info, drone_2_info, navdata):
    roll, pitch, yaw, gaz = [command.linear.y, command.linear.x, command.angular.z, command.linear.z]
    
    #the b & f flight gives the precedence
    if not drone_info.task == DroneTask.FlyH:
        drones = [drone_2_info]
        forbid_pitch_front = False
        forbid_pitch_back = False
        forbid_roll_left = False
        forbid_roll_right = False
        
        for drone in drones:
            distance = ((drone.x - drone_info.x)**2 + (drone.y - drone_info.y)**2)
            if distance < TOL_DRONE:
                angle_between_drones = utils.rad_to_deg(math.atan((drone.y - drone_info.y)/(drone.x - drone_info.x)))
                if ((drone.y > drone_info.y and drone.x < drone_info.x) or 
                    (drone.y < drone_info.y and drone.x < drone_info.x)):
                    angle_between_drones += 180
                while angle_between_drones > 180: angle_between_drones -= 360
                while angle_between_drones < -180: angle_between_drones += 360       
            
            
                control_angle = drone_info.alpha - angle_between_drones
                control_angle = utils.normalize_angle(control_angle) 
            
                if control_angle > -45 and control_angle < 45: forbid_pitch_front = True
                elif control_angle > 45 and control_angle < 135: forbid_roll_right = True
                elif control_angle > -135 and control_angle < -45: forbid_roll_left = True
                else: forbid_pitch_back = True 
        
        if forbid_pitch_front:
            if navdata.vx > 0.2 and pitch > 0: pitch = -1  #if speed is too high, hovering is not sufficient to stop the drone! 
            else: pitch = 0
            
        elif forbid_pitch_back:
            if navdata.vx < -0.2 and pitch < 0: pitch = 1
            else: pitch = 0
            
        if forbid_roll_left:
            if navdata.vy > 0.2 and roll > 0: roll = -1
            else: pitch = 0
        
        elif forbid_roll_right:
            if navdata.vy < -0.2 and roll < 0: roll = 1
            else: pitch = 0
    
        rospy.loginfo("forbid pitch front: %d" %forbid_pitch_front)
        rospy.loginfo("forbid pitch back: %d" %forbid_pitch_back)
        rospy.loginfo("forbid pitch left: %d" %forbid_roll_left)
        rospy.loginfo("forbid pitch right: %d" %forbid_roll_right)
    
        
    return [roll, pitch, yaw, gaz]

def track_room_marker(drone_info):
    yaw = 0
    
    if not drone_info.search_marker:
        if not drone_info.room_marker_tracked == -1:
           yaw = - K_ANG_Z*drone_info.marker_displacement
                
    #search the closest tag wrt the current position    
    else:    
        distances = []
        #append left
        d_left = math.sqrt((RoomTagCoor.LeftX - drone_info.x)**2 + (RoomTagCoor.LeftY - drone_info.y)**2)
        distances.append(d_left)
    
        d_front = math.sqrt((RoomTagCoor.FrontX - drone_info.x)**2 + (RoomTagCoor.FrontY - drone_info.y)**2)
        distances.append(d_front)
    
        #...others#
    
        min_dist = min(distances)
        min_index = distances.index(min_dist)
    
        #this is verbose but preserves the usage of arbitrary tag ids
    
        if min_index == 0:
            new_ref_tag = TagIds.RoomLeftTag
            y = RoomTagCoor.LeftY
            x = RoomTagCoor.LeftX
        elif min_index == 1:
            new_ref_tag = TagIds.RoomFrontTag
            y = RoomTagCoor.FrontY
            x = RoomTagCoor.FrontX
        #...others...#
    
        if drone_info.room_marker_tracked == TagIds.RoomLeftTag:
            cur_index = 0
        elif drone_info.room_marker_tracked == TagIds.RoomFrontTag:
            cur_index = 1    
        #...others...#
    
        #if it is the ref tag, it's ok the current displacement'
        if drone_info.room_marker_tracked == new_ref_tag: #the last condition should not be necessary
            yaw = - K_ANG_Z*drone_info.marker_displacement       
    
        #otherwise, rotate to track the closest room marker
        else:   
            target_yaw = utils.rad_to_deg(math.atan((y - drone_info.y)/(x - drone_info.x)))
            if ((y > drone_info.y and x < drone_info.x) or (y < drone_info.y and x < drone_info.x)):
                target_yaw += 180
            target_yaw = utils.normalize_angle(target_yaw)
        
            rospy.loginfo("target yaw: %f" %target_yaw) 
        
            yaw, dr = rotate(target_yaw, drone_info.alpha, K_ANG_Z_EXPL)
     
    return yaw           

def track_generic_marker(drone_info, marker):
    yaw = 0
    
    if drone_info.room_marker_tracked == marker:
        yaw = - K_ANG_Z*drone_info.marker_displacement
        
    return yaw    
        
