import math
import utils
import rospy

class TagIds:
    MasterTag     = 8
    FollowTag     = 7
    FollowTag2    = 16
    FollowTag3    = 0
    FollowTag4    = 1
    FollowTag5    = 2
    FollowTag6    = 3 
    FollowTag7    = 4
    BeltFakeTag   = 15
    BeltFrontTag  = 77 #7
    BeltLeftTag   = 66 #6
    BeltRightTag  = 166 #16
    BeltBackTag   = 17
    FrontTag      = 9
    RightTag      = 10
    LeftTag       = 12
    BackTag       = 11
    RoomFrontTag  = 90
    RoomLeftTag   = 100
    RoomRightTag  = 111
    RoomBackTag   = 122
    AllFollowTags = [FollowTag, FollowTag2, FollowTag3, FollowTag4, FollowTag5, FollowTag6, FollowTag7] 

class RoomTagCoor:
    DIM_X = 10
    DIM_Y = 5
    LeftX  = 0
    LeftY  = 3.5
    LeftZ  = 1
    FrontX = 3
    FrontY = 5
    FrontZ = 1

class VisionData(object):
    TAG_ON_LEFT  = 0
    TAG_ON_RIGHT = 1
    PITCH_POS    = 2
    PITCH_NEG    = 3    
    
class Tag:
    def __init__(self,x,y,d):
        self.x = x
        self.y = y
        self.d = d
    
class TagAR:
    def __init__(self,id,x,y,z,roll,pitch,yaw):
        #position
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        #orientation as a quaternion
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
def get_angles_from_vision(displacement_x,pitch_med,tag_position,pitch_sign):
    if tag_position == VisionData.TAG_ON_LEFT  and pitch_sign == VisionData.PITCH_NEG:
        beta = -utils.deg_to_rad(displacement_x)
        omega = -pitch_med
        alpha = math.pi/2 - (math.pi/2 + beta - omega)
        gamma = math.pi/2 - alpha - beta
    elif tag_position == VisionData.TAG_ON_RIGHT and pitch_sign == VisionData.PITCH_POS:
        beta = utils.deg_to_rad(displacement_x)
        omega = pitch_med
        alpha = math.pi/2 - (math.pi/2 + beta - omega)
        gamma = math.pi/2 - alpha - beta
    elif tag_position == VisionData.TAG_ON_LEFT and pitch_sign == VisionData.PITCH_POS:
        beta = -utils.deg_to_rad(displacement_x)
        omega = pitch_med
        alpha = math.pi/2 - (math.pi/2 - beta - omega)
        gamma = math.pi/2 - alpha + beta
    else: #tag_on_right and pitch neg
        beta = utils.deg_to_rad(displacement_x)
        omega = -pitch_med
        alpha = math.pi/2 - (math.pi/2 - beta - omega)
        gamma = math.pi/2 - alpha + beta 
    return [beta,omega,alpha,gamma]        

def get_average_tag_info(tags_buffer, tag_id):
    x_med = 0.0
    y_med = 0.0
    dist_med = 0.0
    pitch_med = 0.0
    count = 0
    if tag_id is None:
        return (x_med, y_med, dist_med, pitch_med, count)
    for tag in tags_buffer.get():
        if tag is not None and tag.id == tag_id:
            x_med += tag.x
            y_med += tag.y
            dist_med += tag.z
            pitch_med += tag.pitch
            count += 1
    
    if count is not 0:
        x_med = x_med/count
        y_med = y_med/count
        dist_med = dist_med/count
        pitch_med = pitch_med/count
        
    return (x_med, y_med, dist_med, pitch_med, count)
#0 = front 1 = left 2 = right 3 = back
def get_alpha_by_vision_info(tags_buffer,rot_reference):
    x_med = [0 for i in range(4)]
    dist_med = [0 for i in range(4)]
    pitch_med = [0 for i in range(4)]
    count = [0 for i in range(4)]
    
    for tag in tags_buffer.get():
        if tag is not None:
              
            if tag.id == TagIds.RoomFrontTag:
                x_med[0] += tag.x
                dist_med[0] += tag.z
                pitch_med[0] += tag.pitch
                count[0] += 1
                
            elif tag.id == TagIds.RoomLeftTag:
                x_med[1] += tag.x
                dist_med[1] += tag.z
                pitch_med[1] += tag.pitch
                count[1] += 1
                
            elif tag.id == TagIds.RoomRightTag:
                x_med[2] += tag.x
                dist_med[2] += tag.z
                pitch_med[2] += tag.pitch
                count[2] += 1
                    
            elif tag.id == TagIds.RoomBackTag:
                x_med[3] += tag.x
                dist_med[3] += tag.z
                pitch_med[3] += tag.pitch
                count[3] += 1
    
    #take the most accurate
    index = count.index(max(count))
    #take the most close (should be more accurate)
    #index = dist_med.index(min(dist_med))
    pitch = utils.rad_to_deg(pitch_med[index]/count[index])
    
    if index == 3:
        alpha = pitch + rot_reference + 270
        ref_tag = TagIds.RoomBackTag
    elif index == 0:
        alpha = pitch + rot_reference + 90
        ref_tag = TagIds.RoomFrontTag
    elif index == 2:
        alpha = pitch + rot_reference 
        ref_tag = TagIds.RoomRightTag
    elif index == 1:
        alpha = pitch + rot_reference + 180
        ref_tag = TagIds.RoomLeftTag            
    
    while alpha > 180: alpha -= 360
    while alpha < -180: alpha += 360
    
    return [alpha, ref_tag]

def get_ref_coor(ref_tag):
    if ref_tag == TagIds.RoomLeftTag:
        return [RoomTagCoor.LeftX, RoomTagCoor.LeftY, RoomTagCoor.LeftZ]
    elif ref_tag == TagIds.RoomFrontTag:
        return [RoomTagCoor.FrontX, RoomTagCoor.FrontY, RoomTagCoor.FrontZ]
    else: return [0,0,0]

def get_av_pitch(pitch_med, count, i):
    pitch = pitch_med[i]/count[i]
    return pitch
    
def get_box_ang_pos(tags_buffer, rot_reference):
    x_med = [0 for i in range(4)]
    dist_med = [0 for i in range(4)]
    pitch_med = [0 for i in range(4)]
    count = [0 for i in range(4)]
    
    alpha = None
    for tag in tags_buffer.get():
        if tag is not None:
            if tag.id == TagIds.FrontTag:
                x_med[0] += tag.x
                dist_med[0] += tag.z
                pitch_med[0] += tag.pitch
                count[0] += 1
                
            elif tag.id == TagIds.LeftTag:
                x_med[1] += tag.x
                dist_med[1] += tag.z
                pitch_med[1] += tag.pitch
                count[1] += 1
                
            elif tag.id == TagIds.RightTag:
                x_med[2] += tag.x
                dist_med[2] += tag.z
                pitch_med[2] += tag.pitch
                count[2] += 1
                    
            elif tag.id == TagIds.BackTag:
                x_med[3] += tag.x
                dist_med[3] += tag.z
                pitch_med[3] += tag.pitch
                count[3] += 1
        
    if (count[3] is not 0 and count[2] is 0) or (count[0] is not 0 and count[1] is 0): # i'm back or in front
                
        if count[3] is not 0:
            alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count,3)) + rot_reference + 90    
        else:
            alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count,0)) + rot_reference + 270
            
    elif (count[2] is not 0 and count[0] is 0) or (count[1] is not 0 and count[3] is 0): #i'm on the right or on the left
                
        if count[2] is not 0:
            alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count, 2)) + rot_reference 
        else:
            alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count, 1)) + rot_reference + 180
    
    if alpha is not None:
        while alpha > 180: alpha -= 360
        while alpha < -180: alpha += 360
    
    return alpha

def get_belt_ang_pos(tags_buffer, rot_reference):
    x_med = [0 for i in range(4)]
    dist_med = [0 for i in range(4)]
    pitch_med = [0 for i in range(4)]
    count = [0 for i in range(4)]
    
    alpha = None
    for tag in tags_buffer.get():
        if tag is not None:
            if tag.id == TagIds.BeltFrontTag:
                x_med[0] += tag.x
                dist_med[0] += tag.z
                pitch_med[0] += tag.pitch
                count[0] += 1
                
            elif tag.id == TagIds.BeltLeftTag:
                x_med[1] += tag.x
                dist_med[1] += tag.z
                pitch_med[1] += tag.pitch
                count[1] += 1
                
            elif tag.id == TagIds.BeltRightTag:
                x_med[2] += tag.x
                dist_med[2] += tag.z
                pitch_med[2] += tag.pitch
                count[2] += 1
                    
            elif tag.id == TagIds.BeltBackTag:
                x_med[3] += tag.x
                dist_med[3] += tag.z
                pitch_med[3] += tag.pitch
                count[3] += 1
    
    if count[0] is not 0:
        alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count, 0)) + rot_reference
    
    elif count[1] is not 0:
        alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count, 1)) + rot_reference + 90
    
    elif count[2] is not 0:
         alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count, 2)) + rot_reference + 270    
    
    elif count[3] is not 0:
         alpha = utils.rad_to_deg(get_av_pitch(pitch_med,count, 3)) + rot_reference + 180
    
    if alpha is not None:
        while alpha > 180: alpha -= 360
        while alpha < -180: alpha += 360
    
    return alpha
               