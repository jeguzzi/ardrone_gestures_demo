#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_gesture_demo')
import rospy
from PySide import QtCore, QtGui

from controller_main import ControllerMain
from drone_video_display2 import DroneVideoDisplay
from drone_info import DroneCommands

class KeyMapping(object):
    PitchForward     = QtCore.Qt.Key.Key_E
    PitchBackward    = QtCore.Qt.Key.Key_D
    RollLeft         = QtCore.Qt.Key.Key_S
    RollRight        = QtCore.Qt.Key.Key_F
    YawLeft          = QtCore.Qt.Key.Key_W
    YawRight         = QtCore.Qt.Key.Key_R
    IncreaseAltitude = QtCore.Qt.Key.Key_Q
    DecreaseAltitude = QtCore.Qt.Key.Key_A
    Takeoff          = QtCore.Qt.Key.Key_Y
    Land             = QtCore.Qt.Key.Key_H
    Emergency        = QtCore.Qt.Key.Key_C
    FollowTag        = QtCore.Qt.Key.Key_T
    DebugTag         = QtCore.Qt.Key.Key_B
    FlatTrim         = QtCore.Qt.Key.Key_M

class KeyboardController(DroneVideoDisplay):
    def __init__(self, id):
        #define the controller
        self.controller = ControllerMain(id)
        super(KeyboardController,self).__init__()

    def keyPressEvent(self, event):
        key = event.key()

        if self.controller is not None and not event.isAutoRepeat():
            if key == KeyMapping.Emergency:
                self.controller.low.send_emergency()
            elif key == KeyMapping.Takeoff:
                self.controller.low.send_takeoff()
            elif key == KeyMapping.Land:
                self.controller.low.send_land()     
            elif key == KeyMapping.FlatTrim:
                self.controller.low.send_flat_trim()
            elif key == KeyMapping.FollowTag:
                self.controller.low.send_follow_tag()       
                             	
            else:
                if key == KeyMapping.YawLeft:
                    self.controller.low.receive_input(DroneCommands.YawLeft)
                elif key == KeyMapping.YawRight:
                    self.controller.low.receive_input(DroneCommands.YawRight)
                elif key == KeyMapping.PitchForward:
                    self.controller.low.receive_input(DroneCommands.PitchForward)
                elif key == KeyMapping.PitchBackward:
                    self.controller.low.receive_input(DroneCommands.PitchBackward)
                elif key == KeyMapping.RollLeft:
                    self.controller.low.receive_input(DroneCommands.RollLeft)
                elif key == KeyMapping.RollRight:
                    self.controller.low.receive_input(DroneCommands.RollRight)
                elif key == KeyMapping.IncreaseAltitude:
                    self.controller.low.receive_input(DroneCommands.IncreaseAlt)
                elif key == KeyMapping.DecreaseAltitude:
                    self.controller.low.receive_input(DroneCommands.DecreaseAlt)
                #tag detection (while flying)                 						

    def keyReleaseEvent(self,event):
        key = event.key()
        
        if self.controller is not None and not event.isAutoRepeat():
           
            if key == KeyMapping.YawLeft or key == KeyMapping.YawRight or key == KeyMapping.PitchForward or key == KeyMapping.PitchBackward or key == KeyMapping.RollLeft or key == KeyMapping.RollRight or key == KeyMapping.IncreaseAltitude or key == KeyMapping.DecreaseAltitude:
                self.controller.low.receive_input(DroneCommands.NoCommand)


# Setup the application
if __name__=='__main__':
    import sys
    rospy.init_node('ardrone_keyboard_controller')

    app = QtGui.QApplication(sys.argv)
    args = rospy.myargv(sys.argv)
    id = int(args[1])
    #start all parts of the application
    display = KeyboardController(id)
    
    display.show()

    status = app.exec_()

    rospy.signal_shutdown('Shutdown')
    sys.exit(status)

