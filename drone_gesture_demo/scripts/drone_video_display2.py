#!/usr/bin/env python

import roslib; roslib.load_manifest('drone_gestures_demo')
import rospy

from threading import Lock
from PySide import QtCore, QtGui
from sensor_msgs.msg import Image       
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Int16
from ardrone_autonomy.msg import Navdata 
from ar_track_alvar_msgs_idsia.msg import AlvarMarkers
from gesture_messages.msg import Position
from tf.transformations import euler_from_quaternion

import utils
from drone_info import DroneStatus, DroneTask
from controller_main import ControllerMain
from tag import TagIds
# The GUI libraries



# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 70 #ms


class DroneVideoDisplay(QtGui.QMainWindow):
    status_messages = {
        DroneStatus.Emergency : 'Emergency',
        DroneStatus.Inited    : 'Initialized',
        DroneStatus.Landed    : 'Landed',
        DroneStatus.Flying    : 'Flying',
        DroneStatus.Hovering  : 'Hovering',
        DroneStatus.Test      : 'Test (?)',
        DroneStatus.TakingOff : 'Taking Off',
        DroneStatus.GotoHover : 'Going to Hover Mode',
        DroneStatus.Landing   : 'Landing',
        DroneStatus.Looping   : 'Looping (?)'
    }
    DisconnectedMessage = 'Disconnected'
    UnknownMessage = 'Unknown Status'
    
    def __init__(self):
        super(DroneVideoDisplay, self).__init__()
        
        #define the gui
        self.setWindowTitle('AR.Drone Control Center')
        self.resize(500,300)
        cWidget = QtGui.QWidget(self) #parent widget       
        
        grid = QtGui.QGridLayout(cWidget)        

        ############visualization and feeds################
        #image widget        
        '''self.imageBox = QtGui.QLabel(cWidget) 
        grid.addWidget(self.imageBox, 0, 0)'''
        
        #navdata widget
        '''self.navdataBox = QtGui.QTextEdit(cWidget)
        if (self.navdataBox.isReadOnly() == False):
            self.navdataBox.setReadOnly(True)
        grid.addWidget(self.navdataBox,0,1)
        
        self.navdataBox.setText('Waiting for navdata feed')'''
        
        #markers widget
        self.markersBox = QtGui.QTextEdit(cWidget)
        if (self.markersBox.isReadOnly() == False):
            self.markersBox.setReadOnly(True)
        grid.addWidget(self.markersBox,0,1)
        
        self.markersBox.setText('Waiting for markers feed')
        
        ###########commands#################################
        
        gridCommands = QtGui.QGridLayout()
        grid.addLayout(gridCommands,0,0)
        
        upperLeft = QtGui.QVBoxLayout()
        upperLeft.setSpacing(5)
        gridCommands.addLayout(upperLeft,0,0)
        
        upperRight = QtGui.QVBoxLayout()
        upperRight.setSpacing(5)
        gridCommands.addLayout(upperRight,0,1)
        
        lowerLeft = QtGui.QVBoxLayout()
        lowerLeft.setSpacing(5)
        gridCommands.addLayout(lowerLeft,1,0)
        
        lowerRight = QtGui.QVBoxLayout()
        lowerRight.setSpacing(5)
        gridCommands.addLayout(lowerRight,1,1)
        
        button000 = QtGui.QPushButton('Start demo', cWidget)
        self.connect(button000, QtCore.SIGNAL('clicked()'), self.controller.low.demo_gesture)
        upperLeft.addWidget(button000)
        
        button5 = QtGui.QPushButton('Terminate', cWidget)
        self.connect(button5, QtCore.SIGNAL('clicked()'), self.controller.low.send_land)
        upperLeft.addWidget(button5)
        
        button6 = QtGui.QPushButton('Reset', cWidget)
        self.connect(button6, QtCore.SIGNAL('clicked()'), self.controller.low.reset)
        upperLeft.addWidget(button6)
        
        #first couple of sliders
        self.slider1 = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider1.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.slider1.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.slider1.setTickInterval(255)
        self.slider1.setSingleStep(2)

        self.slider1.valueChanged[int].connect(self.setValue1)
        self.setMaximum1(0)
        self.setMaximum1(255)
        
        self.lbl1 = QtGui.QLabel()
        self.lbl1.setText('Hmin')
        upperRight.addWidget(self.lbl1)
        upperRight.addWidget(self.slider1)
        
        self.slider2 = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider2.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.slider2.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.slider2.setTickInterval(255)
        self.slider2.setSingleStep(2)

        self.slider2.valueChanged[int].connect(self.setValue2)
        self.setMaximum2(0)
        self.setMaximum2(255)
        
        self.lbl2 = QtGui.QLabel()
        self.lbl2.setText('Hmax')
        upperRight.addWidget(self.lbl2)
        upperRight.addWidget(self.slider2)
        
        #second
        
        self.slider3 = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider3.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.slider3.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.slider3.setTickInterval(255)
        self.slider3.setSingleStep(2)

        self.slider3.valueChanged[int].connect(self.setValue3)
        self.setMaximum3(0)
        self.setMaximum3(255)
        
        self.lbl3 = QtGui.QLabel()
        self.lbl3.setText('Smin')
        lowerLeft.addWidget(self.lbl3)
        lowerLeft.addWidget(self.slider3)
        
        self.slider4 = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider4.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.slider4.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.slider4.setTickInterval(255)
        self.slider4.setSingleStep(2)

        self.slider4.valueChanged[int].connect(self.setValue4)
        self.setMaximum4(0)
        self.setMaximum4(255)
        
        self.lbl4 = QtGui.QLabel()
        self.lbl4.setText('Smax')
        lowerLeft.addWidget(self.lbl4)
        lowerLeft.addWidget(self.slider4)
        
        #third
        
        self.slider5 = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider5.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.slider5.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.slider5.setTickInterval(255)
        self.slider5.setSingleStep(2)

        self.slider5.valueChanged[int].connect(self.setValue5)
        self.setMaximum5(0)
        self.setMaximum5(255)
        
        self.lbl5 = QtGui.QLabel()
        self.lbl5.setText('Vmin')
        lowerRight.addWidget(self.lbl5)
        lowerRight.addWidget(self.slider5)
        
        self.slider6 = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.slider6.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.slider6.setTickPosition(QtGui.QSlider.TicksBothSides)
        self.slider6.setTickInterval(255)
        self.slider6.setSingleStep(2)

        self.slider6.valueChanged[int].connect(self.setValue6)
        self.setMaximum6(0)
        self.setMaximum6(255)
        
        self.lbl6 = QtGui.QLabel()
        self.lbl6.setText('Vmax')
        lowerRight.addWidget(self.lbl6)
        lowerRight.addWidget(self.slider6)

        ####################################################
        cWidget.setLayout(grid)
        self.setCentralWidget(cWidget)

        
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.receive_navdata)
        #display textually only infos relative to tag to follow and box tag. 
        self.subARMarkerIndividual = rospy.Subscriber('/ar_pose_marker_individual', AlvarMarkers, self.receive_marker_individual)
        #since they have different dimension, they must be handled separately.
        self.subARMarkerRoom = rospy.Subscriber('/ar_pose_marker_room', AlvarMarkers, self.receive_marker_room)
        self.subPosition = rospy.Subscriber('/ardrone/position',Position, self.receive_position)
        
        #self.subVideo   = rospy.Subscriber('/ardrone/image_raw', Image, self.receive_image)
        self.pubGesture = rospy.Publisher('/ardrone/gesture', String)
        
        self.pubHmin = rospy.Publisher('/ardrone/Hmin', Int16)
        self.pubHmax = rospy.Publisher('/ardrone/Hmax', Int16)
        self.pubSmin = rospy.Publisher('/ardrone/Smin', Int16)
        self.pubSmax = rospy.Publisher('/ardrone/Smax', Int16)
        self.pubVmin = rospy.Publisher('/ardrone/Vmin', Int16)
        self.pubVmax = rospy.Publisher('/ardrone/Vmax', Int16)
        
        self.subHmin = rospy.Subscriber('/ardrone/Hminc', Int16, self.receive_hmin)
        self.subHmax = rospy.Subscriber('/ardrone/Hmaxc', Int16, self.receive_hmax)
        self.subSmin = rospy.Subscriber('/ardrone/Sminc', Int16, self.receive_smin)
        self.subSmax = rospy.Subscriber('/ardrone/Smaxc', Int16, self.receive_smax)
        self.subVmin = rospy.Subscriber('/ardrone/Vminc', Int16, self.receive_vmin)
        self.subVmax = rospy.Subscriber('/ardrone/Vmaxc', Int16, self.receive_vmax)
        
        # Holds the image frame received from the drone and later processed by the GUI
        self.image = None
        self.imageLock = Lock()
        
        # Holds the status message to be displayed on the next GUI update
        self.status_message = ''
        self.marker_message = ''
        self.marker_room_message = ''
        self.position_message = ''
        

        # Tracks whether we have received data since the last connection check
        # This works because data comes in at 50Hz but we're checking for a connection at 4Hz
        self.communicationSinceTimer = False
        self.connected = False

        # A timer to check whether we're still connected
        self.connectionTimer = QtCore.QTimer(self)
        self.connectionTimer.timeout.connect(self.connection_callback)
        self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
         
        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.redraw_callback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)                  
    
    def receive_hmin(self, msg):
        print "received Hmin"
        self.slider1.setValue(msg.data)
        self.lbl1.setText("Hmin: %d" %msg.data)
    
    def receive_hmax(self, msg):
        self.slider2.setValue(msg.data)
        self.lbl2.setText("Hmax: %d" %msg.data)
    
    def receive_smin(self, msg):
        self.slider3.setValue(msg.data)
        self.lbl3.setText("Smin: %d" %msg.data)
    
    def receive_smax(self, msg):
        self.slider4.setValue(msg.data)
        self.lbl4.setText("Smax: %d" %msg.data)
    
    def receive_vmin(self, msg):
        self.slider5.setValue(msg.data)
        self.lbl5.setText("Vmin: %d" %msg.data)
    
    def receive_vmax(self, msg):
        self.slider6.setValue(msg.data)
        self.lbl6.setText("Vmax: %d" %msg.data)
    
    def setValue1(self, value):
        if self.controller.low.drone_info.task != DroneTask.DemoGesture:
            self.slider1.setValue(value)
            self.lbl1.setText("Hmin: %d" %value)
            self.pubHmin.publish(Int16(value))
        
    def setMinimum1(self, value):
        self.slider1.setMinimum(value)
    def setMaximum1(self, value):
        self.slider1.setMaximum(value)
        
    def setValue2(self, value):
        if self.controller.low.drone_info.task != DroneTask.DemoGesture:
            self.slider2.setValue(value)
            self.lbl2.setText("Hmax: %d" %value)
            self.pubHmax.publish(Int16(value))
        
    def setMinimum2(self, value):
        self.slider2.setMinimum(value)
    def setMaximum2(self, value):
        self.slider2.setMaximum(value)
    
    def setValue3(self, value):
        if self.controller.low.drone_info.task != DroneTask.DemoGesture:
            self.slider3.setValue(value)
            self.lbl3.setText("Smin: %d" %value)
            self.pubSmin.publish(Int16(value))
        
    def setMinimum3(self, value):
        self.slider3.setMinimum(value)
    def setMaximum3(self, value):
        self.slider3.setMaximum(value)
        
    def setValue4(self, value):
        if self.controller.low.drone_info.task != DroneTask.DemoGesture:
            self.slider4.setValue(value)
            self.lbl4.setText("Smax: %d" %value)
            self.pubSmax.publish(Int16(value))
        
    def setMinimum4(self, value):
        self.slider4.setMinimum(value)
    def setMaximum4(self, value):
        self.slider4.setMaximum(value)
        
    def setValue5(self, value):
        if self.controller.low.drone_info.task != DroneTask.DemoGesture:
            self.slider5.setValue(value)
            self.lbl5.setText("Vmin: %d" %value)
            self.pubVmin.publish(Int16(value))
        
    def setMinimum5(self, value):
        self.slider5.setMinimum(value)
    def setMaximum5(self, value):
        self.slider5.setMaximum(value)
        
    def setValue6(self, value):
        if self.controller.low.drone_info.task != DroneTask.DemoGesture:
            self.slider6.setValue(value)
            self.lbl6.setText("Vmax: %d" %value)
            self.pubVmax.publish(Int16(value))
        
    def setMinimum6(self, value):
        self.slider6.setMinimum(value)
    def setMaximum6(self, value):
        self.slider6.setMaximum(value)                     
    
    def public_gesture(self):
        self.pubGesture.publish(String("EXPLORE"))
        
    def call_animation(self):
        utils.flight_animation(4)    
    
    def onTextChanged(self,text):
        self.controller.low.drone_info.flyght_tag = int(text)
        self.lbl.setText('Tag to track for cross flight: ' + text)
        rospy.loginfo("tag: %d" %self.controller.low.drone_info.flyght_tag)
        self.lbl.adjustSize()
    # Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
    def connection_callback(self):
        self.connected = self.communicationSinceTimer
        self.communicationSinceTimer = False

    def redraw_callback(self):
        self.markersBox.setText(self.marker_message)
        #if self.image is not None:
            # We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
        '''self.imageLock.acquire()
            try:
                    # Convert the ROS image into a QImage which we can display
                    image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
            finally:
                    self.imageLock.release()'''

            # We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
            #elf.imageBox.resize(image.width(),image.height())
            #self.imageBox.setPixmap(image)
            #self.navdataBox.setText(self.status_message)
        # Update the status bar to show the current drone status & battery level
        #self.statusBar().showMessage(self.status_message if self.connected else self.DisconnectedMessage)

    def receive_image(self,data):
        # Indicate tbelt new data has been received (thus we are connected)
        self.communicationSinceTimer = True

        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            self.image = data # Save the ros image for processing by the display thread
        finally:
            self.imageLock.release()

    def receive_navdata(self,navdata):
        # Indicate tbelt new data has been received (thus we are connected)
        self.communicationSinceTimer = True
        msg = ''
        msg += 'State: %s \n' %self.status_messages[navdata.state]
        msg += 'Battery: %f \n' %navdata.batteryPercent
        '''msg += 'RotX: %f \n' %navdata.rotX
        msg += 'RotY: %f \n' %navdata.rotY
        msg += 'RotZ: %f \n' %navdata.rotZ
        msg += 'Altd: %f \n' %navdata.altd
        msg += 'Velx: %f \n' %navdata.vx
        msg += 'Vely: %f \n' %navdata.vy
        msg += 'Velz: %f \n' %navdata.vz
        msg += 'Accx: %f \n' %navdata.ax
        msg += 'Accy: %f \n' %navdata.ay
        msg += 'Accz: %f \n' %navdata.az'''
        
        # Update the message to be displayed even with position message
        self.status_message = msg #+ '\nPosition: \n' + self.position_message
    
    def receive_marker_individual(self,data):
        msg = 'Individual markers: \n'
        for marker in data.markers:
            if (marker.id in TagIds.AllFollowTags or marker.id == TagIds.FrontTag or marker.id == TagIds.BackTag or marker.id == TagIds.LeftTag or marker.id == TagIds.RightTag or
                marker.id == TagIds.BeltBackTag or marker.id == TagIds.BeltFrontTag or marker.id == TagIds.BeltLeftTag or marker.id == TagIds.BeltRightTag):
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                msg += 'Marker ID: %d\n' %marker.id
                msg += 'X: %f\n' %marker.pose.pose.position.x
                msg += 'Y: %f\n' %marker.pose.pose.position.y
                msg += 'Z: %f\n' %marker.pose.pose.position.z
                msg += 'Yaw: %f\n' %utils.rad_to_deg(yaw)
                msg += '*****************\n'
        
        self.marker_message = self.status_message +  msg + 'Room Markers: \n' + self.marker_room_message
    
    def receive_marker_room(self,data):
        msg = ''
        for marker in data.markers:
            if marker.id == TagIds.RoomFrontTag or marker.id == TagIds.RoomBackTag or marker.id == TagIds.RoomLeftTag or marker.id == TagIds.RoomRightTag:
                q = utils.msg_to_quaternion(marker.pose.pose.orientation)
                roll,pitch,yaw = euler_from_quaternion(q)
                msg += 'Marker ID: %d\n' %marker.id
                msg += 'X: %f\n' %marker.pose.pose.position.x
                msg += 'Y: %f\n' %marker.pose.pose.position.y
                msg += 'Z: %f\n' %marker.pose.pose.position.z
                msg += 'Yaw: %f\n' %utils.rad_to_deg(yaw)
                msg += '*****************\n'
        
        self.marker_room_message = msg
        
    def receive_position(self,position):
        msg = ''
        msg += 'x: %f\n' %(position.x)
        msg += 'y: %f\n' %(position.y)
        msg += 'z: %f\n' %position.z
        msg += 'alpha: %f \n' %position.alpha
        self.position_message = msg
              
