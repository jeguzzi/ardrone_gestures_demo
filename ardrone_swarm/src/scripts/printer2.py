#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_swarm')
import rospy
import numpy
import sys
import math
import matplotlib

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from geometry_msgs.msg import Point
from ardrone_swarm.msg import Position
from ar_track_alvar.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as  NavigationToolbar
from matplotlib.figure import Figure
from matplotlib.path import Path
import matplotlib.patches as patches
from std_msgs.msg import Int8
from std_msgs.msg import String

import utils
from tag import Tag
from tag import TagAR
from tag import TagIds
from tag import RoomTagCoor


class PlotWindow(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('MAP')
        self.create_main_frame()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        self.axes.clear()
        self.axes.set_xlim([-2,10])
        self.axes.set_ylim([-2,10])
        self.axes.grid(True)
        self.draw_room()
        self.axes.set_autoscaley_on(False)
        self.axes.set_autoscalex_on(False)        
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.axes = self.fig.add_subplot(111)
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)     
        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

    def draw_room(self):
        verts = [
                (0., 0.), # left, bottom
                (0., RoomTagCoor.DIM_Y), # left, top
                (RoomTagCoor.DIM_X, RoomTagCoor.DIM_Y), # right, top
                (RoomTagCoor.DIM_X, 0.), # right, bottom
                (0., 0.), # ignored
                ]

        codes = [Path.MOVETO,
                 Path.LINETO,
                 Path.LINETO,
                 Path.LINETO,
                 Path.CLOSEPOLY,
                ]

        path = Path(verts, codes)

        patch = patches.PathPatch(path, facecolor='white', lw=2)
        self.axes.add_patch(patch)


class OnlinePlot(PlotWindow):
    def __init__(self):
        PlotWindow.__init__(self)
        self.count = 0

        self.window_size=30
        
        self.x = 0.0
        self.y = 0.0
        
        self.box_position_received = False
        self.x_box = -999
        self.y_box = -999
        self.tag_located = False

        self.listx = []
        self.listy = []
        
        self.alpha = 0.0
        
        rospy.init_node('printer2', anonymous=True)
        #display graphically the room's marker position + box position (which will be received in a separate topic)
        self.subARMarker = rospy.Subscriber('/ar_pose_marker_room', AlvarMarkers, self.receive_marker_and_print_path)
        self.subPosition = rospy.Subscriber('/ardrone/position', Position, self.receive_position)
        self.subBoxPosition = rospy.Subscriber('/ardrone/box_position', Point, self.receive_box_position)
        self.subResetPosition = rospy.Subscriber('/ardrone/reset_position', String, self.receive_reset)
        
        self.marker_callback_started = False 
    
    def receive_marker_and_print_path(self, data):
        #exploit the callback function of the marker to update the plot of the path
        #the room node must be launched before the main node!
        if not self.marker_callback_started: self.marker_callback_started = True
        
        if self.count == 0:
            self.count += 1
            self.axes.clear()
            self.axes.set_xlim([-2,7])
            self.axes.set_ylim([-2,7])
            self.axes.grid(True)
            self.axes.set_autoscaley_on(False)
            self.axes.set_autoscalex_on(False)             
            for marker in data.markers:
                if marker.id == TagIds.RoomFrontTag or marker.id == TagIds.RoomBackTag or marker.id == TagIds.RoomLeftTag or marker.id == TagIds.RoomRightTag:
                    displacement_x = utils.rad_to_deg(math.atan2(marker.pose.pose.position.x,marker.pose.pose.position.z))
                    orientation = self.alpha - displacement_x
                    while orientation > 180: orientation -= 360
                    while orientation < -180: orientation += 360
                    new_tagX = math.cos(utils.deg_to_rad(orientation))*marker.pose.pose.position.z + self.x
                    new_tagY = math.sin(utils.deg_to_rad(orientation))*marker.pose.pose.position.z + self.y
                
                    self.axes.plot(new_tagX, new_tagY, 'ro')
        
            self.axes.plot(self.listx[0:500], self.listy[0:500],'co', markersize=3)
            self.axes.plot(self.listx[501:999], self.listy[501:999],'mo', markersize=3)
            self.axes.plot(self.x, self.y, 'yo', markersize=5)
            #plot the orientation
            self.axes.plot([self.x, self.x + math.cos(utils.deg_to_rad(self.alpha))],[self.y, self.y + math.sin(utils.deg_to_rad(self.alpha))], 'y')
            if self.box_position_received:
                self.axes.plot(self.x_box, self.y_box, 'go', markersize=5)
            
            self.draw_room()            
            self.canvas.draw()
            
        elif self.count < 60:
            self.count += 1
        else: self.count = 0           
    
    def receive_position(self, position):             
        self.x = position.x
        self.y = position.y
        self.alpha = position.alpha
              
        if self.tag_located is False and position.tag_located is True: #from now, the position should be correct!
            self.tag_located = True
            self.listx = []
            self.listy = []
        
        if len(self.listx) == 1000: self.listx.pop(0)
        if len(self.listy) == 1000: self.listy.pop(0)
              
        self.listx.append(position.x)
        self.listy.append(position.y)
        
        if not self.marker_callback_started:
            if self.count == 0:
                self.count += 1
                self.axes.clear()
                self.axes.set_xlim([-2,7])
                self.axes.set_ylim([-2,7])
                self.axes.grid(True)
                self.axes.set_autoscaley_on(False)
                self.axes.set_autoscalex_on(False)             
                self.axes.plot(self.listx[0:500], self.listy[0:500],'co', markersize=3)
                self.axes.plot(self.listx[501:999], self.listy[501:999],'mo', markersize=3)
                self.axes.plot(self.x, self.y, 'yo', markersize=5)
                #plot the orientation
                self.axes.plot([self.x, self.x + math.cos(utils.deg_to_rad(self.alpha))],[self.y, self.y + math.sin(utils.deg_to_rad(self.alpha))],'y')
                if self.box_position_received:
                    self.axes.plot(self.x_box, self.y_box, 'go', markersize=5)
                
                self.draw_room()        
                self.canvas.draw()
            
            elif self.count < 60:
                self.count += 1
            else: self.count = 0
            
         
    def receive_box_position(self, box_position):
        self.box_position_received = True
        self.x_box = box_position.x
        self.y_box = box_position.y          
    
    def receive_reset(self, msg):
        if msg.data == 'RESET':
            self.x = 0
            self.y = 0
            self.alpha = 0
            self.listx = []
            self.listy = []
            self.box_position_received = False
              
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlinePlot()
    window.show()
    app.exec_()