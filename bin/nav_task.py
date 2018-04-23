#!/usr/bin/env python2

import rospy
import tf
from tf.transformations import *

import numpy as np
import random
from math import sqrt, atan, pi, degrees, floor, atan2

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *

from human_tasks.msg import *
from human_tasks.srv import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from PyQt5.QtCore import pyqtSignal as Signal

from QLabeledValue import *
from QLabeledSlider import *

import RobotIcon
#import ObjectIcon
import QArrow

import os, csv
import rospkg
import struct
import cv2
import copy
from Queue import *
from collections import *

import pdb


class NavTaskWidget(QSplitter):
    scoringComplete = Signal()
    robot_state_changed = Signal()
    goal_changed = Signal()
    goalList_changed = Signal(str)
    
    def __init__(self, map_topic='/map', cam_topic = '/image_raw'):
        super(NavTaskWidget, self).__init__()
        self.initUI(map_topic, cam_topic)

    def initUI(self, map_topic, cam_topic):
        
        self.setWindowTitle('Navigation Task')
        
        vNavLayout = QVBoxLayout()
     
        self.map_view = DEMView(map_topic, parent = self)
        self.cam_view = CamView(cam_topic)
        self.btnDone = QPushButton('Get Score')
        self.btnDone.clicked.connect(self.btnDone_onclick)
        

        self.setOrientation(Qt.Vertical)
        upperHalf = QHBoxLayout()
        
        upperHalf.addWidget(self.cam_view)
        upperHalf.addWidget(self.map_view)
        upperWidget = QWidget()
        upperWidget.setLayout(upperHalf)
        self.addWidget(upperWidget)

        hNavLayout = QHBoxLayout()
               
        poseGroup = QGroupBox("Current Pose")
        poseLayout = QVBoxLayout()


        goalGroup = QGroupBox('Ranging To Goal')
        goalLayout = QVBoxLayout()


        self.poseLabels = [QLabeledValue("X"),
                QLabeledValue("Y"),
                QLabeledValue("Yaw")]
        poseLayout.setSpacing(0)
        for label in self.poseLabels:
            poseLayout.addWidget(label)

        poseGroup.setLayout(poseLayout)

        goalRangeGroup = QGroupBox('Ranging to Goal')
        goalRangeLayout = QVBoxLayout()

        self.goalRangeLabels=[QLabeledValue('Range X'),
                              QLabeledValue('Range Y'),
                              QLabeledValue('Target Yaw')]
        goalRangeLayout.setSpacing(0)
        for label in self.goalRangeLabels:
            goalRangeLayout.addWidget(label)
        goalRangeGroup.setLayout(goalRangeLayout)
            
        lowerLeftGroup = QVBoxLayout()
        lowerLeftGroup.addWidget(poseGroup)
        lowerLeftGroup.addWidget(goalRangeGroup)

        controlsGroup = QGroupBox('Robot Controls')
        controlsLayout = QVBoxLayout()
        self.linVelSlider = QLabeledSlider('Linear Vel:')
        self.angVelSlider = QLabeledSlider('Angular Vel:')
        controlsLayout.addWidget(self.linVelSlider)
        controlsLayout.addWidget(self.angVelSlider)
        
        controlsGroup.setLayout(controlsLayout)
        
        lowerHalf = QHBoxLayout()
        lowerHalf.addLayout(lowerLeftGroup)
        lowerHalf.addWidget(controlsGroup)
        
        lowerWidget = QWidget()
        lowerWidget.setLayout(lowerHalf)

        self.addWidget(lowerWidget)
        self.addWidget(self.btnDone)
        
        #self.stateTimer = QTimer()
        #self.stateTimer.timeout.connect(self._updateState)
        #self.stateTimer.start(100)
        
        self.odom_sub = rospy.Subscriber('state', Pose2D, self.robot_state_cb)
        self.dem_sub = rospy.Subscriber('dem', Image, self.dem_cb)

        #Resize the splitter thing
        self.setSizes([self.height()*0.77, self.height()*0.20, self.height()*0.023])
        #Disable the sizer handles
        self.handle(1).setEnabled(False)
        self.handle(2).setEnabled(False)
        
        self.stateQueue = deque() #Queue() #to handle high volume state updates from bag playback

    def btnDone_onclick(self):
        self.scoringComplete.emit()

    def addGoals(self, goalList):
        allGoals = sorted(goalList, key=lambda param: param[0]) #Sort the combined list by the goal ID
        
        for goal in allGoals:
            item = QStandardItem()
            itemKey = str(goal[0])
            item.setText(itemKey)
            item.setCheckable(True)
            item.setCheckState(Qt.Checked)
        
            self.goalModel.appendRow(item)
            self.goalList.setModel(self.goalModel)
            self._map_view.addGoal(goal[0], goal[1].x, goal[1].y)
            #            self._map_view.goals[itemKey] = goal[1]
            self._map_view.goalVisible[itemKey] = True

        
    def updateRobotState(self):

        #process every message in the queue so far:
        #print 'State queue size:', len(self.stateQueue)
        while True:
            try:
                self._robotState = self.stateQueue.pop()
            except IndexError, e:
                return

            for idx, val in enumerate(self._robotState):
                self.poseLabels[idx].updateValue(val)

                #self.fuelLabel.updateValue(self._robotFuel)

                #Ping unity with a steer if enabled
                '''
                if not self.lastSteerMsg is None:
                self.steer_pub.publish(self.lastSteerMsg)
                '''
                #Add to the current traverse

            currentIndex = len(self._map_view.traverses.keys()) - 1
            if currentIndex >= 0:
                self._map_view.addTraversePoint(currentIndex, (self._robotState[0], self._robotState[1]))

        
    def robot_state_cb(self, msg):
        #Resolve the odometry to a screen coordinate for display

        worldX = msg.pose.position.x
        worldY = msg.pose.position.y
        worldZ = msg.pose.position.z

        
        worldRoll, worldPitch, worldYaw = euler_from_quaternion([msg.pose.orientation.x,
                                                                 msg.pose.orientation.y,
                                                                 msg.pose.orientation.z,
                                                                 msg.pose.orientation.w],'sxyz')


        #TODO: Wrap the inv rpy to [-pi, pi]
        #print 'Orientation:', msg.pose.orientation, ' RPY:', worldRoll, worldPitch, worldYaw
        
        self.stateQueue.appendleft([worldX, worldY, worldZ, worldRoll, worldPitch, worldYaw])
        
        #print 'Robot State:', self._robotState
        #self._robotFuel = msg.fuel

        self.robot_state_changed.emit()

    def dem_cb(self, msg):
        print 'Got dem cb'
        
    def close(self):
        if self.dem_sub:
            self.dem_sub.unregister()
        if self.odom_sub:
            self.odom_sub.unregister()
            
    def mousePressEvent(self, evt):
            print 'Sizes:', self.sizes()
class DEMView(QGraphicsView):
    dem_changed = Signal()
    robot_odom_changed = Signal()

    def __init__(self, dem_topic='dem',
                 tf=None, parent=None):
        super(DEMView, self).__init__()
        self._parent = parent
        self.demDownsample = 4
        #self.dem_changed.connect(self._update)
        
        
        self._dem_item = None
        self._robotIcon = None
        
        self.setDragMode(QGraphicsView.NoDrag)

        self._addedItems = dict()
        self.w = 4000
        self.h = 4000

        #Two color families:
        #Red and green
        self._colors = [(238, 34, 116),
                        (68, 134, 252),
                        (0xff, 0, 0),
                        (0xff, 00, 0xa8),
                        (0xff, 0x5a, 00),
                        (0xff, 0x20, 00),
                        (0, 0, 0xff),
                        (0, 0x60, 0xff),
                        (0, 0x90, 0xff),
                        (0x72, 0, 0xff),
                        (236, 228, 46),
                        (102, 224, 18),
                        (242, 156, 6),
                        (240, 64, 10),
                        (196, 30, 250),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)) ]
        
        self._scene = QGraphicsScene()
        self.setScene(self._scene)
        
        self.traverses = dict()
        self.traverseVisible = dict()

        self._robotLocation = [0,0,0,0,0,0]
        self.goalIcons = dict()
        self.goalVisible = dict()
        
        self.setAcceptDrops(True)

        self.initialPosition = (3725, 2400)
        
    
        
    def l2dist(self, src, dest):
        return math.sqrt((dest[0] - src[0])**2 + (dest[1] - src[1]) ** 2)
    
    def addTraversePoint(self, index, point):
        theColor = QColor(self._colors[index][0], self._colors[index][1], self._colors[index][2])
        theColor.setAlpha(80)
        thePen = QPen(theColor)
        thePen.setWidthF(5)

        theBrush = QBrush(theColor)
        theBrush.setStyle(Qt.SolidPattern)
        
        if self._dem_item is None:
            return

        point_scaled = (point[0] / self.demDownsample, point[1] / self.demDownsample)
        
        if len(self.traverses[index]) == 0:
            
            marker = self._scene.addEllipse(0.0, 0.0, 1.0, 1.0, thePen, theBrush)
            bounds = marker.boundingRect()
            
            #marker.setPos(QPointF(point_scaled[0] - bounds.width()/2, point_scaled[1] - bounds.height()/2))
            marker.setPos(QPointF(point_scaled[0], point_scaled[1]))
            
            self.traverses[index].append(marker)
            marker.setVisible(self.traverseVisible[index])

        elif self.l2dist((self.traverses[index][-1].x(), self.traverses[index][-1].y()),
                         point_scaled) > 1.0:
            marker = self._scene.addEllipse(0.0, 0.0, 1.0, 1.0, thePen, theBrush)
            bounds = marker.boundingRect()
            
            marker.setPos(QPointF(point_scaled[0], point_scaled[1]))
            #marker.setTransformOriginPoint(QPoint(bounds.width()/2, bounds.height()/2))
            self.traverses[index].append(marker)
            marker.setVisible(self.traverseVisible[index])

    def setGoalState(self, index, state):
        #Alter the visibility of the given goal
        if index not in self.goalVisible.keys():
            print 'DEMView: Goal index ', index, ' not in:', self.goalVisible.keys()
            return

        print 'Setting goal ', index, ' to visible state:', state
        self.goalVisible[index] = state
        
        #Go through the GraphicsItems that make up the goals and set their visibility as needed:
        self.goalIcons[index].setVisible(state)

            
    def setTraverseState(self, index, state):
        #Alter the visibility of the given traverse
        if index >= len(self.traverses):
            print 'DEMView: Traverse index ', index, ' out of bounds:', len(self.traverses)
            return

        print 'Setting traverse ', index, ' to visible state:', state
        self.traverseVisible[index] = state
        
        #Go through the GraphicsItems that make up the traverse and set their visibility as needed:
        for item in self.traverses[index]:
            item.setVisible(state)
            
    def addInitial(self, x, y):
        thisGoal = RobotIcon.RobotWidget(str('S'), QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2]))
        thisGoal.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
        thisGoal.setBrush(QBrush(QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2])))
        world = [x, y]
        #print 'Goal raw coord:', world
        iconBounds = thisGoal.boundingRect()

        world[0] /= self.demDownsample
        world[1] /= self.demDownsample

        thisGoal.setPos(QPointF(world[0]-iconBounds.width()/2, world[1]-iconBounds.height()/2))
        print 'Added initial position at :', world
        self.initialPositionIcon = thisGoal
        self._scene.addItem(self.initialPositionIcon)
        
    def addGoal(self, id, x, y):
        #Draw a new goal icon per the directions...
        if id not in self.goalIcons.keys():
            thisGoal = RobotIcon.RobotWidget(str(id), QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2]))
            thisGoal.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            thisGoal.setBrush(QBrush(QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2])))          
            self.goalIcons[id] = thisGoal
            #self._scene.addItem(thisGoal)
            #Update the label's text:
            self.goalIcons[id].setText(str(id))
            
            #Pick up the world coordinates
            world = [x, y]
            #print 'Goal raw coord:', world
            iconBounds = self.goalIcons[id].boundingRect()

            world[0] /= self.demDownsample
            world[1] /= self.demDownsample
            
            #Adjust the world coords so that the icon is centered on the goal
            #Since the font may change, we adjust this later
            #world[0] = world[0] - iconBounds.width()/2 
            #world[1] = world[1] - iconBounds.height()/2 #mirror the y coord
            
            #print 'Drawing goal ', id, ' at ', world
            self.goalIcons[id].setPos(QPointF(world[0], world[1]))
            self.goalVisible[id] = True
            
    def robot_odom_cb(self, msg):

        #Resolve the odometry to a screen coordinate for display from a RobotState message

        worldX = msg.pose.position.x
        worldY = msg.pose.position.y
        worldZ = msg.pose.position.z
        
        worldRoll, worldPitch, worldYaw = euler_from_quaternion([msg.pose.orientation.x,
                                                                 msg.pose.orientation.y,
                                                                 msg.pose.orientation.z,
                                                                 msg.pose.orientation.w],'sxyz')

       
        self._robotLocation = [worldX, worldY, worldZ, worldRoll, worldPitch, worldYaw]
        #print 'Robot at: ', self._robotLocations[0] 
        
        self.robot_odom_changed.emit()
        
    def _updateRobot(self):
        #Redraw the robot locations
        #print 'Updating robot locations'
        #If this is the first time we've seen this robot, create its icon
        if self._robotIcon == None:
            #thisRobot = RobotIcon.RobotWidget('R', color=QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2]))
            #thisRobot.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            #thisRobot.setBrush(QBrush(QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2])))
            thisRobot = QArrow.QArrow(color=QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2]))
            self._robotIcon = thisRobot
            self._scene.addItem(thisRobot)
            
        #Pick up the world coordinates - copy so we can change in place
        world = copy.deepcopy(self._robotLocation)
        #print 'Raw robot loc: ', world[0], world[1]
        
        iconBounds = self._robotIcon.boundingRect()

        world[0] /= self.demDownsample
        world[1] /= self.demDownsample
        
        #Adjust the world coords so that the icon is centered on the robot, rather than top-left
        world[0] = world[0] - iconBounds.width()/2 
        world[1] = world[1] - iconBounds.height()/2 #mirror the y coord

        #print 'Drawing robot at ', world[0], world[1]
        
        self._robotIcon.setPos(QPointF(world[0], world[1]))
        
        #print 'Rotating:', world[5]
        self._robotIcon.setRotation(world[5]*180/math.pi + 90)
        
        #Set the origin so that the caret rotates around its center(ish)
        self._robotIcon.setTransformOriginPoint(QPoint(iconBounds.width()/2, iconBounds.height()/2))

        #print 'Transform origin:', self._robotIcon.transformOriginPoint()
    
        
    def mousePressEvent(self,e):
        return
    
    def hazmap_cb(self, msg):
        #Unlike the dem, the hazmap is pretty standard - gray8 image
        hazmap = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.hazmapImage = QImage(hazmap, msg.width, msg.height, QImage.Format_Grayscale8)
        self.hazmap_changed.emit()

    def _updateHazmap(self):
        print 'Rendering hazmap'

        hazTrans = QImage(self.hazmapImage.width(), self.hazmapImage.height(), QImage.Format_ARGB32)
        hazTrans.fill(Qt.transparent)
        
        for row in range(0, self.hazmapImage.height()):
            for col in range(0, self.hazmapImage.width()):
                #Change the colormap to be clear for clear areas, red translucent for obstacles
                pixColor = self.hazmapImage.pixelColor(col, row)

                if pixColor.rgba() == 0xff000000:
                    hazTrans.setPixelColor(col, row, QColor(255, 0, 0, 32))
                else:
                    hazTrans.setPixelColor(col, row, QColor(0, 0, 0, 0))

        self.hazmapItem = self._scene.addPixmap(QPixmap.fromImage(hazTrans)) #.scaled(self.w*100,self.h*100))
        self.hazmapItem.setPos(QPointF(0, 0))
        trans = QTransform()
        #print 'Translating by:', bounds.width()
        
        trans.scale(self.w/hazTrans.width(),self.h/hazTrans.height())
        #trans.translate(0, -bounds.height())
        self.hazmapItem.setTransform(trans)
        
        # Everything must be mirrored
        #self._mirror(self.hazmapItem)
        
    def dem_cb(self, msg):
        #self.resolution = msg.info.resolution
        self.w = msg.width
        self.h = msg.height

        print 'Got DEM encoded as:', msg.encoding
        print 'message length:', len(msg.data), 'type:', type(msg.data)
        print 'width:', msg.width
        print 'height:', msg.height
        
        a = np.array(struct.unpack('<%dd' % (msg.width*msg.height), msg.data), dtype=np.float64, copy=False, order='C')

        rawDEM = a.reshape((self.h, self.w))
        rawDEM = cv2.resize(rawDEM, (self.h//self.demDownsample, self.w//self.demDownsample), interpolation = cv2.INTER_LINEAR)
        self.h = rawDEM.shape[0]
        self.w = rawDEM.shape[1]
        
        '''
        if self.w % 4:
            e = np.zeros((self.h, 4 - self.w % 4), dtype=rawDEM.dtype, order='C')
            rawDEM = np.append(rawDEM, e, axis=1)
            self.h = rawDEM.shape[0]
            self.w = rawDEM.shape[1]
        ''' 

        #Scale to a 8-bit grayscale image:
        self.grayDEM = np.zeros(rawDEM.shape, dtype=np.uint8)
        minZ = np.min(np.min(rawDEM))
        maxZ = np.max(np.max(rawDEM))
        dynRange = maxZ - minZ

        print 'Max Z:', maxZ
        print 'Min Z:', minZ
        
        for i in range(0, self.h):
            for j in range(0, self.w):
                self.grayDEM[i][j] = (rawDEM[i][j] - minZ) * 255/dynRange

        #use OpenCV2 to interpolate the dem into something reasonably sized
        print 'Grayscale conversion complete'

        #Needs to be a class variable so that at QImage built on top of this numpy array has
        #a valid underlying buffer - local ars
        #self.resizedDEM = cv2.resize(self.grayDEM, (500,500), interpolation = cv2.INTER_LINEAR)

        print 'Image resize complete'
        
        self.h = self.grayDEM.shape[0]
        self.w = self.grayDEM.shape[1]
        image = QImage(self.grayDEM.reshape((self.h*self.w)), self.w, self.h, QImage.Format_Grayscale8)

        #        for i in reversed(range(101)):
        #            image.setColor(100 - i, qRgb(i* 2.55, i * 2.55, i * 2.55))
        #        image.setColor(101, qRgb(255, 0, 0))  # not used indices
        #        image.setColor(255, qRgb(200, 200, 200))  # color for unknown value -1

        self._dem = image       
        self.dem_changed.emit()

    def close(self):
        super().close()
            
    def mousePressEvent(self, e):
        scenePt = self.mapToScene(e.pos())
        print 'Click at:', scenePt.x(), ' ', scenePt.y()

        
    def dragMoveEvent(self, e):
        print('Scene got drag move event')
        
    def resizeEvent(self, evt=None):
        #Resize map to fill window
        scale = 1
        bounds = self._scene.sceneRect()
        if bounds:
            self._scene.setSceneRect(0, 0, self.w*scale, self.h*scale)
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
            #self.centerOn(self._dem_item)
            self.show()
 
    def _update(self):
        if self._dem_item:
            self._scene.removeItem(self._dem_item)

        pixmap = QPixmap.fromImage(self._dem)
        self._dem_item = self._scene.addPixmap(pixmap) #.scaled(self.w*100,self.h*100))
        self._dem_item.setPos(QPointF(0, 0))
        # Everything must be mirrored
        #self._mirror(self._dem_item)

        # Add drag and drop functionality
        self.add_dragdrop(self._dem_item)

        #Resize map to fill window
        scale = 1
        self.setSceneRect(0, 0, self.w*scale, self.h*scale)
        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
        #self.centerOn(self._dem_item)
        self.show()
        bounds = self._scene.sceneRect()
        #print 'Bounds:', bounds
        #Allow the robot position to be drawn on the DEM 
        #self.robot_odom_changed.connect(self._updateRobot)

        #Overlay the hazmap now that the dem is loaded
        self.hazmap_sub = rospy.Subscriber('hazmap', Image, self.hazmap_cb)

        #Allow goals to be drawn as well
        #self.goalList_changed.connect(self._updateGoalList)

        #Add the policy as well
        self.policy_sub = rospy.Subscriber('policy', Policy, self.policy_cb)

        #Make sure the goals stack above this
        for item in self.goalIcons.items():
            self._scene.addItem(item[1])
            item[1].setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            #When the font size changes, the boundingRect changes - adjust the position:
            bounds = item[1].boundingRect()
            pos = item[1].pos()
            item[1].setPos(QPointF(pos.x() - bounds.width()/2.0, pos.y() - bounds.height()/2.0))
        #Draw the initial position
        self.addInitial(self.initialPosition[0], self.initialPosition[1])
        
    def _mirror(self, item):
        #Get the width from the item's bounds...
        bounds = item.sceneBoundingRect()
        #print 'Bounds:', bounds
        #print 'item:', item

        trans = QTransform()
        #print 'Translating by:', bounds.width()
        
        trans.scale(1,-1)
        trans.translate(0, -bounds.height())
        item.setTransform(trans)

        #bounds = item.sceneBoundingRect()
        #print 'Bounds:', bounds

class CamView(QGraphicsView):
    #Subscribe to a camera view on this topic
    image_changed = Signal()
    def __init__(self, cam_topic='/image_raw',parent=None):
        super(CamView,self).__init__()
        self._parent = parent

        self.setScene(QGraphicsScene())
        self._imageSub = rospy.Subscriber(cam_topic, Image, self.image_cb)
        self.w = 0
        self.h = 0
        self.image_changed.connect(self._updateImage)
        self._image = None
        self._imageItem = None
        self.setMaximumWidth(400)

        
    def image_cb(self, msg):
        self._image = QPixmap.fromImage(QImage(msg.data, msg.width, msg.height, QImage.Format_RGB888))
        self.w = msg.width
        self.h = msg.height
        self.image_changed.emit()
        
    def _updateImage(self):
        if self._imageItem:
            self.scene().removeItem(self._imageItem)
         #self._image.height/2)
        self._imageItem = self._scene.addPixmap(self._image)
        self._imageItem.setZValue(0)
        self.setSceneRect(0, 0, self.w, self.h)

    def resizeEvent(self, evt=None):
        #Resize

        bounds = self.scene().itemsBoundingRect()

        if bounds:
            #Wait until bounds exist, which only happen after the map is updated once
            bounds.setWidth(bounds.width()*1.0)         
            bounds.setHeight(bounds.height()*1.0)
            self.fitInView(bounds, Qt.KeepAspectRatio)
            #self.centerOn(self._imageG)
            self.show()
            
def main():
        if len(sys.argv) < 1:
                print 'Usage:', sys.argv[0], ''

                sys.exit(-1)
                
	app = QApplication(sys.argv)
        rospy.init_node('nav_task')
	navTask = NavTaskWidget()

        mainWindow = QMainWindow()
        mainWindow.setWindowTitle('Navigation task')
        mainWindow.setCentralWidget(navTask)
        mainWindow.setStatusBar(QStatusBar())
        mainWindow.show()
        app.exec_()

	sys.exit(0)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
