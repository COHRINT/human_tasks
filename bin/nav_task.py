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
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

import std_srvs.srv

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
    reinitialize = pyqtSignal(Pose, Pose)
    robot_state_changed = Signal()
    goal_changed = Signal()
    goalList_changed = Signal(str)
    
    def __init__(self, terrain_topic='/terrain', cam_topic = '/image_raw',
                 gazebo_namespace='/gazebo', modelName = 'robot0'):
        super(NavTaskWidget, self).__init__()
        self.gazeboNamespace = gazebo_namespace
        self.modelName = modelName
        self.initUI(terrain_topic, cam_topic)
        self.reinitialize.connect(self.initialize)
        self.score = None
        
    def initUI(self, terrain_topic, cam_topic):
        
        self.setWindowTitle('Navigation Task')
        
        vNavLayout = QVBoxLayout()
     
        self.map_view = TerrainView(terrain_topic, parent = self)
        self.cam_view = CamView(cam_topic)
        self.btnDone = QPushButton('Get Score')
        self.btnDone.clicked.connect(self.btnDone_onclick)
        

        self.setOrientation(Qt.Vertical)
        upperHalf = QHBoxLayout()

        camViewGroup = QGroupBox('Camera View')
        camViewLayout = QVBoxLayout()
        camViewLayout.addWidget(self.cam_view)
        camViewGroup.setLayout(camViewLayout)
        
        upperHalf.addWidget(camViewGroup)

        mapViewGroup = QGroupBox('Overhead View')
        mapViewLayout = QVBoxLayout()
        mapViewLayout.addWidget(self.map_view)
        mapViewGroup.setLayout(mapViewLayout)
        
        upperHalf.addWidget(mapViewGroup)
        
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
                QLabeledValue("Z"),
                QLabeledValue("Yaw")]
        
        poseLayout.setSpacing(0)
        for label in self.poseLabels:
            poseLayout.addWidget(label)

        poseGroup.setLayout(poseLayout)
        poseGroup.setMinimumWidth(200)
        
        goalRangeGroup = QGroupBox('Ranging to Goal')
        goalRangeLayout = QVBoxLayout()

        self.goalRangeLabels=[QLabeledValue('Range X'),
                              QLabeledValue('Range Y'),
                              QLabeledValue('Yaw Delta')]
        goalRangeLayout.setSpacing(0)
        for label in self.goalRangeLabels:
            goalRangeLayout.addWidget(label)
        goalRangeGroup.setLayout(goalRangeLayout)
            
        lowerLeftGroup = QVBoxLayout()
        lowerLeftGroup.addWidget(poseGroup)
        lowerLeftGroup.addWidget(goalRangeGroup)

        controlsGroup = QGroupBox('Robot Controls')
        controlsGroup.keyPressEvent = self.keyPressEvent
        
        controlsLayout = QVBoxLayout()
        self.linVelSlider = QLabeledSlider('Linear Velocity', -50, 50, 0)
        self.angVelSlider = QLabeledSlider('Angular Velocity', -20, 20, 0)
        self.linVelSlider.valueChanged.connect(self.onNewLinVel)
        self.angVelSlider.valueChanged.connect(self.onNewAngVel)
        
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
        self.currentGoal = (0,0)
        
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.robot_state_cb)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


        
        
        self.stateQueue = deque() #Queue() #to handle high volume state updates from bag playback
        
        map(self.robot_state_changed.connect, [self.updateRobotState])

    def initialize(self, start, goal):
        #Resize the splitter thing
        self.setSizes([self.height()*0.75, self.height()*0.20, self.height()*0.023])

        #self.setSizes([767, 262, 25])
        #Disable the sizer handles
        #self.handle(1).setEnabled(False)
        #self.handle(2).setEnabled(False)
        
        #Teleport the robot to the start position
        self.setRobotPosition((start.position.x, start.position.y, start.position.z), start.orientation)

        #Resume physics:
        self.resumePhysics()
       

        #Add the goal to the map
        self.currentGoal = (goal.position.x, goal.position.y)
        self.startPos = (start.position.x, start.position.y)
        self.map_view.addGoal('*', goal.position.x, goal.position.y)
        #self.map_view.goalVisible[itemKey] = True
        
        self.score = None
        
        self.btnDone.setFocus()
        self.grabKeyboard()
        
    def resumePhysics(self):
         try:
            unpause_physics = rospy.ServiceProxy(self.gazeboNamespace+'/unpause_physics', std_srvs.srv.Empty)
            rospy.loginfo("Calling service %s/unpause_physics" % self.gazeboNamespace)
                        
            resp = unpause_physics(std_srvs.srv.EmptyRequest())
            return 
         except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def pausePhysics(self):
        try:
            pause_physics = rospy.ServiceProxy(self.gazeboNamespace+'/pause_physics', std_srvs.srv.Empty)
            rospy.loginfo("Calling service %s/pause_physics" % self.gazeboNamespace)
                        
            resp = pause_physics(std_srvs.srv.EmptyRequest())
            return 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def setRobotPosition(self, pose, quat):
        try:
            set_model_state = rospy.ServiceProxy(self.gazeboNamespace+'/set_model_state', SetModelState)
            rospy.loginfo("Calling service %s/set_model_state" % self.gazeboNamespace)
            st = ModelState()
            st.model_name = self.modelName
            st.pose.position.x = pose[0]
            st.pose.position.y = pose[1]
            st.pose.position.z = pose[2]
            st.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
            
            resp = set_model_state(st)
            rospy.loginfo("State set status: %s"%resp.status_message)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def onNewLinVel(self, newVal):
        msg = Twist()

        msg.angular.z = -self.angVelSlider.getValue()
        msg.linear.x = newVal
        self.twist_pub.publish(msg)
        
    def onNewAngVel(self, newVal):
        msg = Twist()

        msg.angular.z = -newVal
        msg.linear.x = self.linVelSlider.getValue()
        self.twist_pub.publish(msg)
        
    def btnDone_onclick(self):
        self.releaseKeyboard()
        
        if self.score is None:
            #Stop the robot
            msg = Twist()
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.twist_pub.publish(msg)
            self.finalTime = rospy.Time.now()
            self.pausePhysics()
            #Compute and display the score:
            #Score is the % of distance travelled between goal and end state
            distToGoal = self.l2dist((self.currentGoal[0], self.currentGoal[1]),
                                     (self.robot_state_latest[0], self.robot_state_latest[1]))
            totalDist = self.l2dist((self.currentGoal[0], self.currentGoal[1]),
                                     (self.startPos[0], self.startPos[1]))
            self.score = 1.0 - distToGoal / totalDist
            self.btnDone.setText('Score: %1.2f, press for next task' % self.score)
        else:
            #Clean up the goals and the traverses
            self.map_view.removeGoal('*')
            self.map_view.removeTraverse(0)
            self.btnDone.setText('Get Score')
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
            self.map_view.addGoal(goal[0], goal[1].x, goal[1].y)
            #            self._map_view.goals[itemKey] = goal[1]
            self.map_view.goalVisible[itemKey] = True

    def l2dist(self, src, dest):
        return math.sqrt((dest[0] - src[0])**2 + (dest[1] - src[1]) ** 2)
    
    def updateRobotState(self):
        self.map_view.updateRobotState(self.robot_state_latest)
        #process every message in the queue so far:
        #print 'State queue size:', len(self.stateQueue)
        while True:
            try:
                self._robotState = self.stateQueue.pop()
            except IndexError, e:
                return

            for idx, val in enumerate(self._robotState):
                self.poseLabels[idx].updateValue(val)

            #Update the goal ranging
            self.goalRangeLabels[0].updateValue(self.currentGoal[0] - self.robot_state_latest[0])
            self.goalRangeLabels[1].updateValue(self.currentGoal[1] - self.robot_state_latest[1])

            #Compute the atan - desired yaw to get to goal
            self.goalRangeLabels[2].updateValue('%1.1f' % (math.atan2(self.currentGoal[1] - self.robot_state_latest[1],
                                                                      self.currentGoal[0] - self.robot_state_latest[0])- self.robot_state_latest[3]))         
            #Add to the current traverse

            currentIndex = len(self.map_view.traverses.keys()) - 1
            if currentIndex >= 0:
                self.map_view.addTraversePoint(currentIndex, (self._robotState[0], self._robotState[1]))
                
            #Zoom in on the robot

            left = min(self._robotState[0], self.currentGoal[0])
            top = max(self._robotState[1], self.currentGoal[1])
            width = abs(self._robotState[0] - self.currentGoal[0])
            height = abs(self._robotState[1] - self.currentGoal[1])

           
            #print 'Raw window: l:', left, ' t:' , top, ' w:', width, ' h:', height
            #Translate to scene coords from gazebo coords
            margin = 50
            self.map_view.setSceneRect(self.map_view.w/2 + left - margin, self.map_view.h/2 - top - margin, width+margin*2, height+margin*2)
            #print 'Scene rect:', self.map_view.sceneRect()
            self.map_view.fitInView(self.map_view.sceneRect(), Qt.KeepAspectRatio)
            #print 'Set scene rect to:', self.sceneRect()
        
    def robot_state_cb(self, msg):
        #Resolve the odometry to a screen coordinate for display

        worldX = msg.pose.pose.position.x
        worldY = msg.pose.pose.position.y
        worldZ = msg.pose.pose.position.z

        #print 'WorldZ:', worldZ
        
        worldRoll, worldPitch, worldYaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                 msg.pose.pose.orientation.y,
                                                                 msg.pose.pose.orientation.z,
                                                                 msg.pose.pose.orientation.w],'sxyz')


        #TODO: Wrap the inv rpy to [-pi, pi]
        #print 'Orientation:', msg.pose.orientation, ' RPY:', worldRoll, worldPitch, worldYaw
        
        self.stateQueue.appendleft([worldX, worldY, worldZ, worldYaw])
        self.robot_state_latest = [worldX, worldY, worldZ, worldYaw]
        
        self.robot_state_changed.emit()

    def close(self):
        if self.dem_sub:
            self.dem_sub.unregister()
        if self.odom_sub:
            self.odom_sub.unregister()
            
    def mousePressEvent(self, evt):
        #print 'Sizes:', self.sizes()
        pass

    def keyPressEvent(self, evt):
        evt.accept()
        #print 'Got keypress:', evt.key()
        if evt.key() == Qt.Key_Up:
            #This fires events as needed and percolates through the system
            self.linVelSlider.triggerAction(QAbstractSlider.SliderSingleStepAdd)
        elif evt.key() == Qt.Key_Down:
            #This fires events as needed and percolates through the system
            self.linVelSlider.triggerAction(QAbstractSlider.SliderSingleStepSub)
        elif evt.key() == Qt.Key_Left:
            self.angVelSlider.triggerAction(QAbstractSlider.SliderSingleStepSub)
        elif evt.key() == Qt.Key_Right:
            self.angVelSlider.triggerAction(QAbstractSlider.SliderSingleStepAdd)
            
class TerrainView(QGraphicsView):
    terrain_changed = Signal()

    def __init__(self, terrain_topic='terrain',
                 tf=None, parent=None):
        super(TerrainView, self).__init__()
        self._parent = parent
        self.demDownsample = 1
        #self.dem_changed.connect(self._update)
        
        
        self._dem_item = None
        self._robotIcon = None
        
        self.setDragMode(QGraphicsView.NoDrag)

        self._addedItems = dict()
        self.w = 2000
        self.h = 2000

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
        self.traverses[0] = []
        self.traverseVisible[0] = True
        
        self.robotLocation = [0,0,0]
        self.goalIcons = dict()
        self.goalVisible = dict()
        
        self.setAcceptDrops(True)

        self.initialPosition = (3725, 2400)
        
        #Overlay the hazmap now that the dem is loaded
        self.terrain_changed.connect(self.updateTerrain)
        self.terrain_sub = rospy.Subscriber(terrain_topic, Image, self.terrain_cb)

        
    def l2dist(self, src, dest):
        return math.sqrt((dest[0] - src[0])**2 + (dest[1] - src[1]) ** 2)

    def removeTraverse(self, index):
        #Remove all of the graphics items associated with this traverse:
        for item in self.traverses[index]:
            self.scene().removeItem(item)

        #Unref the GraphicsItems
        self.traverses[index] = []
        
    def addTraversePoint(self, index, point):
        theColor = QColor(self._colors[index][0], self._colors[index][1], self._colors[index][2])
        theColor.setAlpha(80)
        thePen = QPen(theColor)
        thePen.setWidthF(5)

        theBrush = QBrush(theColor)
        theBrush.setStyle(Qt.SolidPattern)
        
        #if self.traverse_item is None:
        #    return

        point_scaled = [point[0] / self.demDownsample, point[1] / self.demDownsample]

        point_scaled[0] += self.w/2
        point_scaled[1] = self.h/2 - point_scaled[1] 
        #print 'Adding traverse point:', point_scaled
        
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
            thisGoal.setFont(QFont("SansSerif", max(self.h / 30.0,3), QFont.Bold))
            thisGoal.setBrush(QBrush(QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2])))          
            self.goalIcons[id] = thisGoal
            #self._scene.addItem(thisGoal)
            #Update the label's text:
            self.goalIcons[id].setText(str(id))

            
            #Pick up the world coordinates
            world = [x, y]
            #print 'Goal raw coord:', world

            world[0] /= self.demDownsample
            world[1] /= self.demDownsample

            #Move the coords - goals are expressed in Gazebo coords
            world[0] += self.w/2
            world[1] = self.h/2 - world[1] #invert the y
            
            #Adjust the world coords so that the icon is centered on the goal
            #Since the font may change, we adjust this later
            
            iconBounds = self.goalIcons[id].boundingRect()
            #print 'Goal bounds:', iconBounds
            world[0] = world[0] - 24 #magic number determined by rendering the bounding box and looking at click data 
            world[1] = world[1] - 35 #ditto
            
            #print 'Drawing goal ', id, ' at ', world
            self.goalIcons[id].setPos(QPointF(world[0], world[1]))
            self.goalIcons[id].setZValue(100)
            self.scene().addItem(self.goalIcons[id])
            self.goalVisible[id] = True

    def removeGoal(self, id):
        self.scene().removeItem(self.goalIcons[id])
        del self.goalIcons[id]
        
    def updateRobotState(self, robotLocation):
        #Redraw the robot locations
        #print 'Updating robot location:', robotLocation
        
        #If this is the first time we've seen this robot, create its icon
        if self._robotIcon == None:
            thisRobot = QArrow.QArrow(color=QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2]))
            #print 'Robot icon:', thisRobot
            
            self._robotIcon = thisRobot
            self._scene.addItem(thisRobot)
            
        #Pick up the world coordinates - copy so we can change in place
        world = list(copy.deepcopy(robotLocation))
        #print 'Raw robot loc: ', world[0], world[1]

        #Translate to the center:
        world[0] += self.w/2
        world[1] = self.h/2 - world[1] #invert y coord
        iconBounds = self._robotIcon.boundingRect()

        #world[0] /= self.demDownsample
        #world[1] /= self.demDownsample

        #Updated the QArrow so that its natural origin is at its origin
        #Adjust the world coords so that the icon is centered on the robot, rather than top-left
        #world[0] = world[0] - iconBounds.width()/2 
        #world[1] = world[1] - iconBounds.height()/2 #mirror the y coord

        #print 'Drawing robot at ', world[0], world[1]
        
        self._robotIcon.setPos(QPointF(world[0], world[1]))
        
        #Set the origin so that the caret rotates around its center(ish)
        #self._robotIcon.setTransformOriginPoint(QPoint(iconBounds.width()/2, iconBounds.height()/2))

        #print 'Rotating:', world[5]
        self._robotIcon.setRotation(-world[3]*180/math.pi + 90)
        
       
        #print 'Transform origin:', self._robotIcon.transformOriginPoint()
        self._robotIcon.setZValue(100)


    def terrain_cb(self, msg):
        #Unlike the dem, the terrain is pretty standard - rgb8 image
        terrain = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.w = msg.width
        self.h = msg.height
        self.terrainImage = QImage(terrain, msg.width, msg.height, QImage.Format_RGB888)
        self.terrain_changed.emit()

    def updateTerrain(self):
        print 'Rendering terrain'

        self.terrainItem = self._scene.addPixmap(QPixmap.fromImage(self.terrainImage))
        self.terrainItem.setPos(QPointF(0, 0))
        self.setSceneRect(0, 0, self.terrainItem.pixmap().width(), self.terrainItem.pixmap().width())
        self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
        trans = QTransform()
        #print 'Translating by:', bounds.width()
        
        #trans.scale(self.w/hazTrans.width(),self.h/hazTrans.height())
        #trans.translate(0, -bounds.height())
        #self.terrainItem.setTransform(trans)
        
        # Everything must be mirrored
        #self._mirror(self.hazmapItem)
 

    def close(self):
        super().close()
            
    def mousePressEvent(self, e):
        scenePt = self.mapToScene(e.pos())
        print 'Click at:', scenePt.x(), ' ', scenePt.y()

    def keyPressEvent(self, evt):
        evt.ignore()
        
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
        
    def mirror(self, item):
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
        

        
    def image_cb(self, msg):
        self._image = QPixmap.fromImage(QImage(msg.data, msg.width, msg.height, QImage.Format_RGB888))
        self.w = msg.width
        self.h = msg.height
        self.image_changed.emit()
        
    def _updateImage(self):
        #Refresh the pixmap instead of removing/adding the image...
        #print 'Updating camera view'
        if self._imageItem is None:
            self._imageItem = self.scene().addPixmap(self._image)
         #self._image.height/2)
        self._imageItem.setPixmap(self._image)
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

    def keyPressEvent(self, evt):
        evt.ignore()
        
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
