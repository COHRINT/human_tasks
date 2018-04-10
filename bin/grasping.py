#!/usr/bin/env python2

import sys
import csv
import math
import rospy
import signal
import time

import numpy as np


from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from QDirection import *
from QCrosshair import *
from QGripper import *
from QPolygonGenerator import *

from human_tasks.msg import *

#Main window area: Contains a QGraphicsView and some buttons
class GraspingWidget(QWidget):
    def __init__(self, srcObject):
        super(QWidget, self).__init__()
        self.srcPolygon = srcObject
        self.initUI()
       
        
    def initUI(self):
        rospy.init_node('grasping')
        self.setWindowTitle('Grasping')
        mainLayout = QVBoxLayout()
        self.graspView = GraspView(parent = self, obj=self.srcPolygon)
        self.w = 500
        self.h = 500
        
        mainLayout.addWidget(self.graspView)
        self.btnDone = QPushButton('Done')
        self.btnDone.clicked.connect(self.btnDone_onclick)
        self.btnDone.keyPressEvent = self.btnDone_keyPress
        
        mainLayout.addWidget(self.btnDone)

        self.setLayout(mainLayout)
        
        self.score = 0.0
        self.finalTime = None
        
    def btnDone_keyPress(self, evt):
        if evt.key() == Qt.Key_Return:
            self.btnDone_onclick()
        
    def btnDone_onclick(self):
        if self.finalTime is None:
            self.runScoring()
        else:
            self.parent().close()
        
    def runScoring(self):
        self.finalTime = rospy.Time.now()
        self.score = self.graspView.getScore()
        self.graspView.setDisabled(True)
        self.parent().statusBar().showMessage('Earned score: %d%%' % (self.score*100))
        self.btnDone.setFocus(Qt.TabFocusReason)
        
class GraspView(QGraphicsView):
    def __init__(self, parent=None, obj=None):
        super(GraspView, self).__init__(parent=parent)
        self._parent = parent

        self.setScene(QGraphicsScene())
        self._scene = self.scene()
        self.w = 500
        self.h = 500

        #Add the robot arm (nominal) to the scene
        self.robotArmLength = 600
        self.robotArm = QGraphicsRectItem(0,0,40, self.robotArmLength)
        self.robotArm.setBrush(QBrush(Qt.gray))
        self.robotArm.setTransformOriginPoint(QPointF(20,self.robotArmLength/2))
        
        self.robotArm.setRotation(-45)
        self._scene.addItem(self.robotArm)

        
        
        self.gripper = QGripper()
        self._scene.addItem(self.gripper)
        scale = 1
        self.gripper.setScale(scale)
        bounds = self.gripper.boundingRect()
        print 'Bounds:', bounds

        #Since the center of the gripper isn't the center of the bounding box, this needs some help...
        #curPos = self.gripper.getPivot()
        gripperPos = QPointF(self.w/2, self.h/3)
        self.gripper.setPos(gripperPos)

        #Join the arm to the gripper:
        armEndpoint = self.robotArm.mapToScene(QPointF(self.robotArm.boundingRect().width()/2, self.robotArm.boundingRect().height()))
        offset = gripperPos - armEndpoint
        self.robotArm.setPos(self.robotArm.pos() + offset)
        

        #Draw in the ground..
        self.groundPath = QPainterPath()
        self.groundPath.moveTo(-1000, 0)
        self.groundPath.cubicTo(QPointF(-100,40), QPointF(100,-40), QPointF(1000, 0))
        self.groundPath.lineTo(QPointF(1000, 100))
        self.groundPath.lineTo(QPointF(-1000, 100))
        
        self.groundItem = QGraphicsPathItem(self.groundPath)
        self._scene.addItem(self.groundItem)
        self.groundItem.setPos(QPointF(self.w/2, 470))
        groundPen = QPen(Qt.darkRed)
        groundBrush = QBrush(Qt.darkRed)
        self.groundItem.setPen(groundPen)
        self.groundItem.setBrush(groundBrush)

        #self.fitInView(0,0,self.w, self.h)

        #Add an object to grasp:
        self.srcPolygon = obj
        self.addObjectPolygon(self.srcPolygon)
        self.isectItem = None
        self.polyCount = 0
        self.gripAreaItem = None
        
        #Draw some directions:
        directions = QGraphicsSimpleTextItem('Use arrow keys to move\nPgUp/PgDown to rotate\nPress spacebar to grab')
        directions.setFont(QFont("SansSerif", max(self.h / 60.0,3), QFont.Bold))
        self._scene.addItem(directions)
        dirBounds = directions.boundingRect()
        directions.setPos(QPointF(self.w - dirBounds.width(), 0.0))
        
    def addObjectPolygon(self, obj):
        self.gen = QPolygonGenerator()
        objs = self.gen.getRandomPolygons(1, 10)

        if not obj is None:
            self.objItem = QGraphicsPolygonItem(obj)
        else:
            self.objItem = QGraphicsPolygonItem(objs[0])
        for index in range(0, self.objItem.polygon().count()):
            print 'x:', self.objItem.polygon().value(index).x(), ' y:', self.objItem.polygon().value(index).y()

        self.objItem.setFillRule(Qt.WindingFill)
        self.objItem.setBrush(Qt.green)
        objPen = QPen()
        objPen.setWidth(0)
        objPen.setColor(Qt.black)
        objPen.setJoinStyle(Qt.MiterJoin)
        self.objItem.setPen(objPen)
        self.objItem.setScale(4)
        self._scene.addItem(self.objItem)

        #Gradually increase the y coordinate until collision:
        #todo: add a rotation such that the object is at least sitting on a face instead of a vertex...
        
        self.objItem.setPos(self.w/2, self.h/2)

        curPos = self.objItem.pos()
        inCollision = False
        while not self.objItem.collidesWithItem(self.groundItem):
            curPos.setY(curPos.y() + 1)
            self.objItem.setPos(curPos)

        #There is at least one point in contact - find it:
        #print 'Finding contact point'
        
        srcPoly = self.objItem.polygon()
        foundIndex = None
        curPos = self.objItem.pos()
        while foundIndex is None:
            for j in range(0, srcPoly.count()):
                vertex = srcPoly.value(j)
                #print 'Checking vertex:', vertex
                if self.groundItem.contains(self.groundItem.mapFromItem(self.objItem, vertex)):
                    foundIndex = j
                    break
            #print 'lowering object'
            curPos.setY(curPos.y() + 1)
            self.objItem.setPos(curPos)

        #print 'Contact with vertex ', foundIndex
        #print 'Contains?:', self.groundItem.contains(self.groundItem.mapFromItem(self.objItem, vertex))
        
        #Rotate and check vertex (foundIndex + 1) % count until it's in contact as well (guaranteed to be ccw from hull algorithm)
        self.objItem.setTransformOriginPoint(srcPoly.value(foundIndex))

        foundContact = False
        nextVertIndex = (foundIndex+1)% srcPoly.count()
        nextVert = srcPoly.value(nextVertIndex)
        while not foundContact:
            #print 'Checking rotation:', self.objItem.rotation()
            if self.groundItem.contains(self.groundItem.mapFromItem(self.objItem, nextVert)):
                break
            
            self.objItem.setRotation(self.objItem.rotation() - 1)
            while not self.groundItem.contains(self.groundItem.mapFromItem(self.objItem, vertex)):
                #print 'Gluing object back to ground'
                curPos = self.objItem.pos()
                curPos.setY(curPos.y() + 1)
                self.objItem.setPos(curPos)
                
        curPos = self.objItem.pos()  
        curPos.setY(curPos.y() - 1) #back off one step
        self.objItem.setPos(curPos)

    def getMaxChord(self, target):
        #use a n^2 algorithm to compute the distances bewteen every vertex and then find the max
        polygon = target
        chords = np.zeros((polygon.count(), polygon.count()))
        #Convention: chord[from][to]
        for src in range(0, polygon.count()):
            for dest in range(0, src):
                chords[src][dest] = np.linalg.norm(np.array((polygon.value(src).x(), polygon.value(src).y()))-
                                                   np.array((polygon.value(dest).x(), polygon.value(dest).y())))


        #print 'Chords:', chords
        maxChord = np.amax(chords)
        print 'Max chord:', maxChord
        return maxChord

    
    def l2dist(self, src, dest):
        return math.sqrt((dest.y() - src.y())**2 + (dest.x() - src.x()) ** 2)
    
    def reducePolygon(self, target):
        #ToFill seems to munge the corners, giving a bit of a chamfer
        #Make a new polygon with duplicate vertexes removed
        dest = QPolygonF()
        lastPoint = QPointF()
        for index in range(0, target.count()):
            foundDupe = False
            for other in range(index+1, target.count()):
                if self.l2dist(target.value(other), target.value(index)) < 1e-4:
                    foundDupe = True
                    break
            if not foundDupe:
                print 'Adding vertex:', index, ':', target.value(index)
                dest.append(target.value(index))
            #lastPoint = target.value(index)

        #The last point is a duplicate of the first, closing the path...
        dest.prepend(dest.value(dest.count()-1)) 
        return dest
    
    def getEdgeAngles(self, target):
        angles = np.zeros((target.count()-1, 1))
        index = 0
        for src in range(0, target.count()-1):
            dest = src + 1
            angles[index] = math.pi - math.atan2(target.value(dest).y() - target.value(src).y(),
                                                 target.value(dest).x() - target.value(src).x())
            index += 1
        return angles
    
    def printPolygon(self, poly):
        for index in range(0, poly.count()):
            print 'x:', poly.value(index).x(), ' y:', poly.value(index).y()
        
    def getScore(self):
        maxChord = self.getMaxChord(self.objItem.mapToScene(self.objItem.polygon()))

        #Implemented from https://stackoverflow.com/questions/8837109/qpolygonf-qlinef-intersection
        gripLine_l = QPainterPath()
        leftJaw = self.gripper.getLeftJaw()
        rightJaw = self.gripper.getRightJaw()

        #order is inside, outside
        leftJaw_l = (self.gripper.mapToScene(leftJaw[0]),
                     self.gripper.mapToScene(leftJaw[1]))
        rightJaw_l = (self.gripper.mapToScene(rightJaw[0]),
                     self.gripper.mapToScene(rightJaw[1]))

        print 'Left jaw:', leftJaw_l
        
        gripLine_l.moveTo(leftJaw_l[0])
        gripLine_l.lineTo(leftJaw_l[1])
        gripLine_l.lineTo(rightJaw_l[1])
        gripLine_l.lineTo(rightJaw_l[0])
        gripLine_l.lineTo(leftJaw_l[0])
        
        #object painter path:

        objPath = QPainterPath()
        obj_poly = self.objItem.mapToScene(self.objItem.polygon())
        objPath.addPolygon(obj_poly) #polygon in scene coords
        
        #isect_path = self.objItem.mapToScene(self.objItem.shape()).intersected(gripLine_l)

        #intersect in scene coords
        isect_path = objPath.intersected(gripLine_l)
        isect_path.setFillRule(Qt.WindingFill)
        isect_all = isect_path.toFillPolygon(QTransform())

        #Map back to the gripper's coord sys:
        isect_all = self.gripper.mapFromScene(isect_all)
        
        #check # vertices > 0
        if isect_all.count() == 0:
           
            print 'No intersection!'
            return 0.0
        
       
        print 'Starting with:', len(isect_all)
        self.printPolygon(isect_all)
        
        isect = self.reducePolygon(isect_all)
        print 'After reduction:'
        self.printPolygon(isect)
        
        #print 'Max chord of intersection:', self.getMaxChord(isect) #already in scene coords

        gripLine = QPainterPath()
        gripLine.moveTo(leftJaw_l[1])
        gripLine.lineTo(rightJaw_l[1])

        isect_gripper = self.gripper.mapToScene(isect)
        
  
        print 'Isect gripper:'
        self.printPolygon(isect_gripper)
        
        self.isectItem = self._scene.addPolygon(self.gripper.mapToScene(isect), QPen(), QBrush(Qt.darkGreen))
        print 'Isect vis:', self.isectItem.isVisible()
        print 'Isect pos:', self.isectItem.pos().x(), ',', self.isectItem.pos().y()
       
        #self.isectItem.setPos(self.w/2, self.h/2)
        
        gripAreaColor = QColor(Qt.blue)
        gripAreaColor.setAlpha(50)
        gripAreaBrush = QBrush(gripAreaColor)
        self.gripAreaItem = self._scene.addPath(gripLine_l, QPen(), gripAreaBrush)
        
        #Put the isect item into jaw coords
        #Find the two line segments that are most parallel to the gripper's jaws

        print 'Gripper rotation:', self.gripper.rotation()
        #gripAngle = (self.gripper.rotation()*math.pi/180 + math.pi/2) % math.pi #since opposites are fine here

        #Since we're in gripper-local coords now:
        gripAngle = math.pi/2
        
        print 'Grip angle:', gripAngle
        print 'Vertex count:', isect.count()
        
        isectAngles = self.getEdgeAngles(isect)
        print 'Isect angles:', isectAngles
   
        #find the two with the smallest absolute difference from the gripper angle
        angleDiffs = np.abs(isectAngles.ravel() - gripAngle)

        print 'Raw diffs:', angleDiffs
        
        for index in range(0, len(isectAngles)):
            if angleDiffs[index] > math.pi/2:
                angleDiffs[index] = np.abs(math.pi  - angleDiffs[index]) 
        print 'Corrected diffs:', angleDiffs
        
        angleDiffs_ordered = np.sort(angleDiffs)

        print 'ordered angle diffs:', angleDiffs_ordered

        #Worst score is if the lowest two differences are pi/2, best is zero difference:
        score = (math.pi - angleDiffs_ordered[0] - angleDiffs_ordered[1])/math.pi


        return score
                        
    def resizeEvent(self, evt=None):
        #Resize map to fill window
        scale = 1
        bounds = self._scene.sceneRect()
        if bounds:
            self._scene.setSceneRect(0, 0, self.w, self.h)
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
            self.show()
            
    def keyPressEvent(self, event):
        currentPos = self.gripper.pos()
        currentRot = self.gripper.rotation()
        
        keyMove = 2
        keyRotate = 1

        candidateRot = self.gripper.rotation()
        candidatePos = self.gripper.pos()
        
        if event.key() == Qt.Key_PageUp:
            candidateRot -= keyRotate
        elif event.key() == Qt.Key_PageDown:
            candidateRot += keyRotate
        elif event.key() == Qt.Key_Up:
            candidatePos.setY(candidatePos.y() - keyMove)
        elif event.key() == Qt.Key_Down:
            candidatePos.setY(candidatePos.y() + keyMove)
        elif event.key() == Qt.Key_Left:
            candidatePos.setX(candidatePos.x() - keyMove)
        elif event.key() == Qt.Key_Right:
            candidatePos.setX(candidatePos.x() + keyMove)
        elif event.key() == Qt.Key_Space:
            self.parent().runScoring()
        elif event.key() == Qt.Key_Return:
            self.parent().runScoring()
            
            #Ignore other keys...

        #Check for collisions:
        self.gripper.setRotation(candidateRot)
        self.gripper.setPos(candidatePos)
        
        collisions = self.gripper.collidingItems()
        if len(collisions) > 1:
            print 'Collisions:'
            print collisions
            self.gripper.setRotation(currentRot)
            self.gripper.setPos(currentPos)

        #Join the arm to the gripper:
        armEndpoint = self.robotArm.mapToScene(QPointF(self.robotArm.boundingRect().width()/2, self.robotArm.boundingRect().height()))
        offset = self.gripper.pos() - armEndpoint
        self.robotArm.setPos(self.robotArm.pos() + offset)
        
    def mousePressEvent(self, event):
        #implement a mousing interface - translate and rotate
        sceneCoords = self.mapToScene(event.pos())
        print 'Button:', event.button()
        
        
        if event.button() == Qt.LeftButton:
            self._scene.removeItem(self.objItem)
            self.addObjectPolygon(self.srcPolygon)
            self.gripper.setPos(QPointF(self.w/2, self.h/8))
            #Join the arm to the gripper:
            armEndpoint = self.robotArm.mapToScene(QPointF(self.robotArm.boundingRect().width()/2, self.robotArm.boundingRect().height()))
            offset = self.gripper.pos() - armEndpoint
            self.robotArm.setPos(self.robotArm.pos() + offset)
            if not self.isectItem is None:
                self._scene.removeItem(self.isectItem)
            if not self.gripAreaItem is None:
                self._scene.removeItem(self.gripAreaItem)
                
        elif event.button() == Qt.RightButton:
            '''
            print 'Polygon:',
            poly = self.objItem.polygon()
            for j in range(0, poly.count()):
                print '(', poly.value(j).x(), ',', poly.value(j).y(), ') ',
            '''
            print 'Score:', self.getScore()
        elif event.button() == 8: #the little button closest to the battery indicator on my trackball
            #Save the polygon to a file to be loaded later
            self.polyCount += 1
            print 'Polygon count:', self.polyCount
            polyFile = open('polygons.csv', 'a')

            polygon = self.objItem.polygon()
            polyFile.write('%d' % polygon.count())
            for index in range(0, polygon.count()):
                polyFile.write(',%1.2f, %1.2f' % (polygon.value(index).x(), polygon.value(index).y()))
            polyFile.write('\n')
            polyFile.close()
            
def loadObjects(fileName):
    #Load a set of polygons from a csv file
    csvFile = open(fileName, 'r')
    reader = csv.reader(csvFile, delimiter=',')
    objects = []
    for row in reader:
        dest = QPolygonF()
        for vertIndex in range(0, int(row[0])):
            dest.append(QPointF(float(row[1+2*vertIndex]), float(row[1+2*vertIndex+1])))
        objects.append(dest)
    return objects


def main():
    
        if len(sys.argv) > 1 and len(sys.argv) < 3:
                print 'Usage:', sys.argv[0], ' <objects.csv> objectIndex'
                print 'Please provide an objects file and an objectIndex'
                print 'Or leave blank to use a random polygon'
                sys.exit(-1)
	app = QApplication(sys.argv)
        if len(sys.argv) > 1:
            objectsFile = sys.argv[1]
            objectIndex = int(sys.argv[2])
            
            objects = loadObjects(objectsFile)
            if objectIndex > len(objects):
                print 'Specified index:', objectIndex, ' exceeds objects defined in: ', objectsFile
                sys.exit(-1)
            graspApp = GraspingWidget(objects[objectIndex])
        else:
            graspApp = GraspingWidget(None)    

	

        mainWindow = QMainWindow()
        mainWindow.setWindowTitle('Grasping')
        mainWindow.setCentralWidget(graspApp)
        mainWindow.resize(graspApp.w, graspApp.h)
        mainWindow.setStatusBar(QStatusBar())
        graspPub = rospy.Publisher('grasping', Grasping, queue_size=10)
        
        startTime = rospy.Time.now()
        mainWindow.show()
        app.exec_()
        
        
        totalTime = graspApp.finalTime - startTime #a Duration
        print 'Publishing score:', graspApp.score, ' duration:', totalTime.to_sec(), ' sec'

        msg = Grasping()
        msg.header.stamp = rospy.Time.now()
        
        msg.score = graspApp.score
        msg.duration = totalTime.to_sec()
        msg.objectIndex = objectIndex
        
        graspPub.publish(msg)
        
        
	sys.exit(0)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
