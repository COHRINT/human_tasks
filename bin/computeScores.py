#!/usr/bin/python2
'''
Compute the score from a given data bag
Attention: %correct classification / #messages
Workload: 1 - (low + high) / total
tasks: Count achieved out of baseline (full?)

'''
import rosbag
import rospy

import sys
from human_tasks.msg import *
from rqt_py_common.topic_helpers import get_field_type
import pdb
from attention_task import CommCategory

topics = ['/experiment_ui/attention',
          '/control_node/workload_task',
          '/control_node/grasp_task',
          '/control_node/nav_task',
          '/control_node/sample_task']
def accepted_topic(topic):
    msg_types = [OccupancyGrid, Path, PolygonStamped, PointStamped]
    msg_type, array = get_field_type(topic)

    if not array and msg_type in msg_types:
        return True
    else:
        return False
    

    
class WorkloadScore(object):
    def __init__(self):
        self.totalTicks = 0
        self.lowTicks = 0
        self.highTicks = 0
        self.msgCount = 0
        
    def addMessage(self, msg):
        self.msgCount += 1
        self.totalTicks += msg.totalTicks
        self.lowTicks += msg.lowTicks
        self.highTicks += msg.highTicks

    def getScore(self):
        if self.msgCount > 0:
            return float(1 - (self.lowTicks + self.highTicks) / float(self.totalTicks))
        else:
            return 0.0

class AttentionScore(object):
    def __init__(self):
        self.correct = 0
        self.incorrect = 0
        self.totalTime = 0.0
        self.msgCount = 0
        
    def addMessage(self, msg):
        category = CommCategory(msg.category)
        
        if 'EVA' in msg.msgSrc and category == CommCategory.High:
            self.correct += 1
        elif 'EVA' in msg.msgSrc and category == CommCategory.Low:
            self.incorrect += 1
        elif 'EVA' not in msg.msgSrc and category == CommCategory.High:
            self.incorrect += 1
        elif 'EVA' not in msg.msgSrc and category == CommCategory.Low:
            self.correct += 1
        else:
            print 'Problem with attention message:', msgSrc, ' category:', msg.category

        self.totalTime += msg.spent.to_sec()
        self.msgCount += 1
        
    def getScore(self):
        if self.msgCount > 0:
            score = 0.5 * self.correct / self.msgCount + 0.5*1.0 #for now, change to spent / reference time
        else:
            score = 0.0

        return score
    
def main():
    if len(sys.argv) < 2:
        print 'Usage:', sys.argv[0], ' <bagfile>'
        sys.exit(-1)
    bag = rosbag.Bag(sys.argv[1])
    workload = WorkloadScore()
    attention = AttentionScore()

    graspCount = 0
    navCount = 0
    sampleCount = 0
    
    for topic, msg, t in bag.read_messages(topics=topics):
        #msg_type, array = get_field_type(topic)
        #pdb.set_trace()

        #Clumsy, but effective...
        #print 'Message type:', str(type(msg)).split('__')[1].split("'")[0]

        msgType = str(type(msg)).split('__')[1].split("'")[0]
        if msgType == 'AttentionTask':
            #print 'found attention'
            attention.addMessage(msg)
            
        elif msgType == 'Workload':
            #print 'Adding workload'
            workload.addMessage(msg)
        elif msgType == 'Grasping':
            graspCount += 1
        elif msgType == 'Navigation':
            navCount += 1
        elif msgType == 'SampleHandling':
            sampleCount += 1
        else:
            print 'Unknown msg type:', msgType
            
    bag.close()

    totalCount = graspCount + navCount + sampleCount
    print 'Workload score:', workload.getScore(), ' total events:', workload.msgCount
    print 'Attention score:', attention.getScore(), ' total events:', attention.msgCount
    print 'Total tasks:', totalCount
    
if __name__ == '__main__':
    main()

