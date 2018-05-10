#!/usr/bin/python2
'''
Run a node to run ls -l and report the contents as a string[] via ROS node 
'''


import inotify.adapters
import subprocess
import rospy
import sys
import time
from human_tasks.msg import *
import os

import signal
import pdb

class FileMonitor(object):
    def __init__(self, directory, mask):
        #Monitor the directory for changes
        self.monPub = rospy.Publisher('file_updates', FileWatcher, queue_size = 10, latch=True)


        self.i = inotify.adapters.Inotify()
        self.dirToWatch = os.path.join(os.path.expanduser('~'), directory)
        
        print 'Watching ', self.dirToWatch, ' for ROS bags'
        self.i.add_watch(self.dirToWatch)
        self.mask = mask
        self.shouldQuit = False
        signal.signal(signal.SIGUSR1, self.onSigUSR1)
        
    def onSigUSR1(self, signum, frame):
        self.shouldQuit = True
        print 'Posted sigusr1'
        
    def postFileMessage(self, fileName, fileSize):
        msg = FileWatcher()
        msg.header.stamp = rospy.Time.now()
        msg.fileName = fileName.encode('ascii', 'ignore')
        msg.fileSize = int(fileSize)

        self.monPub.publish(msg)
        
    def start(self):
        try:
            while not rospy.is_shutdown():
                #time.sleep(1000)
                #print 'Checking files...'
                events = self.i.event_gen(timeout_s=1)
                evtList = list(events)
                
                for event in evtList:
                    if event is not None:
                        (header, type_names, watch_path, filename) = event
                        #_LOGGER.info("WD=(%d) MASK=(%d) COOKIE=(%d) LEN=(%d) MASK->NAMES=%s "
                        #             "WATCH-PATH=[%s] FILENAME=[%s]",
                        #             header.wd, header.mask, header.cookie, header.len, type_names,
                        #             watch_path.decode('utf-8'), filename.decode('utf-8'))
                        #print 'Header:', header, ' Type:', type_names, ' Filename:', filename.decode('utf-8')
                        #if 'IN_MODIFY' in type_names and '.bag' in filename:
                        if True:
                            statInfo = os.stat(os.path.join(self.dirToWatch, filename))
                            fileSize = statInfo.st_size
                            self.postFileMessage(filename, fileSize)

        finally:
            self.i.remove_watch(self.dirToWatch)
            
def main():
    if len(sys.argv) < 2:
        print 'Usage: ', sys.argv[0], ' <directory to monitor>'
        sys.exit(0)
        
    rospy.init_node('filemonitor_node')
    
    mon = FileMonitor(sys.argv[1], '*.bag*')
    mon.start()
    rospy.spin()
    
    

        
if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
	pass
