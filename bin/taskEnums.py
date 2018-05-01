#!/usr/bin/python2

from enum import Enum

class TaskType(Enum):
    Navigation = 0
    Grasping = 1
    Handling = 2
    
class TaskDifficulty(Enum):
    Easy = 0
    Medium = 1
    Hard = 2
    
