#!/usr/bin/env python
#8/2/22

# import pandas as pd
import random
import time
import socket
import sys
import numpy as np

import rospy
import math
import actionlib
import local_plan # imports the Movement class from the movement.py file



# Just here for imports in case the Movement class variables need to see it in this file
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from math import atan2,degrees





#######################################
# CLASS NAME: Initiator

#
#######################################
class RelativeDirection:
    def __init__(self,offsets,x_ref,y_ref,direction):
        # Where is the robot relative to the global 0,0?
        # (Each robot thinks its starting pose is 0,0)
        self.offset_x = offsets[0]
        self.offset_y = offsets[1]

        self.x_ref = x_ref
        self.y_ref = y_ref

        self.direction = direction

    def away(self):
        #        ^
        #        |
        #       +x
        # <- +y
        #move to -y and -x
        goal_x = -self.x_ref + self.offset_x
        goal_y = -self.y_ref + self.offset_y

        awayGoal = [goal_x,goal_y]

        return awayGoal


    def towards(self):
        #        ^
        #        |
        #       +x
        # <- +y
        goal_x = self.x_ref + self.offset_x
        goal_y = self.y_ref + self.offset_y

        towardsGoal = [goal_x,goal_y]

        return towardsGoal

    def get_rel_dir_goal(self):
        if direction == 'away':
            rel_dir_goal = self.away()
        else:
            rel_dir_goal = self.towards()

        return rel_dir_goal

class Proximity:

    def __init__(self, proximity,goal):
        # Where is the robot relative to the global 0,0?
        # (Each robot thinks its starting pose is 0,0)
        self.proximity = proximity
        self.goal_x = goal[0]
        self.goal_y = goal[1]

    def get_prox_goal(self):






class Initiator:
    def __init__(self,mover,rob_id,num_rob):
        self.mover = mover

        self.rob_id = 2 # all robots will be from 0 - num robots
        self.num_robots = 4


    def reset_to_home(self):
        goal = Point()
        goal.x = 0
        goal.y = 0

        rospy.loginfo("Going home")
        self.mover.return_to_starting_pos(goal)
        self.mover.final_formation_orientation(0)







if __name__ == '__main__':
    try:
        rospy.init_node('please_work',anonymous=True)
        mover = local_plan.Movement()

        initiator = Initiator(mover)


        l_speed = float(input('What do you want the forward speed to be. Default to 0.6: '))
        r_speed = float(input('What do you want the rotational speed to be? Default to 0.25: '))

        # Set robot offsets relative to global 0,0
        offsets = (0,0)

        # THE REFERENCE POINT IS RELATIVE TO ROBOT 0, ROBOT 0 IS CONSIDERED 0,0
        ref_point_input = input('Where would you like the robot to go? Ex. 3,3: ')
        ref_point = ref_point_input

        x_ref = ref_point[0]
        y_ref = ref_point[1]


        ready=raw_input('Go? y/n').lower()

        if ready == 'y':
            # Layer relative direction parameter
            direction = raw_input('Relative Direction? (towards/away)').lower()
            rel_dir = RelativeDirection(offsets,x_ref,y_ref,direction)
            rel_dir_goals = rel_dir.get_rel_dir_goal()

            mover.move_to_goal_avoidance(rel_dir_goals)

            # # Layer proximity parameter
            # prox = raw_input('Proximity? (y/n)').lower()
            # ('(in meters)').lower()
            # proximity = Proximity(rel_dir_goals)

            # Layer geometry parameter


        else:
            pass


        again = raw_input('Another waypoint? (y/n): ').lower()



        while again == 'y':
            direction = raw_input('Relative Direction? (towards/away)').lower()
            rel_dir = RelativeDirection(offsets, x_ref, y_ref, direction)
            rel_dir_goals = rel_dir.get_rel_dir_goal()

            mover.move_to_goal_avoidance(rel_dir_goals)

            again = raw_input('Another waypoint? (y/n): ').lower()

        home = raw_input("return home? y/n")
        if home == 'y':
            mover.return_to_starting_pos()
        else:
            pass


    except rospy.ROSInterruptException:
        rospy.loginfo("Didn't work, so cry")