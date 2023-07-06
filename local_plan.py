#!/usr/bin/env python

import rospy
import math
import actionlib
import socket


from std_msgs.msg import String
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan

from math import atan2



#msg_pub = rospy.Publisher('msgTest', String, queue_size=10)
#move_pub = rospy.Publisher('moveTest', MoveBaseGoal, queue_size=10)

# rospy.init_node('move_bot', anonymous=True)
# vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# rate = rospy.Rate(10)
# move = Twist()


class Movement:
    def __init__(self):
        # self.goal = Point()
        # self.goal.x = 0
        # self.goal_y = 0

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.theta = 0.0

        self.delta = 0.1

        self.rot_speed = 0.3
        self.forward_speed = 0.6
        # rospy.init_node("speed_controller")

        # create publisher and subscriber
        self.sub = rospy.Subscriber("/odom", Odometry,
                                    self.newOdom)  # our odometry node is called /odom and not /odometry/filtered
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber("/base_scan", LaserScan, self.newLaserScan)

        # create a move variable
        self.move = Twist()

        # set the rate
        self.r = rospy.Rate(4)

        self.moveScan = {}

        # def set_goal_point(self, goal_x, goal_y):
        #     self.goal.x = goal_x
        #     self.goal.y = goal_y

    def newOdom(self, msg):
        # get the current x and y position values for the robot
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        # rospy.loginfo("Current x-coordinate: " + str(self.cur_x))
        # rospy.loginfo("Current y-coordinate: " + str(self.cur_y))
        # rospy.loginfo("Current theta value: " + str(self.theta))

    def newLaserScan(self, msg):
        self.moveScan = {
            'fleft': min(min(msg.ranges[315:]), 0.6),
            'fright': min(min(msg.ranges[:60]), 0.6)
        }

        # curGoal is of type Point()

    def move_to_goal_point(self, curGoal):
        reached = False
        x = curGoal.x
        y = curGoal.y
        reached = False
        rospy.loginfo("Inside move_to_goal_point()")
        while not rospy.is_shutdown() and not reached:
            inc_x = x - self.cur_x
            inc_y = y - self.cur_y

            rospy.loginfo("Incrementation of x: " + str(inc_x))
            rospy.loginfo("Incrementation of y: " + str(inc_y))
            rospy.loginfo("Current goal: " + str(curGoal))

            angle_to_goal = atan2(inc_y, inc_x)
            dist = math.sqrt(((x - self.cur_x) ** 2) + ((y - self.cur_y) ** 2))
            # rospy.loginfo("Current distance to goal: " + str(dist))

            # IS NOT UPDATING TO THE NEW ANGLE SEEN, SO IT IS STUCK AT 0.78
            if dist <= self.delta:  # and abs(self.theta) <= self.delta * 0.5:
                rospy.loginfo("Theta: " + str(self.theta))
                rospy.loginfo("Robot is close enough to the participants. Stopping now!")
                self.move.linear.x = 0.0
                self.move.angular.z = 0.0
                reached = True


            elif abs(angle_to_goal - self.theta) > 0.3:  # self.delta:
                if y > 0:
                    self.move.linear.x = 0.0
                    self.move.angular.z = self.rot_speed  # 0.25
                    # do something
                else:
                    self.move.linear.x = 0.0
                    self.move.angular.z = -1 * self.rot_speed  # -0.25

            else:
                self.move.linear.x = self.forward_speed  # 0.5
                self.move.angular.z = 0.0

            self.pub.publish(self.move)
            self.r.sleep()

    def move_to_goal_avoidance(self, curGoal):
        reached = False
        x = curGoal.x
        y = curGoal.y
        rospy.loginfo("Inside move_to_goal_point()")
        while not rospy.is_shutdown() and not reached:
            inc_x = x - self.cur_x
            inc_y = y - self.cur_y

            rospy.loginfo("Incrementation of x: " + str(inc_x))
            rospy.loginfo("Incrementation of y: " + str(inc_y))
            rospy.loginfo("Current goal: " + str(curGoal))

            angle_to_goal = atan2(inc_y, inc_x)
            dist = math.sqrt(((x - self.cur_x) ** 2) + ((y - self.cur_y) ** 2))

          
            if dist <= self.delta:  # and abs(self.theta) <= self.delta * 0.5:
                rospy.loginfo("Theta: " + str(self.theta))
                rospy.loginfo("Robot is close enough to the participants. Stopping now!")
                self.move.linear.x = 0.0
                self.move.angular.z = 0.0
                reached = True


            elif abs(angle_to_goal - self.theta) > 0.3:  # self.delta:
                if y > 0:
                    self.move.linear.x = 0.0
                    self.move.angular.z = self.rot_speed  # 0.25
                    # do something
                else:
                    self.move.linear.x = 0.0
                    self.move.angular.z = -1 * self.rot_speed  # -0.25

            else:
                if self.moveScan['fleft'] < 0.6 or self.moveScan['fright'] < 0.6:
                    rospy.loginfo("Fleft range is: " + str(self.moveScan['fleft']))
                    rospy.loginfo("Fright range is: " + str(self.moveScan['fleft']))
                    self.move.linear.x = 0.0
                    self.move.angular.z = 0.0
                else:
                    self.move.linear.x = self.forward_speed  # 0.5
                    self.move.angular.z = 0.0

            self.pub.publish(self.move)
            self.r.sleep()

    def return_to_starting_pos(self):
        # FUNCTION: returns robot to starting position if desired by user
        # INPUTS: terminal input y or n
        # OUTPUTS:

        # Create point with starting goal of 0,0
        # 0,0 is where the robot was when ROS was started up on it
        startGoal = Point()
        startGoal.x = 0
        startGoal.y = 0

        # Ask user if ready to return to start
        ready = raw_input("Should the robot return to the starting position (y/n)?")

        # If yes, return to start with obstacle avoidance
        if ready.lower() == "y":
            self.move_to_goal_avoidance(startGoal)
        else:
            rospy.loginfo("Returned to start position")


    def stop(self):
        # FUNCTION: stops robot
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        self.pub.publish(self.move)

    def correct_orientation(self, curGoal):
        # FUNCTION: rotate towards goal if not pointed at the goal, or at the goal
        # INPUTS: current goal
        # OUTPUTS: robot rotation

        # Set reached to false
        reached = False
        x = curGoal.x
        y = curGoal.y
        rospy.loginfo("Inside move_to_goal_point()")

        # While not at the goal
        while not rospy.is_shutdown() and not reached:
            inc_x = x - self.cur_x
            inc_y = y - self.cur_y

            # Get angle to goal and distance to goal
            angle_to_goal = atan2(inc_y, inc_x)
            dist = math.sqrt(((x - self.cur_x)**2) + ((y - self.cur_y)**2))

            if dist <= self.delta and abs(angle_to_goal- self.theta) > self.delta * 0.5:
                rospy.loginfo("Theta: " + str(self.theta))
                if y > self.cur_y:
                     self.move.linear.x = 0.0
                     self.move.angular.z = self.rot_speed * -0.5
                else:
                     self.move.linear.x = 0.0
                     self.move.angular.z = self.rot_speed * 0.5
            else:
                 self.move.linear.x = 0.0
                 self.move.angular.z = 0.0
                 reached = True
            self.pub.publish(self.move)

    def final_formation_orientation(self,orientation):
        # FUNCTION: Rotate to a final specified orientation
        # INPUTS: desired final orientation
        # OUTPUTS: rotates robot until at desired final orientation

        rospy.loginfo("Rotating to: "+str(orientation))

        # While not at the desired final orientation
        while abs(self.theta - orientation) > self.delta*0.25:
            rospy.loginfo("Need to rotate: "+str(abs(self.theta - orientation)))

            # Rotate left or right
            if self.theta < orientation:
                self.move.linear.x = 0.0
                self.move.angular.z = self.rot_speed  # 0.25
            else:
                self.move.linear.x = 0.0
                self.move.angular.z = -self.rot_speed

            #
            self.pub.publish(self.move)
