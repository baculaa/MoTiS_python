#!/usr/bin/env python

import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import LaserScan

from math import atan2


class Movement:
    def __init__(self):
        # Init robot x,y,theta
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.theta = 0.0

        # Error tolerance in meters and radians
        self.delta = 0.1

        # Rotational speed
        self.rot_speed = 0.3
        # Linear speed
        self.forward_speed = 0.6


        # create publisher and subscriber
        self.sub = rospy.Subscriber("/odom", Odometry,
                                    self.newOdom)  # our odometry node is called /odom and not /odometry/filtered
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber("/base_scan", LaserScan, self.newLaserScan)

        # create a move variable
        self.move = Twist()

        # set the rate
        self.r = rospy.Rate(4)

        # Init laser scan dict
        self.moveScan = {}

        # How close in meters an object can be before the robot stops for it
        self.obstacleThreshold = 0.6


    def newOdom(self, msg):
        # FUNCTION: Gets the current odometry of the robot
        # INPUTS: odometry message
        # OUTPUTS: current x,y,theta

        # get the current x and y position values for the robot
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation

        # Get roll pitch and yaw from quaternion
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # rospy.loginfo("Current x-coordinate: " + str(self.cur_x))
        # rospy.loginfo("Current y-coordinate: " + str(self.cur_y))
        # rospy.loginfo("Current theta value: " + str(self.theta))

    def newLaserScan(self, msg):
        # FUNCTION: Takes in the laser scan data in front of the robot
        # INPUTS: laser scan message
        # OUTPUTS: dictionary of the closest object detected on the front right and left
        self.moveScan = {
            'fleft': min(min(msg.ranges[315:]), self.obstacleThreshold),
            'fright': min(min(msg.ranges[:60]), self.obstacleThreshold)
        }


    def move_to_goal_avoidance(self, curGoal):
        # FUNCTION: Moves robot from current position to goal position
        #           while stopping if an obstacle enters its space
        # INPUTS: Current goal as a Point() type
        # OUTPUTS: self.move Twist() type with modified linear and angular velocities
        reached = False
        x = curGoal.x
        y = curGoal.y
        rospy.loginfo("Inside move_to_goal_point()")
        while not rospy.is_shutdown() and not reached:

            # How far in x and y from goal
            inc_x = x - self.cur_x
            inc_y = y - self.cur_y

            rospy.loginfo("Incrementation of x: " + str(inc_x))
            rospy.loginfo("Incrementation of y: " + str(inc_y))
            rospy.loginfo("Current goal: " + str(curGoal))

            # Get angle to the goal
            angle_to_goal = atan2(inc_y, inc_x)
            dist = math.sqrt(((x - self.cur_x) ** 2) + ((y - self.cur_y) ** 2))

            if dist <= self.delta:
                # If distance to goal is less than the delta
                rospy.loginfo("Robot is close enough to goal. Stopping now!")

                # Set velocities to zero
                self.move.linear.x = 0.0
                self.move.angular.z = 0.0

                # Set reached to True
                reached = True

            elif abs(angle_to_goal - self.theta) > self.delta:
                # If not at goal or pointed towards goal, turn to goal
                self.correct_orientation(y)

            else:
                # If pointed towards the goal and not at the goal
                # If an obstacle is detected within threshold, stop
                if self.moveScan['fleft'] < self.obstacleThreshold or self.moveScan['fright'] < self.obstacleThreshold:
                    rospy.loginfo("Fleft range is: " + str(self.moveScan['fleft']))
                    rospy.loginfo("Fright range is: " + str(self.moveScan['fleft']))

                    # Set velocities to zero
                    self.move.linear.x = 0.0
                    self.move.angular.z = 0.0
                else:
                    # Move forward at set speed
                    self.move.linear.x = self.forward_speed
                    self.move.angular.z = 0.0

            # Publish velocities
            self.pub.publish(self.move)
            # Sleep
            self.r.sleep()

    def return_to_starting_pos(self):
        # FUNCTION: returns robot to starting position if desired by user
        # INPUTS: terminal input y or n
        # OUTPUTS: robot goes to its 0,0

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

    def correct_orientation(self, y):
        # FUNCTION: rotate towards goal if not pointed at the goal, or at the goal
        # INPUTS: current goal
        # OUTPUTS: robot rotation

        if y > self.cur_y:
             self.move.linear.x = 0.0
             self.move.angular.z = -self.rot_speed
        else:
             self.move.linear.x = 0.0
             self.move.angular.z = self.rot_speed

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
