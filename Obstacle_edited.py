#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time # time
import lights # RGB
import led
import ledr

LINEAR_VEL = 0.20 # max vel
RELATIVE_LIN_VEL = LINEAR_VEL / 100 # for easier adjustment

ANGULAR_VEL = 2.50 # max angular vel
RELATIVE_ANG_VEL = ANGULAR_VEL / 100 # for easier adjustment

LIDAR_ERROR = 0.05
COLLISION_DISTANCE = 0.08 + LIDAR_ERROR

SOFT_TURN_DISTANCE = LIDAR_ERROR + 0.44
MEDIUM_TURN_DISTANCE = LIDAR_ERROR + 0.32
HARD_TURN_DISTANCE = LIDAR_ERROR + 0.20

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 360          # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            if scan_filter[i] < 0.01: # if any angle is not working ie. making false readings close to 0
                scan_filter[i] = 1    # set angle to 1
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        avg_linear_speed = 0
        speed_updates = 0
        speed_accumulation = 0
        collision_counter = 0
        collision_delay = time.time()
        # make robot run for 120 seconds
        endtime = time.time() + 120
        while time.time() < endtime:
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)
            
            
            twist.linear.x = LINEAR_VEL
            if min_distance < HARD_TURN_DISTANCE:
                if turtlebot_moving:

                    if min_distance in lidar_distances[180:210]:
                        twist.linear.x = RELATIVE_LIN_VEL * 45
                        twist.angular.z = RELATIVE_ANG_VEL * -80
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Hard turn right')
                    elif min_distance in lidar_distances[210:240]:
                        twist.linear.x = RELATIVE_LIN_VEL * 55
                        twist.angular.z = RELATIVE_ANG_VEL * -76
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Hard turn right')

                    elif min_distance in lidar_distances[120:150]:
                        twist.linear.x = RELATIVE_LIN_VEL * 55
                        twist.angular.z = RELATIVE_ANG_VEL * 76
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Hard turn left')
                    elif min_distance in lidar_distances[150:180]:
                        twist.linear.x = RELATIVE_LIN_VEL * 45
                        twist.angular.z = RELATIVE_ANG_VEL * 80
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Hard turn left')

                    elif min_distance in lidar_distances[60:90] or lidar_distances[30:60] or lidar_distances[0:30] or lidar_distances[330:360] or lidar_distances[300:330] or lidar_distances[270:300] or lidar_distances[240:270] or lidar_distances[90:120]:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = True
                        rospy.loginfo('Keep going, never look back')

            elif min_distance < MEDIUM_TURN_DISTANCE:
                if turtlebot_moving:

                    if min_distance in lidar_distances[180:210]:
                        twist.linear.x = RELATIVE_LIN_VEL * 80
                        twist.angular.z = RELATIVE_ANG_VEL * -66
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Medium turn left')
                    elif min_distance in lidar_distances[210:240]:
                        twist.linear.x = RELATIVE_LIN_VEL * 85
                        twist.angular.z = RELATIVE_ANG_VEL * -60
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Medium turn left')

                    elif min_distance in lidar_distances[120:150]:
                        twist.linear.x = RELATIVE_LIN_VEL * 85
                        twist.angular.z = RELATIVE_ANG_VEL * 60
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Medium turn right')
                    elif min_distance in lidar_distances[150:180]:
                        twist.linear.x = RELATIVE_LIN_VEL * 80
                        twist.angular.z = RELATIVE_ANG_VEL * 66
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Medium turn right')

                    elif min_distance in lidar_distances[60:90] or lidar_distances[30:60] or lidar_distances[0:30] or lidar_distances[330:360] or lidar_distances[300:330] or lidar_distances[270:300] or lidar_distances[90:120] or lidar_distances[240:270]:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = True
                        rospy.loginfo('Keep going, never look back')

            elif min_distance < SOFT_TURN_DISTANCE:
                if turtlebot_moving:

                    if min_distance in lidar_distances[180:210]:
                        twist.linear.x = RELATIVE_LIN_VEL * 90
                        twist.angular.z = RELATIVE_ANG_VEL * -52
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Soft turn right')
                    elif min_distance in lidar_distances[210:240]:
                        twist.linear.x = RELATIVE_LIN_VEL * 95
                        twist.angular.z = RELATIVE_ANG_VEL * -44
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Soft turn right')

                    elif min_distance in lidar_distances[120:150]:
                        twist.linear.x = RELATIVE_LIN_VEL * 95
                        twist.angular.z = RELATIVE_ANG_VEL * 44
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Soft turn left')
                    elif min_distance in lidar_distances[150:180]:
                        twist.linear.x = RELATIVE_LIN_VEL * 90
                        twist.angular.z = RELATIVE_ANG_VEL * 52
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Soft turn left')

                    elif min_distance in lidar_distances[60:90] or lidar_distances[30:60] or lidar_distances[0:30] or lidar_distances[330:360] or lidar_distances[300:330] or lidar_distances[270:300] or lidar_distances[240:270] or lidar_distances[90:120]:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = True
                        rospy.loginfo('Keep going, never look back')
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True

            if min_distance < COLLISION_DISTANCE:
                ledr.collision
                collision_counter =+ 1
            if lights.getAndUpdateColour().color == "red":
                rospy.loginfo("victim found!")
                led.victim


            speed_updates+= 1
            speed_accumulation += twist.linear.x
            avg_linear_speed = speed_accumulation / speed_updates

        rospy.loginfo("final average speed %f", avg_linear_speed)
        rospy.loginfo("final collison count %f", collision_counter)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


