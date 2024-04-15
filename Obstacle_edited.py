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
import array # arrays
import time # time

LINEAR_VEL = 0.20 # max vel
RELATIVE_VEL = LINEAR_VEL / 100
LIDAR_ERROR = 0.05
COLLISION_DISTANCE = LIDAR_ERROR

SAFE_STOP_DISTANCE = LIDAR_ERROR + 0.05

SOFT_TURN_DISTANCE = LIDAR_ERROR + 0.30
MEDIUM_TURN_DISTANCE = LIDAR_ERROR + 0.20
HARD_TURN_DISTANCE = LIDAR_ERROR + 0.10

U_TURN = 1 #TBD

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

        # make robot run for 120 seconds 
        endtime = time.time + 120 
        while time.time() < endtime:
        
            
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)
            min_distance_angle = -1

            angle_intervals = 11
            angle_range = len(lidar_distances)

            for i in range(0, len(angle_intervals)):
                index1 = int((angle_range / angle_intervals) * i)
                index2 = int((angle_range / angle_intervals) * (i+1))
                if min_distance in lidar_distances[index1:index2]:
                    min_distance_angle = index1 + 15
                print(str(min_distance_angle - 15) +"-" + str((angle_range / angle_intervals) + min_distance_angle - 15) + " Distance of the obstacle: " + str(min_distance))
            
            twist.linear.x = LINEAR_VEL
            if min_distance < U_TURN:
                print('you spin me right round baby, right round')
            elif min_distance < HARD_TURN_DISTANCE:
                if turtlebot_moving:

                    if min_distance in lidar_distances[0:30]:
                        twist.linear.x = RELATIVE_VEL * 10
                        twist.angular.z = RELATIVE_VEL * 90
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[30:60]:
                        twist.linear.x = RELATIVE_VEL * 20
                        twist.angular.z = RELATIVE_VEL * 80
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[60:90]:
                        twist.linear.x = RELATIVE_VEL * 30
                        twist.angular.z = RELATIVE_VEL * 70
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')

                    elif min_distance in lidar_distances[90:270]:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Keep going, never look back')

                    elif min_distance in lidar_distances[270:300]:
                        twist.linear.x = RELATIVE_VEL * 30
                        twist.angular.z = RELATIVE_VEL * -70
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[300:330]:
                        twist.linear.x = RELATIVE_VEL * 20
                        twist.angular.z = RELATIVE_VEL * -80
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[330:360]:
                        twist.linear.x = RELATIVE_VEL * 10
                        twist.angular.z = RELATIVE_VEL * -90
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
            

            elif min_distance < MEDIUM_TURN_DISTANCE:
                if turtlebot_moving:

                    if min_distance in lidar_distances[0:30]:
                        twist.linear.x = RELATIVE_VEL * 40
                        twist.angular.z = RELATIVE_VEL * 60
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn')
                    elif min_distance in lidar_distances[30:60]:
                        twist.linear.x = RELATIVE_VEL * 50
                        twist.angular.z = RELATIVE_VEL * 50
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn')
                    elif min_distance in lidar_distances[60:90]:
                        twist.linear.x = RELATIVE_VEL * 60
                        twist.angular.z = RELATIVE_VEL * 40
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn')

                    elif min_distance in lidar_distances[90:270]:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Keep going, never look back')

                    elif min_distance in lidar_distances[270:300]:
                        twist.linear.x = RELATIVE_VEL * 60
                        twist.angular.z = RELATIVE_VEL * -40
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[300:330]:
                        twist.linear.x = RELATIVE_VEL * 50
                        twist.angular.z = RELATIVE_VEL * -50
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[330:360]:
                        twist.linear.x = RELATIVE_VEL * 40
                        twist.angular.z = RELATIVE_VEL * -60
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                        

            elif min_distance < SOFT_TURN_DISTANCE:
                if turtlebot_moving:

                    if min_distance in lidar_distances[0:30]:
                        twist.linear.x = RELATIVE_VEL * 70
                        twist.angular.z = RELATIVE_VEL * 30
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn slightly')
                    elif min_distance in lidar_distances[30:60]:
                        twist.linear.x = RELATIVE_VEL * 80
                        twist.angular.z = RELATIVE_VEL * 20
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn slightly')
                    elif min_distance in lidar_distances[60:90]:
                        twist.linear.x = RELATIVE_VEL * 90
                        twist.angular.z = RELATIVE_VEL * 10
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn slightly')

                    elif min_distance in lidar_distances[90:270]:
                        twist.linear.x = LINEAR_VEL
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Keep going, never look back')

                    elif min_distance in lidar_distances[270:300]:
                        twist.linear.x = RELATIVE_VEL * 90
                        twist.angular.z = RELATIVE_VEL * -10
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[300:330]:
                        twist.linear.x = RELATIVE_VEL * 80
                        twist.angular.z = RELATIVE_VEL * -20
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')
                    elif min_distance in lidar_distances[330:360]:
                        twist.linear.x = RELATIVE_VEL * 70
                        twist.angular.z = RELATIVE_VEL * -30
                        self._cmd_pub.publish(twist)
                        rospy.loginfo('Turn sharply')

                    #twist.linear.x = 0.0 
                    #twist.angular.z = 0.1
                    #self._cmd_pub.publish(twist)
                    #turtlebot_moving = False
                    #rospy.loginfo('Stop!')
                
            # else:
                 #twist.linear.x = LINEAR_VEL
                 #twist.angular.z = 0.0
                 #self._cmd_pub.publish(twist)
                 #turtlebot_moving = True
                 #rospy.loginfo('Distance of the obstacle : %f', min_distance)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


