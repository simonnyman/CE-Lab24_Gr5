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
import smbus # RGB
import RPi.GPIO as GPIO

GPIO.setwarnings(False)    # Ignore warning for now
GPIO.setmode(GPIO.BOARD)   # Use physical pin numbering
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)   # Set pin 18 to be an output pin and set initial value to low (off)
GPIO.setup(16, GPIO.OUT, initial=GPIO.LOW)   # Set pin 16 to be an output pin and set initial value to low (off)
GPIO.setup(26, GPIO.OUT, initial=GPIO.HIGH)   # Set pin 26 to be an output pin and set initial value to high (on)


# Get I2C bus
bus = smbus.SMBus(1)
# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

LINEAR_VEL = 0.20 # max vel
RELATIVE_LIN_VEL = LINEAR_VEL / 100 # for easier adjustment

ANGULAR_VEL = 2.50 # max angular vel
RELATIVE_ANG_VEL = ANGULAR_VEL / 100 # for easier adjustment

LIDAR_ERROR = 0.05
COLLISION_DISTANCE = 0.06 + LIDAR_ERROR

# SOFT_TURN_DISTANCE = LIDAR_ERROR + 0.47
# MEDIUM_TURN_DISTANCE = LIDAR_ERROR + 0.35
MIX_TURN_DISTANCE = LIDAR_ERROR + 0.425
HARD_TURN_DISTANCE = LIDAR_ERROR + 0.23

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

    def getAndUpdateColour(self):
    # Read the data from the sensor
        # ISL29125 address, 0x44
        # Reads the data from 0x44, address of ISL29125 from register 0x09 (low green) to 0x0E (high blue)
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        # Convert the data to green, red and blue int values
        green = data[1] * 256 + data[0]
        red = data[3] * 256 + data[2]
        blue = data[5] * 256 + data[4]
        blue=blue * 1.8 # Compensates for the low blue readings
        return red, green, blue

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        avg_linear_speed = 0
        speed_updates = 0
        speed_accumulation = 0

        collision_counter = 0
        victim_counter = 0
        collision_delay = time.time()
        led_delay = time.time()

        # make robot run for 120 seconds
        endtime = time.time() + 120
        while time.time() < endtime:
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            speed_updates = speed_updates + 1
            speed_accumulation += twist.linear.x
            avg_linear_speed = speed_accumulation / speed_updates

            red, green, blue = self.getAndUpdateColour()

            if red > green and red > blue and red - blue > 1000 and red - green > 1000 and led_delay <= time.time():
                GPIO.output(18, GPIO.HIGH) # Turn on
                victim_counter += 1
                led_delay = time.time() + 3
                rospy.loginfo("Victim")
            else:
                GPIO.output(18, GPIO.LOW)

            twist.linear.x = LINEAR_VEL
            if turtlebot_moving:
                if min_distance in lidar_distances[180:205]:
                    if min_distance < HARD_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 20
                        twist.angular.z = RELATIVE_ANG_VEL * -100
                    elif min_distance < MIX_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 60
                        twist.angular.z = RELATIVE_ANG_VEL * -100

                elif min_distance in lidar_distances[155:180]:
                    if min_distance < HARD_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 20
                        twist.angular.z = RELATIVE_ANG_VEL * 100
                    elif min_distance < MIX_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 60
                        twist.angular.z = RELATIVE_ANG_VEL * 100

                elif min_distance in lidar_distances[205:230]:
                    if min_distance < HARD_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 40
                        twist.angular.z = RELATIVE_ANG_VEL * -80
                    elif min_distance < MIX_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 80
                        twist.angular.z = RELATIVE_ANG_VEL * -80

                elif min_distance in lidar_distances[130:155]:
                    if min_distance < HARD_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 40
                        twist.angular.z = RELATIVE_ANG_VEL * 80
                    elif min_distance < MIX_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 80
                        twist.angular.z = RELATIVE_ANG_VEL * 80

                elif min_distance in lidar_distances[230:255]:
                    if min_distance < HARD_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 60
                        twist.angular.z = RELATIVE_ANG_VEL * -60
                    elif min_distance < MIX_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 100
                        twist.angular.z = RELATIVE_ANG_VEL * -60

                elif min_distance in lidar_distances[105:130]:
                    if min_distance < HARD_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 60
                        twist.angular.z = RELATIVE_ANG_VEL * 60
                    elif min_distance < MIX_TURN_DISTANCE:
                        twist.linear.x = RELATIVE_LIN_VEL * 100
                        twist.angular.z = RELATIVE_ANG_VEL * 60

                elif min_distance in lidar_distances[255:360] or lidar_distances[0:105]:
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 0.0

                self._cmd_pub.publish(twist)

            if min_distance < COLLISION_DISTANCE and collision_delay <= time.time():
                GPIO.output(16, GPIO.HIGH) # Turn on
                collision_counter = collision_counter + 1
                collision_delay = time.time()+5
                rospy.loginfo("Collison count %f", collision_counter)
            else:
                GPIO.output(16, GPIO.LOW)

        rospy.loginfo("final average speed: %f", avg_linear_speed)
        rospy.loginfo("final collison count: %f", collision_counter)
        rospy.loginfo("final victim count: %f", victim_counter)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


