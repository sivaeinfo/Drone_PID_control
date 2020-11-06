#!/usr/bin/env python


# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # Format for drone_command
        self.cmd_drone = edrone_cmd()
        self.cmd_drone.rcRoll = 1500
        self.cmd_drone.rcPitch = 1500
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 1500

        # The latitude, longitude and altitude of the drone
        self.latitude = 19.0
        self.longitude = 72.0
        self.altitude = 0

        # The coordinates in the target postion vector is in the order latitude, longitude and altitude
        self.target = [19.000000000, 72.000000, 3.0] #19.0000451700

        # Initial settings for the values of Kp, Ki and Kd
        self.Kp = [4000000, 50]
        self.Ki = [0, 0.32]
        self.Kd = [5000000, 80]
        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.error_sum = [0, 0, 0]

        self.out_latitude = 0
        self.out_longitude = 0
        self.out_altitude = 0

        self.min_value = [1450, 1450, 1000]
        self.max_value = [1550, 1550, 2000]
        #
        
        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        # ------------------------------------------------------------------------------------------------------------


    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude


    def pid(self):

        # Calculating the error
        self.error[0]  =self.target[0] - self.latitude
        self.error[1] = self.target[1] - self.longitude
        self.error[2] = self.target[2] - self.altitude


        self.error_sum[0] = self.error_sum[0] + self.error[0]
        self.error_sum[1] = self.error_sum[1] + self.error[1]
        self.error_sum[2] = self.error_sum[2] + self.error[2]

        # Calculating pid values
        self.out_latitude = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.error_sum[0]) + ((self.Kd[0] * (self.error[0] - self.prev_error[0]))/self.sample_time)
        self.out_longitude = (self.Kp[0] * self.error[1]) + (self.Ki[0] * self.error_sum[1]) + ((self.Kd[0] * (self.error[1] - self.prev_error[1]))/self.sample_time)
        self.out_altitude = (self.Kp[1] * self.error[2]) + (self.Ki[1] * self.error_sum[2]) + ((self.Kd[1] * (self.error[2] - self.prev_error[2]))/self.sample_time)


        # Changing the previous sum value
        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        # Giving drone command
        self.cmd_drone.rcRoll = 1500 + self.out_latitude
        self.cmd_drone.rcPitch = 1500 + self.out_longitude
        self.cmd_drone.rcYaw = 1500
        self.cmd_drone.rcThrottle = 1500 + self.out_altitude

        if self.cmd_drone.rcRoll > self.max_value[0]:
            self.cmd_drone.rcRoll = self.max_value[0]
        elif self.cmd_drone.rcRoll < self.min_value[0]:
            self.cmd_drone.rcRoll = self.min_value[0]
        else:
            self.cmd_drone.rcRoll = self.cmd_drone.rcRoll

        if self.cmd_drone.rcPitch > self.max_value[1]:
            self.cmd_drone.rcPitch = self.max_value[1]
        elif self.cmd_drone.rcPitch < self.min_value[1]:
            self.cmd_drone.rcPitch = self.min_value[1]
        else:
            self.cmd_drone.rcPitch = self.cmd_drone.rcPitch

        if self.cmd_drone.rcThrottle > self.max_value[2]:
            self.cmd_drone.rcThrottle = self.max_value[2]
        elif self.cmd_drone.rcThrottle < self.min_value[2]:
            self.cmd_drone.rcThrottle = self.min_value[2]
        else:
            self.cmd_drone.rcThrottle = self.cmd_drone.rcThrottle

        #
        #
        #
        # ------------------------------------------------------------------------------------------------------------------------

        self.cmd_pub.publish(self.cmd_drone)

        print(self.latitude, self.longitude, self.altitude)

        if self.altitude  >= 2.9:
            self.target[0] = 19.00004571
            print(self.target)
           #time.sleep(2)

        if self.latitude >= 19.00004571:
             self.target[2] = 0.31
             print(self.target)
            #time.sleep(2)

if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
