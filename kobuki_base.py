#!/usr/bin/env python


# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
import math
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
import threading
import time

def plus_or_minus(num, bound, compare):
    return compare >= num - bound and compare <= num + bound

'''
KobukiBase ROS NODE

PUBLISHED TOPICS:
 - `cmd_vel_mux/input/navi` -- Publishes velocity data as `Twist` to tell kobuki to move at a speed
 - `/mobile_base/commands/reset_odometry`
    - Reset the odometry with self.reset_odometry()

SUBSCRIBED TOPICS:
 - odom (nav_msgs/Odometry) -- Callback: self.OdometryDataReceivedEvent
    - self.pose_with_covariance (geometry_msgs/PoseWithCovariance)
        - Pose data, position in x,y,z, with a covariance uncertainty vector
    - self.twist_with_covariance (geometry_msgs/TwistWithCovariance
        - Twist data, both linear and angular velocities in x,y,z axis, with a covariance vector
 
 - /mobile_base/events/imu_data -- Callback: self.ImuDataReceivedEvent 
    - self.imu (sensor_msgs/Imu) - IMU data of bot in realtime, updated asynchronously

 - /mobile_base/events/button -- Callback: self.ButtonPressReceivedEvent

 - 
'''
class KobukiBase():
    # Initialize ROS Node
    def __init__(self, init_node = False):
        if init_node:
            rospy.init_node('KobukiBase')

        # PUBLISHERS
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.cmd_reset_odometry = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size=10)

        # SUBSCRIBERS
        rospy.Subscriber("/odom", Odometry, self.OdometryDataReceivedEvent)
        rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, self.ImuDataReceivedEvent)
        
    def OdometryDataReceivedEvent(self, data):
        self.pose_with_covariance = data.pose
        # print(self.pose_with_covariance.pose)
        self.twist_with_covariance = data.twist

    def ImuDataReceivedEvent(self, data):
        self.imu = data
        # print "IMU Z: ",
        # print self.imu.orientation.z
    
    def reset_odometry(self):
        self.cmd_reset_odometry.publish(Empty())


    # call this function to spawn a new thread which will constantly
    # spin the turtlebot at the specified speed until self.stop()
    # is called from any other thread or timeout expires (seconds)
    def spin_async(self, speed = 0.3, timeout = 1):
        # spins endlessly, if called async
        def spin(speed):
            self.moving = True
            print('attempting to spin asyncrounously')
            
            polling_rate = 10 #Hz
            r = rospy.Rate(polling_rate)
            
            turn_cmd = Twist()
            turn_cmd.angular.z = speed
            time_run = 0

            while self.moving and time_run < timeout:
                self.cmd_vel.publish(turn_cmd)
                r.sleep()
                time_run += (1.0 / float(polling_rate))

            print('timeout, stopping spinning')
            self.stop()
            self.moving = False

        if not (self.moving):
            spin_thread = threading.Thread(target = spin, args=(speed,))
            print('starting spinning thread')
            spin_thread.start()
        else:
            print('already moving, can\'t spin more')


    def stop(self):
        print('attempting to stop with KobukiBase.stop()')
        stop_cmd = Twist()
        self.moving = False
        self.cmd_vel.publish(stop_cmd)    


origin = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_1 = Pose(Point(1.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_2 = Pose(Point(-1, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

 
if __name__ == '__main__':
    turtlebot = TurtlebotControl(init_node = True)
    turtlebot.reset_odometry()
    turtlebot.spin_async(0.3)
    print('sleeping')
    time.sleep(3)
    print('slept')
    turtlebot.stop()
