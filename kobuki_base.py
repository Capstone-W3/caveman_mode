#!/usr/bin/env python


# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
import math
from fifo import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from angle_calculator import *
import threading
import time


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
 
 - /mobile_base/events/imu_data_raw -- Callback: self.ImuDataReceivedEvent 
    - self.imu (sensor_msgs/Imu) - IMU data of bot in realtime, updated asynchronously

 - /mobile_base/events/button -- Callback: self.ButtonPressReceivedEvent

 - 
'''
class KobukiBase():
    # Initialize ROS Node
    def __init__(self, init_node = False):
        self.moving = False
        if init_node:
            rospy.init_node('KobukiBase')

        # PUBLISHERS
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.cmd_reset_odometry = rospy.Publisher('mobile_base/commands/reset_odometry', Empty, queue_size=10)

        # SUBSCRIBERS
        rospy.Subscriber("/odom", Odometry, self.OdometryDataReceivedEvent)
        rospy.Subscriber("/mobile_base/sensors/imu_data_raw", Imu, self.ImuDataReceivedEvent)

        # track the last n frames of pose data with timestamps
        self.location_data = FixedLengthFifo(1000)

        # angle margin of error when spinning, percentage
        self.angular_error = 0.05
        
    def OdometryDataReceivedEvent(self, data):
        self.pose_with_covariance = data.pose
        # print(self.pose_with_covariance.pose)
        self.twist_with_covariance = data.twist

        # print("Odometry Z: %f" % self.pose_with_covariance.pose.orientation.z)

        # push the most recent odometry data to the fifo to trach movement
        self.location_data.push((data.header.stamp, self.pose_with_covariance.pose))

    def ImuDataReceivedEvent(self, data):
        self.imu = data
        # print "IMU Z: ",
        # print self.imu.orientation.z
        # print("IMU Z: %f" % self.imu.orientation.z)
    
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


    def turn_to_angle(self, destination_z):
        start_z = self.pose_with_covariance.pose.orientation.z
        print('turn_to_angle start_z: %f destination_z: %f' % (start_z, destination_z))

        # keep track which way we should turn
        turn_ccw = False

        # determine which direction is most efficient to turn

        # if both start and finish have the same sign,
        if (start_z >= 0 and destination_z >= 0 or (start_z <= 0 and destination_z <= 0)):
            # turn counterclockwise if start < end
            turn_ccw = start_z < destination_z
        else:
            # else if one is positive and one is negative
            distance_cw = 0
            distance_ccw = 0


            # if the start_z is positive, clockwise distance is subtraction
            if start_z > 0:
                distance_cw = abs(start_z - destination_z)
                distance_ccw = 2 - distance_cw
            else:
            # else the counterclockwise distance is subtraction
                distance_ccw = abs(start_z - destination_z)
                distance_cw = 2 - distance_ccw

            if (distance_ccw < distance_cw):
                turn_ccw = True
            else:
                turn_ccw = False

        if (turn_ccw):
            print('Turning Counterclockwise to try and get to %f' % destination_z)
        else:
            print('Turning Clockwise to try and get to %f' % destination_z)
        
        # negate the speed if we are turning clockwise
        velocity = self.angular_speed
        if (not turn_ccw):
            velocity *= -1

        # define the window which we say to go towards
        window_buffer = abs(destination_z) * self.angular_error
        
        bound_ccw = destination_z + window_buffer
        bound_cw = destination_z - window_buffer

        # if we overshoot and go past -1 or 1, need to correct with
        # the correct angle within the bounds of [-1,1]
        if (bound_ccw > 1):
            bound_ccw = (2 - bound_ccw) * -1.0
        if (bound_cw < -1):
            bound_cw = 2 + bound_cw
       
        # start spinning
        self.spin_async(velocity, timeout=10)
        
        while not angle_in_between(self.pose_with_covariance.pose.orientation.z, bound_cw, bound_ccw):
            continue

        self.stop()


    def stop(self):
        print('attempting to stop with KobukiBase.stop()')
        stop_cmd = Twist()
        self.moving = False
        self.cmd_vel.publish(stop_cmd)    


origin = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_1 = Pose(Point(1.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_2 = Pose(Point(-1, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

 
if __name__ == '__main__':
    turtlebot = KobukiBase(init_node = True)
    turtlebot.reset_odometry()
    turtlebot.spin_async(0.4, timeout=500)
    print('sleeping')
    time.sleep(200)
    print('slept')
    turtlebot.stop()
