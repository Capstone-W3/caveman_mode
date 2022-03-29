#!/usr/bin/env python
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
    def __init__(self, init_node = False, angular_speed = 0.3, linear_speed = 0.2):
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

        # default speeds
        self.angular_speed = angular_speed
        self.linear_speed = linear_speed
        
        # angle margin of error when spinning (pi * radians)
        self.angular_error = 0.01

        # linear margin of error when moving forward/backwards (meters)
        self.linear_error = 0.01

        # angle odometry parameter
        self.kP = 4
        self.kD = -0.2

        # time to stop moving if we haven't reached destination (seconds)
        self.movement_timeout = 5

    def OdometryDataReceivedEvent(self, data):
        self.pose_with_covariance = data.pose
        # print(self.pose_with_covariance.pose)
        self.twist_with_covariance = data.twist

        #print("Odometry Z: %f" % self.pose_with_covariance.pose.orientation.z)

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

        if (self.moving):
            return
        
        self.moving = True

        command= Twist()
        polling_rate = 10.0 # Hz
        dt = 1.0 / polling_rate
        r = rospy.Rate(polling_rate)
        time_run = 0

        last_err = 0

        while time_run < self.movement_timeout and not within_plus_or_minus(self.pose_with_covariance.pose.orientation.z, destination_z, self.angular_error) and self.moving:
            time_run += dt

            err = destination_z - self.pose_with_covariance.pose.orientation.z

            velocity_val = (self.kP * err) + ((last_err - err) / dt) * self.kD
            
            min_turning_speed = 0.5

            if velocity_val > 0:
                velocity_val = max(velocity_val, min_turning_speed)
            elif velocity_val < 0:
                velocity_val = min(velocity_val, -1 * min_turning_speed)

            command.angular.z = velocity_val

            last_err = err
            
            # print(command.angular.z)
            self.cmd_vel.publish(command)
            r.sleep()

        print('end z: %f' % self.pose_with_covariance.pose.orientation.z)
        print('reached destination: %s' % within_plus_or_minus(self.pose_with_covariance.pose.orientation.z, destination_z, self.angular_error))
        
        
        
        self.stop()
        

    def stop(self):
        # print('attempting to stop with KobukiBase.stop()')
        stop_cmd = Twist()
        self.moving = False
        self.cmd_vel.publish(stop_cmd)

    def move(self, distance_meters):
        if (self.moving):
            return

        self.moving = True

        start_angle_quaternion = self.pose_with_covariance.pose.orientation.z
        start_x = self.pose_with_covariance.pose.position.x
        start_y = self.pose_with_covariance.pose.position.y
        start_angle_radians = start_angle_quaternion * math.pi

        print('Start = (%f, %f), Angle = %f (pi*radians)' % (start_x, start_y, start_angle_quaternion))
        
        delta_x = distance_meters * math.cos(start_angle_radians)
        delta_y = distance_meters * math.sin(start_angle_radians)

        destination_x = start_x + delta_x
        destination_y = start_y + delta_y

        print('Destination = (%f, %f), delta_x = %f, delta_y = %f' % (destination_x, destination_y, delta_x, delta_y))

        move_cmd = Twist()
        polling_rate = 10.0 #Hz
        dt = 1.0 / polling_rate
        r = rospy.Rate(polling_rate)
        time_run = 0

        last_err = 0

        last_angle_err = 0
        angle_err = float("inf")

        # measuring success by distance from starting point relation to goal,
        # calculated in 2 dimensions
        # start with infinity so it isn't within bounds, recalculate once per cycle
        err = float("inf")

        while time_run < self.movement_timeout and not within_plus_or_minus(err, 0, self.linear_error) and self.moving:
            time_run += dt

            curr_pose = self.pose_with_covariance.pose.position
            curr_x = curr_pose.x
            curr_y = curr_pose.y
            curr_angle = self.pose_with_covariance.pose.orientation.z
            
            distance_traveled = distance_between_points(curr_x, curr_y, start_x, start_y)

            err = distance_meters - distance_traveled

            min_movement_speed = 0.1
            max_movement_speed = 0.5

            velocity_val = ((self.kP * err) / 3) + ((last_err - err) / dt) * self.kD

            if velocity_val > 0:
                velocity_val = max(velocity_val, min_movement_speed)
                velocity_val = min(velocity_val, max_movement_speed)
            elif velocity_val < 0:
                velocity_val = min(velocity_val, -1 * min_movement_speed)
                velocity_val = max(velocity_val, -1 * max_movement_speed)

            # print('distance moved: %f, error: %f, velocity: %f m/s' % (distance_traveled, err, velocity_val))

            move_cmd.linear.x = velocity_val
            self.cmd_vel.publish(move_cmd)
            last_err = err
            r.sleep()

        self.stop()

        print('Final position: (%f, %f)' % (curr_x, curr_y))
        print('Destination goal: (%f, %f)' % (destination_x, destination_y))
        print('distance from goal: %f' % distance_between_points(curr_x, curr_y, destination_x, destination_y))
        print('time_run: %f' % time_run)








origin = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_1 = Pose(Point(1.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_2 = Pose(Point(-1, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

 
if __name__ == '__main__':
    turtlebot = KobukiBase(init_node = True)
    turtlebot.reset_odometry()
    rospy.sleep(1)
    turtlebot.move(1)
    rospy.sleep(1)

