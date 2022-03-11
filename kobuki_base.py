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
        
        # angle margin of error when spinning
        self.angular_error = 0.01

        # angle odometry parameter
        self.kP = 4
        self.kD = -0.2

        # time to stop moving if we haven't reached destination
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

        while time_run < self.movement_timeout and not within_plus_or_minus(self.pose_with_covariance.pose.orientation.z, destination_z, self.angular_error):
            time_run += dt

            err = destination_z - self.pose_with_covariance.pose.orientation.z

            velocity_val = (self.kP * err) + ((last_err - err) / dt) * self.kD
            
            min_turning_speed = 0.4

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
        



        '''

        # keep track which way we should turn
        turn_ccw = False
        distance_cw = 0
        distance_ccw = 0
        distance = 0


        # determine which direction is most efficient to turn

        # if both start and finish have the same sign,
        if (start_z >= 0 and destination_z >= 0 or (start_z <= 0 and destination_z <= 0)):
            # turn counterclockwise if start < end
            turn_ccw = start_z < destination_z
            distance = abs(start_z - destination_z)
        else:
            # else if one is positive and one is negative

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


        
        destination_adjusted = 0 

        if (turn_ccw):
            print('Turning Counterclockwise to try and get to %f' % destination_z)
            destination_adjusted = destination_z + distance_ccw
	    else:
            print('Turning Clockwise to try and get to %f' % destination_z)
            destination_adjusted = destination_z - distance_ccw
        
        # define the window which we say to go towards
        window_buffer = abs(destination_z) * self.angular_error
        
        

        # negate the speed if we are turning clockwise
        velocity = self.angular_speed
        if (not turn_ccw):
            velocity *= -1

        bound_ccw = destination_z + window_buffer
        bound_cw = destination_z - window_buffer

        # if we overshoot and go past -1 or 1, need to correct with
        # the correct angle within the bounds of [-1,1]
        if (bound_ccw > 1):
            bound_ccw = (2 - bound_ccw) * -1.0
        if (bound_cw < -1):
            bound_cw = 2 + bound_cw

        # additional control for the amount of time should help prevent overshooting
        # by timing out after we reach the time that should be taken by the turn
        if (turn_ccw):
            distance = distance_ccw
        else:
            distance = distance_cw

        

        # time that should be taken by the turn
        approximate_turn_time = (distance / self.angular_speed)
        # add a bit of a buffer since this is a timeout
        approximate_turn_time += (approximate_turn_time * 0.05)

        # start spinning
        self.spin_async(velocity, timeout = approximate_turn_time)
        
        # check 10 times/s:
        polling_rate = 10 #Hz
        r = rospy.Rate(polling_rate)
        time_run = 0

        print('Timeout Time: %f' % approximate_turn_time)

        while not angle_is_between(self.pose_with_covariance.pose.orientation.z, bound_cw, bound_ccw) and time_run <= approximate_turn_time:
            time_run += (1.0 / polling_rate)
            r.sleep()
            continue

        self.stop()

        
        '''

    def stop(self):
        # print('attempting to stop with KobukiBase.stop()')
        stop_cmd = Twist()
        self.moving = False
        self.cmd_vel.publish(stop_cmd)    


origin = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_1 = Pose(Point(1.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_2 = Pose(Point(-1, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

 
if __name__ == '__main__':
    turtlebot = KobukiBase(init_node = True)
    turtlebot.reset_odometry()
    rospy.sleep(1)
    turtlebot.turn_to_angle(1)
    turtlebot.turn_to_angle(0)
    turtlebot.turn_to_angle(0.5)
    turtlebot.turn_to_angle(-0.5)
    turtlebot.turn_to_angle(0)
    turtlebot.turn_to_angle(-1)
