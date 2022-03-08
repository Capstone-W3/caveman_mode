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


def plus_or_minus(num, bound, compare):
    return compare >= num - bound and compare <= num + bound

'''
TurtlebotControl ROS NODE

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
class TurtlebotControl():
    # Initialize ROS Node
    def __init__(self):
        #rospy.init_node('TurtlebotControl')

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

    
    def move_to_pose(self, destination):
        start_pose = self.pose_with_covariance.pose

        x_start = start_pose.position.x
        y_start = start_pose.position.y

        x_dest = destination.position.x
        y_dest = destination.position.y

        delta_y = y_dest - y_start
        delta_x = x_dest - x_start

        distance = math.sqrt((delta_x)**2 + (delta_y)**2)

        theta_radians = 0

        print("Start: (%.3f,%.3f) Target: (%.3f,%.3f)" % (x_start, y_start, x_dest, y_dest))
        print("delta_y: %.3f delta_x: %.3f" % (delta_y, delta_x))

        print "Side Ratio: ",
        print(delta_y / delta_x)

        theta_radians = math.atan(delta_y / delta_x)


        print 'theta_radians: ',
        print(theta_radians)

        if (delta_x < 0):
            if (delta_y > 0):
                theta_radians = math.pi - theta_radians                
            elif (delta_y < 0):
                theta_radians = (-1 * math.pi) - theta_radians
            else:
                theta_radians = math.pi
        elif (delta_x == 0):
            if (delta_y > 0):
                theta_radians = math.pi / 2.0                
            elif (delta_y < 0):
                theta_radians = -1.0 * math.pi / 2.0
            else:
                theta_radians = 0

        print 'theta_radians (adjusted): '
        print(theta_radians)
            
            

        theta_quaternary = theta_radians / math.pi
        
        print 'theta_quaternary: ',
        print(theta_quaternary)

        self.turn_to_angle(theta_quaternary)
        


    # Turns turtlebot to face a specified angle, which is Quaternion.z
    # range is [-1,1] in units of pi*radians, so [-pi,pi]
    def turn_to_angle(self, z, speed = 0.25):
        
        polling_rate = 10 #Hz
        r = rospy.Rate(polling_rate)

        print("Speed: %.3f Radians/s" % (speed))
        print("Start Z: %.3f Target Z: %.3f" % (self.imu.orientation.z, z))


        while not plus_or_minus(z, 0.01, self.imu.orientation.z):
            self.rotate(speed)
            # print("Current Z: %.3f" % (self.pose_with_covariance.pose.orientation.z))
            r.sleep()

        
        print("Final Z: %.3f" % (self.pose_with_covariance.pose.orientation.z))
        self.stop()


    # Move forward `distance` meters at `speed` m/s
    def move_forward(self, distance, speed = 0.3):
        time = abs(distance) / abs(speed);
        polling_rate = 10.0 #Hz
        # send commands at 10Hz
        r = rospy.Rate(polling_rate)

        print("Speed: %i m/s" % (speed))
        print("Distance: %i m" % (distance))
        print("Time: %i s" % (time))

        # Twist is a type representing velocity in both linear x,y,z and angular x,y,z rates
        # linear is in meters/s and angular is in radians/s
        move_cmd = Twist()
        # forward speed in m/s
        move_cmd.linear.x = speed
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0

        time_run = 0

        # as long as you haven't ctrl + c keeping doing...
        while time_run < time:
            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            
            time_run += (1.0 / polling_rate)
            r.sleep()
            
        print('attempting to stop')
        self.stop()

    # rotate a set number of radians relative to current orientation
    def rotate_radians(self, radians, speed = 0.25):
        polling_rate = 10 #Hz
        r = rospy.Rate(polling_rate)

        time = abs(radians) / abs(speed)

        normalized_radians = radians / math.pi

        print("Speed: %.3f Radians/s" % (speed))
        print("Distance: %.3f Radians" % (radians))
        print("Time: %.1f s" % (time))
        time_run = 0.0

        while time_run < time:
            self.rotate(speed)
            time_run += (1.0 / polling_rate)
            r.sleep()

        self.stop()

    def rotate(self, speed = 0.25):
        turn_cmd = Twist()
        turn_cmd.angular.z = speed
        self.cmd_vel.publish(turn_cmd)

    def stop(self):
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)
    
    
    def shutdown(self): 
        # publish to STOP MOVING

        # stop turtlebot
        rospy.loginfo("Stopping TurtleBot")
        self.stop()
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


origin = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_1 = Pose(Point(1.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
point_2 = Pose(Point(-1, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

 
if __name__ == '__main__':
    turtlebot = TurtlebotControl()
    turtlebot.reset_odometry()
    #turtlebot.move_forward(1)
    # turtlebot.rotate_radians(3.141592566758)
    #turtlebot.turn_to_angle(1.00)
    #rospy.sleep(2)
    #turtlebot.move_to_pose(point_1)
    #rospy.sleep(2)
    #turtlebot.move_to_pose(point_2)
    rospy.sleep(1)
    turtlebot.turn_to_angle(0, 0.3)
    rospy.sleep(5)
    turtlebot.move_forward(1)
    rospy.sleep(5)
    turtlebot.turn_to_angle(1, 0.3)
    rospy.sleep(5)
    turtlebot.move_forward(1)
    rospy.sleep(5)
    turtlebot.turn_to_angle(0,0.3)
