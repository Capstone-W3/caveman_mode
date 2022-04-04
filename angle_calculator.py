# Figure out relative angle based on x coordinate of a picture within
# a frame

import math
import rospy
from geometry_msgs.msg import *

# frame_width is the width of the frame used by YOLO

frame_width = 640 # pixels
camera_fov = 69.4 # degrees
half_fov = camera_fov / 2.0
leg_length = 462.139 # leg length between center of  frame and camera
camera_kobuki_distance = 0.6096 # meters between kobuki and camera


def find_destination_z(pixel_x, reference_z, distance_from_camera):
    # left is positive in terms of z angle

    # returns relative angle between pixel_x and camera in units of radians
    def angle_to_turn(pixel_x):
        
        theta_degrees = 0

        if pixel_x < frame_width / 2:
            side_length = (frame_width / 2) - pixel_x
            theta_radians = math.atan(side_length / leg_length)
        elif pixel_x > frame_width / 2:
            side_length = pixel_x - (frame_width / 2)
            theta_radians = math.atan(side_length / leg_length) * -1
        else:
            theta_radians = 0

        # print('X: %i Theta (radians): %f' % (pixel_x, theta_radians))

        theta_degrees = (theta_radians * 180.0) / math.pi
        
        # print('X: %i Theta (degrees): %f' % (pixel_x, theta_degrees))

        theta_kobuki = theta_radians / math.pi

        return theta_radians

    angle_relative_to_camera = angle_to_turn(pixel_x)

    # beta is the angle between the leg spanning the distance from the camera
    # to the trash and the leg between the camera and the center of the kobuki
    beta = math.pi - abs(angle_relative_to_camera)

    # since we have the distance from the camera to the trash, the distance from
    # the kobuki to the camera, and the angle between these legs, we can use
    # the side-angle-side trigonometric theorem and then the law of sins
    # to find the angle between the trash and the kobuki
    # math explanation below
    # we are solving for side c first, the leg between the kobuki and the trash piece
    # a => camera_kobuki_distance
    # b => distance_from_camera
    # c^2 = a^2 + b^2 - 2ab*cos(beta)
    c = math.sqrt(camera_kobuki_distance**2 + distance_from_camera**2 - (2 * camera_kobuki_distance * distance_from_camera * math.cos(beta)))

    # we can now use the law of sins to find the angle between trash and kobuki
    # sin(relative_to_kobuki)/distance_from_camera = sin(beta)/c
    # sin(relative_to_kobuki) = (distance_from_camera * sin(beta))/c
    # relative_to_kobuki = sin^-1((distance_from_camera*sin(beta)))/c
    to_be_arcsined = (distance_from_camera * math.sin(beta)) / c
    relative_to_kobuki = math.asin(to_be_arcsined)

    # remember to negate the angle relative to the kobuki if the angle relative to the
    # camera is negative
    if angle_relative_to_camera < 0:
        relative_to_kobuki = -1 * relative_to_kobuki
    
    # adjust to units of pi * radians
    relative_to_kobuki_normalized = relative_to_kobuki / math.pi
    
    destination_z = reference_z + relative_to_kobuki_normalized


    

    # since angles are in terms of pi * radians,
    # adding or subtracting 2 (2pi) gets the same angle
    # but within the bounds of [-1,1]/[-pi,pi]
    if (destination_z > 1):
        final_destination = destination_z - 2
    elif (destination_z < -1):
        final_destination = destination_z + 2
    else:
        final_destination = destination_z

    return (final_destination, c)


def angle_is_between(input_z, bound_cw, bound_ccw):
    
    cw_bound_sign = bound_cw >= 0
    ccw_bound_sign = bound_ccw >= 0

    # if we have the same sign, just make sure its between the two values
    if cw_bound_sign == ccw_bound_sign:
        #print("same sign")
	return input_z >= bound_cw and input_z < bound_ccw    
    elif bound_cw > 0.5:
        #print("diff sign, crossing -1/1 border")
        # else if the clockwise bound is > 0.5, which is only possible
        # if the bounds are on the opposite side of the -1/1 border
        return input_z >= bound_cw or input_z <= bound_ccw
    else:
        #print("diff sign, crossing 0 border")
        # this case only triggers if the bounds are on opposite sides of 0, which
        # normal math works for
        return input_z >= bound_cw and input_z < bound_ccw

def within_plus_or_minus(val, target, margin):
    if margin < 0:
        margin = -1 * margin
    return val >= (target - margin) and (val <= target + margin)

def distance_between_points(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


# returns a pose `distance away` and between start_pose and destination_pose,
# with the angle of the returned pose pointing towards the destination point
def get_goal_pose(start_pose, destination_pose, distance_from_goal):
    start_x = start_pose.position.x
    start_y = start_pose.position.y

    dest_x = destination_pose.position.x
    dest_y = destination_pose.position.y

    # angle between the line formed by these two points and the x axis
    theta_radians = math.atan((dest_y - start_y) / (dest_x - start_x))
    
    # normalize this to the unit sphere by dividing pi
    theta_quat = theta_radians / math.pi

    # distance between the two points
    distance = distance_between_points(start_x, start_y, dest_x, dest_y)

    destination_pose = Pose()
    destination_pose.orientation.z = theta_quat
    
    distance_to_go = 0

    if distance > distance_from_goal:
        distance_to_go = distance - distance_from_goal

    near_x = start_x + (distance_to_go * math.cos(theta_radians))
    near_y = start_y + (distance_to_go * math.sin(theta_radians))

    destination_pose.position.x = near_x
    destination_pose.position.y = near_y
    
    return destination_pose

def distance_between_poses(pose1, pose2):
    x_1 = pose1.position.x
    y_1 = pose1.position.y

    x_2 = pose2.position.x
    y_2 = pose2.position.y

    return distance_between_points(x_1, y_1, x_2, y_2)


# returns the closest pose to `pose` in the `pose_list`, also POPs the closest
# pose from the list
def get_closest_pose(pose_list, pose):
    closest_pose = None
    closest_index = 0
    closest_distance = float(99999999.9999)

    # iterate through all the poses and find the closest
    for i in range(len(pose_list)):
        distance = distance_between_poses(pose, pose_list[i])
        
        if distance < closest_distance:
            closest_distance = distance
            closest_pose = pose_list[i]
            closest_index = i

    if len(pose_list) > 0:
        pose_list.pop(closest_index)

    return closest_pose

if __name__ == '__main__':
    #print('Simulating a trash found at pixel_x = 400 1m away, reference_z = 0')
    #print('Angle relative to world and distance from base: %s' % find_destination_z(400, 0.00, 1.00))
    start_pose = Pose()
    dest_pose = Pose()

    start_pose.position.x = 0
    start_pose.position.y = 0
    start_pose.orientation.z = -0.3

    dest_pose.position.x = 2
    dest_pose.position.y = 2
    dest_pose.orientation.z = 0.557

    distance_from_goal = 0.5

    print('Start Pose:')
    print(start_pose)
    print('Destination Pose:')
    print(dest_pose)
    print('Goal Pose (should be %f m away)' % distance_from_goal)
    goal_pose = get_goal_pose(start_pose, dest_pose, distance_from_goal)
    print(goal_pose)

    print('Actual Distance between goal pose and destination pose: %f m' % distance_between_points(goal_pose.position.x, goal_pose.position.y, dest_pose.position.x, dest_pose.position.y))

    test_pose_1 = Pose()
    test_pose_1.position.x = -1
    test_pose_1.position.y = -1

    test_pose_2 = Pose()
    test_pose_2.position.x = 10
    
    test_pose_3 = Pose()
    test_pose_3.position.y = 4

    pose_list = [start_pose, dest_pose, test_pose_2, test_pose_3]

    print('\n\n')

    print('TESTING GETTING CLOSEST POSE:')
    print('STARTING POSE: %s' % test_pose_1)

    i = 0

    while(len(pose_list) > 0):
        print('Iteration %i :' % i)
        #print(pose_list)
        print('Closest Pose:')
        print(get_closest_pose(pose_list, test_pose_1))
        print('\n')
        i += 1
