# Figure out relative angle based on x coordinate of a picture within
# a frame

import math

# frame_width is the width of the frame used by YOLO

frame_width = 640 # pixels
camera_fov = 69.4 # degrees
half_fov = camera_fov / 2.0
leg_length = 462.139 # leg length between center of  frame and camera


def find_destination_z(pixel_x, reference_z):
    # left is positive in terms of z angle

    # returns relative angle between pixel_x and camera in units of
    # pi * radians, which is what the turtlebot's rotation odometry
    # uses
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

        return theta_kobuki

    angle_to_move = angle_to_turn(pixel_x)

    destination_z = reference_z + angle_to_move

    # since angles are in terms of pi * radians,
    # adding or subtracting 2 (2pi) gets the same angle
    # but within the bounds of [-1,1]/[-pi,pi]
    if (destination_z > 1):
        final_destination = destination_z - 2
    elif (destination_z < -1):
        final_destination = destination_z + 2
    else:
        final_destination = destination_z

    return final_destination


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

if __name__ == '__main__':
    xs = [i for i in range(frame_width)]
    angle_dict = {}

    for pixel in xs:
        angle_dict[pixel] = angle_to_turn(pixel)

    for key in angle_dict:
        print('X: %i, Angle: %f' % (key, angle_dict[key]))
