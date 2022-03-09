# trash_bot is the top level representation of our turtlebot

from kobuki_base import *
from yolo_subscriber import *
from serial_motor import *
import threading
import time

class TrashBot():
    def __init__(self):
        rospy.init_node('trash_bot')

        self.trash_detector = TrashYoloSubscriber(self.TrashDetected)
        self.kobuki_base = KobukiBase()
        self.collection_mechanism = SerialMotor()
        # self.collection_mechanism.Connect()
        
        self.linear_speed = 0.4 # m/s
        # note: angular velocity direction is positive => counter clockwise
        self.angular_speed = 0.3 # radians/s

    def TrashDetected(self, trash_data):
        print("Deteceted Trash!")
        # print(type(trash_data))

        # isolate the closest piece of trash
        closest_piece = trash_data[0]

        # we define closest as the one with the highest upper bound y value in
        # the frame, since down is positive and bottom right is (640,480)
        if len(trash_data) > 1:
            max_y = 0
            for piece in trash_data:
                if piece.y_bounds[1] > max_y and piece.confidence > 0.6:
                    min_y = piece.y_bounds[0]
                    closest_piece = piece

        print('closest piece is at (%i, %i)' % (closest_piece.x, closest_piece.y))


        # if we aren't confident, stop the turtlebot and do nothing
        if closest_piece.confidence < 0.6:
            self.kobuki_base.stop()
            print('unconfident, not moving')
            return


        # find our reference Z at the time the picture was taken
        reference_z = 0

        # do this by getting the timestamp of the yolo image and find the closest odometry frame
        # with that type
        yolo_stamp = trash_data.timestamp

        closest_pose = None
        closest_time = timestamp
        smallest_difference = None

        # right now this scans through all of them, make it stop in future when it finds a good
        # match
        for (timestamp, pose) in self.kobuki_base.location_data:
            difference_s = math.abs(yolo_stamp.sec - timestamp.sec)
            difference_ns = math.abs(yolo_stamp.nsec - timestamp.nsec)

            ns_to_s = float(difference_ns) * (10**-9)

            difference = difference_s + ns_to_s
            
            if closest_stamp == None:
                closest_stamp = timestamp
                closest_pose = pose
                smallest_difference = difference
                continue
            else:
                if difference < smallest_difference:
                    closest_stamp = timestamp
                    closest_pose = pose
                    smallest_difference = difference
            
        reference_z = closest_pose.orientation.z
        
        destination_angle = find_destination_z(closest_piece.x, reference_z)

        
        '''
        # Now that we've isolated the closest piece, lets figure out which
        # way to turn
        center_x = self.trash_detector.frame_width // 2

        # how much can we can miss the center on either side percentage
        margin_of_error = 0.05
        
        # define bounds of the window which we deem in the center
        window_left = center_x - (margin_of_error * float(center_x))
        window_right = center_x + (margin_of_error * float(center_x))
        
        # if trash left:
        if closest_piece.x <= window_left:
            # turn left
            print('attempting to turn left')
            self.kobuki_base.stop()
            self.kobuki_base.spin_async(self.angular_speed)
        # elif trash right:
        elif closest_piece.x >= window_right:
            # turn right
            print('attempting to turn right')
            self.kobuki_base.stop()
            self.kobuki_base.spin_async(-1 * self.angular_speed)
        # else IT MUST BE IN MIDDLE
        else:
            # firstly, stop rotating
            print('attempting to attack da trash')
            self.kobuki_base.stop()

            # secondly, start the motor
            #self.collection_mechanism.StartMotor()

            # thirdly, move forward one meter
            #self.move_forward(1, self.linear_speed)

            # fourthly, turn off the collection mechanism
            #self.collection_mechanism.StopMotor()
        '''


if __name__ == '__main__':
    t = TrashBot()

    while True:
        continue
