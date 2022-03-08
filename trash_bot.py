# trash_bot is the top level representation of our turtlebot

from move_bot import *
from yolo_subscriber import *
from serial_motor import *



class TrashBot():
    def __init__(self):
        rospy.init_node('trash_bot')

        self.trash_detector = TrashYoloSubscriber(self.TrashDetected)
        self.kobuki_base = TurtlebotControl()
        self.collection_mechanism = SerialMotor()
        # self.collection_mechanism.Connect()
        
        self.linear_speed = 0.4 # m/s
        # note: angular velocity direction is positive => counter clockwise
        self.angular_speed = 0.3 # radians/s

    def TrashDetected(self, trash_data):
        print("Deteceted Trash in TrashBot.TrashDetected()")
        print(type(trash_data))

        # isolate the closest piece of trash
        closest_piece = trash_data[0]

        # we define closest as the one with the lowest average Y value in
        # the frame
        if len(trash_data) > 1:
            min_y = self.trash_detector.frame_height + 1
            for piece in trash_data:
                if piece.y_bounds[0] < min_y and piece.confidence > 0.6:
                    min_y = piece.y_bounds[0]
                    closest_piece = piece

        # if we aren't confident, stop the turtlebot and do nothing
        if closest_piece.confidence < 0.6:
            self.kobuki_base.stop()
            print('unconfident, not moving')
            return

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
            self.kobuki_base.rotate(self.angular_speed)
        # elif trash right:
        elif closest_piece.x >= window_right:
            # turn right
            print('attempting to turn right')
            self.kobuki_base.rotate(-1 * self.angular_speed)
        # else IT MUST BE IN MIDDLE
        else:
            # firstly, stop rotating
            print('attempting to attack da trash')
            self.stop()

            # secondly, start the motor
            #self.collection_mechanism.StartMotor()

            # thirdly, move forward one meter
            #self.move_forward(1, self.linear_speed)

            # fourthly, turn off the collection mechanism
            #self.collection_mechanism.StopMotor()



if __name__ == '__main__':
    t = TrashBot()

    while True:
        continue
