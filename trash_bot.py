# trash_bot is the top level representation of our turtlebot

from kobuki_base import *
from yolo_subscriber import *
from serial_motor import *
from angle_calculator import *
from realsense_subscriber import *
from image_controller import *
import threading
import time

class TrashBot():
    def __init__(self):
        rospy.init_node('trash_bot')

        # if this is set to true, it will start responding to trash 
        # it sees in the environment via the YOLO subscriber
        self.respond_to_trash = False
        
        self.linear_speed = 0.4 # m/s
        # note: angular velocity direction is positive => counter clockwise
        self.angular_speed = 0.3 # radians/s
        
        self.confidence_threshold = 0.8
        self.locked_on = threading.Lock()
	
        self.depth_camera = RealsenseSubscriber()
        self.kobuki_base = KobukiBase()
        self.collection_mechanism = SerialMotor()
        motor_connected = self.collection_mechanism.Connect()
        self.republisher = ImageController()
        
        print('Motor Connected: %s' % motor_connected)
        
        # declare the trash detector after a delay to ensure that everything
        # else has received at least one frame of data before yolo detects
        # trash and expects everything to be initialized
        rospy.sleep(0.1)
        self.trash_detector = TrashYoloSubscriber(self.TrashDetected)

        # publisher that tells if we are listening to yolo or not
        self.active_publisher = rospy.Publisher('/trash_bot_active', Bool, queue_size=1)

        # how far to overshoot the destination when roving forward to pick up trash
        self.overshoot_distance = 0.305 # meters

    def TrashDetected(self, trash_data):
        
        if (not self.respond_to_trash):
            print('Found trash but I\'m not started up')
            return
        elif (self.locked_on.locked()):
            print('already locked on, ignoring found trash')
            return
        
        print("Detected Trash!")
        # print(type(trash_data))

        # isolate the closest piece of trash
        closest_piece = trash_data[0]

        # we define closest as the one with the highest upper bound y value in
        # the frame, since down is positive and bottom right is (640,480)
        if len(trash_data) > 1:
            max_y = 0
            for piece in trash_data:
                if piece.y_bounds[1] > max_y and piece.confidence > self.confidence_threshold:
                    min_y = piece.y_bounds[0]
                    closest_piece = piece

        # if we aren't confident, stop the turtlebot and do nothing
        if closest_piece.confidence < self.confidence_threshold:
            self.kobuki_base.stop()
            print('unconfident, not moving')
            return

        # If we aren't locked on, acquire the lock so two trash pieces aren't
        # locked onto simultaneously
        if not self.locked_on.locked():
            self.locked_on.acquire()
        else:
            return
 
        print('Closest piece is at (%i, %i)' % (closest_piece.x, closest_piece.y))
        print('Locked on: %s' % self.locked_on.locked())

        # find our reference Z at the time the picture was taken
        reference_z = 0

        # do this by getting the timestamp of the yolo image and find the closest odometry frame
        # with that type
        yolo_stamp = trash_data[0].timestamp

        closest_pose = None
        closest_stamp = None
        smallest_difference = None

        # right now this scans through all of them, make it stop in future when it finds a good
        # match
        for (timestamp, pose) in self.kobuki_base.location_data:
            #print('timestamp: %s, Type: %s' % (timestamp, type(timestamp)))
            #print('yolo_stamp: %s, Type: %s' % (yolo_stamp, type(yolo_stamp)))
            
            time_difference = abs(yolo_stamp - timestamp)

            if closest_stamp == None:
                closest_stamp = timestamp
                closest_pose = pose
                smallest_difference = time_difference
                continue
            else:
                if time_difference < smallest_difference:
                    closest_stamp = timestamp
                    closest_pose = pose
                    smallest_difference = time_difference
 
        closest_image = None
        closest_stamp = None
        smallest_difference = None

        # Find the distance that the trash is away from us by cross-referencing the yolo timestamp
        # with the depth image timestamp
        for (timestamp, image) in self.depth_camera.depth_history:
            time_difference = abs(yolo_stamp - timestamp)

            if closest_stamp == None:
                closest_stamp = timestamp
                closest_image = image
                smallest_difference = time_difference
                continue
            else:
                if time_difference < smallest_difference:
                    closest_stamp = timestamp
                    closest_image = image
                    smallest_difference = time_difference

        # distance_away = closest_image[closest_piece.y][closest_piece.x] # meters, unadjusted for angle
        distance_away = self.depth_camera.get_depth_at_pixel(closest_image, closest_piece.x, closest_piece.y)

        reference_z = closest_pose.orientation.z 
        destination_angle = find_destination_z(closest_piece.x, reference_z, distance_away)
        
        print('Attempting to turn to destination angle %f' % destination_angle)

        self.kobuki_base.turn_to_angle(destination_angle)

        print('Starting the Collection Mechanism')
        self.collection_mechanism.StartMotor()

        # sleeping for half a second here gives the motor time to spin up to max speed	
        rospy.sleep(0.5)

        print('Trash is %3f m away and I want to overshoot %f m, attempting to attack' % (distance_away, self.overshoot_distance))
        self.kobuki_base.move(distance_away + self.overshoot_distance)
        
        print('Stopping the Collection Mechanism')
        self.collection_mechanism.StopMotor()
        
        # Stop listening to YOLO now that we've reached
        self.ShutDown()

        self.locked_on.release() 
        print('locked off')


    def StartUp(self):
        active_message = Bool()
        active_message.data = True
        self.active_publisher.publish(active_message)
        self.respond_to_trash = True

    def ShutDown(self):
        active_message = Bool()
        active_message.data = False
        self.active_publisher.publish(active_message)
        self.kobuki_base.stop()
        self.respond_to_trash = False
        

if __name__ == '__main__':
    t = TrashBot()

    while True:
        continue
