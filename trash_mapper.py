from image_controller import *
from angle_calculator import *
from realsense_subscriber import *
from yolo_subscriber import *
from kobuki_base import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
import rospy
import math

class TrashMapper():
    def __init__(self, init_node = False):
        if init_node:
            rospy.init_node('trash_mapper')
        
        # self.kobuki_base = KobukiBase()
        self.yolo_subscriber = TrashYoloSubscriber(self.TrashDetected)
        # self.image_controller = ImageController(False, False)
        self.depth_camera = RealsenseSubscriber()

        # are we responding to trash
        self.respond_to_trash = False

        # publisher that tells if we want to listen to yolo or not
        self.active_publisher = rospy.Publisher('/trash_bot_active', Bool, queue_size=1)
        
        # publisher that publishes trash poses in the world when we detect one
        self.trash_point_publisher = rospy.Publisher('/trash_mapper/trash_points', PoseStamped, queue_size=1)
        
        # how confident do we need to be
        self.confidence_threshold = 0.85

        # subscriber to orbslam path topic
        rospy.Subscriber('/cam_path', Path, self.OrbslamPathReceivedEvent)

        # Path that the bot has traveled.
        # List of PoseStamped
        self.path = []

    # update our local path every time orbslammyboi gives us an update of where
    # in the world he's been
    def OrbslamPathReceivedEvent(self, orb_path):
        print('TrashMapper: Received ORBSLAM path with %i poses' % len(orb_path.poses))
        self.path = orb_path.poses

    def TrashDetected(self, trash_data):
        if (not self.respond_to_trash):
            print('TrashMapper: Found trash but I\'m not supposed to be looking')
            return

        # keep track of all the pieces we are confident about
        confident_pieces = []

        for item in trash_data:
            if item.confidence > self.confidence_threshold:
                confident_pieces.append(item)

        if len(confident_pieces) > 0:
            print('TrashMapper: Trash Detected!')
        else:
            return

        # find our reference Z at the time the picture was taken
        reference_z = 0

        # do this by getting the timestamp of the yolo image and find the closest odometry frame
        # with that type
        yolo_stamp = trash_data[0].timestamp

        # This generates points based on odometry and is replaced with ORBSLAM
        # map position data
        '''

        closest_pose = None
        closest_odom_stamp = None
        smallest_difference = None

        # right now this scans through all of them, make it stop in future when it finds a good
        # match
        for (timestamp, pose) in self.kobuki_base.location_data:
            #print('timestamp: %s, Type: %s' % (timestamp, type(timestamp)))
            #print('yolo_stamp: %s, Type: %s' % (yolo_stamp, type(yolo_stamp)))
            
            time_difference = abs(yolo_stamp - timestamp)

            if closest_odom_stamp == None:
                closest_odom_stamp = timestamp
                closest_pose = pose
                smallest_difference = time_difference
                continue
            else:
                if time_difference < smallest_difference:
                    closest_odom_stamp = timestamp
                    closest_pose = pose
                    smallest_difference = time_difference
 
        '''

        closest_pose = None
        closest_stamp = None
        smallest_difference = None
        closest_header = None

        # get the pose closest to the yolo timestamp in the path vector that we
        # store from ORBSLAMMYBOI
        for pose_stamped in self.path:

            orb_stamp = pose_stamped.header.stamp
            
            time_difference = abs(yolo_stamp - orb_stamp)

            if closest_stamp == None:
                closest_stamp = orb_stamp
                closest_pose = pose_stamped.pose
                closest_header = pose_stamped.header
                smallest_difference = time_difference
                continue
            else:
                if time_difference < smallest_difference:
                    closest_stamp = orb_stamp
                    closest_pose = pose_stamped.pose
                    closest_header = pose_stamped.header
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

        # rotation when picture was taken
        reference_z = closest_pose.orientation.z

        # position of the bot (REPLACE THIS WITH ODOM AND TRANSFORM FUNNINESS)
        #listener = tf.transformListener();
        #pos, quat = listener.lookupTransform("odom", "map", closest_odom_stamp)
        #print('pos', pos)
        #print('quat',quat)
        
        # These are based on odometry, replaced with ORBSLAM map position
        #bot_x = closest_pose.position.x
        #bot_y = closest_pose.position.y

        bot_x = closest_pose.position.x
        bot_y = closest_pose.position.y
        
        # find the map location of each piece and add it to self.trash_points
        for piece in confident_pieces:
            # get the exact distance away from us and the angle relative to the bot
            distance_away = self.depth_camera.get_depth_at_pixel(closest_image, piece.x, piece.y)
            
            (angle_from_bot_location, distance_from_base) = find_destination_z(piece.x, reference_z, distance_away)
            
            angle_radians = angle_from_bot_location * math.pi

            trash_x = bot_x + (distance_from_base * math.cos(angle_radians))
            trash_y = bot_y + (distance_from_base * math.sin(angle_radians))
            
            # self.trash_points.append((trash_x, trash_y))
            trash_point = PoseStamped()
            trash_point.pose.position.x = trash_x
            trash_point.pose.position.y = trash_y
            trash_point.header = closest_header
            self.trash_point_publisher.publish(trash_point)

    # sends a message to image_controller to start feeding yolo data
    # also sets self.respond_to_trash to true
    def StartListeningToYolo(self): 
        active_message = Bool()
        active_message.data = True
        self.active_publisher.publish(active_message)
        self.respond_to_trash = True

    # stop listening to yolo and tell image controller to shut up and not
    # republish any more images
    def StopListeningToYolo(self): 
        active_message = Bool()
        active_message.data = False
        self.active_publisher.publish(active_message)
        self.respond_to_trash = False


if __name__ == '__main__':
    mapper = TrashMapper(True)
    rospy.sleep(3)
    mapper.StartListeningToYolo()

    while True:
        continue

    
