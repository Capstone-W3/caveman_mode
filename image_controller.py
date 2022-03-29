import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *

# Image Controller
# Acts as a `valve` between the realsense and YOLOv4 to limit the amount of images it has
# to process to cut down on battery life and processing time

'''
PUBLISHED TOPICS:

- `/image_controller/republisher` -- republishes the rgb image from
                                     the realsense when we want it to
                                     message type is `Image`
SUBSCRIBED TOPICS:

- `/camera/color/image_raw`       -- RGB image feed from the kobuki
                                  -- message type is `Image`
                                  -- callback is ImageReceivedEvent

- `/trash_bot_active`             -- Is the trash bot looking for YOLO?
                                  -- message type is `Bool`
                                  -- callback is TrashBotActiveReceivedEvent
'''

class ImageController:
    def __init__(self, init_node = False, default_value = False):
        if init_node:
            rospy.init_node('image_controller')

        # are we republishing images?
        self.republishing = default_value

        # PUBLISHERS
        self.republisher = rospy.Publisher('/image_controller/republisher', Image, queue_size=1)

        # SUBSCRIBERS
        rospy.Subscriber("/camera/color/image_raw", Image, self.ImageReceivedEvent)
        rospy.Subscriber("/trash_bot_active", Bool, self.TrashBotActiveReceivedEvent)

        # FPS of input camera
        self.fps = 30
        # amount of frames to skip between republishing
        self.frame_skips = 30
        # count of processed frames
        self.frame_count = 0
    
    def ImageReceivedEvent(self, data):
        if self.republishing:
            self.frame_count += 1
            if self.frame_count % self.frame_skips == 0:
                self.republisher.publish(data)

    def TrashBotActiveReceivedEvent(self, data):
        print 'Image Controller Republishing Status: ',
        if data.data:
            print('Republishing')
        else:
            print('Waiting')
        self.republishing = data.data

if __name__ == '__main__':
    i = ImageController(True, True)
    while True:
        continue
