# Yolo Subscriber
# Subscribes to darknet_ros (YOLOv4) topics to determine where the trash at

import rospy
from darknet_ros_msgs.msg import *

'''
    PUBLISHERS:
    SUBSCRIBERS:

    - /darknet_ros/found_object : darknet_ros_msgs/ObjectCount
    - /darknet_ros/bounding_boxes : darknet_ros_msgs/BoundingBoxes

'''
class TrashYoloSubscriber():
    def __init__(self, TrashSpottedCallback = None):
        #rospy.init_node('TrashYoloSubscriber')

        # PUBLISHERS

        # SUBSCRIBERS
        rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.DarknetFoundObjectReceivedEvent)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.DarknetBoundingBoxesReceivedEvent)

        # Keeps track of all trash within the current frame
        # type: List<TrashPiece>
        self.trash = []
        # Keeps track of how many pieces are within frame
        self.count = 0

        # The callback function which we will execute when we see trash
        self.TrashSpottedCallbackFunction = TrashSpottedCallback

        # Dimensions of Camera
        self.frame_width = 640
        self.frame_height = 480
    
    def DarknetFoundObjectReceivedEvent(self, data):
        self.count = data.count

    def DarknetBoundingBoxesReceivedEvent(self, data):
        # print('got a bounding box for trash')
        # print(data)
        # clear the trash list
        self.trash = []

        # Add all trash to the list of currently spotted trash
        for bounding_box in data.bounding_boxes:
            self.trash.append(TrashPiece(bounding_box, data.image_header))

        # print('trash count: %i' % len(self.trash))

        if self.TrashSpottedCallbackFunction != None:
            self.TrashSpottedCallbackFunction(self.trash)


class TrashPiece():
    def __init__(self, bounding_box, header):
        # Confidence that this trash is actually trash
        self.confidence = bounding_box.probability

        self.timestamp = header.stamp

        # Bounding box coordinates
        self.x_bounds = (bounding_box.xmin, bounding_box.xmax)
        self.y_bounds = (bounding_box.ymin, bounding_box.ymax)
        
        # Centered coordinates
        self.x = self.x_bounds[1] - ((self.x_bounds[1] - self.x_bounds[0]) / 2)
        self.y = self.y_bounds[1] - ((self.y_bounds[1] - self.y_bounds[0]) / 2)


if __name__ == '__main__':
    subscribe = TrashYoloSubscriber()

    while True:
        continue
