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
class YoloSubscriber():
    def __init__(self):
        rospy.init_node('YoloSubscriber')

        # PUBLISHERS

        # SUBSCRIBERS
        rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.DarknetFoundObjectReceivedEvent)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.DarknetBoundingBoxesReceivedEvent)

    def DarknetFoundObjectReceivedEvent(self, data):
        #print('found trash lol')
        #print(data)
        print()

    def DarknetBoundingBoxesReceivedEvent(self, data):
        print('got a bounding box for trash')
        print(data)
        self.trash = data


if __name__ == '__main__':
    subscribe = YoloSubscriber()

    while True:
        continue
