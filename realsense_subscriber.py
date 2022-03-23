# Realsense Subscriber
# Subscribes to realsense2_camera topics

import rospy
from sensor_msgs.msg import *
from fifo import *
from cv_bridge import CvBridge

'''
    PUBLISHERS:
    SUBSCRIBERS:

    -/camera/aligned_depth_to_color/image_raw : Image
'''

class RealsenseSubscriber():
    def __init__(self, init_node = False):
        if init_node:
            rospy.init_node('realsense_subscriber')

        # PUBLISHERS
        # SUBSCRIBERS
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.DepthImageReceivedEvent)

        # save last 9000 frames (30 seconds) of depth camera data 
	# Note: data will be stored in tuples of (timestamp, depth_image)
	# depth_image is indexed like [y][x]        
	self.depth_history = FixedLengthFifo(900)
	
	# convert ros images form strings to opencv images
	self.image_bridge = CvBridge()


    # records
    def DepthImageReceivedEvent(self, depth_image):        
        cv_image = self.image_bridge.imgmsg_to_cv2(depth_image, desired_encoding='16UC1')
        self.depth_history.push((depth_image.header.stamp, cv_image))


    def get_depth_at_pixel(self, image_array, pixel_x, pixel_y):
	raw_depth = image_array[pixel_y][pixel_x]
	corrected_depth = float(raw_depth) * 0.001
	print('Depth at (%f, %f) is %f %f' % (pixel_x, pixel_y, raw_depth, corrected_depth))        
	return corrected_depth # convert from mm to m

if __name__ == '__main__':
    r = RealsenseSubscriber(True)
    distance = [[0]*480]*640
    rospy.sleep(0.3)
    
    print('Calculating distances')
    for x in range(0, 640):
        for y in range(0, 480):
            # print (x,y)
            dist = r.get_depth_at_pixel(r.depth_history[0][1], x, y) * 0.001
            distance[x][y] = dist            
            # print dist, 'm'    
    print('Done Calculating Distances')
    print(distance)
    
    while True:
        continue
    
        
