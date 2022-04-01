from geometry_msgs.msg import *
import rospy
from collections import namedTuple

class AntiCluster:
    def __init__(self, x, y, init_node=False):
        if init_node:
            rospy.init_node('aggregate_points')

        rospy.Subscriber("/trash_mapper/trash_points", PoseStamped, self.addPoint)
        self._XLENGTH_ = x  # m
        self._YLENGTH_ = y  # m
        self.listOfPoints = []
        self.avgTrashPosesTuple : List[Tuple[Pose, int]] = []
        self.avgTrashPoses : List[Pose] = []

        self.trash_points_publisher = rospy.Publisher('/trash_mapper/culled_trash_points', PoseArray, queue_size = 1)
        self.trash_avg_points_publisher = rospy.Publisher('/trash_mapper/avg_culled_trash_points', PoseArray, queue_size = 1)

    def addPoint(self, data):
        self.listOfPoints.append(data.pose)
        self.listOfPoints = self.aggregatePoints(self.listOfPoints)
        self.aggregatePoseAvg(data)

        arr = PoseArray()
        arr.header = data.header
        arr.poses = self.listOfPoints
        self.trash_points_publisher.publish(arr)

        avg_arr = PoseArray()
        avg_arr.header = data.header
        avg_arr.poses = self.avgTrashPoses
        self.trash_avg_points_publisher.publish(avg_arr)
	print('Points: %s' % self.listOfPoints)
        print('Avg Points: %s' % self.avgTrashPoses)
    
    def aggregatePoints(self, points):
        x = 0
        y = 1
        endX = len(points)

        while x < endX:
            y = x + 1
            xRangeTop = points[x].position.x + self._XLENGTH_
            yRangeTop = points[x].position.y + self._YLENGTH_
            xRangeBottom = points[x].position.x - self._XLENGTH_
            yRangeBottom = points[x].position.y - self._YLENGTH_

            while y < endX:
                if xRangeBottom <= points[y].position.x <= xRangeTop and yRangeBottom <= points[y].position.y <= yRangeTop:
                    #print(points[x])
                    #print(points[y])
                    points.pop(y)
                    endX = endX - 1
                y = y + 1

            x = x + 1

        return points

    def aggregatePointsAvg(self, pose : Pose):

        for poseTuple in self.avgTrashPosesTuple:
            
            xRangeTop = poseTuple[0].position.x + self._XLENGTH_
            yRangeTop = poseTuple[0].position.y + self._YLENGTH_
            xRangeBottom = poseTuple[0].position.x - self._XLENGTH_
            yRangeBottom = poseTuple[0].position.y - self._YLENGTH_
            if xRangeBottom <= pose.position.x <= xRangeTop and yRangeBottom <= pose.position.y <= yRangeTop:
               
                poseTuple[0].position.x = ((poseTuple[0].position.x * poseTuple[1]) + pose.position.x)/ (poseTuple[1] + 1)
                poseTuple[0].position.y = ((poseTuple[0].position.y * poseTuple[1]) + pose.position.y)/ (poseTuple[1] + 1)
                poseTuple[1] += 1
            else:
               self.avgTrashPosesTuple.push(pose)
               self.avgTrashPoses.push(pose)


if __name__ == "__main__":
    '''
    first = [(1, 2), (20, 4), (3, 20), (20, 20), (1, 1), (21, 20), (40, 40)]
    print(first)
    
    cluster = AntiCluster(0.2, 0.2, True)
    last = cluster.aggregatePoints(first)
    print(last)
    '''
    cluster = AntiCluster(1,1, True)

    while True:
        continue
