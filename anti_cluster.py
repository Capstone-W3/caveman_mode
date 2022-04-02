#from geometry_msgs.msg import *
#import rospy
#from collections import namedTuple
from multiprocessing.dummy import Array
from operator import truediv
from queue import Empty


class AntiCluster:
    def __init__(self, x, y): #, init_node=False):
        #if init_node:
            #rospy.init_node('aggregate_points')

        #rospy.Subscriber("/trash_mapper/trash_points", PoseStamped, self.addPoint)
        self._XLENGTH_ = x  # m
        self._YLENGTH_ = y  # m
        self.listOfPoints = []
        self.avgTrashPosesTuple = []#: List[Tuple[Pose, int]] = []
        self.avgTrashPoses = [] #: List[Pose] = []

        #self.trash_points_publisher = rospy.Publisher('/trash_mapper/culled_trash_points', PoseArray, queue_size = 1)
        #self.trash_avg_points_publisher = rospy.Publisher('/trash_mapper/avg_culled_trash_points', PoseArray, queue_size = 1)

    def addPoint(self, data):
        # self.listOfPoints.append(data.pose)
        # self.listOfPoints = self.aggregatePoints(self.listOfPoints)
        #print('data: ', data)
        self.aggregatePoseAvg(data)

        #arr = PoseArray()
        # arr = Array()
        # arr.header = data.header
        # arr.poses = self.listOfPoints
        # self.trash_points_publisher.publish(arr)

        #avg_arr = PoseArray()
        #avg_arr = Array()

        #avg_arr.header = data.header
        #avg_arr.poses = self.avgTrashPoses
        #self.trash_avg_points_publisher.publish(avg_arr)
        
        #print('Points: ', self.listOfPoints)
        #print('Avg Points: ', self.avgTrashPosesTuple)
    
    def aggregatePoints(self, points):
        x = 0
        y = 1
        endX = len(points)

        while x < endX:
            y = x + 1
            
            #xRangeTop = points[x].position.x + self._XLENGTH_

            xRangeTop = points[x][0] + self._XLENGTH_
            #print('xrange top', xRangeTop)
            
            #yRangeTop = points[x].position.y + self._YLENGTH_
            yRangeTop = points[x][1] + self._YLENGTH_
            #xRangeBottom = points[x].position.x - self._XLENGTH_
            xRangeBottom = points[x][0] - self._XLENGTH_
            #yRangeBottom = points[x].position.y - self._YLENGTH_
            yRangeBottom = points[x][1] - self._YLENGTH_


            while y < endX:
                if xRangeBottom <= points[y][0] <= xRangeTop and yRangeBottom <= points[y][1] <= yRangeTop:
                    #print(points[x])
                    #print(points[y])
                    points.pop(y)
                    endX = endX - 1
                y = y + 1

            x = x + 1

        return points

    def aggregatePoseAvg(self, pose):
        #print('pose: ', pose)

        if len(self.avgTrashPosesTuple) is 0 :
            #print('inside empty')
            self.avgTrashPosesTuple.append([pose, 1])
            self.avgTrashPoses.append(pose)
        else:
            editedpose = False
            for poseTuple in self.avgTrashPosesTuple:
                #print('existing pose', poseTuple[0])
            #xRangeTop = poseTuple[0][.position.x] + self._XLENGTH_
            #yRangeTop = poseTuple[0].position.y + self._YLENGTH_
            #xRangeBottom = poseTuple[0].position.x - self._XLENGTH_
            #yRangeBottom = poseTuple[0].position.y - self._YLENGTH_
            #xRangeTop = points[x][0] + self._XLENGTH_
            #print('xrange top', xRangeTop)
                xRangeTop = poseTuple[0][0] + self._XLENGTH_
                #print('xrangetop', xRangeTop)
            #yRangeTop = points[x].position.y + self._YLENGTH_
                yRangeTop = poseTuple[0][1] + self._YLENGTH_
                #print('yrangetop', yRangeTop)
            #xRangeBottom = points[x].position.x - self._XLENGTH_
                xRangeBottom = poseTuple[0][0] - self._XLENGTH_
                #print('xrangebot', xRangeBottom)
            #yRangeBottom = points[x].position.y - self._YLENGTH_
                yRangeBottom = poseTuple[0][1] - self._YLENGTH_
                #print('yrangebot', yRangeBottom)
                #print(xRangeBottom <= pose[0] <= xRangeTop and yRangeBottom <= pose[1] <= yRangeTop)
                if xRangeBottom <= pose[0] <= xRangeTop and yRangeBottom <= pose[1] <= yRangeTop:
                    #print('inside edit')
                    #print('pose tuple x', poseTuple[0][0])
                    #print('pose tuple times called', poseTuple[1])
                    #print('pose x', pose[0])
                    #poseTuple = list(poseTuple)
                    #poseTuple[0] = list(poseTuple[0])
                    poseTuple[0][0] = ((poseTuple[0][0] * poseTuple[1]) + pose[0]) / (poseTuple[1] + 1.0)
                    poseTuple[0][1] = ((poseTuple[0][1] * poseTuple[1]) + pose[1]) / (poseTuple[1] + 1.0)
                    poseTuple[1] += 1
                    #poseTuple = tuple(poseTuple)
                    #print('edited posetuple', poseTuple)
                    editedpose = True
                    break
                    #poseTuple[0] = tuple(poseTuple[0])
            if not editedpose:
                self.avgTrashPosesTuple.append([pose, 1])
                self.avgTrashPoses.append(pose)
            
            #print('tuplelist:', self.avgTrashPoses)
        


if __name__ == "__main__":
    
    first = [[1.0, 2.0], [20.0, 4.0], [24.0, 4.0], [20.0, 20.0], [1.0, 1.0], [21.0, 20.0], [40.0, 40.0]]
    print(first)

    
    cluster = AntiCluster(5, 5)#, True)
    for point in first:
        cluster.addPoint(point)
    #print('avg cluster tuple', cluster.avgTrashPosesTuple)
    print('avg cluster', cluster.avgTrashPoses)
    #last = cluster.aggregatePoints(first)
    #print(last)

    #cluster = AntiCluster(1,1, True)

    #while True:
    #    continue
