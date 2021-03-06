import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from angle_calculator import *

# Amerigo Vespucci is an epic 14th century Italian Navigator
class Vespucci():
    def __init__(self, init_node = False):
        if init_node:
            rospy.init_node('vespucci')

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.ShutDown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

        # distance away from the target we should get near
        self.near_target_distance = 0.5

        # list of trash points, constantly updated as it subscibes to the
        # declustered points topic
        self.trash_points = PoseArray()

        # list of trash poses that we save in case another poseArray message comes in
        self.trash_points_saved = None

        # current pose of the robot in the map
        # note: this is a PoseWithCovarianceStamped!!!
        self.current_position = None

        # subscribe to the anti-clustered list of points
        rospy.Subscriber('/trash_mapper/avg_culled_trash_points', PoseArray, self.UpdateTrashPoints)

        # subscribe to AMCL's current robot PoseWithCovarianceStamped
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.UpdatePosition)        

        # distance to get near a trash point
        self.goal_distance = 1.5 # (m)

        # are we moving
        self.started_up = False

        self.points_to_travel_publisher = rospy.Publisher('/vespucci/pickup_points', PoseArray, queue_size=1)

    def UpdateTrashPoints(self, data):
        self.trash_points = data;

    def UpdatePosition(self, data):
        self.current_position = data

    def PublishPickupPoints(self):
        pose_array = PoseArray()
        if self.trash_points_saved != None:
            pose_array.poses = self.trash_points_saved
            pose_array.header = self.trash_points.header
        else:
            pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        self.points_to_travel_publisher.publish(pose_array)

    def GoToPose(self, pose):

        print('Vespucci: GoToPose attempting to go to:')
        print(pose)

        if pose.orientation.w == 0.0:
            pose.orientation.w = 1.0

        self.goal_sent = True
	goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

		# Start moving
        self.move_base.send_goal(goal)

		# Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def GoNearPose(self,  destination_pose):
        goal_pose = get_goal_pose(self.current_position.pose.pose, destination_pose, self.goal_distance)
        print('Vespucci: Goal Pose (GoNearPose):')
        print(goal_pose)

        self.GoToPose(goal_pose)

    # do we have points to go towards still or are we done
    def HasPoints(self):
        if self.trash_points_saved == None:
            return len(self.trash_points.poses) > 0
        else:
            return len(self.trash_points_saved) > 0

    # Navigate "near" the closest trash pose to our current position
    def GoNearClosestPose(self):
        if (self.started_up):
            return Pose()
        else:
            self.started_up = True

        print('Vespucci: Attempting to navigate to nearest trash point')

        if self.trash_points_saved == None:
            self.trash_points_saved = self.trash_points.poses
        elif self.trash_points_saved == []:
            print('No points left')
            return Pose()

        current_pose = self.current_position.pose.pose
        print('Vespucci: Current Position:')
        print(current_pose)
        
        closest_pose = get_closest_pose(self.trash_points_saved, current_pose)
        print('Vespucci: Closest Trash Position:')
        print(closest_pose)

        print()

        print('Vespucci: Sending a movement command')
        self.GoNearPose(closest_pose)

        self.started_up = False
	return closest_pose
	
    def GetFullShortestPath(self):
        if (self.started_up):
            return
        else:
            self.started_up = True

        print('Vespucci: Attempting to navigate to nearest trash point')

        if self.trash_points_saved == None:
            self.trash_points_saved = self.trash_points.poses
        elif self.trash_points_saved == []:
            print('No points left')
            return

        current_pose = self.current_position.pose.pose
        print('Vespucci: Current Position:')
        print(current_pose)
        
	
	heuristicTrashPairs = {} # create a dictionary of the distance between each pair of trash points
        for path in itertools.permutations(self.trash_points_saved, r=2):
            heuristicTrashPairs[(path[0], path[1])] = distance_between_poses(path[0], path[1])
	
	# This will be a pretty good heuristic as long as we only have a couple of trash points
	# It needs to actually check each permutation though so a path of 13 pieces of trash would need 6,227,020,800 iterations
	minCostToAllTrash = 999999
	currentBestPath = []
	for path in itertools.permutations(self.trash_points_saved):

	    costToFirst = distance_between_poses(current_pose, path[0])
	    costForRest = sum( distance_between_poses(path[i], path[i+1]) for i in range(len(path)-1))
	    pathCost = costToFirst+costForRest

	    if pathCost < minCostToCorners: 
	        minCostToCorners = pathCost
		currentBestPath = path

	return currentBestPath


	
	
    def ShutDown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
        self.started_up = False

#if __name__ == '__main__':
