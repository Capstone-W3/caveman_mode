from trash_bot import *
from trash_mapper import *
from anti_cluster import *
from vespucci import *
import rospy
from geometry_msgs.msg import *

# Controls caveman mode modes
# can only be in one mode at a time
class ModeController():
    def __init__(self, init_node = False):
        if init_node:
            rospy.init_node('caveman_mode_controller')

	self.image_controller = ImageController(False, False)
        self.trash_bot = TrashBot()
        self.trash_mapper = TrashMapper()
        self.anti_cluster = AntiCluster(1.0,1.0, False)
        self.navigator = Vespucci()
        print('ModeController: Initialized')

        self.nav_start_pose = None

        self.nav_start_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    def StartMapper(self):
        print("ModeController: Starting Mapper")
        self.trash_bot.ShutDown()
        self.trash_mapper.StartListeningToYolo()

    def StopMapper(self):
        print("ModeController: Stopping Mapper")
        self.trash_mapper.StopListeningToYolo()
        print('Mapped Points (anti-clustered):' % self.anti_cluster.listOfPoints)
        poses = self.trash_mapper.path

        if len(poses) == 0:
            return

        pose_stamped = poses[len(poses) - 1]

        self.nav_start_pose = PoseWithCovarianceStamped()
        self.nav_start_pose.pose.pose = pose_stamped.pose
        self.nav_start_pose.header = pose_stamped.header

    def StartTrashBot(self, direction):
        print("ModeController: Starting TrashBot")
        self.trash_mapper.StopListeningToYolo()
        self.trash_bot.StartUp(direction)

    def StopTrashBot(self):
        print("ModeController: Stopping TrashBot")
        self.trash_bot.ShutDown()

    def Halt(self):
        print("ModeController: Halting")
        self.navigator.ShutDown()
        self.trash_mapper.StopListeningToYolo()
        self.trash_bot.ShutDown()

    def PickUpTrash(self):
        print('ModeController: PickUpTrash()')
        print('Publishing initial pose for Vespucci...')
        print(self.nav_start_pose)

        if self.nav_start_pose != None:
            self.nav_start_publisher.publish(self.nav_start_pose)
        
        print('Sleeping for 5s to let AMCL catch up...')
        rospy.sleep(5)
        
        refresh_rate = 10 #Hz
        caveman_time_limit = 10 # (seconds)
        r = rospy.Rate(refresh_rate)
        dt = 0

        print('ModeController: Attempting to pick up trash')

        # while there are still points to go to
        while(self.navigator.HasPoints()):
            # publishthe posearray of the points we need to get
            self.navigator.PublishPickupPoints()

            # go near the closest trash point and return where that trash point is
            trash_pose = self.navigator.GoNearClosestPose()

            curr_pose = self.navigator.current_position.pose.pose

            start_x = curr_pose.position.x
            start_y = curr_pose.position.y

            dest_x = trash_pose.position.x
            dest_y = trash_pose.position.y
    
            # RADIANS
            theta_trash = math.atan((start_y - dest_y) / (start_x - dest_x))
            
            delta_x = dest_x - start_x
            delta_y = dest_y - start_y
            
            if (delta_x < 0):
                if (delta_y > 0):
                    theta_trash = math.pi + theta_trash
                elif(delta_y < 0):
                    theta_trash = (-1 * math.pi) + theta_trash

            # convert radians to unit sphere
            theta_trash = theta_trash / math.pi
            
            sub = theta_trash - curr_pose.orientation.z

            curr_z = theta_trash
            destination_z = curr_pose.orientation.z

            # These variables track if we need to overlap over the -1/1 angle boundary
            # in the course of our turn on on the shortest path
            overlap_cw = False
            overlap_ccw = False

            # if the start and destination are in quadrants 2 and 3
            if (curr_z > 0.5) and (destination_z < -0.5):
                # move COUNTER CLOCKWISE
                overlap_ccw = True
            elif (curr_z < -0.5) and (destination_z > 0.5):
                # move CLOCKWISE
                overlap_cw = True
            else:
                overlap_cw = False
                overlap_ccw = False

            direction = False


            if overlap_ccw:
                direction = True
            elif overlap_cw:
                direction = False
            elif sub > 0:
                direction = True          

            # start up the TrashBot
            dt = 0
            self.StartTrashBot(direction)

            # while the bot is running and time elapsed hasnt breached our limit,
            # wait
            # the trashbot should toggle `trash_bot.running` if it successfully
            # finds and attacks a piece of trash and shuts down, otherwise we 
            # just observe the time limit and manually shut trashbot down
            while(self.trash_bot.running and dt < caveman_time_limit):
                dt += 1.0/float(refresh_rate)
                print('dt: %f' % dt)
                r.sleep()

            if dt >= caveman_time_limit:
                self.trash_bot.ShutDown()

            
        print('ModeController: Done picking up trash i hope')
            


if __name__ == '__main__':
    m = ModeController(True)
    rospy.sleep(3) # sleep 3s to let everything bring up
    m.StartMapper()
    while(True):
        continue
