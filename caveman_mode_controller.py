from trash_bot import *
from trash_mapper import *
from anti_cluster import *
from vespucci import *
import rospy

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

    def StartMapper(self):
        print("ModeController: Starting Mapper")
        self.trash_bot.ShutDown()
        self.trash_mapper.StartListeningToYolo()

    def StopMapper(self):
        print("ModeController: Stopping Mapper")
        self.trash_mapper.StopListeningToYolo()
        print('Mapped Points (anti-clustered):' % self.anti_cluster.listOfPoints)

    def StartTrashBot(self):
        print("ModeController: Starting TrashBot")
        self.trash_mapper.StopListeningToYolo()
        self.trash_bot.StartUp()

    def StopTrashBot(self):
        print("ModeController: Stopping TrashBot")
        self.trash_bot.ShutDown()

    def Halt(self):
        print("ModeController: Halting")
        self.navigator.ShutDown()
        self.trash_mapper.StopListeningToYolo()
        self.trash_bot.ShutDown()

    def PickUpTrash(self):
        refresh_rate = 10 #Hz
        caveman_time_limit = 10 (seconds)
        r = rospy.Rate(refresh_rate)
        dt = 0

        print('ModeController: Attempting to pick up trash')

        # while there are still points to go to
        while(self.navigator.HasPoints()):
            # go near the closest trash point
            self.navigator.GoNearClosestPose()

            # start up the TrashBot
            dt = 0
            self.StartTrashBot()

            # while the bot is running and time elapsed hasnt breached our limit,
            # wait
            # the trashbot should toggle `trash_bot.running` if it successfully
            # finds and attacks a piece of trash and shuts down, otherwise we 
            # just observe the time limit and manually shut trashbot down
            while(self.trash_bot.running and dt < caveman_time_limit):
                dt += 1.0/float(refresh_rate)
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
