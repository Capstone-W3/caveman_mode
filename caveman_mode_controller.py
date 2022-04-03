from trash_bot import *
from trash_mapper import *
from anti_cluster import *
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
        self.trash_mapper.StopListeningToYolo()
        self.trash_bot.ShutDown()

if __name__ == '__main__':
    m = ModeController(True)
    rospy.sleep(3) # sleep 3s to let everything bring up
    m.StartMapper()
    while(True):
        continue
