from trash_bot import *
from trash_mapper import *
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

    def StartMapper(self):
        self.trash_bot.ShutDown()
        self.trash_mapper.StartListeningToYolo()

    def StopMapper(self):
        self.trash_mapper.StopListeningToYolo()

    def StartTrashBot(self):
        self.trash_mapper.StopListeningToYolo()
        self.trash_bot.StartUp()

    def StopTrashBot(self):
        self.trash_bot.ShutDown()

    def Halt(self):
        self.trash_mapper.StopListeningToYolo()
        self.trash_bot.ShutDown()
    
