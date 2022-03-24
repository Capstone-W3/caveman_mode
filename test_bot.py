from trash_bot import *
import rospy

if __name__ == '__main__':
    t = TrashBot()
    print('STARTING UP THE TRASHBOT')
    t.StartUp()
    print('WAITING 10 SECONDS')
    rospy.sleep(10)
    print('STOPPING THE TRASHBOT')
    t.ShutDown()
    print('WAITING 5 SECONDS')
    rospy.sleep(5)
    print('STARTING UP TRASHBOT')
    t.StartUp()
    print('WAITING 5 SECONDS THEN SHUTTING DOWN')
    rospy.sleep(5)
    t.ShutDown()

