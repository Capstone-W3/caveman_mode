from trash_bot import *
import rospy

if __name__ == '__main__':
    t = TrashBot()


    # start up the trashbot and pay attention to YOLO
    t.StartUp()
    print('Waiting until pickup')

    # pause until we aren't running anymore
    # ie: the robot has found and picked up a piece of trash
    while t.running:
        rospy.sleep(0.1)
    print('PIECE 1 picked up (hopefully)')
    
    print('TURNING AROUND')

    # turn around (this goes to a defined angle, just for testing)
    t.kobuki_base.turn_to_angle(-0.95)

    # start up the bot again
    t.StartUp()

    print('WAITING FOR ANOTHER DETECTION')

    # pause until we aren't runningh anymore
    # ie: the robot has found and picked up a piece of trash
    while t.running:
        rospy.sleep(0.1)

    print('PIECE 2 picked up (hopefully)')
    print('Sleeping for 5 seconds and exiting')

    rospy.sleep(5)
