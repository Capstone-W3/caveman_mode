# caveman_mode

Oonga Boonga if see trash turn and pick it up

# Mode Controller Quick Guide

## Running TrashMapper alongside a bag file
Run darknet from its own terminal window
`source ~/darknet_v4_workspace/devel/setup.bash`
`roslaunch darknet_ros trash_darknet_ros.launch`

Open new terminal window and source the darknet workspace for yolo messages

`source ~/darknet_v4_workspace/devel/setup.bash`

Right now I just interface with ModeController in the python terminal

`cd ~/caveman_mode`

`python`

`> from caveman_mode_controller import *`

`> m = ModeController(True)`

`> m.StartMapper()`

Once mapper is started, run the bag file with orbslam path vector data and RGBD images

To run the visualization in RVIZ, use the rviz config file in ~orb_slam2_ws/OTHER_FOLDERS/rviz
Not 100% on the command or file names but its something like
`rosrun rviz rviz -d orb_slam_somethingsomething.rviz`
If you can't get that running use rviz normally and subscribe to the relevant topics

top-level object is ModeController()
This encapsulates TrashBot and TrashMapper and switches between them
ModeController also contains the node ImageController which controls the input data to YOLO and the AntiClutter node which currently takes in the poses from TrashMapper and republishes a PoseArray of trash points (Currently)

`ModeController.StartMapper()` - starts TrashMapper

`ModeController.StopMapper()` - stops TrashMapper

`ModeController.StartTrashBot()` - starts caveman mode

`ModeController.StopTrashBot()` - Stops trashbot mode

`ModeController.Halt()` - Stops both modes and image controller
