#!/usr/bin/env python
import os
import roslaunch
import rospy

BAGS_FOLDER = os.listdir(f"{rospy.get_param('/record_folder/bagpath')}/{rospy.get_param('/record_folder/bagfolder')}/")
LAUNCH_FILE = f"{rospy.get_param('/record_folder/launchpath')}/{rospy.get_param('/record_folder/algorithm')}.launch"

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

for file in BAGS_FOLDER:
    if file.endswith(".bag"):
        print(f"PLAYING BAG: {file}")

        cli_args = [LAUNCH_FILE, f'runname:={file[:-4]}']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        parent.spin()