#!/usr/bin/env python
import os
import roslaunch
import rospy

BAGS_FOLDER = os.listdir(f"{rospy.get_param('/record_folder/bagpath')}/{rospy.get_param('/record_folder/bagfolder')}/")
LAUNCH_FILE = f"{rospy.get_param('/record_folder/launchpath')}/{rospy.get_param('/record_folder/algorithm')}.launch"
SPECIFIC_BAGS = str(rospy.get_param('/record_folder/bags')).split()

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

for file in BAGS_FOLDER:
    if file.endswith(".bag"):
        if len(SPECIFIC_BAGS) > 0:
            found = False
            for bag in SPECIFIC_BAGS:
                if bag in file:
                    print(bag, file)
                    found = True
            
            if not found: continue

        print(f"PLAYING BAG: {file}")

        cli_args = [LAUNCH_FILE, f'runname:={file[:-4]}', f'bagfolder:={rospy.get_param("/record_folder/bagfolder")}', f'commit:={rospy.get_param("/record_folder/commit")}']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        parent.spin()