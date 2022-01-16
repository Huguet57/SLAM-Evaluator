# SLAM Evaluator
Evaluator to save and compare results from different SLAM algorithms/commits.

## How does it work?
Run the ``record_folder.py`` script found in the scripts folder.
This will run the ``record_bag.launch`` where you can specify via the ``config`` folder which algorithm and parameters are you using.
****Important:**** the algorithms have to detect when the bag ends and *end naturally* in order to work.

## Record a folder of rosbags
Launch the following command to record a folder with your rosbags: ``roslaunch slam_evaluator record_folder.launch folder:="XXX" algorithm:="YYY"``. In ``record_folder.launch`` you will see that you can also define the absolute paths for convinience and for each new algorithm, you have to define its topics in a YAML file.
