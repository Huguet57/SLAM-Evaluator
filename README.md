# SLAM Evaluator
Evaluator to save and compare results from different SLAM algorithms/commits.

## How does it work?
****Important:**** the algorithms have to detect when the bag ends and *end naturally* in order to work.

## Record a folder of rosbags
Launch the following command to record a folder with your rosbags: ``roslaunch slam_evaluator record_folder.launch bagfolder:="XXX" algorithm:="YYY"``. In ``record_folder.launch`` you will see that you can also define the absolute paths for convinience and for each new algorithm, you have to define its topics in a YAML file.
