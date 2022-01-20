# SLAM Evaluator
Evaluator to save and compare results from different SLAM algorithms/commits.

## How does it work?
****Important:**** the algorithms have to detect when the bag ends and *end naturally* in order to work.

## Record a folder of rosbags
Launch the following command to record a folder with your rosbags: ``roslaunch slam_evaluator record_folder.launch bagfolder:="XXX" algorithm:="YYY" commit:="ZZZ" bags:="AA BB CC"``. In ``record_folder.launch`` you will see that you can also define the absolute paths for convinience and for each new algorithm, you have to define its topics in a YAML file.

## Studying data
Run ``data/evalutate.py`` to generate CSV files containing difference with the ground truth data.
Run ``data/compare.py [commit]`` to compare one commit's difference with ground truth with the others.
Run ``data/plot_inter.py [algorithm] [bag1 bag2 bag3 ...]`` to plot the outputs of the algorithm vs. others and ground truth.
