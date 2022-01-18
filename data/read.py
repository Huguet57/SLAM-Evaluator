import pandas as pd
import os

COMMIT_FOLDERS = [
    "/media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits/gt/",
    "/media/andreu/Movies/ws_fastlio/src/SLAM-Evaluator/data/commits/fast_lio/",
    "/media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits/limovelo/7afca02dc5e7505eaac35a68e9a17090a787f320/",
    "/media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits/limovelo/489917e1278ac2adf5d4fcb617646f3d0374d882/"
]

COMMITS = {}

for folder in COMMIT_FOLDERS:
    commit_name = os.path.basename(os.path.normpath(folder))
    COMMITS[commit_name] = {}

    for file in os.listdir(folder):
        if file.endswith(".csv"):
            COMMITS[commit_name][file] = pd.read_csv(
                os.path.join(folder, file),
                header=None,
                names=["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
            )