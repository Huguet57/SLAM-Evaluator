import pandas as pd
import os

ALGORITHMS = [
    "/media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits/gt/",
    "/media/andreu/Movies/ws_fastlio/src/SLAM-Evaluator/data/commits/fast_lio/",
    "/media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits/limovelo/",
]

COMMITS = {}

for ALGORITHM in ALGORITHMS:
    for COMMIT in os.listdir(ALGORITHM):
        commit_name = os.path.basename(os.path.normpath(COMMIT))
        COMMITS[commit_name] = {}

        for file in os.listdir(COMMIT):
            if file.endswith(".csv"):
                COMMITS[commit_name][file] = pd.read_csv(
                    os.path.join(COMMIT, file),
                    header=None,
                    names=["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
                )