import pandas as pd
import numpy as np
import quaternion
import matplotlib.pyplot as plt
from read import COMMITS
from math import cos, sin, pi
from copy import copy

def interpolate(b1, a, b2):
    # Interpolated b
    bint = copy(b1)
    bint.stamp = a.stamp

    # 0: b1, 1: b2
    alpha = (bint.stamp - b1.stamp)/(b2.stamp - b1.stamp)

    # R3 interpolation
    bint.x = (1-alpha)*b1.x + alpha*b2.x
    bint.y = (1-alpha)*b1.y + alpha*b2.y
    bint.z = (1-alpha)*b1.z + alpha*b2.z

    # Quaternion slerp interpolation
    q1 = np.quaternion(b1.qw, b1.qx, b1.qy, b1.qz)
    q2 = np.quaternion(b2.qw, b2.qx, b2.qy, b2.qz)
    qint = quaternion.slerp_evaluate(q1, q2, alpha)

    bint.qw = qint.w
    bint.qx = qint.x
    bint.qy = qint.y
    bint.qz = qint.z

    return bint

def construct_int(ours, gts):
    op = 0
    gp = 0

    bint = copy(gts)
    counter = 0

    while gp < len(gts) and op < len(ours) - 1:
        if ours.iloc[op + 1].stamp < gts.iloc[gp].stamp:
            op += 1
            continue
        
        if gts.iloc[gp].stamp < ours.iloc[op].stamp:
            gp += 1
            continue
        
        # Add to data frame
        bint.iloc[counter] = interpolate(ours.iloc[op], gts.iloc[gp], ours.iloc[op + 1])
        
        counter += 1
        op += 1
        gp += 1

    return bint.head(counter)

def localDiff(a1, a2, b1, b2):
    qa = np.quaternion(a1.qw, a1.qx, a1.qy, a1.qz)
    qb = np.quaternion(b1.qw, b1.qx, b1.qy, b1.qz)

    da = np.array([(a2-a1).x, (a2-a1).y, (a2-a1).z])
    db = np.array([(b2-b1).x, (b2-b1).y, (b2-b1).z])

    local_diff = np.matmul(quaternion.as_rotation_matrix(qa).transpose(), da) - np.matmul(quaternion.as_rotation_matrix(qb).transpose(), db)
    return local_diff

def point_first_odom(ours, gt):
    op = 0
    gp = 0

    while op < len(ours) and gp < len(gt):
        if abs(ours.iloc[op].stamp - gt.iloc[gp].stamp) < 1e-4: break
        elif ours.iloc[op].stamp < gt.iloc[gp].stamp: op += 1
        elif ours.iloc[op].stamp > gt.iloc[gp].stamp: gp += 1

    return op, gp

# Program
if __name__ == '__main__':    
    import sys
    if len(sys.argv) > 1: custom_algorithm = sys.argv[1]
    else: custom_algorithm = None

    for algorithm in COMMITS:
        if algorithm == "gt": continue
        if not custom_algorithm is None and algorithm != custom_algorithm: continue

        df = pd.DataFrame(columns=["run", "xy_error", "xy_total", "perc"])
        print(f"Algorithm: {algorithm}")
        print("Progress: ", end="")

        for key in COMMITS["gt"]:
            ours_int = construct_int(COMMITS[algorithm][key], COMMITS["gt"][key])
            op, gp = point_first_odom(ours_int, COMMITS["gt"][key])

            ours_int = ours_int[op:]
            gts_ours = COMMITS["gt"][key][gp:]

            xy_err = 0
            xy_total = 0
            N = min(len(ours_int), len(gts_ours))

            for i in range(N-1):
                diff = localDiff(ours_int.iloc[i], ours_int.iloc[i+1], gts_ours.iloc[i], gts_ours.iloc[i+1])
                xy_err += np.linalg.norm(diff[:2])
                xy_total += (gts_ours.iloc[i].x - gts_ours.iloc[i+1].x)**2 + (gts_ours.iloc[i].y - gts_ours.iloc[i+1].y)**2

            row = pd.DataFrame(
                [[key, xy_err, xy_total, xy_err/xy_total*100]],
                columns=["run", "xy_error", "xy_total", "perc"]
            )

            df = df.append(row, ignore_index=True)
            
            print("I", end="", flush=True)

        # Save to CSV
        df.to_csv(
            f"/media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits/{algorithm}.csv",
            index=False
        )

        print(f"\nSaved in /media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits/{algorithm}.csv")