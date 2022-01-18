import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from read import COMMITS
from math import cos, sin, pi

def rot(xy, yaw):
    return (
        cos(yaw)*xy.x - sin(yaw)*xy.y,
        sin(yaw)*xy.x + cos(yaw)*xy.y
    )

def drot(xy, yaw):
    return (
        - sin(yaw)*xy.x - cos(yaw)*xy.y,
        cos(yaw)*xy.x - sin(yaw)*xy.y
    )

def errors(xys, targets, yaw):
    err = 0
    for j in range(min(len(xys), len(targets))):
        err += error(xys.iloc[j], targets.iloc[j], yaw)
    
    return err

def error(xy, target, yaw):
    return (rot(xy, yaw)[0] - target.x)**2 + (rot(xy, yaw)[1] - target.y)**2

def Drot(xy, target, yaw):
    return 2*(rot(xy, yaw)[0] - target.x)*drot(xy, yaw)[0] + 2*(rot(xy, yaw)[1] - target.y)*drot(xy, yaw)[1]

def gradDesc(xy, target, yaw):    
    err = 0

    for i in range(20):
        D = 0
        err = 0

        for j in range(min(len(xy), len(target))):
            D += Drot(xy.iloc[j], target.iloc[j], yaw)
            err += error(xy.iloc[j], target.iloc[j], yaw)
        
        yaw = yaw - D*1.e-8
        # print(i, yaw, err)
    
    return (yaw, err)
    
def minSquares(xy, target):
    res = []
    N = 4
    for i in range(N):
        res.append(errors(xy, target, 2*pi/N*i))

    print(res)
    m = np.argmin(res)
    yaw = 2*pi/N*m

    yaw, err = gradDesc(xy, target, yaw)
    print("final: ", err)
    return yaw

# Program
if __name__ == '__main__':
    import sys
    target_algorithm = sys.argv[1]

    if len(sys.argv) >= 3: keys = sys.argv[2:]
    else: keys = COMMITS["gt"].keys()

    for key in keys:
        # Interpret key
        for full_key in COMMITS["gt"].keys():
            if key in full_key:
                key = full_key
                break

        counter = 0
        yaw = -minSquares(COMMITS["gt"][key], COMMITS[target_algorithm][key])

        for algorithm in COMMITS:
            if algorithm == "gt": plt.plot(COMMITS[algorithm][key].x, COMMITS[algorithm][key].y, color="red")
            elif key in COMMITS[algorithm]: plt.plot(rot(COMMITS[algorithm][key], yaw)[0], rot(COMMITS[algorithm][key], yaw)[1], color=np.random.rand(3,))
            counter = counter + 1

        plt.title(key)
        plt.legend(COMMITS.keys())
        plt.show()