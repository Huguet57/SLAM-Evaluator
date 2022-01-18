import os
import pandas as pd
import numpy as np
import quaternion
import matplotlib.pyplot as plt
from math import cos, sin, pi
from copy import copy

DATA_FOLDER = "/media/andreu/Movies/ws_limovelo/src/slam_evaluator/data/commits"

class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'

def compare_run(old, new):
    # Improvement
    abs_improvement = new.perc - old.perc
    rel_improvement = abs_improvement/old.perc
    
    # Give absolute values to calculate mean improvement
    return abs_improvement < 0, abs_improvement, rel_improvement*100, new.xy_error, new.xy_total, old.xy_error, old.xy_total

def add_info_to_summary(info, summary, runname):
    better, abs_imp, rel_imp, new_err, new_total, old_err, old_total = info
    betters, worses = summary

    # Too short, can bring noisy percentages
    if new_total < 10: return

    if better:
        betters[runname] = [abs_imp, rel_imp, new_err, new_total]
        if "MEAN" in betters: betters["MEAN"] += np.asarray([new_err, new_total, old_err, old_total])
        else: betters["MEAN"] = np.asarray([new_err, new_total, old_err, old_total])
    else:
        worses[runname] = [abs_imp, rel_imp, new_err, new_total]
        if "MEAN" in worses: worses["MEAN"] += np.asarray([new_err, new_total, old_err, old_total])
        else: worses["MEAN"] = np.asarray([new_err, new_total, old_err, old_total])

def calculate_means(summary):
    betters, worses = summary
    total_mean = np.asarray([0.,0.,0.,0.])

    total_mean += betters["MEAN"]
    new_err, new_total, old_err, old_total = betters["MEAN"].tolist()
    better_improvement = new_err/new_total - old_err/old_total
    del betters["MEAN"]

    total_mean += worses["MEAN"]
    new_err, new_total, old_err, old_total = worses["MEAN"].tolist()
    worse_improvement = new_err/new_total - old_err/old_total
    del worses["MEAN"]
    
    new_err, new_total, old_err, old_total = total_mean.tolist()
    total_improvement = new_err/new_total - old_err/old_total

    return total_improvement*100, better_improvement*100, worse_improvement*100

def interpret_improvement(improvement, absolute = False):
    rounded = round(improvement, 2)

    if absolute:
        if rounded < -5: return color.GREEN + str(rounded) + color.END
        elif rounded < -2: return color.CYAN + str(rounded) + color.END
        elif rounded > 5: return color.RED + '+' + str(rounded) + color.END
        elif rounded > 2: return color.YELLOW + '+' + str(rounded) + color.END
    else:
        if rounded < -20: return color.GREEN + str(rounded) + color.END
        elif rounded < -10: return color.CYAN + str(rounded) + color.END
        elif rounded > 20: return color.RED + '+' + str(rounded) + color.END
        elif rounded > 10: return color.YELLOW + '+' + str(rounded) + color.END
    
    if rounded > 0: return '+' + str(rounded)
    else: return str(rounded)

def interpret_meters(mts):
    if mts < 10: return color.RED + str(round(mts)) + 'm' + color.END
    if mts < 20: return color.YELLOW + str(round(mts)) + 'm' + color.END
    return str(round(mts)) + 'm'

def print_summary(name, summary, topN):
    betters, worses = summary
    total_mean, better_mean, worse_mean = calculate_means(summary)

    betters = dict(sorted(betters.items(), key=lambda item: item[1][1]))
    worses = dict(sorted(worses.items(), key=lambda item: item[1][1], reverse=True))

    print(f'{color.UNDERLINE}Commit "{color.BOLD}{name}{color.END}"{color.END}\n')

    count = 0
    print(f"\tIs better than {color.BOLD}{name}{color.END} in {color.BLUE}{len(betters)}{color.END} runs.")
    print(f"\t\tTop {color.BOLD}{min(len(betters), topN)}{color.END} runs:")
    for key, improvement in betters.items():
        count = count + 1
        if count <= topN: 
            print(end="\t\t\t")
            print(f"{key} ({interpret_meters(improvement[3])}): {interpret_improvement(improvement[1])}% ({interpret_improvement(improvement[0], True)}% abs.)")

    print(f"\n\t\tMean improvement when is better: {color.GREEN}{round(better_mean, 2)}%{color.END}\n")

    count = 0
    print(f"\tIs worse than {color.BOLD}{name}{color.END} in {color.BLUE}{len(worses)}{color.END} runs.")
    print(f"\t\tBottom {color.BOLD}{min(len(worses), topN)}{color.END} runs:")
    for key, improvement in worses.items():
        count = count + 1
        if count <= topN:
            print(end="\t\t\t")
            print(f"{key} ({interpret_meters(improvement[3])}): {interpret_improvement(improvement[1])}% ({interpret_improvement(improvement[0], True)}% abs.)")

    print(f"\n\t\tMean unimprovement when is worse: {color.RED}+{round(worse_mean, 2)}%{color.END}")
    
    if total_mean < 0: print(f"\n\tTotal improvement vs. {color.BOLD}{name}{color.END}: {color.GREEN}{round(total_mean, 2)}%{color.END}\n")
    else: print(f"\n\tTotal unimprovement vs. {color.BOLD}{name}{color.END}: {color.RED}+{round(total_mean, 2)}%{color.END}\n")

    return total_mean < 0

# Program
if __name__ == '__main__':
    commits = {}
    import sys
    new_commit = sys.argv[1]
    if len(sys.argv) > 2: topN = int(sys.argv[2])
    else: topN = 1e8

    for file in os.listdir(DATA_FOLDER):
        if file.endswith(".csv"):
            results = pd.read_csv(os.path.join(DATA_FOLDER, file))
            commits[file[:-4]] = results

    successes = 0

    for name, old_commit in commits.items():
        old_commit = old_commit.sort_values(by=['run'])
        if new_commit in name: continue
        summary = [{}, {}]
        
        for run, old_values in old_commit.iterrows():
            if old_values.run != commits[new_commit].iloc[run].run: continue
            info = compare_run(old_values, commits[new_commit].iloc[run])
            add_info_to_summary(info, summary, old_values.run)

        successes += print_summary(name, summary, topN)

    print(f"Total successes: {color.GREEN}{successes}{color.END} and failures: {color.RED}{len(commits)-1-successes}{color.END}")