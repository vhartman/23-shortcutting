import numpy as np
import matplotlib.pyplot as plt

import csv
import collections
import os

from scipy import interpolate

colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

plt.style.use('./scripts/paper_2.mplstyle')

# loads the data from a single csvfile.
def load_file(filepath):
    all_runs = []
    with open(filepath, newline='\n') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        single_run = []
        for i, row in enumerate(reader):
            data = [float(num) for num in row]
            single_run.append(data)

            if i % 2 == 1:
                all_runs.append(single_run)
                single_run = []

    return all_runs

def plot_folder(foldername, save=False):
    files = [os.path.join(foldername, f) for f in os.listdir(foldername) if os.path.isfile(os.path.join(foldername, f))]

    scenario_name = " ".join(foldername.split('/')[-1].split("_")[:-2])
    plot_files(files, scenario_name)

    if save:
        experiment = foldername.split('/')[-1]
        plt.savefig(f'./out/plots/{experiment}.png', format='png', dpi=300, bbox_inches = 'tight')

def interpolate_at_pts(new_x, x, y):
    f = interpolate.interp1d(x, y, kind='zero', fill_value=y[-1], bounds_error=False)
    new_y = f(new_x)
    return new_y

def plot_files(filenames, scenario_name):
    plt.figure(figsize=(8, 5))
    for i, filename in enumerate(filenames):
        method_name = filename.split('/')[-1].split('.')[-2]
        d = load_file(filename)

        set_label = False
        max_time = 0
        min_time = 1
        for time, length in d:
            min_time = min([time[1], min_time])
            max_time = max([time[-1], max_time])

        x = np.linspace(.9*min_time, max_time, 10000)
        ys = []
        for time, length in d:
            ys.append(interpolate_at_pts(x, time, length))

        plt.semilogx(x, np.median(ys, axis=0), label=method_name, drawstyle="steps-post")
        #plt.semilogx(x, np.median(ys, axis=0), label=method_name.replace("_", r"\_"), drawstyle="steps-post")
        plt.fill_between(x, np.quantile(ys, 0.05, axis=0), np.quantile(ys, 0.95, axis=0), alpha=0.5)

    plt.title(scenario_name)
    plt.grid(which="both", linestyle='--')

    plt.legend()

def main():
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/Wall_0_20231110_230909"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/Wall_1_20231110_230909"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MoveTable_0_20231110_231028"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MoveTable_1_20231110_231028"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/svetlana_0_20231110_233704"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/svetlana_1_20231110_233704"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_0_20231110_233821"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231110_233821"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_0_20231111_000727"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_000727"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_001106"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/quim_0_20231111_000631"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/quim_1_20231110_234036"
    #folder="/home/valentin/git/personal-projects/23-shortcutting/out/quim_1_20231111_000631"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/FourRooms_0_20231111_001550"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/FourRooms_1_20231111_001550"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/puzzle_1_20231111_001808"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/BrickWall_0_20231111_001940"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/BrickWall_1_20231111_001940"
    folder= "/home/valentin/git/personal-projects/23-shortcutting/out/Handover_1_20231110_230542"

    folder= "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_0_20231111_003028"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_003028"

    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_0_20231111_000727"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_000727"

    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_0_20231111_125445"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/BrickWall_0_20231111_130406"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/BrickWall_1_20231111_130406"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/BrickWall_1_20231111_130749"

    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_0_20231111_131031"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_131031"

    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/Handover_0_20231111_131218"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/Handover_0_20231111_131218"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/Handover_1_20231111_131218"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MoveTable_0_20231111_174024"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MoveTable_1_20231111_174024"

    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_210655"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_214446"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231111_215632"

    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MultipleSofa_1_20231111_221144"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/cage_2d_1_20231113_173958"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/move_table_walls_1_20231113_180818"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/rotate_cube_1_20231113_234146"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MultipleSofa_1_20231115_153356"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/move_table_walls_1_20231115_153457"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MultipleSofa_1_20231115_154952"
    folder = "/home/valentin/git/personal-projects/23-shortcutting/out/MultipleSofa_1_20231115_161529"
    #folder = "/home/valentin/git/personal-projects/23-shortcutting/out/sofa_1_20231115_161631"

    plot_folder(folder, save=True)

    plt.show()

main()
