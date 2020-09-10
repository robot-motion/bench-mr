from utils import parse_run_ids, parse_steer_functions, parse_planners
from retrieve import retrieve_planner_stats_by_run, retrieve_planner_stats_by_steering
from trajectory import visualize
from plot_stats import plot_smoother_stats
from plot_stats import plot_planner_stats
import json
import os
import statistics
import matplotlib as mpl
mpl.rcParams['mathtext.fontset'] = 'cm'
mpl.rcParams['pdf.fonttype'] = 42

def write_result_to_json(result: dict, path: str):
    with open(path, 'w') as rj:
        json.dump(result, rj)

def retrieve_useful_stats_from_result(result: dict):
    useful = {}
    # key_0 is run id ?
    for key_0 in result:
        useful[key_0] = {}
        # key_1 is planner
        for key_1 in result[key_0]:
            useful[key_0][key_1] = {}
            # key_2 is also run id??
            for key_2 in result[key_0][key_1]:
                # get curvature
                useful[key_0][key_1]["max_curvature"] = result[key_0][key_1][key_2]["max_curvature"]

                # get path length
                useful[key_0][key_1]["path_length"] = result[key_0][key_1][key_2]["path_length"]

                # get smoothness
                useful[key_0][key_1]["smoothness"] = result[key_0][key_1][key_2]["smoothness"]

                #get planning time
                useful[key_0][key_1]["planning_time"] = result[key_0][key_1][key_2]["planning_time"]

    return useful

def compute_average_and_std(useful: dict):
    total_sum = {}
    total_count = {}
    avg_result = {}
    result_list = {}
    std_result = {}

    #initialize
    for key_0 in useful:
        for planner in useful[key_0]:
            avg_result[planner] = {}
            total_sum[planner] = {}
            total_count[planner] = {}
            result_list[planner] = {}
            std_result[planner] = {}
            for att in useful[key_0][planner]:
                    avg_result[planner][att] = 0.0
                    total_sum[planner][att] = 0.0
                    total_count[planner][att] = 0
                    result_list[planner][att] = []
                    std_result[planner][att] = 0.0

    for key_0 in useful:
        for planner in useful[key_0]:
            for att in useful[key_0][planner]:
                if useful[key_0][planner][att] != None:
                    total_count[planner][att] += 1
                    total_sum[planner][att] += useful[key_0][planner][att]
                    result_list[planner][att].append(useful[key_0][planner][att])
                    # print(key_0, planner, att, result_list[planner][att])

    for planner in result_list:
        for att in result_list[planner]:
            # print(planner, att, result_list[planner][att])
            if len(result_list[planner][att]) > 1:
                # print(result_list[planner][att])
                std_result[planner][att] = statistics.stdev(result_list[planner][att])
            else:
                if len(result_list[planner][att]) == 0:
                    std_result[planner][att] = None
                else:
                    std_result[planner][att] = 0.0

    for planner in total_sum:
        for att in total_sum[planner]:
            if total_count[planner][att] > 0:
                avg_result[planner][att] = float("%0.4f" % (total_sum[planner][att] / total_count[planner][att]))
            else:
                avg_result[planner][att] = None
    
    for planner in std_result:
        for att in std_result[planner]:
            if std_result[planner][att] != None:
                std_result[planner][att] = float("%0.4f" % std_result[planner][att])
    return avg_result, std_result

def get_directory():
    file_list = []
    for fname in os.listdir('../results'):
        if fname != 'smoothers.json':
            file_list.append(fname)
    return file_list

with open('../docs/scenario_list.json', 'w') as rj:
    json.dump(get_directory(), rj)

for fname in os.listdir('../results'):
    if fname == 'smoothers.json':
        continue
    
    print(fname)

    try:
        result = retrieve_planner_stats_by_steering('../results/' + fname)
        useful_result = retrieve_useful_stats_from_result(result)
        avg_result, std_result = compute_average_and_std(useful_result)

        write_result_to_json(avg_result, '../docs/avg_results/avg_result_' + fname)
        write_result_to_json(std_result, '../docs/std_results/std_result_' + fname)
    except Exception:
        print('Error parsing: ' + fname)
        continue
'''
# plot planners stats violin plots
for fname in os.listdir('../results'):
    print("Plotting planner stats for: " + fname)
    plot_planner_stats('../results/%s' % fname,
                    combine_views=True,
                    plot_violins=True,
                    save_file="../docs/violin_plots/%s" % fname.replace('.json', '.png'),
                    ticks_rotation=90,
                    max_plots_per_line=3,
                    fig_width=10,
                    fig_height=7,
                    ignore_planners="SBPL_ADstar,SBPL_ARstar,SBPL_MHA"
    )
    print('done')

# plot smoother trajectory
for fname in os.listdir('../results'):
    if fname == 'smoothers.json':
        continue

    try:
        visualize('../results/%s' % fname,
                plot_every_nth_polygon=0,
                set_title=False,
                max_plots_per_line=2,
                combine_views=True,
                draw_nodes=True,
                draw_arrows=True,
                fig_width=6, fig_height=6,
                run_id='0,1,3,4',
                line_width=2,
                line_alpha=1,
                node_alpha=0.3,
                #draw_cusps=True,
                show_smoother=True,
                color_map_name='tab10',
                max_colors=10,
                draw_start_goal_thetas=True,
                #show_only_smoother=True,
                # ignore_smoothers="GRIPS, bspline, simplifymax, shortcut",
                ignore_smoothers='CHOMP, bspline, shortcut',
                save_file='../docs/smoothers_trajectory_plots/%s' % fname.replace('.json', '.png')
        )
    except Exception:
        print("Error visualizing " + fname)


# plot smoother stats seperated
plot_smoother_stats('../results/smoothers.json',
                    combine_views=True,
                    plot_violins=True,
                    fig_width=25,
                    max_plots_per_line=3,
                    separate_planners=True,
                    #ignore_smoothers='chomp',
                    save_file='../docs/smoothers_stats/smoothers_stats_seperated.png'
)

# plot smoother stats merged
plot_smoother_stats('../results/smoothers.json',
                    combine_views=True,
                    plot_violins=True,
                    fig_width=15,
                    max_plots_per_line=3,
                    separate_planners=False,
                    ignore_smoothers='chomp',
                    save_file='../docs/smoothers_stats/smoothers_stats_merged.png'
)

'''