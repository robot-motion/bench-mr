from utils import *
from definitions import *
import sys

def json_table(results_filename: str,
                planners='all',
                row_label: str = '',
                metrics: [str] = ['path_found', 'planning_time', 'path_length', 'max_curvature', 'mean_clearing_distance',
                                  'cusps'],
                time_limit: float = 3) -> str:
    for metric in metrics:
        metric_properties[metric]["max"] = 1e-8
        metric_properties[metric]["min"] = 1e8
    parsed_planners = parse_planners(planners)
    if planners != 'all':
        planners = [planner for planner in planners if planner in parsed_planners]
    else:
        planners = get_planners(results_filename)
    if len(planners) == 0:
        print("Warning: No planners were selected for generating a table.", file=sys.stderr)
        return 'No planners were selected for %s.' % results_filename
    stats = {metric: {planner: [] for planner in planners} for metric in metrics}
    stats["colliding"] = {planner: [] for planner in planners}
    total_runs = 1
    with open(results_filename, 'r') as rf:
        runs = json.load(rf)["runs"]
        total_runs = len(runs)
        for run in runs:
            for planner, plan in run["plans"].items():
                if planner not in planners:
                    continue
                for metric in metrics:
                    if metric == "path_found":
                        stats[metric][planner].append(int(plan["stats"][metric]))
                    elif metric == "cusps":
                        stats[metric][planner].append(len(plan["stats"][metric]))
                    else:
                        stats[metric][planner].append(plan["stats"][metric])
                stats["colliding"][planner].append(1 - int(plan["stats"]["path_collides"]))
    for metric in metrics:
        metric_properties[metric]["max"] = safe_max([safe_mean(stats[metric][planner]) for planner in planners])
        metric_properties[metric]["min"] = safe_min([safe_mean(stats[metric][planner]) for planner in planners])
        
    metric_properties["planning_time"]["max"] = time_limit
    
    output = ''
    output += '[\n'
    if row_label != '':
        output += '\\rowlabel{%s}\n\\\\\n' % row_label

    check = ['time', 'path_length', 'max_curvature', 'clearance', 'cusps']

    id = 1
    max_sol = 0
    bar_max = [0, 0, 0, 0, 0]
    for planner in planners:
        output += '\t{\"id\":%d, \"planner\":\"%s\",' % (id, planner)
        for i, metric in enumerate(metrics):
            if metric == 'path_found' and safe_sum(stats[metric][planner]) == 0:
                # no paths have been found
                output += ' \"solutions\":\"0\", \"time\":\"N / A\", \"path_length\":\"N / A\", \"curvature\":\"N / A\", \"clearance\":\"N / A\", \"cusps\":\"N / A\"'
                break
            elif metric == 'path_found':
                nc = safe_sum(stats["colliding"][planner])
                pf = safe_sum(stats["path_found"][planner])
                max_sol = max(max_sol, pf)
                output += ' \"solutions\":\"%i\",' % nc      
            else:
                mu = safe_mean(stats[metric][planner])
                if "max" in metric_properties[metric]:
                    shown_mu = mu / metric_properties[metric]["max"]
                else:
                    shown_mu = mu
                if metric_properties[metric].get("show_std", False):
                    output += ' \"%s\":\"%.2f \xB1 %.2f \"' % (check[i-1], mu, safe_std(stats[metric][planner]))
                    bar_max[i-1] = max(bar_max[i-1], mu)
                elif metric_properties[metric].get("percent", False):
                    output += ' \"%s\":\"%i\"' % (check[i-1], (mu * 100))
                    bar_max[i-1] = max(bar_max[i-1], mu * 100)
                elif metric_properties[metric].get("sum", False):
                    output += ' \"%s\":\"%i\"' % (check[i-1], safe_sum(stats[metric][planner]))
                    bar_max[i-1] = max(bar_max[i-1], safe_sum(stats[metric][planner]))
                else:
                    output += ' \"%s\":\"%i\"' % (check[i-1], safe_mean(stats[metric][planner]))
                    bar_max[i-1] = max(bar_max[i-1], safe_mean(stats[metric][planner]))
                if i < len(metrics) - 1 :
                    output += ','
        output += '},'
        output += '\n'
    output += '\t{\"max_sol\":%i, \"max_t\":%.2f, \"max_pl\":%.2f,  \"max_curv\":%.2f, \"max_clear\":%.2f, \"max_cusps\": %.2f}\n' \
                % (max_sol, bar_max[0], bar_max[1], bar_max[2], bar_max[3], bar_max[4])
    output += ']'
    return output


def main() :
    out_file = open("docs/table_json/moving_ai_berlin_256_reeds_shepp_table.json", "w")
    output = json_table("results/moving_ai_berlin_256_reeds_shepp.json")
    out_file.write(output)

main()