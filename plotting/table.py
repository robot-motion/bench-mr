from utils import *
from definitions import *


def latex_table(results_filename: str,
                planners='all',
                row_label: str = '',
                metrics: [str] = ['path_found', 'planning_time', 'path_length', 'curvature', 'cusps'],
                time_limit: float = 3) -> str:
    metric_properties["planning_time"]["max"] = time_limit
    parsed_planners = parse_planners(planners)
    if planners != 'all':
        planners = [planner for planner in planners if planner in parsed_planners]
    else:
        planners = get_planners(results_filename)
    if len(planners) == 0:
        print("Warning: No planners were selected for generating a table.", file=sys.stderr)
        return 'No planners were selected for %s.' % results_filename
    stats = {metric: {planner: [] for planner in planners} for metric in metrics}
    with open(results_filename, 'r') as rf:
        for run in json.load(rf)["runs"]:
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
    output = ''
    if row_label != '':
        output += '\\rowlabel{%s}\n\\\\' % row_label
    for planner in planners:
        output += '%s & %% %s\n' % (latexify(planner).ljust(40), convert_planner_name(planner))
        for i, metric in enumerate(metrics):
            if metric == 'path_found' and safe_sum(stats[metric][planner]) == 0:
                # no paths have been found
                output += '\t0 &\n'
                for j in range(len(metrics) - 1):
                    output += '\tN / A' + ('&' if j < len(metrics) - 2 else '%') + '\n'
                break
            else:
                mu = safe_mean(stats[metric][planner])
                if "max" in metric_properties[metric]:
                    shown_mu = mu / metric_properties[metric]["max"]
                else:
                    shown_mu = mu
                line = '\t{\\databar{%.2f}}' % (min(1., shown_mu))
                if metric_properties[metric].get("show_std", False):
                    line += '\t%.2f \\pm %.2f' % (mu, safe_std(stats[metric][planner]))
                elif metric_properties[metric].get("percent", False):
                    line += '\t%i \\%%' % (mu * 100)
                elif metric_properties[metric].get("sum", False):
                    line += '\t%i' % (safe_sum(stats[metric][planner]))
                else:
                    line += '\t%.2f' % safe_mean(stats[metric][planner])
                output += line + ('&' if i < len(metrics) - 1 else '%') + '\n'
        output += '\\\\\n'
    return output
