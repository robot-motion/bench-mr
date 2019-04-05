from utils import parse_run_ids, parse_steer_functions, parse_planners
import json


def retrieve_planner_stats_by_run(json_file: str, planners: str = 'all', run_id: str = 'all'):
    data = json.load(open(json_file, "r"))
    run_ids = parse_run_ids(run_id, len(data["runs"]))
    all_planners = (planners == 'all')
    planners = parse_planners(planners)
    result = {}
    for run_id in run_ids:
        run = data["runs"][run_id]
        result[run_id] = {}
        if all_planners:
            planners = run["plans"].keys()
        for planner in planners:
            if planner in run["plans"]:
                plan = run["plans"][planner]
                result[run_id][planner] = plan["stats"]
                result[run_id][planner]["steer_function"] = run["settings"]["steer"]["steering_type"]
                result[run_id][planner]["planner"] = planner
                result[run_id][planner]["run_id"] = run_id
                result[run_id][planner]["intermediary"] = []
                if len(plan["intermediary_solutions"]) > 0:
                    for sol in plan["intermediary_solutions"]:
                        inter = sol["stats"]
                        inter["cost"] = sol["cost"]
                        result[run_id][planner]["intermediary"].append(inter)
    return result


def retrieve_planner_stats_by_steering(json_file: str, steer_funcs: str = 'all', planners: str = 'all',
                                       run_id: str = 'all'):
    retrieval = retrieve_planner_stats_by_run(json_file, planners=planners, run_id=run_id)
    steer_funcs = parse_steer_functions(steer_funcs)
    result = {}
    for sf in steer_funcs:
        result[sf] = {}
        for run_id, plans in retrieval.items():
            if plans is None:
                continue
            for planner, stats in plans.items():
                if stats['steer_function'] == sf:
                    if planner not in result[sf]:
                        result[sf][planner] = {}
                    result[sf][planner][stats["run_id"]] = stats
    return result


def write_result_to_json(result: dict, path: str):
    with open('../' + path, 'w') as rj:
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
                useful[key_0][key_1]["curvature"] = result[key_0][key_1][key_2]["curvature"]
                # print(key_0, key_1, useful[key_0][key_1]["curvature"])

                # get path length
                useful[key_0][key_1]["path_length"] = result[key_0][key_1][key_2]["path_length"]

                # get smoothness
                useful[key_0][key_1]["smoothness"] = result[key_0][key_1][key_2]["smoothness"]

                #get planning time
                useful[key_0][key_1]["planning_time"] = result[key_0][key_1][key_2]["planning_time"]

    return useful

def compute_average(useful: dict):
    total_sum = {}
    total_count = {}
    avg_result = {}

    #initialize
    for key_0 in useful:
        for planner in useful[key_0]:
            avg_result[planner] = {}
            total_sum[planner] = {}
            total_count[planner] = {}
            for att in useful[key_0][planner]:
                    avg_result[planner][att] = 0.0
                    total_sum[planner][att] = 0.0
                    total_count[planner][att] = 0

    for key_0 in useful:
        for planner in useful[key_0]:
            for att in useful[key_0][planner]:
                if useful[key_0][planner][att] != None:
                    total_count[planner][att] += 1
                    total_sum[planner][att] += useful[key_0][planner][att]

    for planner in total_sum:
        for att in total_sum[planner]:
            if total_count[planner][att] > 0:
                avg_result[planner][att] = float("%0.4f" % (total_sum[planner][att] / total_count[planner][att]))
            else:
                avg_result[planner][att] = None
    
    return avg_result




result = retrieve_planner_stats_by_steering("../parking1.json")

useful = retrieve_useful_stats_from_result(result)

avg_result = compute_average(useful)

write_result_to_json(result, 'result.json')
write_result_to_json(useful, 'useful.json')
write_result_to_json(avg_result, 'avg.json')
