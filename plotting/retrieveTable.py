from utils import parse_run_ids, parse_steer_functions, parse_planners
from retrieve import retrieve_planner_stats_by_run, retrieve_planner_stats_by_steering
import json

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




result = retrieve_planner_stats_by_steering("../results/warehouse.json")

useful = retrieve_useful_stats_from_result(result)

avg_result = compute_average(useful)

write_result_to_json(result, '../docs/clean_results/clean_result_warehouse.json')
write_result_to_json(useful, '../useful_results/useful_result_warehouse.json')
write_result_to_json(avg_result, '../avg_results/avg_result_warehouse.json')