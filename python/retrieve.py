from utils import parse_run_ids, parse_steer_functions, parse_planners
import json


def retrieve_planner_stats_by_run(json_file: str, planners: str = 'all', run_id: str = 'all'):
    file = open(json_file, "r")
    data = json.load(file)
    file.close()
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
                if plan is None or "stats" not in plan:
                    continue
                result[run_id][planner] = plan["stats"]
                result[run_id][planner]["steer_function"] = run["settings"]["steer"]["steering_type"]
                result[run_id][planner]["planner"] = planner
                result[run_id][planner]["run_id"] = run_id
                result[run_id][planner]["intermediary"] = []
                if plan["intermediary_solutions"] is not None and len(plan["intermediary_solutions"]) > 0:
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
