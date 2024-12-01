import sys

sys.path.append('build')
sys.path.append('scripts')
import fire
import gin
from map import Map
import json
import numpy as np
from env_search.warehouse import get_packages
from env_search.warehouse.config import WarehouseConfig
from env_search.utils import (
    kiva_env_str2number, get_chute_loc, read_in_kiva_map,
    read_in_sortation_map, sortation_env_str2number, get_n_valid_edges,
    get_n_valid_vertices, DIRS, read_in_sortation_map, sortation_obj_types,
    get_Manhattan_distance_coor, load_pibt_default_config, get_workstation_loc)
import py_driver  # type: ignore # ignore pylance warning
import json


def compress_vertex_matrix(map, vertex_matrix):
    assert map.width * map.height == len(vertex_matrix)
    compressed_vertex_matrix = []
    for y in range(map.height):
        for x in range(map.width):
            pos = y * map.width + x
            if map.graph[y, x] == 1:
                continue
            compressed_vertex_matrix.append(vertex_matrix[pos])
    return compressed_vertex_matrix


def compress_edge_matrix(map, edge_matrix):
    # edge matrix: h*w*[right,up,left,down]
    assert map.width * map.height * 4 == len(edge_matrix)

    compressed_edge_matrix = []
    for y in range(map.height):
        for x in range(map.width):
            pos = y * map.width + x
            if map.graph[y, x] == 1:
                continue

            if (x + 1) < map.width and map.graph[y, x + 1] == 0:  # right
                compressed_edge_matrix.append(edge_matrix[4 * pos + 0])

            if (y - 1) >= 0 and map.graph[y - 1, x] == 0:  # up
                compressed_edge_matrix.append(edge_matrix[4 * pos + 1])

            if (x - 1) >= 0 and map.graph[y, x - 1] == 0:  # left
                compressed_edge_matrix.append(edge_matrix[4 * pos + 2])

            if (y + 1) < map.height and map.graph[y + 1, x] == 0:  # down
                compressed_edge_matrix.append(edge_matrix[4 * pos + 3])
    return compressed_edge_matrix


def uncompress_vertex_matrix(map, compressed_vertex_matrix, fill_value=0):
    j = 0
    vertex_matrix = []
    for y in range(map.height):
        for x in range(map.width):
            if map.graph[y, x] == 1:
                vertex_matrix.append(fill_value)
            else:
                vertex_matrix.append(compressed_vertex_matrix[j])
                j += 1
    return vertex_matrix


def uncompress_edge_matrix(map, compressed_edge_matrix, fill_value=0):
    # edge matrix: h*w*[right,up,left,down]

    j = 0
    edge_matrix = []
    for y in range(map.height):
        for x in range(map.width):
            if map.graph[y, x] == 1:
                for i in range(4):
                    edge_matrix.append(fill_value)
            else:
                if (x + 1) < map.width and map.graph[y, x + 1] == 0:  # right
                    edge_matrix.append(compressed_edge_matrix[j])
                    j += 1
                else:
                    edge_matrix.append(fill_value)

                if (y - 1) >= 0 and map.graph[y - 1, x] == 0:  # up
                    edge_matrix.append(compressed_edge_matrix[j])
                    j += 1
                else:
                    edge_matrix.append(fill_value)

                if (x - 1) >= 0 and map.graph[y, x - 1] == 0:  # left
                    edge_matrix.append(compressed_edge_matrix[j])
                    j += 1
                else:
                    edge_matrix.append(fill_value)

                if (y + 1) < map.height and map.graph[y + 1, x] == 0:  # down
                    edge_matrix.append(compressed_edge_matrix[j])
                    j += 1
                else:
                    edge_matrix.append(fill_value)

    return edge_matrix


def main(warehouse_config, map_filepath, chute_mapping_file, seed=0):

    np.random.seed(seed)

    gin.parse_config_file(warehouse_config)
    warehouse_config = WarehouseConfig()

    # Read in map
    with open(map_filepath, "r") as f:
        raw_env_json = json.load(f)
    map_json_str = json.dumps(raw_env_json)
    map_str, _ = read_in_sortation_map(map_filepath)
    map_np = sortation_env_str2number(map_str)
    h, w = map_np.shape

    # Read in packages, chute mapping
    _, package_dist_weight_json = get_packages(
        warehouse_config.package_mode,
        warehouse_config.package_dist_type,
        warehouse_config.package_path,
        warehouse_config.n_destinations,
    )
    with open(chute_mapping_file, "r") as f:
        chute_mapping_json = json.load(f)
        chute_mapping_json = json.dumps(chute_mapping_json)

    config_path = "configs/pibt_default_no_rot.json"
    with open(config_path) as f:
        config = json.load(f)
        config_str = json.dumps(config)

    # list of destinations
    # n_destinations = 300
    # packages = np.random.randint(0, n_destinations, size=100000).tolist()
    # package_dist_weight = np.random.rand(n_destinations).tolist()
    # print(packages)

    all_chutes = get_chute_loc(map_np).tolist()
    all_workstations = get_workstation_loc(map_np).tolist()
    n_chutes = len(all_chutes)
    print(all_chutes)
    print(all_workstations)
    n_valid_edges = get_n_valid_edges(map_np,
                                      bi_directed=True,
                                      domain="sortation")
    n_valid_vertices = get_n_valid_vertices(map_np, domain="sortation")
    compressed_weights_json_str = json.dumps(np.ones(n_valid_edges).tolist())
    compressed_wait_costs_json_str = json.dumps(
        np.ones(n_valid_vertices).tolist())

    # Task assignment policy
    task_assignment_params = np.random.rand(10).tolist()

    ret = py_driver.run(
        scenario="SORTING",  # one of ["KIVA", "COMPETITION", "SORTING"]
        # For map, it uses map_path by default. If not provided, it'll use map_json
        # which contains json string of the map
        # map_path=map_path,
        map_json_str=map_json_str,
        # map_json_path=map_json_path,
        simulation_steps=5000,
        # for the problem instance we use:
        # if random then we need specify the number of agents and total tasks, also random seed,
        gen_random=True,
        num_agents=800,
        num_tasks=100000,
        seed=seed,
        save_paths=True,
        # weight of the left/right workstation, only applicable for maps with workstations
        left_w_weight=1,
        right_w_weight=1,
        # else we need specify agents and tasks path to load data.
        # agents_path="example_problems/random.domain/agents/random_600.agents",
        # tasks_path="example_problems/random.domain/tasks/random-32-32-20-600.tasks",
        # weights are the edge weights, wait_costs are the vertex wait costs
        # if not specified here, then the program will use the one specified in the config file.
        weights=compressed_weights_json_str,
        wait_costs=compressed_wait_costs_json_str,
        # if we don't load config here, the program will load the default config file.
        config=config_str,
        # the following are some things we don't need to change in the weight optimization case.
        plan_time_limit=
        1,  # in seconds, time limit for planning at each step, no need to change
        preprocess_time_limit=
        1800,  # in seconds, time limit for preprocessing, no need to change
        file_storage_path=
        "large_files/",  # where to store the precomputed large files, no need to change
        task_assignment_strategy=
        "roundrobin",  # how to assign tasks to agents, no need to change
        num_tasks_reveal=1,  # how many new tasks are revealed, no need to change
        # Chute mapping related
        # packages=json.dumps(packages),
        package_dist_weight=package_dist_weight_json,
        package_mode="dist",
        chute_mapping=chute_mapping_json,
        task_assignment_cost="heuristic+num_agents",
        task_assignment_params=json.dumps(task_assignment_params),
        recirc_mechanism=True,
        task_waiting_time=0,
        workstation_waiting_time=0,
        task_change_time=100,
        task_gaussian_sigma=0.01,
        time_sigma=1000,
        time_dist=True,
        # assign_C=15,
    )

    analysis = json.loads(ret)
    print(analysis.keys())
    print(np.array(analysis["tile_usage"]).shape)
    print(np.array(analysis["vertex_wait_matrix"]).shape)
    # print(np.array(analysis["edge_pair_usage"]).shape)

    # print(analysis["throughput"], analysis["edge_pair_usage_mean"],
    #   analysis["edge_pair_usage_std"])
    print("throughput", analysis["throughput"])
    print(analysis["n_finish_task_plus_n_recirs"])
    print(analysis["n_recirs"])
    print(analysis["recirc_rate"])
    print("Chute sleep count ", analysis["chute_sleep_count"])

    # ##### Only use the following for weight opt case #####
    # # because the order of orientation is different in competition code and weight opt code.

    # # h*w*4: 0: right, 1: up, 2: left, 3:down
    # edge_usage_matrix = analysis["edge_usage_matrix"]
    # vertex_wait_matrix = analysis["vertex_wait_matrix"]

    # compressed_vertex_waits = compress_vertex_matrix(map, vertex_wait_matrix)
    # compressed_edge_usages = compress_edge_matrix(map, edge_usage_matrix)

    # # print(compressed_vertex_waits,compressed_edge_usages)

    # # print(len(compressed_vertex_waits),len(compressed_edge_usages))

    # _vertex_waits = uncompress_vertex_matrix(map, compressed_vertex_waits)
    # _edge_usages = uncompress_edge_matrix(map, compressed_edge_usages)

    # assert len(_vertex_waits) == len(vertex_wait_matrix)
    # assert len(_edge_usages) == len(edge_usage_matrix), "{} vs {}".format(
    #     len(_edge_usages), len(edge_usage_matrix))

    # for a, b in zip(_vertex_waits, vertex_wait_matrix):
    #     assert a == b

    # for a, b in zip(_edge_usages, edge_usage_matrix):
    #     assert a == b


if __name__ == "__main__":
    fire.Fire(main)
