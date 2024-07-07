import sys
sys.path.append('build')
sys.path.append('scripts')
from map import Map
import json

map_path="example_problems/warehouse.domain/maps/kiva_large_w_mode.map"
# full_weight_path="scripts/random_weight_001.w"
with_wait_costs=True

map=Map(map_path)
map.print_graph(map.graph)

map_json_path = "../maps/warehouse/human/kiva_large_w_mode.json"
with open(map_json_path, "r") as f:
    map_json = json.load(f)
    map_json_str = json.dumps(map_json)

import py_driver # type: ignore # ignore pylance warning
print(py_driver.playground())

import json
config_path="configs/pibt_default_no_rot.json"
with open(config_path) as f:
    config=json.load(f)
    config_str=json.dumps(config)

ret=py_driver.run(
    # For map, it uses map_path by default. If not provided, it'll use map_json
    # which contains json string of the map
    # map_path=map_path,
    # map_json_str = map_json_str,
    map_json_path = map_json_path,
    simulation_steps=2,
    # for the problem instance we use:
    # if random then we need specify the number of agents and total tasks, also random seed,
    gen_random=True,
    num_agents=10,
    init_task=True, 
    init_task_ids=str([719, 1008, 1008, 1008, 1125, 792, 503, 1043, 1151, 468]), 
    init_agent=True,  
    init_agent_pos=str([1123, 485, 165, 694, 718, 357, 890, 845, 640, 623]), 
    # network_params=str([1, ]*2693), 
    num_tasks=100000,
    seed=0,
    save_paths=True,
    # weight of the left/right workstation, only applicable for maps with workstations
    left_w_weight=1,
    right_w_weight=1,
    # else we need specify agents and tasks path to load data.
    # agents_path="example_problems/random.domain/agents/random_600.agents",
    # tasks_path="example_problems/random.domain/tasks/random-32-32-20-600.tasks",
    # weights are the edge weights, wait_costs are the vertex wait costs
    # if not specified here, then the program will use the one specified in the config file.
    # weights=compressed_weights_json_str,
    # wait_costs=compressed_wait_costs_json_str,    
    # if we don't load config here, the program will load the default config file.
    config=config_str,    
    # the following are some things we don't need to change in the weight optimization case.
    plan_time_limit=1, # in seconds, time limit for planning at each step, no need to change
    preprocess_time_limit=1800, # in seconds, time limit for preprocessing, no need to change
    file_storage_path="large_files/", # where to store the precomputed large files, no need to change
    task_assignment_strategy="roundrobin", # how to assign tasks to agents, no need to change
    num_tasks_reveal=1, # how many new tasks are revealed, no need to change
)

import json
import numpy as np

analysis=json.loads(ret)
print(analysis.keys())
print(np.array(analysis["tile_usage"]).shape)
print(np.array(analysis["edge_pair_usage"]).shape)

print(analysis["throughput"],analysis["edge_pair_usage_mean"],analysis["edge_pair_usage_std"])
print(analysis["throughput"], analysis["num_task_finished"])
print("final_pos:", analysis["final_pos"])
print("final_tasks:", analysis["final_tasks"])
print("====")
print(analysis["actual_paths"])
print(analysis["starts"])
print("exec_f:", analysis["exec_future"])
print("plan_f:", analysis["plan_future"])
print("exec_m:", analysis["exec_move"])
print("plan_m:", analysis["plan_move"])
##### Only use the following for weight opt case #####
# because the order of orientation is different in competition code and weight opt code.

# h*w*4: 0: right, 1: up, 2: left, 3:down
edge_usage_matrix=analysis["edge_usage_matrix"]
vertex_wait_matrix=analysis["vertex_wait_matrix"]

def compress_vertex_matrix(map,vertex_matrix):
    assert map.width*map.height==len(vertex_matrix)
    compressed_vertex_matrix=[]    
    for y in range(map.height):
        for x in range(map.width):
            pos=y*map.width+x
            if map.graph[y,x]==1:
                continue
            compressed_vertex_matrix.append(vertex_matrix[pos])
    return compressed_vertex_matrix

def compress_edge_matrix(map,edge_matrix):
    # edge matrix: h*w*[right,up,left,down]
    assert map.width*map.height*4==len(edge_matrix)
    
    compressed_edge_matrix=[]
    for y in range(map.height):
        for x in range(map.width):
            pos=y*map.width+x
            if map.graph[y,x]==1:
                continue
            
            if (x+1)<map.width and map.graph[y,x+1]==0: # right
                compressed_edge_matrix.append(edge_matrix[4*pos+0])
                
            if (y-1)>=0 and map.graph[y-1,x]==0: # up
                compressed_edge_matrix.append(edge_matrix[4*pos+1])
                
            if (x-1)>=0 and map.graph[y,x-1]==0: # left 
                compressed_edge_matrix.append(edge_matrix[4*pos+2])
            
            if (y+1)<map.height and map.graph[y+1,x]==0: # down
                compressed_edge_matrix.append(edge_matrix[4*pos+3])
    return compressed_edge_matrix


def uncompress_vertex_matrix(map,compressed_vertex_matrix,fill_value=0):
    j=0
    vertex_matrix=[]    
    for y in range(map.height):
        for x in range(map.width):
            if map.graph[y,x]==1:
                vertex_matrix.append(fill_value)
            else:
                vertex_matrix.append(compressed_vertex_matrix[j])
                j+=1
    return vertex_matrix


def uncompress_edge_matrix(map,compressed_edge_matrix,fill_value=0):
    # edge matrix: h*w*[right,up,left,down]
    
    j=0
    edge_matrix=[]
    for y in range(map.height):
        for x in range(map.width):
            if map.graph[y,x]==1:
                for i in range(4):
                    edge_matrix.append(fill_value)
            else:
                if (x+1)<map.width and map.graph[y,x+1]==0: # right
                    edge_matrix.append(compressed_edge_matrix[j])
                    j+=1
                else:
                    edge_matrix.append(fill_value)
                    
                if (y-1)>=0 and map.graph[y-1,x]==0: # up
                    edge_matrix.append(compressed_edge_matrix[j])
                    j+=1
                else:
                    edge_matrix.append(fill_value)
                    
                if (x-1)>=0 and map.graph[y,x-1]==0: # left 
                    edge_matrix.append(compressed_edge_matrix[j])
                    j+=1
                else:
                    edge_matrix.append(fill_value)
                
                if (y+1)<map.height and map.graph[y+1,x]==0: # down
                    edge_matrix.append(compressed_edge_matrix[j])
                    j+=1
                else:
                    edge_matrix.append(fill_value)
    
    return edge_matrix

compressed_vertex_waits=compress_vertex_matrix(map,vertex_wait_matrix)
compressed_edge_usages=compress_edge_matrix(map,edge_usage_matrix)

# print(compressed_vertex_waits,compressed_edge_usages)

# print(len(compressed_vertex_waits),len(compressed_edge_usages))


_vertex_waits=uncompress_vertex_matrix(map,compressed_vertex_waits)
_edge_usages=uncompress_edge_matrix(map,compressed_edge_usages)

assert len(_vertex_waits)==len(vertex_wait_matrix)
assert len(_edge_usages)==len(edge_usage_matrix), "{} vs {}".format(len(_edge_usages),len(edge_usage_matrix))

for a,b in zip(_vertex_waits,vertex_wait_matrix):
    assert a==b
    
for a,b in zip(_edge_usages,edge_usage_matrix):
    assert a==b
                
            