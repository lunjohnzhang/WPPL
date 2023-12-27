import sys
sys.path.append('build')
sys.path.append('scripts')
from map import Map

map_path="example_problems/random.domain/maps/random-32-32-20.map"
full_weight_path="scripts/random_weight_050.w"
with_wait_costs=True

map=Map(map_path)
map.print_graph(map.graph)

import json
with open(full_weight_path) as f:
    full_weights=json.load(f)

def compress_weights_with_wait_costs(map,full_weights):
    compressed_wait_costs=[]
    for y in range(map.height):
        for x in range(map.width):
            weight_idx=y*map.width+x
            
            if map.graph[y,x]==1:
                continue
            
            compressed_wait_costs.append(full_weights[5*weight_idx+4])
            
    print("wait costs size",compressed_wait_costs.__len__())

    compressed_wait_costs="["+",".join([str(w) for w in compressed_wait_costs])+"]"
    
    
    compressed_weights=[]
    
    for y in range(map.height):
        for x in range(map.width):
            weight_idx=y*map.width+x
            
            if map.graph[y,x]==1:
                continue
            
            if (x+1)<map.width and map.graph[y,x+1]==0:
                compressed_weights.append(full_weights[5*weight_idx+0])
                
            if (y-1)>=0 and map.graph[y-1,x]==0:
                compressed_weights.append(full_weights[5*weight_idx+3])
                
            if (x-1)>=0 and map.graph[y,x-1]==0:
                compressed_weights.append(full_weights[5*weight_idx+2])
            
            if (y+1)<map.height and map.graph[y+1,x]==0:
                compressed_weights.append(full_weights[5*weight_idx+1])
                
            if (weight_idx==0):
                print(compressed_weights)
                
    print("edge weights size",compressed_weights.__len__())

    compressed_weights="["+",".join([str(w) for w in compressed_weights])+"]"
    return compressed_wait_costs, compressed_weights


compressed_wait_costs_json_str, compressed_weights_json_str=compress_weights_with_wait_costs(map,full_weights)

print(compressed_wait_costs_json_str)
print(compressed_weights_json_str)


import py_driver
print(py_driver.playground())

import json
# py_configs contains the best configs used for random-32-32-20 map with 100,200,400,600,800 agents.
config_path="py_configs/random-32-32-20_800.json"
with open(config_path) as f:
    config=json.load(f)
    config_str=json.dumps(config)

# the return values now include:
# 1. throughput double
# 2. vertexUsage 1-d double json array, N_v
# 3. edgeUsage  2-d double json array, N_v*N_v
ret=py_driver.run(
    map_path="example_problems/random.domain/maps/random-32-32-20.map",
    simulation_steps=1000,
    # for the problem instance we use:
    # if random then we need specify the number of agents and total tasks, also random seed,
    gen_random=True,
    num_agents=600,
    num_tasks=100000,
    seed=0,
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
                
            