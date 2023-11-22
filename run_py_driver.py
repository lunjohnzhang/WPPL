import sys
sys.path.append('build')
sys.path.append('scripts')
from map import Map

map_path="example_problems/random.domain/maps/random-32-32-20.map"
full_weight_path="scripts/random_weight_001.w"
with_wait_costs=True

map=Map(map_path)
map.print_graph(map.graph)

import json
with open(full_weight_path) as f:
    full_weights=json.load(f)

def compress_weights(map,full_weights, with_wait_costs=False):
    compressed_weights=[]
    
    if with_wait_costs:
        for y in range(map.height):
            for x in range(map.width):
                weight_idx=y*map.width+x
                if map.graph[y,x]==1:
                    continue
                compressed_weights.append(full_weights[5*weight_idx+4])
    else:
        compressed_weights.append(full_weights[4]) # wait
    
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
                
    print("size",compressed_weights.__len__())

    compressed_weights="["+",".join([str(w) for w in compressed_weights])+"]"
    return compressed_weights

compressed_weights_json_str=compress_weights(map,full_weights,with_wait_costs)

print(compressed_weights_json_str)


import py_driver
print(py_driver.playground())

import os
# how do we specify OMP_NUM_THREADS=1, may be directly set in environ?
os.environ["OMP_NUM_THREADS"] = "1"

EXECUTABLE="./build/lifelong_comp"
INPUT_FILE="example_problems/random.domain/random_400.json" # random_xxx means random map with xxx agents. we care about 100,200,400,600,800.
OUTPUT_FILE="test_py_driver.json" # 
SIMULATION_TIME=1000 # simulate how many steps # for random_100,200,400,600,800, we use 500,500,1000,1000,1000 steps accordingly.
PLAN_TIME_LIMIT=1 # how many seconds to plan for each step, no need to change
FILE_STORAGE_PATH="large_files/" # where to store the precomputed large files, no need (to change) in the weight opt case.

cmd=f"{EXECUTABLE} --inputFile {INPUT_FILE} -o {OUTPUT_FILE} --simulationTime {SIMULATION_TIME} --planTimeLimit {PLAN_TIME_LIMIT} --fileStoragePath {FILE_STORAGE_PATH}"

# the cmd is the string command above, which is the same command to run ./build/lifelong in the terminal.
# the compress_weights_json_str is a weight represented as [w0,w1,w2,....] as required by weight opt program.
# the return values now include:
# 1. throughput double
# 2. vertexUsage 1-d double json array, N_v
# 3. edgeUsage  2-d double json array, N_v*N_v
ret=py_driver.run(cmd=cmd,weights=compressed_weights_json_str,with_wait_costs=with_wait_costs)

import json
import numpy as np

analysis=json.loads(ret)
print(analysis.keys())
print(np.array(analysis["tile_usage"]).shape)
print(np.array(analysis["edge_pair_usage"]).shape)

print(analysis["throughput"],analysis["edge_pair_usage_mean"],analysis["edge_pair_usage_std"],len(analysis["action_ctrs"]))


##### Only use the following for weight opt case #####
# because the order of orientation is different in competition code and weight opt code.

# h*w*6: 0: right, 1: up, 2: left, 3:down, 4: wait, 5: wait+rotation
action_ctrs=analysis["action_ctrs"]

def get_compressed_action_ctrs(map,action_ctrs):
    assert map.width*map.height*6==len(action_ctrs)
    
    compressed_vertex_waits=[]    
    for y in range(map.height):
        for x in range(map.width):
            pos=y*map.width+x
            if map.graph[y,x]==1:
                continue
            compressed_vertex_waits.append(action_ctrs[6*pos+4])
    
    compressed_edge_usages=[]
    for y in range(map.height):
        for x in range(map.width):
            pos=y*map.width+x
            if map.graph[y,x]==1:
                continue
            
            if (x+1)<map.width and map.graph[y,x+1]==0: # right
                compressed_edge_usages.append(action_ctrs[6*pos+0])
                
            if (y-1)>=0 and map.graph[y-1,x]==0: # up
                compressed_edge_usages.append(action_ctrs[6*pos+1])
                
            if (x-1)>=0 and map.graph[y,x-1]==0: # left 
                compressed_edge_usages.append(action_ctrs[6*pos+2])
            
            if (y+1)<map.height and map.graph[y+1,x]==0: # down
                compressed_edge_usages.append(action_ctrs[6*pos+3])
                
    return compressed_vertex_waits,compressed_edge_usages


compressed_vertex_waits,compressed_edge_usages=get_compressed_action_ctrs(map,action_ctrs)

print(compressed_vertex_waits,compressed_edge_usages)

print(len(compressed_vertex_waits),len(compressed_edge_usages))
                
            