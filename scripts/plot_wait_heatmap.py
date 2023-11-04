import json
import numpy as np
import matplotlib.pyplot as plt

snapshot_step=5000
input_fp='test_warehouse_fix_nb.json'
# output_fn='test_100_lns_weight1_step_{}'.format(snapshot_step)
h=140
w=500


with open(input_fp) as f:
    data = json.load(f)

print(data.keys())
print(data["actionModel"])

# print(data["start"])


def get_orient_idx(orient):
    return {"E": 0, 'S': 1, 'W': 2, 'N': 3}[orient]

def move(x,y,orient,action):
    if action=="F":
        if orient==0:
            x+=1
        elif orient==1:
            y+=1
        elif orient==2:
            x-=1
        elif orient==3:
            y-=1
    elif action=="R":
        orient=(orient+1)%4
    elif action=="C":
        orient=(orient-1)%4
    elif action=="W":
        pass
    return x,y,orient


team_size=data["teamSize"]
actual_paths=data["actualPaths"]
starts=data["start"]
events=data["events"]
tasks=data["tasks"]


MAXT=(len(actual_paths[0])+1)//2

T=snapshot_step

arr=np.zeros((team_size,T+1,3),dtype=int)

goal_locs=np.zeros((team_size,),dtype=int)

wait_heatmap=np.zeros((h,w),dtype=int)

for aid in range(team_size):
    start=starts[aid]
    arr[aid,0,0]=start[1]
    arr[aid,0,1]=start[0]
    arr[aid,0,2]=get_orient_idx(start[2])
    
    path=actual_paths[aid]
    actions=path.split(",")
    x=arr[aid,0,0]
    y=arr[aid,0,1]
    orient=arr[aid,0,2]
    for t in range(T):
        action=actions[t]
        if action=="W":
            wait_heatmap[y,x]+=1
        x,y,orient=move(x,y,orient,action)
        arr[aid,t+1,0]=x
        arr[aid,t+1,1]=y
        arr[aid,t+1,2]=orient
        
print(wait_heatmap.sum())

plt.imshow(wait_heatmap)
plt.show()

# for aid in range(team_size):
#     events_for_agent=events[aid]
#     for event in range(len(events_for_agent)-1,-1,-1):
#         tid,timestep,info=events_for_agent[event]
#         if info=="assigned":
#             goal_x = tasks[tid][2]
#             goal_y = tasks[tid][1]
#             goal_locs[aid]=goal_y*w+goal_x
#             break



# print(arr[0])
# print("team_size",team_size)

# with open(output_fn+".snapshot",'w') as f:
#     f.write("{}\n".format(team_size))
#     for aid in range(team_size):
#         f.write("{} {} {}\n".format(arr[aid,snapshot_step,0],arr[aid,snapshot_step,1],arr[aid,snapshot_step,2]))
        
# with open(output_fn+".goals",'w') as f:
#     for aid in range(team_size):
#         f.write("{}\n".format(goal_locs[aid]))
