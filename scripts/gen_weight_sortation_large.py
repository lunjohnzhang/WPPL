# for warehouse_large.map

import numpy as np

def gen_weights_sortation_large_001():
    weights=np.ones((140,500,5),dtype=np.int32)*2

    c=0
    for row in range(0,140,4):
        if row%4==0 or row%4==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=100000
        elif row%4==2 or row%4==3:
            # go east
            weights[row,:,0]=100000
            # go west
            weights[row,:,2]=2-c
        else:
            assert False,row
            
    for col in range(0,500,4):
        if col%4==0 or col%4==1:
            # go south
            weights[:,col,1]=2-c
            # go north
            weights[:,col,3]=100000
        elif col%4==2 or col%4==3:
            # go south
            weights[:,col,1]=100000
            # go north
            weights[:,col,3]=2-c
        else:
            assert False
            
    with open("sortation_large_weight_001.weight","w") as f:
        f.write("[")
        for row in range(140):
            for col in range(500):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(139,499,4):
                        f.write(",")
        f.write("]")
        
gen_weights_sortation_large_001()