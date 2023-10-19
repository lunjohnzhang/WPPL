# for paris_1_256.map

import numpy as np
        
def gen_weights_paris_001():
    weights=np.ones((32,32,5),dtype=np.int32)*2

    c=1
    
    #TODO(rivers): we need to add some consistent checking for weight assignment...
    #TODO(rivers): we need a tool to visualize weight map...
    
    # TODO(rivers): probably need better, make boarder and cental consistent
    ### boarder ###
    
    #### top ####
    for col in range(0,32,1): 
        # any
        if col%2==1:
            # go south
            weights[:,col,1]=2-c
            # go north
            weights[:,col,3]=2+c
        elif col%2==0:
            # go south
            weights[:,col,1]=2+c
            # go north
            weights[:,col,3]=2-c
    
    #### left ####
    for row in range(0,32,1):
        # any
        if row%2==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=2+c
        elif row%2==0:
            # go east
            weights[row,:,0]=2+c
            # go west
            weights[row,:,2]=2-c
            
    with open("random_weight_001.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")        

gen_weights_paris_001()