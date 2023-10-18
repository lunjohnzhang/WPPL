# for paris_1_256.map

import numpy as np
        
def gen_weights_paris_001():
    weights=np.ones((256,256,5),dtype=np.int32)*2

    c=0
    
    #TODO(rivers): we need to add some consistent checking for weight assignment...
    #TODO(rivers): we need a tool to visualize weight map...
    
    # TODO(rivers): probably need better, make boarder and cental consistent
    ### boarder ###
    
    #### top ####
    for col in range(0,256,1): 
        # any
        if col%2==1:
            # go south
            weights[:,col,1]=2-c
            # go north
            weights[:,col,3]=100000
        elif col%2==0:
            # go south
            weights[:,col,1]=100000
            # go north
            weights[:,col,3]=2-c
    
    #### left ####
    for row in range(0,256,1):
        # any
        if row%2==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=100000
        elif row%2==0:
            # go east
            weights[row,:,0]=100000
            # go west
            weights[row,:,2]=2-c
            
    with open("paris_weight_001.weight","w") as f:
        f.write("[")
        for row in range(256):
            for col in range(256):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(255,255,4):
                        f.write(",")
        f.write("]")        

gen_weights_paris_001()