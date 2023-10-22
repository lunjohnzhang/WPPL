# for warehouse_large.map

import numpy as np

def gen_weights_warehouse_large_001():
    weights=np.ones((140,500,5),dtype=np.int32)*2

    c=0
    for row in range(7,131,3):
        if (row-7)%6==0:
            # go east
            weights[row,7:492,0]=2-c
            # go west
            weights[row,7:492,2]=100000
        elif (row-7)%6==3:
            # go east
            weights[row,7:492,0]=100000
            # go west
            weights[row,7:492,2]=2-c
        else:
            assert False,row
            
    for col in range(7,492,4):
        if (col-7)%8==0:
            # go south
            weights[7:131,col,1]=2-c
            # go north
            weights[7:131,col,3]=100000
        elif (col-7)%8==4:
            # go south
            weights[7:131,col,1]=100000
            # go north
            weights[7:131,col,3]=2-c
        else:
            assert False
            
    with open("warehouse_large_weight_test3.weight","w") as f:
        f.write("[")
        for row in range(140):
            for col in range(500):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(139,499,4):
                        f.write(",")
        f.write("]")
        
        
def gen_weights_warehouse_large_002():
    weights=np.ones((140,500,5),dtype=np.int32)*2

    c=0
    
    #TODO(rivers): we need to add some consistent checking for weight assignment...
    #TODO(rivers): we need a tool to visualize weight map...
    
    ### central boarder ###
    
    for col in range(0,8,1):
        # see col 7 
        if (col-4)%2==1:
            # go south
            weights[:,col,1]=2-c
            # go north
            weights[:,col,3]=100000
        elif (col-4)%2==0:
            # go south
            weights[:,col,1]=100000
            # go north
            weights[:,col,3]=2-c
            
    for col in range(491,500,1):
        # see col 491
        if (col-491)%2==0:
            # go south
            weights[:,col,1]=100000
            # go north
            weights[:,col,3]=2-c
        elif (col-491)%2==1:
            # go south
            weights[:,col,1]=2-c
            # go north
            weights[:,col,3]=100000
    
    for row in range(0,8,1):
        # see row 7
        if (row-4)%2==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=100000
        elif (row-4)%2==0:
            # go east
            weights[row,:,0]=100000
            # go west
            weights[row,:,2]=2-c
            
    for row in range(130,140,1):
        # see row 130
        if (row-130)%2==0:
            # go east
            weights[row,:,0]=100000
            # go west
            weights[row,:,2]=2-c
        elif (row-130)%2==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=100000        
    
    
    ### central ###
    
    for row in range(7,131,3):
        if (row-7)%6==0:
            # go east
            weights[row,7:492,0]=2-c
            # go west
            weights[row,7:492,2]=100000
        elif (row-7)%6==3:
            # go east
            weights[row,7:492,0]=100000
            # go west
            weights[row,7:492,2]=2-c
        else:
            assert False,row
            
    for col in range(7,492,4):
        if (col-7)%8==0:
            # go south
            weights[7:131,col,1]=2-c
            # go north
            weights[7:131,col,3]=100000
        elif (col-7)%8==4:
            # go south
            weights[7:131,col,1]=100000
            # go north
            weights[7:131,col,3]=2-c
        else:
            assert False
            
    with open("warehouse_large_weight_002.weight","w") as f:
        f.write("[")
        for row in range(140):
            for col in range(500):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(139,499,4):
                        f.write(",")
        f.write("]")
        
def gen_weights_warehouse_large_003():
    weights=np.ones((140,500,5),dtype=np.int32)*2

    c=0
    
    #TODO(rivers): we need to add some consistent checking for weight assignment...
    #TODO(rivers): we need a tool to visualize weight map...
    
    # TODO(rivers): probably need better, make boarder and cental consistent
    ### boarder ###
    
    #### top ####
    for col in range(0,500,1): 
        # any
        if col%2==1:
            # go south
            weights[0:4,col,1]=2-c
            # go north
            weights[0:4,col,3]=100000
        elif col%2==0:
            # go south
            weights[0:4,col,1]=100000
            # go north
            weights[0:4,col,3]=2-c
            
    #### bottom ####
    for col in range(0,500,1): 
        # any
        if col%2==1:
            # go south
            weights[136:140,col,1]=2-c
            # go north
            weights[136:140,col,3]=100000
        elif col%2==0:
            # go south
            weights[136:140,col,1]=100000
            # go north
            weights[136:140,col,3]=2-c
    
    #### left ####
    for row in range(0,140,1):
        # any
        if row%2==1:
            # go east
            weights[row,0:4,0]=2-c
            # go west
            weights[row,0:4,2]=100000
        elif row%2==0:
            # go east
            weights[row,0:4,0]=100000
            # go west
            weights[row,0:4,2]=2-c
    
    
    #### right ####
    for row in range(0,140,1):
        # any
        if row%2==1:
            # go east
            weights[row,496:500,0]=2-c
            # go west
            weights[row,496:500,2]=100000
        elif row%2==0:
            # go east
            weights[row,496:500,0]=100000
            # go west
            weights[row,496:500,2]=2-c
    
    ### buffer ###
    
    for col in range(0,8,1):
        # see col 7 
        if (col-4)%2==1:
            # go south
            weights[:,col,1]=2-c
            # go north
            weights[:,col,3]=100000
        elif (col-4)%2==0:
            # go south
            weights[:,col,1]=100000
            # go north
            weights[:,col,3]=2-c
            
    for col in range(491,500,1):
        # see col 491
        if (col-491)%2==0:
            # go south
            weights[:,col,1]=100000
            # go north
            weights[:,col,3]=2-c
        elif (col-491)%2==1:
            # go south
            weights[:,col,1]=2-c
            # go north
            weights[:,col,3]=100000
    
    for row in range(0,8,1):
        # see row 7
        if (row-4)%2==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=100000
        elif (row-4)%2==0:
            # go east
            weights[row,:,0]=100000
            # go west
            weights[row,:,2]=2-c
            
    for row in range(130,140,1):
        # see row 130
        if (row-130)%2==0:
            # go east
            weights[row,:,0]=100000
            # go west
            weights[row,:,2]=2-c
        elif (row-130)%2==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=100000        
    
    
    ### central ###
    
    for row in range(7,131,3):
        if (row-7)%6==0:
            # go east
            weights[row,7:492,0]=2-c
            # go west
            weights[row,7:492,2]=100000
        elif (row-7)%6==3:
            # go east
            weights[row,7:492,0]=100000
            # go west
            weights[row,7:492,2]=2-c
        else:
            assert False,row
            
    for col in range(7,492,4):
        if (col-7)%8==0:
            # go south
            weights[7:131,col,1]=2-c
            # go north
            weights[7:131,col,3]=100000
        elif (col-7)%8==4:
            # go south
            weights[7:131,col,1]=100000
            # go north
            weights[7:131,col,3]=2-c
        else:
            assert False
            
    with open("warehouse_large_weight_003.weight","w") as f:
        f.write("[")
        for row in range(140):
            for col in range(500):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(139,499,4):
                        f.write(",")
        f.write("]")        
        
        
        
        
        
gen_weights_warehouse_large_003()