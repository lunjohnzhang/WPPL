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

def gen_weights_paris_002():
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
            weights[:,col,3]=10
        elif col%2==0:
            # go south
            weights[:,col,1]=10
            # go north
            weights[:,col,3]=2-c
    
    #### left ####
    for row in range(0,32,1):
        # any
        if row%2==1:
            # go east
            weights[row,:,0]=2-c
            # go west
            weights[row,:,2]=10
        elif row%2==0:
            # go east
            weights[row,:,0]=10
            # go west
            weights[row,:,2]=2-c
            
    with open("random_weight_002.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")        

        
def gen_weights_paris_010():
    weights=np.ones((32,32,5),dtype=np.int32)*2

    c=-1
    
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
            
    with open("random_weight_010.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")       

def gen_weights_paris_011():
    weights=np.ones((32,32,5),dtype=float)*2

    c=-1
    
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
    
    for row in range(10,21):
        for col in range(10,21):
            weights[row,col,:]+=0.01
            
    for row in range(20,32):
        for col in range(10,32):
            weights[row,col,:]+=0.01

            
    with open("random_weight_011.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")      

def gen_weights_paris_012():
    weights=np.ones((32,32,5),dtype=float)*2

    c=-1
    
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
    
    for row in range(15,32):
        for col in range(10,20):
            weights[row,col,:]+=0.01
            
    with open("random_weight_012.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")      

def gen_weights_paris_013():
    weights=np.ones((32,32,5),dtype=float)*2

    c=-1
    
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
    
    ## try specify a path
    weights[26:28,0,3]=1.0 # north
    weights[26:28,0,1]=3.0 # south
    weights[27:29,1,3]=1.0 # north
    weights[27:29,1,1]=3.0 # south
    weights[28:32,2,3]=1.0 # north
    weights[28:32,2,1]=3.0 # south
    weights[29:32,4,3]=1.0 # north
    weights[29:32,4,1]=3.0 # south
    weights[30:32,5,3]=1.0 # north
    weights[30:32,5,1]=3.0 # south
    
    weights[31,2:8,0]=3.0 # east
    weights[31,2:8,2]=1.0 # west
    weights[30,4:6,0]=3.0 # east
    weights[30,4:6,2]=1.0 # west
    weights[29,1:5,0]=3.0 # east
    weights[29,1:5,2]=1.0 # west
    weights[28,1:3,0]=3.0 # east
    weights[28,1:3,2]=1.0 # west
    weights[27,0:2,0]=3.0 # east
    weights[27,0:2,2]=1.0 # west
            
    with open("random_weight_013.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")      

def gen_weights_paris_020():
    weights=np.ones((32,32,5),dtype=float)*2

    c=-1
    
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
    
    for row in range(15,32):
        for col in range(10,20):
            weights[row,col,:]+=0.2
            
    with open("random_weight_020.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")      
        
        
def gen_weights_paris_022():
    weights=np.ones((32,32,5),dtype=float)*2

    c=-1
    
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
    
    for row in range(15,32):
        for col in range(10,20):
            weights[row,col,:]+=0.2
    
    for row in range(5,23):
        for col in range(10,23):
            weights[row,col,:]+=0.1
            
    for row in range(25,32):
        for col in range(21,32):
            weights[row,col,:]+=0.1
    
            
    with open("random_weight_022.weight","w") as f:
        f.write("[")
        for row in range(32):
            for col in range(32):
                for dir in range(5):
                    f.write(str(weights[row,col,dir]))
                    if (row,col,dir)!=(31,31,4):
                        f.write(",")
        f.write("]")    
        
        
        
            
gen_weights_paris_022()