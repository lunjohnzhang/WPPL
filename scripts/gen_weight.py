# for warehouse_large.map

import numpy as np

weights=np.ones((140,500,5),dtype=np.int32)*2

c=1
for row in range(7,131,3):
    if (row-7)%6==0:
        # go east
        weights[row,7:492,0]=2-c
        # go west
        weights[row,7:492,2]=2+c
    elif (row-7)%6==3:
        # go east
        weights[row,7:492,0]=2+c
        # go west
        weights[row,7:492,2]=2-c
    else:
        assert False,row
        
for col in range(7,492,4):
    if (col-7)%8==0:
        # go south
        weights[7:131,col,1]=2-c
        # go north
        weights[7:131,col,3]=2+c
    elif (col-7)%8==4:
        # go south
        weights[7:131,col,1]=2+c
        # go north
        weights[7:131,col,3]=2-c
    else:
        assert False
        
with open("warehouse_large_weight_test.weight","w") as f:
    f.write("[")
    for row in range(140):
        for col in range(500):
            for dir in range(5):
                f.write(str(weights[row,col,dir]))
                if (row,col,dir)!=(139,499,4):
                    f.write(",")
    f.write("]")