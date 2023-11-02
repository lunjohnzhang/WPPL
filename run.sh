#!/bin/bash
set -ex

if [ ! -d large_files ]
then
    mkdir large_files
fi

cd build && make -j16 lifelong && cd ..

#export OMP_NUM_THREADS=1
ARGS="-o test.json --simulationTime 1000 --planTimeLimit 1 --fileStoragePath large_files/"

# random:random
# ./build/lifelong --inputFile example_problems/random.domain/random_20.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_50.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS
./build/lifelong --inputFile example_problems/random.domain/random_600.json $ARGS
# ./build/lifelong --inputFile example_problems/random.domain/random_800.json $ARGS

# warehouse:warehouse_small
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_10.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_50.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_60.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_100.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_200.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_400.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_800.json $ARGS

# warehouse:warehouse_large
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_200.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_400.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_600.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_800.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_1000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_2000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_3000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_5000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_8000.json $ARGS

# warehouse:sortation
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_400.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_600.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_800.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_1200.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_1600.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_2000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_2400.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_3000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_8000.json $ARGS

# game:brc202_d
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_200.json $ARGS
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_300.json $ARGS
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_400.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_500.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_1000.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_2000.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_3000.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_5000.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_8000.json $ARGS 

# city:paris
# ./build/lifelong --inputFile example_problems/city.domain/paris_200.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_300.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_400.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_500.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_1000.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_2000.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_3000.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_5000.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_8000.json $ARGS