#!/bin/bash
set -ex

if [ ! -d large_files ]
then
    mkdir large_files
fi

cd build && make -j16 lifelong_comp && cd ..

# export OMP_NUM_THREADS=1
ARGS="--planTimeLimit 1 --fileStoragePath large_files/"

./build/lifelong_comp --inputFile example_problems/warehouse.domain/kiva_200.json $ARGS -o test_kiva_200_lacam2.json --simulationTime 1000
./build/lifelong_comp --inputFile example_problems/warehouse.domain/kiva_400.json $ARGS -o test_kiva_400_lacam2.json --simulationTime 1000
./build/lifelong_comp --inputFile example_problems/warehouse.domain/kiva_600.json $ARGS -o test_kiva_600_lacam2.json --simulationTime 1000
./build/lifelong_comp --inputFile example_problems/warehouse.domain/kiva_800.json $ARGS -o test_kiva_800_lacam2.json --simulationTime 1000
./build/lifelong_comp --inputFile example_problems/warehouse.domain/kiva_1000.json $ARGS -o test_kiva_1000_lacam2.json --simulationTime 1000
./build/lifelong_comp --inputFile example_problems/warehouse.domain/kiva_1200.json $ARGS -o test_kiva_1200_lacam2.json --simulationTime 1000

# random:random
#./build/lifelong_comp --inputFile example_problems/random.domain/random_20.json $ARGS --simulationTime 500 
# ./build/lifelong_comp --inputFile example_problems/random.domain/random_50.json $ARGS --simulationTime 500 
# LOAD_FP="random_100_lns_step_500.snapshot" 
#./build/lifelong_comp --inputFile example_problems/random.domain/random_100.json $ARGS -o test_random_100.json --simulationTime 1000
#./build/lifelong_comp --inputFile example_problems/random.domain/random_200.json $ARGS -o test_random_200.json --simulationTime 1000
# ./build/lifelong_comp --inputFile example_problems/random.domain/random_400.json $ARGS -o test_random_400.json --simulationTime 1000 
# ./build/lifelong_comp --inputFile example_problems/random.domain/random_600.json $ARGS -o test_random_600.json --simulationTime 1000 
#./build/lifelong_comp --inputFile example_problems/random.domain/random_800.json $ARGS -o test_random_800.json --simulationTime 1000 

# # warehouse:warehouse_small
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_small_10.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_small_50.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_small_60.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_small_100.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_small_200.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_small_400.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_small_800.json $ARGS

# # warehouse:warehouse_large
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_200.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_400.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_600.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_800.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_1000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_2000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_3000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_5000.json $ARGS
# ./build/lifelong_comp --inputFile example_problems/warehouse.domain/warehouse_large_8000.json $ARGS -o test_warehouse.json --simulationTime 5000 

# # warehouse:sortation
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_400.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_600.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_800.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_1200.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_1600.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_2000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_2400.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_3000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_8000.json $ARGS
# ./build/lifelong_comp --inputFile example_problems/warehouse.domain/sortation_large_10000.json $ARGS -o test_sortation.json --simulationTime 5000 

# # game:brc202_d
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_200.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_300.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_400.json $ARGS 
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_500.json $ARGS 
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_1000.json $ARGS 
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_2000.json $ARGS 
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_3000.json $ARGS 
# ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_4000.json $ARGS  -o test_brc202d.json --simulationTime 5000 
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_5000.json $ARGS 
# # ./build/lifelong_comp --inputFile example_problems/game.domain/brc202d_8000.json $ARGS 

# # city:paris
# # ./build/lifelong_comp --inputFile example_problems/city.domain/paris_200.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/city.domain/paris_300.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/city.domain/paris_400.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/city.domain/paris_500.json $ARGS
# ./build/lifelong_comp --inputFile example_problems/city.domain/paris_1000.json $ARGS -o test_paris_1000.json --simulationTime 2000 
# # ./build/lifelong_comp --inputFile example_problems/city.domain/paris_2000.json $ARGS
# ./build/lifelong_comp --inputFile example_problems/city.domain/paris_3000.json $ARGS -o test_paris_3000.json --simulationTime 4000 
# # ./build/lifelong_comp --inputFile example_problems/city.domain/paris_5000.json $ARGS
# # ./build/lifelong_comp --inputFile example_problems/city.domain/paris_8000.json $ARGS