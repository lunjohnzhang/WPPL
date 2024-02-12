#!/bin/bash
set -ex

if [ ! -d large_files ]
then
    mkdir large_files
fi

cd build && make -j16 lifelong && cd ..

# export OMP_NUM_THREADS=1

mkdir -p offline_eval/0211

ARGS="--planTimeLimit 10 --fileStoragePath large_files/"

LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS -o offline_eval/0211/test_1t_045s_random_100.json --simulationTime 500
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS -o offline_eval/0211/test_1t_045s_random_200.json --simulationTime 500 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS -o offline_eval/0211/test_1t_045s_random_400.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_600.json $ARGS -o offline_eval/0211/test_1t_045s_random_600.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_800.json $ARGS -o offline_eval/0211/test_1t_045s_random_800.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_1000.json $ARGS -o offline_eval/0211/test_1t_045s_paris_1000.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_3000.json $ARGS -o offline_eval/0211/test_1t_045s_paris_3000.json --simulationTime 4000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/brc202d.json" ./build/lifelong --inputFile example_problems/game.domain/brc202d_4000.json $ARGS  -o offline_eval/0211/test_1t_045s_brc202d.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/warehouse_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_8000.json $ARGS -o offline_eval/0211/test_1t_045s_warehouse.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_045s/sortation_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_10000.json $ARGS -o offline_eval/0211/test_1t_045s_sortation.json --simulationTime 5000 

LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS -o offline_eval/0211/test_1t_095s_random_100.json --simulationTime 500
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS -o offline_eval/0211/test_1t_095s_random_200.json --simulationTime 500 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS -o offline_eval/0211/test_1t_095s_random_400.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_600.json $ARGS -o offline_eval/0211/test_1t_095s_random_600.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_800.json $ARGS -o offline_eval/0211/test_1t_095s_random_800.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_1000.json $ARGS -o offline_eval/0211/test_1t_095s_paris_1000.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_3000.json $ARGS -o offline_eval/0211/test_1t_095s_paris_3000.json --simulationTime 4000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/brc202d.json" ./build/lifelong --inputFile example_problems/game.domain/brc202d_4000.json $ARGS  -o offline_eval/0211/test_1t_095s_brc202d.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/warehouse_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_8000.json $ARGS -o offline_eval/0211/test_1t_095s_warehouse.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_095s/sortation_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_10000.json $ARGS -o offline_eval/0211/test_1t_095s_sortation.json --simulationTime 5000 

LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS -o offline_eval/0211/test_1t_145s_random_100.json --simulationTime 500
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS -o offline_eval/0211/test_1t_145s_random_200.json --simulationTime 500 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS -o offline_eval/0211/test_1t_145s_random_400.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_600.json $ARGS -o offline_eval/0211/test_1t_145s_random_600.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_800.json $ARGS -o offline_eval/0211/test_1t_145s_random_800.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_1000.json $ARGS -o offline_eval/0211/test_1t_145s_paris_1000.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_3000.json $ARGS -o offline_eval/0211/test_1t_145s_paris_3000.json --simulationTime 4000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/brc202d.json" ./build/lifelong --inputFile example_problems/game.domain/brc202d_4000.json $ARGS  -o offline_eval/0211/test_1t_145s_brc202d.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/warehouse_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_8000.json $ARGS -o offline_eval/0211/test_1t_145s_warehouse.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_145s/sortation_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_10000.json $ARGS -o offline_eval/0211/test_1t_145s_sortation.json --simulationTime 5000 

LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS -o offline_eval/0211/test_1t_195s_random_100.json --simulationTime 500
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS -o offline_eval/0211/test_1t_195s_random_200.json --simulationTime 500 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS -o offline_eval/0211/test_1t_195s_random_400.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_600.json $ARGS -o offline_eval/0211/test_1t_195s_random_600.json --simulationTime 1000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/random-32-32-20.json" ./build/lifelong --inputFile example_problems/random.domain/random_800.json $ARGS -o offline_eval/0211/test_1t_195s_random_800.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_1000.json $ARGS -o offline_eval/0211/test_1t_195s_paris_1000.json --simulationTime 2000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/Paris_1_256.json" ./build/lifelong --inputFile example_problems/city.domain/paris_3000.json $ARGS -o offline_eval/0211/test_1t_195s_paris_3000.json --simulationTime 4000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/brc202d.json" ./build/lifelong --inputFile example_problems/game.domain/brc202d_4000.json $ARGS  -o offline_eval/0211/test_1t_195s_brc202d.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/warehouse_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_8000.json $ARGS -o offline_eval/0211/test_1t_195s_warehouse.json --simulationTime 5000 
LNS_NUM_THREADS=1 CONFIG_PATH="configs/exp_1t_195s/sortation_large.json" ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_10000.json $ARGS -o offline_eval/0211/test_1t_195s_sortation.json --simulationTime 5000 


# # random:random
# # ./build/lifelong --inputFile example_problems/random.domain/random_20.json $ARGS
# # ./build/lifelong --inputFile example_problems/random.domain/random_50.json $ARGS
# # LOAD_FP="random_100_lns_step_500.snapshot" 
# ./build/lifelong --inputFile example_problems/random.domain/random_100.json $ARGS -o test_random_100.json --simulationTime 5
# ./build/lifelong --inputFile example_problems/random.domain/random_200.json $ARGS -o offline_eval/1124/test_random_200.json --simulationTime 500 
# ./build/lifelong --inputFile example_problems/random.domain/random_400.json $ARGS -o offline_eval/1124/test_random_400.json --simulationTime 1000 
# ./build/lifelong --inputFile example_problems/random.domain/random_600.json $ARGS -o offline_eval/1124/test_random_600.json --simulationTime 1000 
# ./build/lifelong --inputFile example_problems/random.domain/random_800.json $ARGS -o offline_eval/1124/test_random_800.json --simulationTime 2000 

# # warehouse:warehouse_small
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_10.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_50.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_60.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_100.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_200.json $ARGS -o offline_eval/1210/test_warehouse_small_200.json --simulationTime 5000
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_400.json $ARGS -o offline_eval/1210/test_warehouse_small_400.json --simulationTime 5000 
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_600.json $ARGS -o offline_eval/1210/test_warehouse_small_600.json --simulationTime 5000 
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_800.json $ARGS -o offline_eval/1210/test_warehouse_small_800.json --simulationTime 5000 
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_1000.json $ARGS -o offline_eval/1210/test_warehouse_small_1000.json --simulationTime 5000 

# # warehouse:warehouse_large
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_200.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_400.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_600.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_800.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_1000.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_2000.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_3000.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_4000.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_5000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_8000.json $ARGS -o offline_eval/1124/test_warehouse.json --simulationTime 5000 

# # warehouse:sortation
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_400.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_600.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_800.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_1200.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_1600.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_2000.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_2400.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_3000.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_5000.json $ARGS
# # ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_8000.json $ARGS
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_10000.json $ARGS -o test_sortation.json --simulationTime 5000 

# # game:brc202_d
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_200.json $ARGS
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_300.json $ARGS
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_400.json $ARGS 
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_500.json $ARGS 
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_1000.json $ARGS 
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_2000.json $ARGS 
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_3000.json $ARGS 
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_4000.json $ARGS  -o offline_eval/1124/test_brc202d.json --simulationTime 5000 
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_5000.json $ARGS 
# # ./build/lifelong --inputFile example_problems/game.domain/brc202d_8000.json $ARGS 

# # city:paris
# # ./build/lifelong --inputFile example_problems/city.domain/paris_200.json $ARGS
# # ./build/lifelong --inputFile example_problems/city.domain/paris_300.json $ARGS
# # ./build/lifelong --inputFile example_problems/city.domain/paris_400.json $ARGS
# # ./build/lifelong --inputFile example_problems/city.domain/paris_500.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_1000.json $ARGS -o offline_eval/1124/test_paris_1000.json --simulationTime 2000 
# # ./build/lifelong --inputFile example_problems/city.domain/paris_2000.json $ARGS
# ./build/lifelong --inputFile example_problems/city.domain/paris_3000.json $ARGS -o offline_eval/1124/test_paris_3000.json --simulationTime 4000 
# # ./build/lifelong --inputFile example_problems/city.domain/paris_5000.json $ARGS
# # ./build/lifelong --inputFile example_problems/city.domain/paris_8000.json $ARGS