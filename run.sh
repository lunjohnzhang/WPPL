cd build && make -j32 && cd ..

# export OMP_NUM_THREADS=128
./build/lifelong --inputFile example_problems/random.domain/random_20.json -o test.json --simulationTime 100 --planTimeLimit 1

# ./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_large_200.json -o test.json --simulationTime 100
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_400.json -o test.json --simulationTime 100 --planTimeLimit 1 --planning_window 10
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_200.json -o test.json --simulationTime 100 --planTimeLimit 1 --planning_window 1
# ./build/lifelong --inputFile example_problems/city.domain/paris_200.json -o test.json --simulationTime 100 --planTimeLimit 1 --planning_window 1