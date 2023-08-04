export OMP_NUM_THREADS=4

./build/lifelong --inputFile example_problems/warehouse.domain/warehouse_small_200.json -o test.json --simulationTime 0
# ./build/lifelong --inputFile example_problems/warehouse.domain/sortation_large_400.json -o test.json --simulationTime 0
# ./build/lifelong --inputFile example_problems/game.domain/brc202d_200.json -o test.json --simulationTime 0
# ./build/lifelong --inputFile example_problems/city.domain/paris_200.json -o test.json --simulationTime 0