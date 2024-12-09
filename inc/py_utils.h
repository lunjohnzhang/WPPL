#include "nlohmann/json.hpp"
#include "Grid.h"
#include <vector>
#include <memory>
#include <random>

std::shared_ptr<std::vector<float>> weight_format_conversion(
    Grid &grid, std::vector<float> &weights);

std::shared_ptr<std::vector<float>> weight_format_conversion_with_wait_costs(
    Grid &grid, std::vector<float> &edge_weights,
    std::vector<float> &wait_costs);

void gen_random_instance(Grid &grid, std::vector<int> &agents,
    std::vector<int> &tasks, int num_agents, int num_tasks, uint seed);