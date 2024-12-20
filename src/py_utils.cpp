#include "py_utils.h"

std::shared_ptr<std::vector<float>> weight_format_conversion(Grid &grid, std::vector<float> &weights)
{
    const int max_weight = 100000;
    std::shared_ptr<std::vector<float>> map_weights_ptr = std::make_shared<std::vector<float>>(grid.map.size() * 5, max_weight);
    auto &map_weights = *map_weights_ptr;

    const int dirs[4] = {1, -grid.cols, -1, grid.cols};
    const int map_weights_idxs[4] = {0, 3, 2, 1};

    int j = 0;

    ++j; // the 0 indexed weight is for wait

    for (int i = 0; i < grid.map.size(); ++i)
    {
        if (grid.map[i] == 1)
        {
            continue;
        }

        map_weights[i * 5 + 4] = weights[0];

        for (int d = 0; d < 4; ++d)
        {
            int dir = dirs[d];
            if (
                0 <= i + dir && i + dir < grid.map.size() &&
                _get_Manhattan_distance(i, i + dir, grid.cols) <= 1 &&
                grid.map[i + dir] != 1)
            {
                float weight = weights.at(j);
                if (weight == -1)
                {
                    weight = max_weight;
                }

                int map_weight_idx = map_weights_idxs[d];
                map_weights[i * 5 + map_weight_idx] = weight;
                ++j;
            }
        }
    }

    // std::cout<<"map weights: ";
    // for (auto i=0;i<map_weights.size();++i) {
    //     std::cout<<map_weights[i]<<" ";
    // }
    // std::cout<<endl;

    if (j != weights.size())
    {
        std::cout << "weight size mismatch: " << j << " vs " << weights.size() << std::endl;
        exit(1);
    }

    return map_weights_ptr;
}

std::shared_ptr<std::vector<float>> weight_format_conversion_with_wait_costs(Grid &grid, std::vector<float> &edge_weights, std::vector<float> &wait_costs)
{
    const int max_weight = 100000;
    std::shared_ptr<std::vector<float>> map_weights_ptr = std::make_shared<std::vector<float>>(grid.map.size() * 5, max_weight);
    auto &map_weights = *map_weights_ptr;

    const int dirs[4] = {1, -grid.cols, -1, grid.cols};
    const int map_weights_idxs[4] = {0, 3, 2, 1};

    // read wait cost
    int j = 0;
    for (int i = 0; i < grid.map.size(); ++i)
    {
        if (grid.map[i] == 1)
        {
            continue;
        }
        map_weights[i * 5 + 4] = wait_costs[j];
        ++j;
    }

    if (j != wait_costs.size())
    {
        std::cout << "wait cost size mismatch: " << j << " vs " << wait_costs.size() << std::endl;
        exit(1);
    }

    // read edge cost
    j = 0;
    for (int i = 0; i < grid.map.size(); ++i)
    {
        if (grid.map[i] == 1)
        {
            continue;
        }

        for (int d = 0; d < 4; ++d)
        {
            int dir = dirs[d];
            if (
                0 <= i + dir && i + dir < grid.map.size() &&
                _get_Manhattan_distance(i, i + dir, grid.cols) <= 1 &&
                grid.map[i + dir] != 1)
            {
                float weight = edge_weights.at(j);
                if (weight == -1)
                {
                    weight = max_weight;
                }

                int map_weight_idx = map_weights_idxs[d];
                map_weights[i * 5 + map_weight_idx] = weight;
                ++j;
            }
        }
    }

    // std::cout<<"map weights: ";
    // for (auto i=0;i<map_weights.size();++i) {
    //     std::cout<<map_weights[i]<<" ";
    // }
    // std::cout<<endl;

    if (j != edge_weights.size())
    {
        std::cout << "edge weight size mismatch: " << j << " vs " << edge_weights.size() << std::endl;
        exit(1);
    }

    return map_weights_ptr;
}

void gen_random_instance(Grid &grid, std::vector<int> &agents, std::vector<int> &tasks, int num_agents, int num_tasks, uint seed)
{
    std::mt19937 MT(seed);

    std::vector<int> empty_locs;
    for (int i = 0; i < grid.map.size(); ++i)
    {
        if (grid.map[i] == 0)
        {
            empty_locs.push_back(i);
        }
    }

    std::shuffle(empty_locs.begin(), empty_locs.end(), MT);

    for (int i = 0; i < num_agents; ++i)
    {
        agents.push_back(empty_locs[i]);
    }

    if (grid.end_points.size() > 0)
    {
        // only sample goal locations from end_points
        std::cout << "sample goal locations from end points" << std::endl;
        for (int i = 0; i < num_tasks; ++i)
        {
            int rnd_idx = MT() % grid.end_points.size();
            tasks.push_back(grid.end_points[rnd_idx]);
        }
    }
    else
    {
        std::cout << "sample goal locations from empty locations" << std::endl;
        for (int i = 0; i < num_tasks; ++i)
        {
            int rnd_idx = MT() % empty_locs.size();
            tasks.push_back(empty_locs[rnd_idx]);
        }
    }
}
