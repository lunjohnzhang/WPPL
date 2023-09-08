/*
 * post processing, e.g., calculating solution quality
 */
#pragma once
#include "MyLaCAM2/instance.hpp"
#include "MyLaCAM2/utils.hpp"
#include "PIBT/HeuristicTable.h"
#include <memory>

namespace MyLaCAM2 {

bool is_feasible_solution(const Instance& ins, const Solution& solution,
                          const int verbose = 0);
int get_makespan(const Solution& solution);
int get_path_cost(const Solution& solution, uint i);  // single-agent path cost
int get_sum_of_costs(const Solution& solution);
int get_sum_of_loss(const Solution& solution);
int get_makespan_lower_bound(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT);
int get_sum_of_costs_lower_bound(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT);
void print_stats(const int verbose, const Instance& ins, const std::shared_ptr<HeuristicTable>& HT,
                 const Solution& solution, const double comp_time_ms);
void make_log(const Instance& ins, const std::shared_ptr<HeuristicTable>& HT, const Solution& solution,
              const std::string& output_name, const double comp_time_ms,
              const std::string& map_name, const int seed,
              const std::string& additional_info,
              const bool log_short = false  // true -> paths not appear
);

}  // namespace MyLaCAM2