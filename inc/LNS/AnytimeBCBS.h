#pragma once
#include "LNS/Instance.h"

namespace LNS {


class AnytimeBCBS
{
public:
    vector<Path> solution;
    list<IterationStats> iteration_stats; //stats about each iteration
    double preprocessing_time = 0;
    double runtime = 0;
    int sum_of_costs = MAX_COST;
    int sum_of_costs_lowerbound = 0;
    int sum_of_distances = -1;
    AnytimeBCBS(const Instance& instance, double time_limit, int screen) :
        instance(instance), time_limit(time_limit), screen(screen) {}

    void run();
    void validateSolution() const;
    void writeIterStatsToFile(string file_name) const;
    void writeResultToFile(string file_name) const;
    string getSolverName() const { return "AnytimeBCBS"; }

private:
    // intput params
    const Instance& instance; // avoid making copies of this variable as much as possible
    double time_limit;
    int screen;
};
} // end namespace LNS
