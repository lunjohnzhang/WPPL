#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "common.h"
#include "nlohmann/json.hpp"
#include <signal.h>
#include <ctime>
#include <climits>
#include <memory>
#include <util/Analyzer.h>
#include "py_utils.h"
#include "SortationSystem.h"

#ifdef MAP_OPT
#include "util/analyze.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

namespace py = pybind11;
class py_sim
{
public:
    py_sim(py::kwargs kwargs);
    std::string warmup();
    std::vector<int> get_curr_pos();
    std::string update_gg_and_step(std::vector<float> edge_weights, std::vector<float> wait_costs);
    // void update_tasks_base_distribution(std::vector<double>& new_distribution);
    // std::vector<double> get_tasks_distribution();

private:
    // std::vector<int> init_agents(py::kwargs& kwargs);
    // std::vector<int> init_tasks(py::kwargs& kwargs);

    // TODO: setting in init
    int warmup_steps = 100;
    int update_gg_interval = 100;

    int simulation_steps;
    double plan_time_limit;
    double preprocess_time_limit;
    std::string file_storage_path;
    std::string task_assignment_strategy;
    int num_tasks_reveal;

    // task distribution weights
    double left_w_weight = 1.0;
    double right_w_weight = 1.0;

    Grid grid;

    Logger *logger;
    MAPFPlanner *planner = nullptr;
    ActionModelWithRotate *model;
    std::unique_ptr<BaseSystem> system_ptr;

    bool save_path = false;
    boost::filesystem::path path_file;
};