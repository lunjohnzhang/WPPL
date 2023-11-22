#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>
#include <climits>
#include <memory>
#include <util/Analyzer.h>
#include "nlohmann/json.hpp"

#ifdef MAP_OPT
#include "util/analyze.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

namespace py = pybind11;

po::variables_map vm;
std::unique_ptr<BaseSystem> system_ptr;


void sigint_handler(int a)
{
    fprintf(stdout, "stop the simulation...\n");
    if (!vm["evaluationMode"].as<bool>())
    {
        system_ptr->saveResults(vm["output"].as<std::string>());
    }
    _exit(0);
}


int _get_Manhattan_distance(int loc1, int loc2, int cols) {
    return abs(loc1 / cols - loc2 / cols) + abs(loc1 % cols - loc2 % cols);
}

std::shared_ptr<std::vector<float> > weight_format_conversion(Grid & grid, std::vector<float> & weights)
{
    const int max_weight=100000;
    std::shared_ptr<std::vector<float> > map_weights_ptr = std::make_shared<std::vector<float> >(grid.map.size()*5, max_weight);
    auto & map_weights=*map_weights_ptr;

    const int dirs[4]={1,-grid.cols, -1, grid.cols};
    const int map_weights_idxs[4]={0,3,2,1};

    int j=0;

    ++j; // the 0 indexed weight is for wait

    for (int i=0;i<grid.map.size();++i) {
        if (grid.map[i] == 1) {
            continue;
        }

        map_weights[i*5+4] = weights[0];

        for (int d=0;d<4;++d) {
            int dir=dirs[d];
            if (
                0<=i+dir && i+dir<grid.map.size() &&
                _get_Manhattan_distance(i, i+dir, grid.cols) <= 1 &&
                grid.map[i+dir] != 1
            ) {
                float weight = weights.at(j);
                if (weight==-1) {
                    weight=max_weight;
                }

                int map_weight_idx=map_weights_idxs[d];
                map_weights[i*5+map_weight_idx]=weight;
                ++j;
            }
        }
    }

    // std::cout<<"map weights: ";
    // for (auto i=0;i<map_weights.size();++i) {
    //     std::cout<<map_weights[i]<<" ";
    // }
    // std::cout<<endl;

    if (j!=weights.size()) {
        std::cout<<"weight size mismatch: "<<j<<" vs "<<weights.size()<<std::endl;
        exit(1);
    }

    return map_weights_ptr;
}


std::shared_ptr<std::vector<float> > weight_format_conversion_with_wait_costs(Grid & grid, std::vector<float> & edge_weights, std::vector<float> & wait_costs)
{
    const int max_weight=100000;
    std::shared_ptr<std::vector<float> > map_weights_ptr = std::make_shared<std::vector<float> >(grid.map.size()*5, max_weight);
    auto & map_weights=*map_weights_ptr;

    const int dirs[4]={1,-grid.cols, -1, grid.cols};
    const int map_weights_idxs[4]={0,3,2,1};

    // read wait cost
    int j=0;
    for (int i=0;i<grid.map.size();++i) {
        if (grid.map[i] == 1) {
            continue;
        }
        map_weights[i*5+4] = wait_costs[j];
        ++j;
    }

    if (j!=wait_costs.size()) {
        std::cout<<"wait cost size mismatch: "<<j<<" vs "<<wait_costs.size()<<std::endl;
        exit(1);
    }


    // read edge cost
    j=0;
    for (int i=0;i<grid.map.size();++i) {
        if (grid.map[i] == 1) {
            continue;
        }

        for (int d=0;d<4;++d) {
            int dir=dirs[d];
            if (
                0<=i+dir && i+dir<grid.map.size() &&
                _get_Manhattan_distance(i, i+dir, grid.cols) <= 1 &&
                grid.map[i+dir] != 1
            ) {
                float weight = edge_weights.at(j);
                if (weight==-1) {
                    weight=max_weight;
                }

                int map_weight_idx=map_weights_idxs[d];
                map_weights[i*5+map_weight_idx]=weight;
                ++j;
            }
        }
    }

    // std::cout<<"map weights: ";
    // for (auto i=0;i<map_weights.size();++i) {
    //     std::cout<<map_weights[i]<<" ";
    // }
    // std::cout<<endl;

    if (j!=edge_weights.size()) {
        std::cout<<"edge weight size mismatch: "<<j<<" vs "<<edge_weights.size()<<std::endl;
        exit(1);
    }

    return map_weights_ptr;
}


std::string run(const py::kwargs& kwargs)
{
    
    // should be a command line string running the code
    std::string cmd=kwargs["cmd"].cast<std::string>();
    std::cout<<"cmd from python is: "<<cmd<<std::endl;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")
        // ("inputFolder", po::value<std::string>()->default_value("."), "input folder")
        ("inputFile,i", po::value<std::string>()->required(), "input file name")
        ("output,o", po::value<std::string>()->default_value("./test.json"), "output file name")
        ("evaluationMode", po::value<bool>()->default_value(false), "evaluate an existing output file")
        ("simulationTime", po::value<int>()->default_value(5000), "run simulation")
        ("fileStoragePath", po::value<std::string>()->default_value(""), "the path to the storage path")
        ("planTimeLimit", po::value<int>()->default_value(INT_MAX), "the time limit for planner in seconds")
        ("preprocessTimeLimit", po::value<int>()->default_value(INT_MAX), "the time limit for preprocessing in seconds")
        ("logFile,l", po::value<std::string>(), "issue log file name");
    clock_t start_time = clock();
    // po::store(po::parse_command_line(argc, argv, desc), vm);

    po::store(po::command_line_parser(po::split_unix(cmd)).options(desc).run(), vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(-1);
    }

    po::notify(vm);

    // std::string base_folder = vm["inputFolder"].as<std::string>();
    boost::filesystem::path p(vm["inputFile"].as<std::string>());

    ONLYDEV(
        const auto & filename=p.filename();
        analyzer.data["instance"]=filename.c_str();
    )

    boost::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    std::cout << base_folder << std::endl;
    if (base_folder.size() > 0 && base_folder.back() != '/')
    {
        base_folder += "/";
    }

    Logger *logger = new Logger();
    if (vm.count("logFile"))
        logger->set_logfile(vm["logFile"].as<std::string>());

    MAPFPlanner *planner = nullptr;
    // Planner is inited here, but will be managed and deleted by system_ptr deconstructor
    if (vm["evaluationMode"].as<bool>())
    {
        logger->log_info("running the evaluation mode");
        planner = new DummyPlanner(vm["output"].as<std::string>());
    }
    else
    {
        planner = new MAPFPlanner();
    }

    auto input_json_file = vm["inputFile"].as<std::string>();
    json data;
    std::ifstream f(input_json_file);
    try
    {
        data = json::parse(f);
    }
    catch (json::parse_error error)
    {
        std::cerr << "Failed to load " << input_json_file << std::endl;
        std::cerr << "Message: " << error.what() << std::endl;
        exit(1);
    }

    auto map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);

    planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);
    planner->env->file_storage_path = vm["fileStoragePath"].as<std::string>();

    if (kwargs.contains("weights")) {
        std::string weight_str=kwargs["weights"].cast<std::string>();
        nlohmann::json weight_json=nlohmann::json::parse(weight_str);
        std::vector<float> weights;
        for (auto & w:weight_json) {
            weights.push_back(w.get<float>());
        }

        if (kwargs.contains("wait_costs")) {
            std::string wait_costs_str=kwargs["wait_costs"].cast<std::string>();
            nlohmann::json wait_costs_json=nlohmann::json::parse(wait_costs_str);
            std::vector<float> wait_costs;
            for (auto & w:wait_costs_json) {
                wait_costs.push_back(w.get<float>());
            }


            planner->map_weights=weight_format_conversion_with_wait_costs(grid, weights, wait_costs);
        } else {
            planner->map_weights=weight_format_conversion(grid, weights);
        }

    } 

    ActionModelWithRotate *model = new ActionModelWithRotate(grid);
    model->set_logger(logger);

    int team_size = read_param_json<int>(data, "teamSize");

    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<int> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));
    std::cout << agents.size() << " agents and " << tasks.size() << " tasks"<< std::endl;
    if (agents.size() > tasks.size())
        logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

    std::string task_assignment_strategy = data["taskAssignmentStrategy"].get<std::string>();
    if (task_assignment_strategy == "greedy")
    {
        system_ptr = std::make_unique<TaskAssignSystem>(grid, planner, agents, tasks, model);
    }
    else if (task_assignment_strategy == "roundrobin")
    {
        system_ptr = std::make_unique<InfAssignSystem>(grid, planner, agents, tasks, model);
    }
    else if (task_assignment_strategy == "roundrobin_fixed")
    {
        std::vector<vector<int>> assigned_tasks(agents.size());
        for (int i = 0; i < tasks.size(); i++)
        {
            assigned_tasks[i % agents.size()].push_back(tasks[i]);
        }
        system_ptr = std::make_unique<FixedAssignSystem>(grid, planner, agents, assigned_tasks, model);
    }
    else
    {
        std::cerr << "unkown task assignment strategy " << data["taskAssignmentStrategy"].get<std::string>() << std::endl;
        logger->log_fatal("unkown task assignment strategy " + data["taskAssignmentStrategy"].get<std::string>());
        exit(1);
    }

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(vm["planTimeLimit"].as<int>());
    system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());

    system_ptr->set_num_tasks_reveal(read_param_json<int>(data, "numTasksReveal", 1));

    signal(SIGINT, sigint_handler);

    system_ptr->simulate(vm["simulationTime"].as<int>());

    nlohmann::json analysis=system_ptr->analyzeResults();
    return analysis.dump(4);

    // if (!vm["evaluationMode"].as<bool>())
    // {
    //     system_ptr->saveResults(vm["output"].as<std::string>());
    // }

    // delete model;
    // delete logger;
    // return 0;
}

string playground(){
	std::string json_string = R"(
	{
		"pi": 3.141,
		"happy": true
	}
	)";
	json ex1 = json::parse(json_string);

	cout << ex1["pi"] << endl;

	return ex1.dump();
}


PYBIND11_MODULE(py_driver, m) {
	// optional module docstring
    // m.doc() = ;

    m.def("playground", &playground, "Playground function to test everything");
    // m.def("add", &add, py::arg("i")=0, py::arg("j")=1);
    m.def("run", &run, "Function to run warehouse simulation");
}