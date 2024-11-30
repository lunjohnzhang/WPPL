#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>
#include <ctime>
#include <climits>
#include <memory>
#include <util/Analyzer.h>
#include "nlohmann/json.hpp"
#include <cassert>
#include "SortationSystem.h"
#include "py_utils.h"

#ifdef MAP_OPT
#include "util/analyze.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

namespace py = pybind11;

// po::variables_map vm;
std::unique_ptr<BaseSystem> system_ptr;


// void sigint_handler(int a)
// {
//     fprintf(stdout, "stop the simulation...\n");
//     if (!vm["evaluationMode"].as<bool>())
//     {
//         system_ptr->saveResults(vm["output"].as<std::string>());
//     }
//     _exit(0);
// }


std::string run(const py::kwargs& kwargs)
{
    int simulation_steps=kwargs["simulation_steps"].cast<int>();
    double plan_time_limit=kwargs["plan_time_limit"].cast<double>();
    double preprocess_time_limit=kwargs["preprocess_time_limit"].cast<double>();
    // std::string map_path=kwargs["map_path"].cast<std::string>();
    std::string file_storage_path=kwargs["file_storage_path"].cast<std::string>();
    std::string task_assignment_strategy=kwargs["task_assignment_strategy"].cast<std::string>();
    int num_tasks_reveal=kwargs["num_tasks_reveal"].cast<int>();

    // Read in left and right weights
    double left_w_weight = 1;
    double right_w_weight = 1;
    if (kwargs.contains("left_w_weight"))
    {
        left_w_weight = kwargs["left_w_weight"].cast<double>();
    }
    if (kwargs.contains("right_w_weight"))
    {
        right_w_weight = kwargs["right_w_weight"].cast<double>();
    }

    // Read in map. Use map_path by default. If not provided, use map_json
    Grid grid;
    if (kwargs.contains("map_path"))
    {
        std::string map_path = kwargs["map_path"].cast<std::string>();
        grid.load_map_from_path(map_path, left_w_weight, right_w_weight);
    }
    else if (kwargs.contains("map_json_str") ||
             kwargs.contains("map_json_path"))
    {
        json map_json;
        if (kwargs.contains("map_json_str"))
        {
            std::string map_json_str = kwargs["map_json_str"].cast<std::string>();
            map_json = json::parse(map_json_str);
        }
        else
        {
            std::string map_json_path = kwargs["map_json_path"].cast<std::string>();
            std::ifstream f(map_json_path);
            map_json = json::parse(f);
        }
        grid.load_map_from_json(map_json, left_w_weight, right_w_weight);
    }

    // // should be a command line string running the code
    // std::string cmd=kwargs["cmd"].cast<std::string>();
    // std::cout<<"cmd from python is: "<<cmd<<std::endl;

    // // Declare the supported options.
    // po::options_description desc("Allowed options");
    // desc.add_options()("help", "produce help message")
    //     // ("inputFolder", po::value<std::string>()->default_value("."), "input folder")
    //     ("inputFile,i", po::value<std::string>()->required(), "input file name")
    //     ("output,o", po::value<std::string>()->default_value("./test.json"), "output file name")
    //     ("evaluationMode", po::value<bool>()->default_value(false), "evaluate an existing output file")
    //     ("simulationTime", po::value<int>()->default_value(5000), "run simulation")
    //     ("fileStoragePath", po::value<std::string>()->default_value(""), "the path to the storage path")
    //     ("planTimeLimit", po::value<int>()->default_value(INT_MAX), "the time limit for planner in seconds")
    //     ("preprocessTimeLimit", po::value<int>()->default_value(INT_MAX), "the time limit for preprocessing in seconds")
    //     ("logFile,l", po::value<std::string>(), "issue log file name");
    // clock_t start_time = clock();
    // // po::store(po::parse_command_line(argc, argv, desc), vm);

    // po::store(po::command_line_parser(po::split_unix(cmd)).options(desc).run(), vm);

    // if (vm.count("help"))
    // {
    //     std::cout << desc << std::endl;
    //     exit(-1);
    // }

    // po::notify(vm);

    // std::string base_folder = vm["inputFolder"].as<std::string>();
    // boost::filesystem::path p(vm["inputFile"].as<std::string>());

    // ONLYDEV(
    //     const auto & filename=p.filename();
    //     analyzer.data["instance"]=filename.c_str();
    // )

    // boost::filesystem::path dir = p.parent_path();
    // std::string base_folder = dir.string();
    // std::cout << base_folder << std::endl;
    // if (base_folder.size() > 0 && base_folder.back() != '/')
    // {
    //     base_folder += "/";
    // }

    Logger *logger = new Logger();
    // if (vm.count("logFile"))
    //     logger->set_logfile(vm["logFile"].as<std::string>());

    MAPFPlanner *planner = nullptr;
    // Planner is inited here, but will be managed and deleted by system_ptr deconstructor
    // if (vm["evaluationMode"].as<bool>())
    // {
    //     logger->log_info("running the evaluation mode");
    //     planner = new DummyPlanner(vm["output"].as<std::string>());
    // }
    // else
    // {
        planner = new MAPFPlanner();
    // }

    // auto input_json_file = vm["inputFile"].as<std::string>();
    // json data;
    // std::ifstream f(input_json_file);
    // try
    // {
    //     data = json::parse(f);
    // }
    // catch (json::parse_error error)
    // {
    //     std::cerr << "Failed to load " << input_json_file << std::endl;
    //     std::cerr << "Message: " << error.what() << std::endl;
    //     exit(1);
    // }

    // auto map_path = read_param_json<std::string>(data, "mapFile");
    // Grid grid(base_folder + map_path);
    // Grid grid(map_path);

    // planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);
    planner->env->map_name = grid.map_name;
    // planner->env->file_storage_path = vm["fileStoragePath"].as<std::string>();
    planner->env->file_storage_path = file_storage_path;

    if (kwargs.contains("config")){
        std::string config_str=kwargs["config"].cast<std::string>();
        nlohmann::json config=nlohmann::json::parse(config_str);
        planner->config=config;
    }

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

    std::string scenario = kwargs["scenario"].cast<std::string>();

    // Get the scenario
    if (scenario == "COMPETITION")
    {
        assert(grid.agent_home_locations.size()==0 ||
               grid.end_points.size()==0);

        std::vector<int> agents;
        std::vector<int> tasks;

        if (!kwargs.contains("gen_random") || !kwargs["gen_random"].cast<bool>()){
            if (!kwargs.contains("agents_path") || !kwargs.contains("tasks_path")){
                logger->log_fatal("agents_path and tasks_path must be provided if not generate instance randomly");
                exit(1);
            }
            auto agents_path = kwargs["agents_path"].cast<std::string>();
            auto tasks_path = kwargs["tasks_path"].cast<std::string>();
            agents = read_int_vec(agents_path);
            tasks = read_int_vec(tasks_path);
        } else {
            int num_agents=kwargs["num_agents"].cast<int>();
            int num_tasks=kwargs["num_tasks"].cast<int>();
            uint seed=kwargs["seed"].cast<uint>();
            gen_random_instance(grid, agents, tasks, num_agents, num_tasks, seed);
        }

        std::cout << agents.size() << " agents and " << tasks.size() << " tasks"<< std::endl;
        if (agents.size() > tasks.size())
            logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

        // std::string task_assignment_strategy = data["taskAssignmentStrategy"].get<std::string>();
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
            std::cerr << "unkown task assignment strategy " << task_assignment_strategy << std::endl;
            logger->log_fatal("unkown task assignment strategy " + task_assignment_strategy);
            exit(1);
        }
    }

    else if (scenario == "KIVA") {
        if (!kwargs.contains("gen_random") || !kwargs["gen_random"].cast<bool>()){
            logger->log_fatal("must generate instance randomly");
            exit(1);
        }

        int num_agents=kwargs["num_agents"].cast<int>();
        std::cout << "using kiva system (random task generation) with "<<num_agents<<" agents"<<std::endl;

        uint seed=kwargs["seed"].cast<uint>();
        system_ptr = std::make_unique<KivaSystem>(grid,planner,model,num_agents,seed);
    }

    else if (scenario == "SORTING")
    {
        // Sortation system assumes the existence of "packages" and
        // "chute_mapping" arguments, both of which are json strings

        // For packages, the program accepts a list of explicit "packages" or a
        // distribution of packages on each destination
        std::string package_mode = kwargs["package_mode"].cast<std::string>();
        vector<int> packages;
        vector<double> package_dist_weight;
        if (package_mode == "explicit")
        {
            if (!kwargs.contains("packages"))
            {
                logger->log_fatal("packages must be provided for sortation system");
                exit(1);
            }
            nlohmann::json packages_json = nlohmann::json::parse(
                kwargs["packages"].cast<std::string>());
            packages = packages_json.get<vector<int>>();
        }
        else if (package_mode == "dist")
        {
            if (!kwargs.contains("package_dist_weight"))
            {
                logger->log_fatal("package_dist_weight must be provided for sortation system");
                exit(1);
            }
            nlohmann::json package_dist_weight_json = nlohmann::json::parse(
                kwargs["package_dist_weight"].cast<std::string>());
            package_dist_weight = package_dist_weight_json.get<vector<double>>();
        }
        else
        {
            logger->log_fatal("packages or package_dist_weight must be provided for sortation system");
            exit(1);
        }

        nlohmann::json chute_mapping_json = nlohmann::json::parse(
            kwargs["chute_mapping"].cast<std::string>());
        // cout << chute_mapping_json << endl;
        map<string, vector<int>> chute_mapping = chute_mapping_json.get<map<string, vector<int>>>();

        // Transform map<string, vector<int>> to a map<int, vector<int>>
        map<int, vector<int>> chute_mapping_int;
        for (auto const &pair : chute_mapping)
        {
            chute_mapping_int[stoi(pair.first)] = pair.second;
        }

        // // Print content of packages
        // for (int i = 0; i < packages.size(); i++)
        // {
        //     std::cout << packages[i] << " ";
        // }
        // cout << endl;
        // Print content of chute_mapping
        // for (auto const &pair : chute_mapping_int)
        // {
        //     std::cout << "{" << pair.first << ": ";
        //     for (int i = 0; i < pair.second.size(); i++)
        //     {
        //         std::cout << pair.second[i] << " ";
        //     }
        //     std::cout << "}" << std::endl;
        // }

        uint seed=kwargs["seed"].cast<uint>();
        int num_agents=kwargs["num_agents"].cast<int>();

        // Task assignment policy
        string task_assignment_cost=kwargs["task_assignment_cost"].cast<std::string>();
        // If the task assignment policy is of an optimization-based policy,
        // read the parameters
        vector<double> task_assignment_params;
        double assign_C = 8.0; // Default
        if (task_assignment_cost == "opt_quadratic_f")
        {
            nlohmann::json task_assignment_params_json = nlohmann::json::parse(
                kwargs["task_assignment_params"].cast<std::string>());
            task_assignment_params = task_assignment_params_json.get<vector<double>>();
        }
        else if (task_assignment_cost == "heuristic+num_agents")
        {
            if (kwargs.contains("assign_C"))
            {
                assign_C = kwargs["assign_C"].cast<double>();
            }
        }

        bool recirc_mechanism = kwargs["recirc_mechanism"].cast<bool>();
        int task_waiting_time = kwargs["task_waiting_time"].cast<int>();
        int workstation_waiting_time = kwargs["workstation_waiting_time"].cast<int>();
        double task_gaussian_sigma = kwargs["task_gaussian_sigma"].cast<double>();
        int task_change_time = kwargs["task_change_time"].cast<int>();

        system_ptr = std::make_unique<SortationSystem>(grid, planner, model,
            chute_mapping_int, package_mode, packages, package_dist_weight,
            task_assignment_cost, task_assignment_params, assign_C,
            recirc_mechanism, task_waiting_time, workstation_waiting_time,
            task_gaussian_sigma, task_change_time, num_agents, seed);
    }
    else
    {
        logger->log_fatal("unknown scenario: "+scenario);
        exit(1);
    }

    system_ptr->set_logger(logger);
    system_ptr->set_plan_time_limit(plan_time_limit);
    system_ptr->set_preprocess_time_limit(preprocess_time_limit);

    system_ptr->set_num_tasks_reveal(num_tasks_reveal);

    // signal(SIGINT, sigint_handler);

    clock_t start_time = clock();
    system_ptr->simulate(simulation_steps);
    double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;

    nlohmann::json analysis=system_ptr->analyzeResults(false);
    analysis["cpu_runtime"] = runtime;
    if (scenario == "SORTING")
    {
        auto sorting_system = dynamic_cast<SortationSystem*>(system_ptr.get());
        analysis["n_finish_task_plus_n_recirs"] = sorting_system->get_n_finish_task_plus_n_recirs();
        analysis["n_recirs"] = sorting_system->get_n_recirs();
        analysis["recirc_rate"] = (double)sorting_system->get_n_recirs() / sorting_system->get_n_finish_task_plus_n_recirs();
        analysis["chute_sleep_count"]  = sorting_system->get_chute_sleep_count();
    }


    // Save path if applicable
    if (kwargs.contains("save_paths") && kwargs["save_paths"].cast<bool>())
    {
        boost::filesystem::path output_dir(kwargs["file_storage_path"].cast<std::string>());
        boost::filesystem::create_directories(output_dir);
        // boost::filesystem::path result_file = output_dir / "results.json";
        boost::filesystem::path path_file = output_dir / "paths.txt";
        // system_ptr->saveResults(result_file.string());
        system_ptr->savePathsLoc(path_file.string());
    }

    // system_ptr->saveResults("debug.json");
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