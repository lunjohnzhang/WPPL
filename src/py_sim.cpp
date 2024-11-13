#include "py_sim.h"

PYBIND11_MODULE(py_sim, m)
{
    py::class_<py_sim>(m, "py_sim")
        .def(py::init<py::kwargs>())
        .def("warmup", &py_sim::warmup)
        .def("update_gg_and_step", &py_sim::update_gg_and_step);
        // .def("get_curr_pos", &py_sim::get_curr_pos);
    // .def("update_tasks_base_distribution", &py_sim::update_tasks_base_distribution)
    // .def("get_tasks_distribution", &py_sim::get_tasks_distribution);
}


std::string py_sim::update_gg_and_step(std::vector<float> edge_weights, std::vector<float> wait_costs){
    this->planner->map_weights=weight_format_conversion_with_wait_costs(this->grid, edge_weights, wait_costs);
    this->planner->update();
    int actual_sim_steps = this->system_ptr->update_gg_and_step(this->update_gg_interval);
    auto results = this->system_ptr->analyzeCurrResults(actual_sim_steps);

    // Last timestep, save path if necessary
    if (results["done"]){
        if (this->save_path)
        {
            cout << "Saving paths" << endl;
            // this->system_ptr->saveResults(this->path_file.string());
            this->system_ptr->savePathsLoc(this->path_file.string());
        }
        // For sortation system, add additional information
        if (this->scenario == "SORTING")
        {
            auto sorting_system = dynamic_cast<SortationSystem*>(system_ptr.get());
            results["n_finish_task_plus_n_recirs"] = sorting_system->get_n_finish_task_plus_n_recirs();
            results["n_recirs"] = sorting_system->get_n_recirs();
            results["recirc_rate"] = (double)sorting_system->get_n_recirs() / sorting_system->get_n_finish_task_plus_n_recirs();
            results["chute_sleep_count"]  = sorting_system->get_chute_sleep_count();
        }
    }
    return results.dump(4);
}


py_sim::py_sim(py::kwargs kwargs)
{
    this->simulation_steps = kwargs["simulation_steps"].cast<int>();
    this->plan_time_limit = kwargs["plan_time_limit"].cast<double>();
    this->preprocess_time_limit = kwargs["preprocess_time_limit"].cast<double>();
    this->file_storage_path = kwargs["file_storage_path"].cast<std::string>();
    this->task_assignment_strategy = kwargs["task_assignment_strategy"].cast<std::string>();
    this->num_tasks_reveal = kwargs["num_tasks_reveal"].cast<int>();

    if (kwargs.contains("warmup_steps"))
    {
        this->warmup_steps = kwargs["warmup_steps"].cast<int>();
    }
    if (kwargs.contains("update_gg_interval"))
    {
        this->update_gg_interval = kwargs["update_gg_interval"].cast<int>();
    }

    // Read in left and right weights
    if (kwargs.contains("left_w_weight"))
    {
        this->left_w_weight = kwargs["left_w_weight"].cast<double>();
    }
    if (kwargs.contains("right_w_weight"))
    {
        this->right_w_weight = kwargs["right_w_weight"].cast<double>();
    }

    // Read in map. Use map_path by default. If not provided, use map_json
    if (kwargs.contains("map_path"))
    {
        std::string map_path = kwargs["map_path"].cast<std::string>();
        this->grid.load_map_from_path(map_path, left_w_weight, right_w_weight);
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
        this->grid.load_map_from_json(map_json, left_w_weight, right_w_weight);
    }

    this->logger = new Logger();

    cout << "Creating planner" << endl;
    this->planner = new MAPFPlanner();
    this->planner->env->map_name = grid.map_name;
    this->planner->env->file_storage_path = file_storage_path;

    if (kwargs.contains("config"))
    {
        std::string config_str = kwargs["config"].cast<std::string>();
        nlohmann::json config = nlohmann::json::parse(config_str);
        this->planner->config = config;
    }

    if (kwargs.contains("weights"))
    {
        std::string weight_str = kwargs["weights"].cast<std::string>();
        nlohmann::json weight_json = nlohmann::json::parse(weight_str);
        std::vector<float> weights;
        for (auto &w : weight_json)
        {
            weights.push_back(w.get<float>());
        }

        if (kwargs.contains("wait_costs"))
        {
            std::string wait_costs_str = kwargs["wait_costs"].cast<std::string>();
            nlohmann::json wait_costs_json = nlohmann::json::parse(wait_costs_str);
            std::vector<float> wait_costs;
            for (auto &w : wait_costs_json)
            {
                wait_costs.push_back(w.get<float>());
            }
            this->planner->map_weights = weight_format_conversion_with_wait_costs(grid, weights, wait_costs);
        }
        else
        {
            this->planner->map_weights = weight_format_conversion(
                grid, weights);
        }
    }

    this->model = new ActionModelWithRotate(grid);
    this->model->set_logger(logger);

    cout << "Loading scenario" << endl;
    this->scenario = kwargs["scenario"].cast<std::string>();

    // Get the scenario
    if (scenario == "COMPETITION")
    {
        assert(grid.agent_home_locations.size() == 0 ||
               grid.end_points.size() == 0);

        std::vector<int> agents;
        std::vector<int> tasks;

        if (!kwargs.contains("gen_random") || !kwargs["gen_random"].cast<bool>())
        {
            if (!kwargs.contains("agents_path") || !kwargs.contains("tasks_path"))
            {
                logger->log_fatal("agents_path and tasks_path must be provided if not generate instance randomly");
                exit(1);
            }
            auto agents_path = kwargs["agents_path"].cast<std::string>();
            auto tasks_path = kwargs["tasks_path"].cast<std::string>();
            agents = read_int_vec(agents_path);
            tasks = read_int_vec(tasks_path);
        }
        else
        {
            int num_agents = kwargs["num_agents"].cast<int>();
            int num_tasks = kwargs["num_tasks"].cast<int>();
            uint seed = kwargs["seed"].cast<uint>();
            gen_random_instance(grid, agents, tasks, num_agents, num_tasks, seed);
        }

        std::cout << agents.size() << " agents and " << tasks.size() << " tasks" << std::endl;
        if (agents.size() > tasks.size())
            logger->log_warning("Not enough tasks for robots (number of tasks < team size)");

        // std::string task_assignment_strategy = data["taskAssignmentStrategy"].get<std::string>();
        if (task_assignment_strategy == "greedy")
        {
            this->system_ptr = std::make_unique<TaskAssignSystem>(grid, planner, agents, tasks, model);
        }
        else if (task_assignment_strategy == "roundrobin")
        {
            this->system_ptr = std::make_unique<InfAssignSystem>(grid, planner, agents, tasks, model);
        }
        else if (task_assignment_strategy == "roundrobin_fixed")
        {
            std::vector<vector<int>> assigned_tasks(agents.size());
            for (int i = 0; i < tasks.size(); i++)
            {
                assigned_tasks[i % agents.size()].push_back(tasks[i]);
            }
            this->system_ptr = std::make_unique<FixedAssignSystem>(grid, planner, agents, assigned_tasks, model);
        }
        else
        {
            std::cerr << "unkown task assignment strategy " << task_assignment_strategy << std::endl;
            logger->log_fatal("unkown task assignment strategy " + task_assignment_strategy);
            exit(1);
        }
    }

    else if (scenario == "KIVA")
    {
        if (!kwargs.contains("gen_random") || !kwargs["gen_random"].cast<bool>())
        {
            logger->log_fatal("must generate instance randomly");
            exit(1);
        }

        int num_agents = kwargs["num_agents"].cast<int>();
        std::cout << "using kiva system (random task generation) with " << num_agents << " agents" << std::endl;

        uint seed = kwargs["seed"].cast<uint>();
        this->system_ptr = std::make_unique<KivaSystem>(grid, planner, model, num_agents, seed);
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
        cout << "creating package dist" << endl;
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

        cout << "Loading chute mapping" << endl;
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

        uint seed = kwargs["seed"].cast<uint>();
        int num_agents = kwargs["num_agents"].cast<int>();

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
        // cout << "Assign C " << assign_C << endl;

        // Recirculation mechanism
        bool recirc_mechanism = kwargs["recirc_mechanism"].cast<bool>();

        this->system_ptr = std::make_unique<SortationSystem>(grid, planner,
            model, chute_mapping_int, package_mode, packages,
            package_dist_weight, task_assignment_cost, task_assignment_params,
            assign_C, recirc_mechanism, num_agents, seed);
    }
    else
    {
        logger->log_fatal("unknown scenario: " + scenario);
        exit(1);
    }

    this->system_ptr->set_logger(logger);
    this->system_ptr->set_plan_time_limit(plan_time_limit);
    this->system_ptr->set_preprocess_time_limit(preprocess_time_limit);
    this->system_ptr->set_num_tasks_reveal(num_tasks_reveal);

    // signal(SIGINT, sigint_handler);

    // clock_t start_time = clock();
    // system_ptr->simulate(simulation_steps);
    // double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;

    // nlohmann::json analysis=system_ptr->analyzeResults();
    // analysis["cpu_runtime"] = runtime;

    // Save path if applicable
    if (kwargs.contains("save_paths"))
    {
        this->save_path = kwargs["save_paths"].cast<bool>();

        boost::filesystem::path output_dir(kwargs["file_storage_path"].cast<std::string>());
        boost::filesystem::create_directories(output_dir);
        this->path_file = output_dir / "paths.txt";
        // system_ptr->saveResults(path_file.string());
    }

    // return analysis.dump(4);
}

std::string py_sim::warmup()
{
    this->system_ptr->total_simulation_steps = this->simulation_steps;
    this->system_ptr->warmup(this->warmup_steps);
    // if (this->save_path)
    // {
    //     this->system_ptr->savePathsLoc(this->path_file.string());
    // }
    return this->system_ptr->analyzeResults(true).dump(4);
    // return "hi!";
}