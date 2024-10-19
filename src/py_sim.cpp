#include "py_sim.h"

PYBIND11_MODULE(py_sim, m){
    py::class_<py_sim>(m, "py_sim")
        .def(py::init<py::kwargs>())
        .def("warmup", &py_sim::warmup);
        // .def("update_gg_and_step", &py_sim::update_gg_and_step)
        // .def("get_curr_pos", &py_sim::get_curr_pos)
        // .def("update_tasks_base_distribution", &py_sim::update_tasks_base_distribution)
        // .def("get_tasks_distribution", &py_sim::get_tasks_distribution);
}


py_sim::py_sim(py::kwargs kwargs){
    // cout << "py_sim constructor" << endl;
}

std::string py_sim::warmup(){
    // this->system_ptr->total_simulation_steps = this->simulation_steps;
    // this->system_ptr->warmup(this->warmup_steps);
    // if (this->save_path){
    //     this->system_ptr->saveResults(this->path_file.string());
    // }
    // return this->system_ptr->analyzeResults().dump(4);
    return "hi!";
}