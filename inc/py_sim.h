// #include "CompetitionSystem.h"
// #include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
// #include "nlohmann/json.hpp"
// #include <signal.h>
// #include <ctime>
// #include <climits>
// #include <memory>
// #include <util/Analyzer.h>
// #include "nlohmann/json.hpp"

#ifdef MAP_OPT
#include "util/analyze.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

namespace py = pybind11;

class py_sim {
    public:
        py_sim(py::kwargs kwargs);
        std::string warmup();
};