#include <pybind11/pybind11.h>
#include <Agent.hpp>

namespace py = pybind11; 

// Define the python module 
PYBIND11_MODULE(swarm_core, m) {
    m.doc() = "HIgh-Performance Swarm Core Engine";
    
    // 1. Expose the Pose struct so Python can read x, y, and theta
    py::class_<Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("x", &Pose::x)
        .def_readwrite("y", &Pose::y)
        .def_readwrite("theta", &Pose::theta);


    // 2. Expose the RobotAgent class and its public methods
    py::class_<RobotAgent>(m, "RobotAgent")
        // Expose the constructor with default argument max_speed = 2.0
        .def(py::init<int, double, double, double>(),
                py::arg("id"), py::arg("x"), py::arg("y"), py::arg("max_speed") = 2.0)
        // Expose member functions
        .def("step", &RobotAgent::step, py::arg("dt"), "Update kinematic state by dt seconds")
        .def("set_target_velocity", &RobotAgent::set_target_velocity, py::arg("v"), py::arg("omega"))
        .def("get_pose", &RobotAgent::get_pose, "Returns current Pose struct")
        .def("get_id", &RobotAgent::get_id)
        .def("get_battery", &RobotAgent::get_battery, "Returns battery percentage (0-100)");
}
