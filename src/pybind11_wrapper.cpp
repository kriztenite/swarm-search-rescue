#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "Agent.hpp"
#include "CommsManager.hpp"
#include "GridMap.hpp"

namespace py = pybind11;

// Define the python module
PYBIND11_MODULE(swarm_core, m) {
  m.doc() = "C++ HIgh-Performance Swarm Core Engine";

  // 1. Expose CellState enum
  py::enum_<CellState>(m, "CellState")
      .value("FREE", CellState::FREE)
      .value("OBSTACLE", CellState::OBSTACLE)
      .value("SURVIVOR", CellState::SURVIVOR)
      .export_values();

  //  Expose GridMap class
  py::class_<GridMap>(m, "GridMap")
      .def(py::init<int, int, double>(), py::arg("width"), py::arg("height"),
           py::arg("resolution"))
      .def("set_cell", &GridMap::set_cell, py::arg("x"), py::arg("y"),
           py::arg("state"))
      .def("get_cell", &GridMap::get_cell, py::arg("x"), py::arg("y"))
      .def("is_collision", &GridMap::is_collision, py::arg("world_x"),
           py::arg("world_y"))
      .def("check_survivor", &GridMap::check_survivor, py::arg("world_x"),
           py::arg("world_y"))
      .def("get_width", &GridMap::get_width)
      .def("get_height", &GridMap::get_height)
      .def("get_resolution", &GridMap::get_resolution);

  // Expose the Pose struct so Python can read x, y, and theta
  py::class_<Pose>(m, "Pose")
      .def(py::init<>())
      .def_readwrite("x", &Pose::x)
      .def_readwrite("y", &Pose::y)
      .def_readwrite("theta", &Pose::theta);

  //  Expose the RobotAgent class and its public methods
  py::class_<RobotAgent>(m, "RobotAgent")
      // Expose the constructor with default argument max_speed = 2.0
      .def(py::init<int, double, double, double>(), py::arg("id"), py::arg("x"),
           py::arg("y"), py::arg("max_speed") = 2.0)
      // Expose member functions
      .def("step", &RobotAgent::step, py::arg("dt"), py::arg("map"),
           "Update kinematic state by dt seconds")
      .def("set_target_velocity", &RobotAgent::set_target_velocity,
           py::arg("v"), py::arg("omega"))
      .def("get_pose", &RobotAgent::get_pose, "Returns current Pose struct")
      .def("get_id", &RobotAgent::get_id)
      .def("get_battery", &RobotAgent::get_battery,
           "Returns battery percentage (0-100)")
      // Add Messaging and memory methods
      .def("receive_message", &RobotAgent::receive_message, py::arg("msg"))
      .def("get_inbox", &RobotAgent::get_inbox,
           "Returns vector of received messages")
      .def("clear_inbox", &RobotAgent::clear_inbox)
      .def("get_known_survivors", &RobotAgent::get_known_survivors,
           "Returns list of known survivor cordinates")
      .def("survivor_memory", &RobotAgent::add_survivor_memory, py::arg("x"),
           py::arg("y"));

  // Expose CommsManager
  py::class_<CommsManager>(m, "CommsManager")
      .def(py::init<double>(), py::arg("comms_range"))
      .def("communicate", &CommsManager::communicate, py::arg("agents"),
           "Sync messages between agents within range")
      .def("get_comms_range", &CommsManager::get_comms_range);
}
