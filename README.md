# 🤖 Distributed Swarm Search & Rescue Simulator

A high-performance, decentralized multi-agent robotics simulation built in **C++20** and **Python**, designed to simulate autonomous search and rescue operations across complex environments. The core physics and communication engine are written in low-level C++ and bound to Python via **Pybind11**, rendering in real-time using **Pygame**.

Designed with a first-principles engineering approach, this simulator scales smoothly to **100+ robotic agents** operating under strict physical, computational, and communication constraints.

---

## 🌟 Key Features

* **High-Performance C++20 Core Engine:** Low-latency unicycle kinematics, acceleration limits, and occupancy grid mapping optimized for multi-threaded/high-density agent workloads.
* **Multi-Hop Mesh Network Routing:** Decentralized P2P communication manager that simulates limited radio range ($R_{comm}$) where agents act as wireless relay nodes, utilizing unique message IDs and TTL broadcast flood protection.
* **Autonomous Charging & Base Stations:** Behavioral state machine (`SEARCHING`, `RETURNING_TO_BASE`, `RECHARGING`) allowing robots to autonomously monitor battery levels, return to designated docks, recharge, and resume missions.
* **Proactive Obstacle Avoidance:** Raycasting lookahead collision detection with intelligent dead-end escape maneuvers (backing up and vector re-orientation).
* **Dynamic Survivor Memory & TTL Expiry:** Clustered duplicate survivor detection, visited status flags, and time-to-live (TTL) memory pruning for stale unvisited records.
* **Quantitative Metrics Logger:** Automated CSV telemetry export tracking mission progress, rescue rates, and energy consumption over time.

---

## 🛠️ Tech Stack & Architecture

* **Backend:** C++20, CMake
* **Python Integration:** Pybind11
* **Frontend / Visualization:** Python 3.10+, Pygame
* **Testing:** Standard C++ assertions (`<cassert>`), automated unit test suite

---

## 🚀 Getting Started

### Prerequisites
Ensure you have the following installed on your system (Linux/macOS/WSL):
* CMake (3.15+)
* A C++20 compliant compiler (GCC 10+ or Clang 10+)
* Python 3.10+
* Pygame (`pip install pygame`)

### Build and Run Instructions

1. **Clone the repository:**
   ```bash
   git clone [https://github.com/kriztenite/swarm-search-rescue.git](https://github.com/YOUR_USERNAME/swarm-search-rescue.git)
   cd swarm-search-rescue

2. **Compile the c++ core with CMake & Pybind11:**
    ```bash
    
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
    cmake --build build --config Release
    ```
3. **Run the C++ Unit Test Suite:**
    ```bash
    ./build/unit_tests
    ```
4. **Lanunch the Real-Time Python Visualizer:**
    create a venv an source into it then run this command.
    ```bash
    python3 python/main.py
    ```

