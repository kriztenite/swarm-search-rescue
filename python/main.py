import math
import sys
from dataclasses import dataclass
from enum import Enum

import pygame

# import our compiled c++ module
try:
    import swarm_core
except ImportError as e:
    print(
        "Error: 'swarm_core' module not found. Please compile the c++ project with CMake first. "
    )
    print(f"Debug info: {e}")
    sys.exit(1)


@dataclass
class SimulationConfig:
    """Centralized configurations"""

    # Window Dimensions
    WIDTH, HEIGHT = 800, 600
    RESOLUTION = 20.0  # Grid cell size (pixedl == world units)
    GRID_COLS = int(WIDTH / RESOLUTION)
    GRID_ROWS = int(HEIGHT / RESOLUTION)
    FPS = 60
    COMMS_RANGE: float = 180.0
    SENSOR_RANGE: float = 80.0
    METRICS_LOG_INTERVAL: float = 1.0  # Log every 1 second, not every frame
    BASE_X: float = 80.0
    BASE_Y: float = 80.0

    NUM_ROBOTS = 5


class Colors(Enum):
    """All colors in one place for easier themeing"""

    # Colors (RGB)
    BG = (20, 20, 30)
    UNKNOWN = (30, 30, 45)
    FREE = (45, 45, 65)
    OBSTACLE = (60, 60, 80)
    SURVIVOR = (255, 215, 0)
    KNOWN_SURVIVOR = (165, 42, 42)
    VISITED_SURVIVOR = (100, 100, 100)
    SEARCHING = (0, 255, 150)
    RETURNING = (255, 165, 0)
    RECHARGING = (0, 191, 255)
    BASE = (0, 120, 255)
    BASE_INNER = (20, 80, 150)
    HEADING = (255, 50, 50)
    COMMS_LINE = (0, 150, 255)
    SENSOR = (0, 210, 230)
    TEXT = (220, 220, 220)


class SimulationState:
    """Encapsulate all simulation state"""

    def __init__(self, config: SimulationConfig):
        self.config = config
        self.sim_time = 0.0
        self.world_map = None
        self.robots: list[swarm_core.RobotAgent] = []
        self.metrics_logger = None
        self.survivors_found = 0
        self.last_metrics_log = 0.0

    def init_world(self):
        """Initialize grid, obstacles, and robots"""
        grid_cols = int(self.config.WIDTH / self.config.RESOLUTION)
        grid_rows = int(self.config.HEIGHT / self.config.RESOLUTION)

        self.world_map = swarm_core.GridMap(
            grid_cols, grid_rows, self.config.RESOLUTION
        )
        self.comms = swarm_core.CommsManager(self.config.COMMS_RANGE)
        self.metrics_logger = swarm_core.MetricsLogger("simulation_metrics.csv")

        # Build borders (mark as dirty)
        for x in range(grid_cols):
            self.world_map.set_cell(x, 0, swarm_core.CellState.OBSTACLE)
            self.world_map.set_cell(x, grid_rows - 1, swarm_core.CellState.OBSTACLE)
        for y in range(grid_rows):
            self.world_map.set_cell(0, y, swarm_core.CellState.OBSTACLE)
            self.world_map.set_cell(grid_cols - 1, y, swarm_core.CellState.OBSTACLE)

        # Internal obstacles
        for y in range(8, 15):
            self.world_map.set_cell(15, y, swarm_core.CellState.OBSTACLE)
        for x in range(20, 30):
            self.world_map.set_cell(x, 20, swarm_core.CellState.OBSTACLE)

        # Scatter survivors across the map
        self.survivors_locations = [
            (6, 6),
            (12, 22),
            (26, 8),
            (33, 24),
            (18, 18),
            (35, 12),
        ]
        for sx, sy in self.survivors_locations:
            self.world_map.set_cell(sx, sy, swarm_core.CellState.SURVIVOR)

        # Create robots with more realistic spacing
        # self.robots = [
        #     swarm_core.RobotAgent(id=i + 1, x=100.0 + (i * 150), y=180.0 + (i * 80))
        #     for i in range(5)
        # ]

        # Create scattered robots across open areas of the map
        self.robots = []
        for i in range(self.config.NUM_ROBOTS):
            # Deterministic spread pattern avoiding borders and inner obstacles
            rx = 120.0 + ((i * 110.0) % 560.0)
            ry = 120.0 + (((i * 70.0) // 560.0) * 80.0 + (i * 45.0) % 360.0)

            # Clamp inside safe bounds
            rx = max(80.0, min(self.config.WIDTH - 80.0, rx))
            ry = max(80.0, min(self.config.HEIGHT - 80.0, ry))

            robot = swarm_core.RobotAgent(id=i + 1, x=rx, y=ry)
            self.robots.append(robot)

        # TODO: rexamine the movement pattern to closely emulate real robot movement

        # Initialize velocities
        for i, robot in enumerate(self.robots):
            # Drive forward with a slight turning rate to make them move in arcs
            robot.set_target_velocity(v=80.0, omega=0.5 + (i * 0.5))

    def step(self, dt: float) -> None:
        """Single simulation step - called once per frame"""
        self.sim_time += dt

        # 1. Communication (mesh network broadcast)
        self.comms.communicate(self.robots)

        # 2. Physics & sensor updates (single step per robot)
        for robot in self.robots:
            # Perform physics step
            robot.step(dt, self.sim_time, self.world_map)

            pose = robot.get_pose()

            # Sensor: Reveal map cell
            self.world_map.reveal_radius(pose.x, pose.y, self.config.SENSOR_RANGE)

            # Sensor: check for survivors
            if self.world_map.check_survivor(pose.x, pose.y):
                print(
                    f"[DISCOVERY] Robot {robot.get_id()} found a survivor "
                    f"at ({int(pose.x)}, {int(pose.y)})!"
                )
                robot.add_survivor_memory(pose.x, pose.y, True, self.sim_time)
                robot.mark_survivor_visited(pose.x, pose.y)
                self.survivors_found += 1

            # Communication: Process incoming survivor reports
            for msg in robot.get_inbox():
                if msg.survivor_found:
                    robot.add_survivor_memory(
                        msg.survivor_x, msg.survivor_y, msg.visited, self.sim_time
                    )

            # Behavior: Auto-target next unvisited survivor
            if not robot.has_target():
                for surv in robot.get_known_survivors():
                    if not surv.visited:
                        robot.set_navigation_target(surv.x, surv.y)
                        break
        # 3. Periodic metrics logging (not every frame)
        if self.sim_time - self.last_metrics_log >= self.config.METRICS_LOG_INTERVAL:
            self.metrics_logger.log_step(
                self.sim_time,
                self.robots,
                self.world_map,
                len(self.survivors_locations),
            )
            self.last_metrics_log = self.sim_time


class Renderer:
    """Seperate rendering logic from simulation"""

    def __init__(self, config: SimulationConfig):
        self.config = config
        self.screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
        pygame.display.set_caption("Swarm Search & Rescue - Cooperative Exploration")
        self.font_small = pygame.font.SysFont(None, 12)
        self.font_large = pygame.font.SysFont(None, 20)
        self.comms_cache = {}  # Cache comms links

        # Static terrain layer: walls, obstacles, survivors, base background
        self.map_surface = pygame.Surface((config.WIDTH, config.HEIGHT))
        self.map_surface.fill(Colors.BG.value)

    def render(self, state: SimulationState) -> None:
        """Render everything each frame"""
        self.screen.blit(self.map_surface, (0, 0))
        self.render_dirty_cells(state)
        self.render_base_station()
        self.render_comms_links(state.robots)
        self.render_robots(state.robots)
        self.render_hud(state)

    def render_dirty_cells(self, state: SimulationState) -> None:
        """Render only dirty grid cells (optimizing for large grids)"""

        for gx in range(state.world_map.get_width()):
            for gy in range(state.world_map.get_height()):
                cell_state = state.world_map.get_cell(gx, gy)
                rect = pygame.Rect(
                    gx * self.config.RESOLUTION,
                    gy * self.config.RESOLUTION,
                    self.config.RESOLUTION,
                    self.config.RESOLUTION,
                )
                if cell_state == swarm_core.CellState.UNKNOWN:
                    pygame.draw.rect(self.screen, Colors.UNKNOWN.value, rect)
                elif cell_state == swarm_core.CellState.FREE:
                    pygame.draw.rect(self.screen, Colors.FREE.value, rect)
                elif cell_state == swarm_core.CellState.OBSTACLE:
                    self._draw_obstacle(rect)
                elif cell_state == swarm_core.CellState.SURVIVOR:
                    self._draw_survivor(gx, gy)

        # state.render_cache.clear_dirty()

    def _draw_obstacle(self, rect: pygame.Rect) -> None:
        """Draw textured obstacle"""
        pygame.draw.rect(self.screen, (110, 55, 45), rect)
        pygame.draw.line(
            self.screen, (45, 25, 20), (rect.left, rect.top), (rect.right, rect.top), 1
        )
        pygame.draw.line(
            self.screen,
            (45, 25, 20),
            (rect.left, rect.top),
            (rect.left, rect.bottom),
            1,
        )
        pygame.draw.line(
            self.screen,
            (45, 25, 20),
            (rect.left, rect.centery),
            (rect.right, rect.centery),
            1,
        )

    def _draw_survivor(self, gx: int, gy: int) -> None:
        """Draw glowing survivor"""
        center_x = int(gx * self.config.RESOLUTION + self.config.RESOLUTION / 2)
        center_y = int(gy * self.config.RESOLUTION + self.config.RESOLUTION / 2)
        pygame.draw.circle(
            self.screen,
            Colors.SURVIVOR.value,
            (center_x, center_y),
            int(self.config.RESOLUTION / 3),
        )

    def render_base_station(self) -> None:
        """Draw base station"""
        pygame.draw.circle(
            self.screen,
            Colors.BASE.value,
            (int(self.config.BASE_X), int(self.config.BASE_Y)),
            24,
            3,
        )
        pygame.draw.circle(
            self.screen,
            Colors.BASE_INNER.value,
            (int(self.config.BASE_X), int(self.config.BASE_Y)),
            18,
        )

    def render_comms_links(self, robots: list[swarm_core.RobotAgent]) -> None:
        """Render Communication links only when they change"""
        current_links = set()

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                p1 = robots[i].get_pose()
                p2 = robots[j].get_pose()
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= self.config.COMMS_RANGE:
                    current_links.add((i, j))
                    pygame.draw.line(
                        self.screen,
                        Colors.COMMS_LINE.value,
                        (int(p1.x), int(p1.y)),
                        (int(p2.x), int(p2.y)),
                        1,
                    )

    def render_robots(self, robots: list[swarm_core.RobotAgent]) -> None:
        """Render all robots and their metadata"""
        for robot in robots:
            pose = robot.get_pose()
            rx, ry, rtheta = int(pose.x), int(pose.y), pose.theta

            # Draw know survivors in memory
            for surv in robot.get_known_survivors():
                color = (
                    Colors.VISITED_SURVIVOR.value
                    if surv.visited
                    else Colors.KNOWN_SURVIVOR.value
                )
                pygame.draw.circle(self.screen, color, (int(surv.x), int(surv.y)), 8, 1)

            # select color based on behavior state
            behavior = robot.get_behavior()
            if behavior == swarm_core.RobotBehavior.RECHARGING:
                robot_color = Colors.RECHARGING.value
            elif behavior == swarm_core.RobotBehavior.RETURNING_TO_BASE:
                robot_color = Colors.RETURNING.value
            else:
                robot_color = Colors.SEARCHING.value

            # Draw robot body
            pygame.draw.circle(
                self.screen, robot_color, (rx, ry), int(self.config.RESOLUTION / 4.0)
            )
            self._render_status_text(rx, ry, robot)

            # Draw heading vector
            end_x = rx + int(15 * math.cos(rtheta))
            end_y = ry + int(15 * math.sin(rtheta))
            pygame.draw.line(
                self.screen, Colors.HEADING.value, (rx, ry), (end_x, end_y), 3
            )
            # Draw radar lines
            self.render_lidar_arc(robot)
            # Draw robot status text
            self._render_status_text(rx, ry, robot)

    def render_lidar_arc(self, robot: swarm_core.RobotAgent):
        """Render the sensor arrays"""
        pose = robot.get_pose()
        x, y, theta = pose.x, pose.y, pose.theta

        num_rays = 21
        fov = math.radians(100)
        max_range = 90

        for i in range(num_rays):
            ang = theta - fov / 2 + fov / 2 * (i / (num_rays - 1))
            ex = x + max_range * math.cos(ang)
            ey = y + max_range * math.sin(ang)
            pygame.draw.line(
                self.screen,
                Colors.SENSOR.value,
                (int(x), int(y)),
                (int(ex), int(ey)),
                1,
            )

    def _render_status_text(
        self, rx: int, ry: int, robot: swarm_core.RobotAgent
    ) -> None:
        """Display robots status text"""
        text = self.font_small.render(
            f"R{robot.get_id()} ({int(robot.get_battery())}%)",
            True,
            Colors.TEXT.value,
        )
        self.screen.blit(text, (rx - 20, ry - 20))

    def render_hud(self, state: SimulationState):
        """Render heads-up display with simulation stats"""
        stats_text = [
            f"Time: {state.sim_time:.1f}s",
            f"Robots: {len(state.robots)}",
            f"Survivor's Found: {state.survivors_found}/{len(state.survivors_locations)}",
            "FPS: 60",
        ]
        y_offset = 10
        for stat in stats_text:
            text = self.font_small.render(stat, True, Colors.TEXT.value)
            self.screen.blit(text, (10, y_offset))
            y_offset += 20


def main():
    """Main simulation loop"""

    # Initialize Pygame
    pygame.init()

    # Create config and state
    config = SimulationConfig()
    state = SimulationState(config)
    renderer = Renderer(config)

    # Initialize world (grid, obstacles, robots)
    state.init_world()

    # Main loop parameters
    clock = pygame.time.Clock()
    running = True
    dt = 1.0 / config.FPS  # Fixed delta time per frame

    print(f"[SIMULATION] Starting with {len(state.robots)} robots")
    print(f"[SIMULATION] Grid size: {config.GRID_COLS} x {config.GRID_ROWS}")
    print(
        f"[SIMULATION] Comms range: {config.COMMS_RANGE}, Sensor range: {config.SENSOR_RANGE}"
    )

    while running:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        # 2. Single simulation step (Not double!)
        state.step(dt)

        # 3. Rendering (single-threaded)
        renderer.render(state)
        pygame.display.flip()

        # 4. Frame rate control
        clock.tick(config.FPS)

    # Clean up
    print("[SIMULATION] shutting down...")
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()


# import math
# import sys
# from dataclasses import dataclass
# from enum import Enum
#
# import pygame
#
# # Import our compiled C++ module
# try:
#     import swarm_core
# except ImportError as e:
#     print(
#         "Error: 'swarm_core' module not found. Please compile the C++ project with CMake first."
#     )
#     print(f"Debug info: {e}")
#     sys.exit(1)
#
#
# @dataclass
# class SimulationConfig:
#     """Centralized configurations"""
#
#     WIDTH: int = 800
#     HEIGHT: int = 600
#     RESOLUTION: float = 20.0  # Grid cell size
#     FPS: int = 60
#     COMMS_RANGE: float = 180.0
#     SENSOR_RANGE: float = 80.0
#     METRICS_LOG_INTERVAL: float = 1.0  # Log every 1 second
#     BASE_X: float = 100.0
#     BASE_Y: float = 100.0
#     NUM_ROBOTS = 10
#
#     @property
#     def GRID_COLS(self) -> int:
#         return int(self.WIDTH / self.RESOLUTION)
#
#     @property
#     def GRID_ROWS(self) -> int:
#         return int(self.HEIGHT / self.RESOLUTION)
#
#
# class Colors(Enum):
#     """All colors in one place for easier themeing"""
#
#     BG = (20, 20, 30)
#     UNKNOWN = (30, 30, 45)
#     FREE = (45, 45, 65)
#     OBSTACLE = (60, 60, 80)
#     SURVIVOR = (255, 215, 0)
#     KNOWN_SURVIVOR = (255, 140, 0)
#     VISITED_SURVIVOR = (100, 100, 100)
#     SEARCHING = (0, 255, 150)
#     RETURNING = (255, 165, 0)
#     RECHARGING = (0, 191, 255)
#     BASE = (0, 120, 255)
#     HEADING = (255, 50, 50)
#     COMMS_LINE = (0, 150, 255)
#     TEXT = (220, 220, 220)
#
#
# class SimulationState:
#     """Encapsulate all simulation state"""
#
#     def __init__(self, config: SimulationConfig):
#         self.config = config
#         self.sim_time = 0.0
#         self.world_map = None
#         self.comms = None
#         self.robots: list[swarm_core.RobotAgent] = []
#         self.metrics_logger = None
#         self.survivors_found = 0
#         self.last_metrics_log = 0.0
#         self.survivors_locations = []
#
#     def init_world(self):
#         """Initialize grid, obstacles, and robots"""
#         self.world_map = swarm_core.GridMap(
#             self.config.GRID_COLS, self.config.GRID_ROWS, self.config.RESOLUTION
#         )
#         # Fixed: Correct C++ class name 'CommsManager'
#         self.comms = swarm_core.CommsManager(self.config.COMMS_RANGE)
#         self.metrics_logger = swarm_core.MetricsLogger("simulation_metrics.csv")
#
#         # Build borders
#         for x in range(self.config.GRID_COLS):
#             self.world_map.set_cell(x, 0, swarm_core.CellState.OBSTACLE)
#             self.world_map.set_cell(
#                 x, self.config.GRID_ROWS - 1, swarm_core.CellState.OBSTACLE
#             )
#         for y in range(self.config.GRID_ROWS):
#             self.world_map.set_cell(0, y, swarm_core.CellState.OBSTACLE)
#             self.world_map.set_cell(
#                 self.config.GRID_COLS - 1, y, swarm_core.CellState.OBSTACLE
#             )
#
#         # Internal obstacles
#         for y in range(8, 15):
#             self.world_map.set_cell(15, y, swarm_core.CellState.OBSTACLE)
#         for x in range(20, 30):
#             self.world_map.set_cell(x, 20, swarm_core.CellState.OBSTACLE)
#
#         # Scatter survivors across the map
#         self.survivors_locations = [
#             (6, 6),
#             (12, 22),
#             (26, 8),
#             (33, 24),
#             (18, 18),
#             (35, 12),
#         ]
#         for sx, sy in self.survivors_locations:
#             self.world_map.set_cell(sx, sy, swarm_core.CellState.SURVIVOR)
#
#         # Create scattered robots across open areas of the map
#         self.robots = []
#         for i in range(self.config.NUM_ROBOTS):
#             # Deterministic spread pattern avoiding borders and inner obstacles
#             rx = 120.0 + ((i * 110.0) % 560.0)
#             ry = 120.0 + (((i * 70.0) // 560.0) * 80.0 + (i * 45.0) % 360.0)
#
#             # Clamp inside safe bounds
#             rx = max(80.0, min(self.config.WIDTH - 80.0, rx))
#             ry = max(80.0, min(self.config.HEIGHT - 80.0, ry))
#
#             robot = swarm_core.RobotAgent(id=i + 1, x=rx, y=ry)
#             robot.set_target_velocity(v=80.0, omega=0.3 + ((i % 5) * 0.2))
#             self.robots.append(robot)
#
#         # Initialize velocities
#         for i, robot in enumerate(self.robots):
#             robot.set_target_velocity(v=80.0, omega=0.5 + (i * 0.2))
#
#     def step(self, dt: float) -> None:
#         """Single simulation step - called once per frame"""
#         self.sim_time += dt
#
#         # 1. Communication (mesh network broadcast)
#         self.comms.communicate(self.robots)
#
#         # 2. Physics & sensor updates
#         for robot in self.robots:
#             robot.step(dt, self.sim_time, self.world_map)
#             pose = robot.get_pose()
#
#             # Sensor: Reveal map cell
#             self.world_map.reveal_radius(pose.x, pose.y, self.config.SENSOR_RANGE)
#
#             # Sensor: check for survivors
#             if self.world_map.check_survivor(pose.x, pose.y):
#                 print(
#                     f"[DISCOVERY] Robot {robot.get_id()} found a survivor at ({int(pose.x)}, {int(pose.y)})!"
#                 )
#                 robot.add_survivor_memory(pose.x, pose.y, True, self.sim_time)
#                 robot.mark_survivor_visited(pose.x, pose.y)
#                 self.survivors_found += 1
#
#             # Communication: Process incoming survivor reports
#             for msg in robot.get_inbox():
#                 if msg.survivor_found:
#                     robot.add_survivor_memory(
#                         msg.survivor_x, msg.survivor_y, msg.visited, self.sim_time
#                     )
#
#             # Behavior: Auto-target next unvisited survivor if searching
#             if (
#                 robot.get_behavior() == swarm_core.RobotBehavior.SEARCHING
#                 and not robot.has_target()
#             ):
#                 for surv in robot.get_known_survivors():
#                     if not surv.visited:
#                         robot.set_navigation_target(surv.x, surv.y)
#                         break
#
#         # 3. Periodic metrics logging
#         if self.sim_time - self.last_metrics_log >= self.config.METRICS_LOG_INTERVAL:
#             self.metrics_logger.log_step(
#                 self.sim_time,
#                 self.robots,
#                 self.world_map,
#                 len(self.survivors_locations),
#             )
#             self.last_metrics_log = self.sim_time
#
#
# class Renderer:
#     """Separate rendering logic from simulation"""
#
#     def __init__(self, config: SimulationConfig):
#         self.config = config
#         self.screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
#         pygame.display.set_caption(
#             "Swarm Search & Rescue - Mesh Routing & Autonomous Charging"
#         )
#         self.font_small = pygame.font.SysFont(None, 14)
#
#     def render(self, state: SimulationState) -> None:
#         """Render everything each frame"""
#         self.screen.fill(Colors.BG.value)
#         self.render_grid(state)
#         self.render_base_station()
#         self.render_comms_links(state.robots)
#         self.render_robots(state.robots)
#         self.render_hud(state)
#
#     def render_grid(self, state: SimulationState) -> None:
#         """Render grid cells"""
#         for gy in range(state.world_map.get_height()):
#             for gx in range(state.world_map.get_width()):
#                 cell_state = state.world_map.get_cell(gx, gy)
#                 rect = pygame.Rect(
#                     gx * self.config.RESOLUTION,
#                     gy * self.config.RESOLUTION,
#                     self.config.RESOLUTION,
#                     self.config.RESOLUTION,
#                 )
#                 if cell_state == swarm_core.CellState.UNKNOWN:
#                     pygame.draw.rect(self.screen, Colors.UNKNOWN.value, rect)
#                 elif cell_state == swarm_core.CellState.FREE:
#                     pygame.draw.rect(self.screen, Colors.FREE.value, rect)
#                 elif cell_state == swarm_core.CellState.OBSTACLE:
#                     self._draw_obstacle(rect)
#                 elif (
#                     cell_state == swarm_core.CellState.SURVIVOR
#                 ):  # Fixed: cell_state check
#                     self._draw_survivor(gx, gy)
#
#     def _draw_obstacle(self, rect: pygame.Rect) -> None:
#         pygame.draw.rect(self.screen, (110, 55, 45), rect)
#         pygame.draw.line(
#             self.screen, (45, 25, 20), (rect.left, rect.top), (rect.right, rect.top), 1
#         )
#         pygame.draw.line(
#             self.screen,
#             (45, 25, 20),
#             (rect.left, rect.top),
#             (rect.left, rect.bottom),
#             1,
#         )
#
#     def _draw_survivor(self, gx: int, gy: int) -> None:
#         center_x = int(gx * self.config.RESOLUTION + self.config.RESOLUTION / 2)
#         center_y = int(gy * self.config.RESOLUTION + self.config.RESOLUTION / 2)
#         pygame.draw.circle(
#             self.screen,
#             Colors.SURVIVOR.value,
#             (center_x, center_y),
#             int(self.config.RESOLUTION / 3),
#         )
#
#     def render_base_station(self) -> None:
#         pygame.draw.circle(
#             self.screen,
#             Colors.BASE.value,
#             (int(self.config.BASE_X), int(self.config.BASE_Y)),
#             24,
#             3,
#         )
#         pygame.draw.circle(
#             self.screen,
#             (20, 80, 150),
#             (int(self.config.BASE_X), int(self.config.BASE_Y)),
#             18,
#         )
#
#     def render_comms_links(self, robots: list[swarm_core.RobotAgent]) -> None:
#         for i in range(len(robots)):
#             for j in range(i + 1, len(robots)):
#                 p1 = robots[i].get_pose()
#                 p2 = robots[j].get_pose()
#                 dist = math.hypot(p2.x - p1.x, p2.y - p1.y)
#                 if dist <= self.config.COMMS_RANGE:
#                     pygame.draw.line(
#                         self.screen,
#                         Colors.COMMS_LINE.value,
#                         (int(p1.x), int(p1.y)),
#                         (int(p2.x), int(p2.y)),
#                         1,
#                     )
#
#     def render_robots(self, robots: list[swarm_core.RobotAgent]) -> None:
#         for robot in robots:
#             pose = robot.get_pose()
#             rx, ry, rtheta = int(pose.x), int(pose.y), pose.theta
#
#             # Draw known survivors in memory
#             for surv in robot.get_known_survivors():
#                 color = (
#                     Colors.VISITED_SURVIVOR.value
#                     if surv.visited
#                     else Colors.KNOWN_SURVIVOR.value
#                 )
#                 pygame.draw.circle(self.screen, color, (int(surv.x), int(surv.y)), 8, 1)
#
#             # Select color based on behavior state
#             behavior = robot.get_behavior()
#             if behavior == swarm_core.RobotBehavior.RECHARGING:
#                 robot_color = Colors.RECHARGING.value
#             elif behavior == swarm_core.RobotBehavior.RETURNING_TO_BASE:
#                 robot_color = Colors.RETURNING.value
#             else:
#                 robot_color = Colors.SEARCHING.value
#
#             # Draw robot body
#             pygame.draw.circle(
#                 self.screen, robot_color, (rx, ry), int(self.config.RESOLUTION / 2.5)
#             )
#
#             # Draw heading vector
#             end_x = rx + int(15 * math.cos(rtheta))
#             end_y = ry + int(15 * math.sin(rtheta))
#             pygame.draw.line(
#                 self.screen, Colors.HEADING.value, (rx, ry), (end_x, end_y), 3
#             )
#
#             # Draw robot status text (Called once)
#             self._render_status_text(rx, ry, robot)
#
#     def _render_status_text(
#         self, rx: int, ry: int, robot: swarm_core.RobotAgent
#     ) -> None:
#         text = self.font_small.render(
#             f"R{robot.get_id()} ({int(robot.get_battery())}%)", True, Colors.TEXT.value
#         )
#         self.screen.blit(text, (rx - 25, ry - 24))
#
#     def render_hud(self, state: SimulationState):
#         stats_text = [
#             f"Time: {state.sim_time:.1f}s",
#             f"Robots: {len(state.robots)}",
#             f"Survivors Found: {state.survivors_found}/{len(state.survivors_locations)}",
#             "FPS: 60",
#         ]
#         y_offset = 10
#         for stat in stats_text:
#             text = self.font_small.render(stat, True, Colors.TEXT.value)
#             self.screen.blit(text, (10, y_offset))
#             y_offset += 20
#
#
# def main():
#     pygame.init()
#     config = SimulationConfig()
#     state = SimulationState(config)
#     renderer = Renderer(config)
#
#     state.init_world()
#
#     clock = pygame.time.Clock()
#     running = True
#     dt = 1.0 / config.FPS
#
#     print(f"[SIMULATION] Starting with {len(state.robots)} robots")
#     print(f"[SIMULATION] Grid size: {config.GRID_COLS} x {config.GRID_ROWS}")
#
#     while running:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT or (
#                 event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
#             ):
#                 running = False
#
#         state.step(dt)
#         renderer.render(state)
#         pygame.display.flip()
#         clock.tick(config.FPS)
#
#     print("[SIMULATION] Shutting down...")
#     pygame.quit()
#     sys.exit()
#
#
# if __name__ == "__main__":
#     main()
