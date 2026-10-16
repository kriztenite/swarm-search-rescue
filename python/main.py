import math
import sys

import pygame

# module_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "build"))
# sys.path.append(module_path)
#
#
# # Get path to swarm_core
# # build_path = os.path.dirname(__file__), "../build"
# # sys.path.insert(0, build_path)
# print(f"Build path : {module_path}")

# import our compiled c++ module
try:
    import swarm_core
except ImportError as e:
    print(
        "Error: 'swarm_core' module not found. Please compile the c++ project with CMake first. "
    )
    print(f"Debug info: {e}")
    sys.exit(1)


def main():

    # Initialize Pygame
    pygame.init()

    # Window Dimensions

    WIDTH, HEIGHT = 800, 600
    RESOLUTION = 20.0  # Each grid cell is 20x20 pixel (and world units)
    GRID_COLS = int(WIDTH / RESOLUTION)
    GRID_ROWS = int(HEIGHT / RESOLUTION)

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Swarm Search & Rescue -Communication & Memory Test")

    # Colors (RGB)
    BG_COLOR = (20, 20, 30)
    OBSTACLE_COLOR = (60, 60, 80)
    SURVIVOR_COLOR = (255, 215, 0)  # Gold
    KNOWN_SURVIVOR_COLOR = (165, 42, 42)  # Brown(#A52A2A)
    ROBOT_COLOR = (0, 255, 150)
    HEADING_COLOR = (255, 50, 50)
    COMMS_LINE_COLOR = (0, 150, 255)  # cyan
    TEXT_COLOR = (220, 220, 220)

    # Simulation parameters
    FPS = 60
    clock = pygame.time.Clock()

    # 1. Instantiate the GridMap & CommsManager (Range = 180 pixels)
    world_map = swarm_core.GridMap(GRID_COLS, GRID_ROWS, RESOLUTION)
    comms = swarm_core.CommsManager(comms_range=180.0)

    # Build borders and obstacles
    for x in range(GRID_COLS):
        world_map.set_cell(x, 0, swarm_core.CellState.OBSTACLE)
        world_map.set_cell(x, GRID_ROWS - 1, swarm_core.CellState.OBSTACLE)
    for y in range(GRID_ROWS):
        world_map.set_cell(0, y, swarm_core.CellState.OBSTACLE)
        world_map.set_cell(GRID_COLS - 1, y, swarm_core.CellState.OBSTACLE)

    # Add some integral obstacles (e.g. a central barrier or pillars)
    for y in range(5, 15):
        world_map.set_cell(15, y, swarm_core.CellState.OBSTACLE)

    # Scatter survivors across the map
    survivors_locations = [(5, 5), (10, 23), (26, 34), (30, 22), (18, 18)]
    for sx, sy in survivors_locations:
        world_map.set_cell(sx, sy, swarm_core.CellState.SURVIVOR)

    # Instantiate Robots close enough to communicate or wander into range
    robots = [
        swarm_core.RobotAgent(id=1, x=100.0, y=300.0, max_speed=130.0),
        swarm_core.RobotAgent(id=2, x=220.0, y=200.0, max_speed=140.0),
        swarm_core.RobotAgent(id=3, x=600.0, y=400.0, max_speed=150.0),
    ]

    # Give them initial target velocities (linear velocity, angular velocity)
    for i, robot in enumerate(robots):
        # Drive forward with a slight turning rate to make them move in arcs
        robot.set_target_velocity(v=100.0, omega=0.5 + (i * 0.5))

    running = True
    dt = 1.0 / FPS  # Fixed delta time per frame

    while running:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 3. Update Physics & Communication
        # Run peer-to-peer radio sync
        comms.communicate(robots)

        for robot in robots:
            robot.step(dt, world_map)

            # Check survivor collection locally
            pose = robot.get_pose()
            if world_map.check_survivor(pose.x, pose.y):
                print(
                    f"[DISCOVERY] Robot {robot.get_id()} found a survivor at ({int(pose.x)}, {int(pose)})!"
                )
                robot.add_survivor_memory(pose.x, pose.y)

            # Process inbox message received from teammates
            inbox = robot.get_inbox()
            if not inbox:
                for msg in inbox:
                    # Store shared survivor location into local memory
                    robot.add_survivor_memory(msg.survivor_x, msg.survivor_y)

            if robot.get_battery() <= 0:
                robot.set_target_velocity(0.0, 0.0)

        # 3. Rendering
        screen.fill(BG_COLOR)

        # Render Grid  Map (Obstacle and Survivors)
        for gy in range(GRID_ROWS):
            for gx in range(GRID_COLS):
                state = world_map.get_cell(gx, gy)
                rect = pygame.Rect(
                    gx * RESOLUTION, gy * RESOLUTION, RESOLUTION, RESOLUTION
                )
                if state == swarm_core.CellState.OBSTACLE:
                    pygame.draw.rect(screen, OBSTACLE_COLOR, rect)
                    pygame.draw.rect(screen, (30, 30, 40), rect, 1)  # Grid cell border
                elif state == swarm_core.CellState.SURVIVOR:
                    # Draw a glowing circle for survivors
                    center_x = int(gx * RESOLUTION + RESOLUTION / 2)
                    center_y = int(gy * RESOLUTION + RESOLUTION / 2)
                    pygame.draw.circle(
                        screen, SURVIVOR_COLOR, (center_x, center_y), int(RESOLUTION)
                    )
        # Render Robots, Headings and shared Knowledge Markers
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                p1 = robots[i].get_pose()
                p2 = robots[j].get_pose()
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= comms.get_comms_range():
                    pygame.draw.line(
                        screen,
                        (0, 120, 200),
                        (int(p1.x), int(p1.y)),
                        (int(p2.x), int(p2.y)),
                        2,
                    )
                    # Render Robots
        for robot in robots:
            pose = robot.get_pose()
            rx, ry, rtheta = int(pose.x), int(pose.y), pose.theta

            # Draw robot body
            pygame.draw.circle(screen, ROBOT_COLOR, (rx, ry), int(RESOLUTION / 2.5))

            # Draw heading line
            end_x = rx + int(15 * math.cos(rtheta))
            end_y = ry + int(15 * math.sin(rtheta))
            pygame.draw.line(screen, HEADING_COLOR, (rx, ry), (end_x, end_y), 3)

            # Display status text
            font = pygame.font.SysFont(None, 16)
            text = font.render(
                f"R{robot.get_id()} ({int(robot.get_battery())}%)", True, TEXT_COLOR
            )
            screen.blit(text, (rx - 20, ry - 20))

        # Refresh screen
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
