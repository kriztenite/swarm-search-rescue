import math
import sys

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


def main():

    # Initialize Pygame
    pygame.init()

    # Window Dimensions

    WIDTH, HEIGHT = 800, 600
    RESOLUTION = 20.0  # Each grid cell is 20x20 pixel (and world units)
    GRID_COLS = int(WIDTH / RESOLUTION)
    GRID_ROWS = int(HEIGHT / RESOLUTION)

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(
        "Swarm Search & Rescue -Cooperative Goal-Directed Exploration"
    )

    # Colors (RGB)
    BG_COLOR = (20, 20, 30)
    UNKNOWN_COLOR = (30, 30, 45)
    FREE_COLOR = (45, 45, 65)
    OBSTACLE_COLOR = (60, 60, 80)
    SURVIVOR_COLOR = (255, 215, 0)
    KNOWN_SURVIVOR_COLOR = (165, 42, 42)
    VISITED_SURVIVOR_COLOR = (100, 100, 100)
    ROBOT_COLOR = (0, 255, 150)
    HEADING_COLOR = (255, 50, 50)
    # TARGET_LINE_COLOR = (255, 255, 150)
    COMMS_LINE_COLOR = (0, 150, 255)
    TEXT_COLOR = (220, 220, 220)

    # Simulation parameters
    FPS = 60
    clock = pygame.time.Clock()
    sim_time = 0.0

    # 1. Instantiate the GridMap, MetricsLogger and CommsManager(Range = 180 pixels)
    world_map = swarm_core.GridMap(GRID_COLS, GRID_ROWS, RESOLUTION)
    comms = swarm_core.CommsManager(comms_range=180.0)
    metrics_logger = swarm_core.MetricsLogger("simulation_metrics.csv")

    # Build borders and obstacles
    for x in range(GRID_COLS):
        world_map.set_cell(x, 0, swarm_core.CellState.OBSTACLE)
        world_map.set_cell(x, GRID_ROWS - 1, swarm_core.CellState.OBSTACLE)
    for y in range(GRID_ROWS):
        world_map.set_cell(0, y, swarm_core.CellState.OBSTACLE)
        world_map.set_cell(GRID_COLS - 1, y, swarm_core.CellState.OBSTACLE)

    # Add some internal obstacles
    for y in range(5, 15):
        world_map.set_cell(15, y, swarm_core.CellState.OBSTACLE)
    for x in range(20, 30):
        world_map.set_cell(x, 20, swarm_core.CellState.OBSTACLE)

    # Scatter survivors across the map
    survivors_locations = [(6, 6), (12, 22), (26, 6), (33, 24), (18, 10)]
    for sx, sy in survivors_locations:
        world_map.set_cell(sx, sy, swarm_core.CellState.SURVIVOR)

    # Instantiate Robots close enough to communicate or wander into range
    robots = [
        swarm_core.RobotAgent(id=1, x=100.0, y=180.0),
        swarm_core.RobotAgent(id=2, x=250.0, y=450.0),
        swarm_core.RobotAgent(id=3, x=400.0, y=300.0),
        swarm_core.RobotAgent(id=4, x=700.0, y=120.0),
        swarm_core.RobotAgent(id=5, x=550.0, y=500.0),
    ]

    # Give them initial target velocities (linear velocity, angular velocity)
    for i, robot in enumerate(robots):
        # Drive forward with a slight turning rate to make them move in arcs
        robot.set_target_velocity(v=80.0, omega=0.5 + (i * 0.5))

    running = True
    dt = 1.0 / FPS  # Fixed delta time per frame

    while running:
        # 2. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        sim_time += dt
        # 3. Communication sync
        comms.communicate(robots)

        # 4. Physics & Cooperative Target Assignment
        for robot in robots:
            robot.step(dt, sim_time, world_map)

            # Check if there is a survivor locally
            pose = robot.get_pose()

            # Reveal unknown map cells within sensor range
            world_map.reveal_radius(pose.x, pose.y, sensor_range=80)

            if world_map.check_survivor(pose.x, pose.y):
                print(
                    f"[DISCOVERY] Robot {robot.get_id()} found a survivor at ({int(pose.x)}, {int(pose.y)})!"
                )
                robot.add_survivor_memory(pose.x, pose.y, True, sim_time)
                robot.mark_survivor_visited(pose.x, pose.y)

            # Process inbox message received from teammates
            inbox = robot.get_inbox()
            for msg in inbox:
                # Store shared survivor location into local memory
                if msg.survivor_found:
                    robot.add_survivor_memory(
                        msg.survivor_x, msg.survivor_y, msg.visited, sim_time
                    )

            # Pick next unvisited target
            if not robot.has_target():
                for surv in robot.get_known_survivors():
                    if not surv.visited:
                        robot.set_navigation_target(surv.x, surv.y)
                        break

            # Step simulation
            robot.step(dt, sim_time, world_map)

            if robot.get_battery() <= 0:
                robot.set_target_velocity(0.0, 0.0)

        # 5. Log metrics every second
        if int(sim_time * FPS) % FPS == 0:
            metrics_logger.log_step(
                sim_time, robots, world_map, len(survivors_locations)
            )

        # 6. Rendering
        screen.fill(BG_COLOR)

        # Render Grid  Map (Unknown vs Free vs Obstacle vs Survivor)
        for gy in range(GRID_ROWS):
            for gx in range(GRID_COLS):
                state = world_map.get_cell(gx, gy)
                rect = pygame.Rect(
                    gx * RESOLUTION, gy * RESOLUTION, RESOLUTION, RESOLUTION
                )
                if state == swarm_core.CellState.UNKNOWN:
                    pygame.draw.rect(screen, UNKNOWN_COLOR, rect)

                elif state == swarm_core.CellState.FREE:
                    pygame.draw.rect(screen, FREE_COLOR, rect)
                elif state == swarm_core.CellState.OBSTACLE:
                    pygame.draw.rect(screen, OBSTACLE_COLOR, rect)
                    # Draw brick textured obstacle
                    pygame.draw.rect(screen, (110, 55, 45), rect)
                    # Mortar grid lines
                    pygame.draw.line(
                        screen,
                        (45, 25, 20),
                        (rect.left, rect.top),
                        (rect.right, rect.top),
                        1,
                    )
                    pygame.draw.line(
                        screen,
                        (45, 25, 20),
                        (rect.left, rect.top),
                        (rect.left, rect.bottom),
                        1,
                    )
                    # Horizontal brick seam
                    pygame.draw.line(
                        screen,
                        (45, 25, 20),
                        (rect.left, rect.centery),
                        (rect.right, rect.centery),
                        1,
                    )
                elif state == swarm_core.CellState.SURVIVOR:
                    # Draw a glowing circle for survivors
                    center_x = int(gx * RESOLUTION + RESOLUTION / 2)
                    center_y = int(gy * RESOLUTION + RESOLUTION / 2)
                    pygame.draw.circle(
                        screen,
                        SURVIVOR_COLOR,
                        (center_x, center_y),
                        int(RESOLUTION / 3),
                    )
        # Render comms links
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
                        COMMS_LINE_COLOR,
                        (int(p1.x), int(p1.y)),
                        (int(p2.x), int(p2.y)),
                        1,
                    )

        # Render Robots, Headings and Target lines
        for robot in robots:
            pose = robot.get_pose()
            rx, ry, rtheta = int(pose.x), int(pose.y), pose.theta

            # Draw know survivors in memory
            for surv in robot.get_known_survivors():
                color = VISITED_SURVIVOR_COLOR if surv.visited else KNOWN_SURVIVOR_COLOR
                pygame.draw.circle(
                    screen,
                    color,
                    (int(surv.x), int(surv.y)),
                    8,
                    1,
                )

            # Draw robot body
            pygame.draw.circle(screen, ROBOT_COLOR, (rx, ry), int(RESOLUTION / 2.5))

            # Draw heading vector
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
