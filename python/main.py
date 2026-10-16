import math
import sys
from pathlib import Path

import pygame

# Find 'build' directory for cpp component
ROOT_DIR = Path(__file__).resolve().parent.parent
build_path = ROOT_DIR / "build"
sys.path.insert(0, str(build_path))
print(f"Added to sys.path: {build_path}")
print(f"Contents of build dir: {list(build_path.glob('*'))}")

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
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Swarm Search & Rescue -C++ Core Physics Preview")

    # Colors (RGB)
    BG_COLOR = (20, 20, 30)
    ROBOT_COLOR = (0, 255, 150)
    HEADING_COLOR = (255, 50, 50)
    TEXT_COLOR = (220, 220, 220)

    # Simulation parameters
    FPS = 60
    clock = pygame.time.Clock()

    # Create a small swarm of 3 robots using our c++ RobotAgent class
    robots = [
        swarm_core.RobotAgent(id=1, x=200.0, y=300.0, max_speed=150.0),
        swarm_core.RobotAgent(id=2, x=400.0, y=300.0, max_speed=180.0),
        swarm_core.RobotAgent(id=3, x=600.0, y=300.0, max_speed=180.0),
    ]

    # Give them initial target velocities (linear velocity, angular velocity)
    for i, robot in enumerate(robots):
        # Drive forward with a slight turning rate to make them move in arcs
        robot.set_target_velocity(v=100.0, omega=1.0 + (i * 0.5))

    running = True
    dt = 1.0 / FPS  # Fixed delta time per frame

    while running:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 2. Physics Update(Calls the c++ step method for vevry robot)
        for robot in robots:
            robot.step(dt)

            # Simple screen boundary bouncing check (basic world bound)
            pose = robot.get_pose()
            if pose.x < 50 or pose.x > WIDTH - 50:
                robot.set_target_velocity(
                    -100.0, 2.0
                )  # Reverse and turn if hitting wall
            if pose.y < 50 or pose.y > HEIGHT - 50:
                robot.set_target_velocity(-100.0, 2.0)

        # 3. Rendering
        screen.fill(BG_COLOR)

        for robot in robots:
            pose = robot.get_pose()
            rx, ry, rtheta = int(pose.x), int(pose.y), pose.theta

            # Draw robot body as a circle
            pygame.draw.circle(screen, ROBOT_COLOR, (rx, ry), 12)

            # Draw heading line (vector showing where the robot is facing)
            end_x = rx + int(20 * math.cos(rtheta))
            end_y = ry + int(20 * math.sin(rtheta))
            pygame.draw.line(screen, HEADING_COLOR, (rx, ry), (end_x, end_y), 3)

            # Display battery level above robot
            font = pygame.font.SysFont(None, 20)
            battery_text = font.render(
                f"ID:{robot.get_id()} ({int(robot.get_battery())}%)", True, TEXT_COLOR
            )
            screen.blit(battery_text, (rx - 25, ry - 25))

        # Refresh screen
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
