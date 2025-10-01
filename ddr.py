import pygame
import math
import time

# --- Constants ---
# Screen dimensions
SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 600

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

class DifferentialDriveRobot:
    """
    Represents the differential drive robot and its controller.
    """
    def __init__(self, x, y, theta, r=15, L=40):
        # Robot's physical parameters
        self.r = r  # Wheel radius (for visualization)
        self.L = L  # Axle length (distance between wheels)

        # Initial pose, stored for reset
        self.initial_x = x
        self.initial_y = y
        self.initial_theta = theta

        # Robot's pose (position and orientation)
        self.x = x
        self.y = y
        self.theta = theta  # Angle in radians

        # --- Controller Gains (These are the values you will tune!) ---
        
        self.Kp_linear = 6  # Proportional gain for linear velocity
        self.Kp_angular = 10    # Proportional gain for angular velocity

        self.Ki_angular = 0.0  # Integral gain for angular velocity

        self.Kd_linear = 0     # Derivative gain for linear velocity
        self.Kd_angular = 0    # Derivative gain for angular velocity
        
        # --- Variables for control and display ---
        self.prev_rho_error = 0.0
        self.prev_alpha_error = 0.0
        self.integral_alpha = 0.0
        self.alpha = 0.0 # Current angle error for display

    def reset(self):
        """Resets the robot's pose and controller state to its initial configuration."""
        self.x = self.initial_x
        self.y = self.initial_y
        self.theta = self.initial_theta
        self.prev_rho_error = 0.0
        self.prev_alpha_error = 0.0
        self.integral_alpha = 0.0
        self.alpha = 0.0

    def update(self, target_x, target_y, dt, controller_type='P'):
        """
        Calculates errors, applies the selected control law, and updates the robot's pose.
        Returns the distance to the target (rho).
        """
        # ---!!!--- ROBUST dt ---!!!---
        dt = max(min(dt, 0.1), 0.001) 
        # MAX_V = 100.0       # <--- MODIFIED: Safety limit removed
        # MAX_OMEGA = 3.0     # <--- MODIFIED: Safety limit removed

        # 1. Calculate Error Terms
        dx = target_x - self.x
        dy = target_y - self.y
        
        rho = math.sqrt(dx**2 + dy**2)
        theta_target = math.atan2(dy, dx)
        alpha = (theta_target - self.theta + math.pi) % (2 * math.pi) - math.pi
        self.alpha = alpha # Store for display

        v = 0.0
        omega = 0.0

        # 2. Check stopping condition BEFORE calculating velocities
        if rho > 1.0: 
            # --- Apply Control Laws ---
            if controller_type == 'P':
                v = self.Kp_linear * rho
                omega = self.Kp_angular * alpha

            elif controller_type == 'PD':
                rho_derivative = (rho - self.prev_rho_error) / dt
                v = self.Kp_linear * rho + self.Kd_linear * rho_derivative
                omega = self.Kp_angular * alpha

            elif controller_type == 'PI':
                v = self.Kp_linear * rho
                self.integral_alpha += alpha * dt
                self.integral_alpha = max(min(self.integral_alpha, 2.0), -2.0)
                omega = (self.Kp_angular * alpha + self.Ki_angular * self.integral_alpha)

            elif controller_type == 'PID':
                rho_derivative = (rho - self.prev_rho_error) / dt
                v = self.Kp_linear * rho + self.Kd_linear * rho_derivative
                self.integral_alpha += alpha * dt
                self.integral_alpha = max(min(self.integral_alpha, 2.0), -2.0)
                alpha_derivative = (alpha - self.prev_alpha_error) / dt
                omega = (self.Kp_angular * alpha + 
                         self.Ki_angular * self.integral_alpha + 
                         self.Kd_angular * alpha_derivative)
        else:
            self.integral_alpha = 0.0

        # v = max(min(v, MAX_V), -MAX_V)                   # <--- MODIFIED: Safety limit removed
        # omega = max(min(omega, MAX_OMEGA), -MAX_OMEGA)   # <--- MODIFIED: Safety limit removed

        self.prev_rho_error = rho
        self.prev_alpha_error = alpha

        # 3. Update Robot's Pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        return rho

    def draw(self, screen):
        pygame.draw.circle(screen, BLUE, (int(self.x), int(self.y)), self.r)
        end_x = self.x + self.r * math.cos(self.theta)
        end_y = self.y + self.r * math.sin(self.theta)
        pygame.draw.line(screen, WHITE, (int(self.x), int(self.y)), (int(end_x), int(end_y)), 2)

def draw_cartesian_plane(screen, font, grid_spacing=50):
    grid_color = (80, 80, 80)
    for x in range(0, SCREEN_WIDTH, grid_spacing):
        pygame.draw.line(screen, grid_color, (x, 0), (x, SCREEN_HEIGHT))
        if x > 0:
            label = font.render(str(x), True, WHITE)
            screen.blit(label, (x + 5, 5))
    for y in range(0, SCREEN_HEIGHT, grid_spacing):
        pygame.draw.line(screen, grid_color, (0, y), (SCREEN_WIDTH, y))
        if y > 0:
            label = font.render(str(y), True, WHITE)
            screen.blit(label, (5, y + 5))
    pygame.draw.line(screen, WHITE, (0, 0), (SCREEN_WIDTH, 0), 2)
    pygame.draw.line(screen, WHITE, (0, 0), (0, SCREEN_HEIGHT), 2)

# --- Main Simulation Loop ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Interactive Differential Drive Robot Simulation")
    clock = pygame.time.Clock()
    info_font = pygame.font.SysFont(None, 24)
    timer_font = pygame.font.SysFont(None, 36)

    # --- Initial State ---
    initial_robot_pos = (200, 450, -math.pi / 2)
    initial_target_pos = (550, 350)

    robot = DifferentialDriveRobot(*initial_robot_pos)
    target_x, target_y = initial_target_pos
    
    trajectory = []

    # --- Controller Setup ---
    controllers = ['P', 'PI', 'PD', 'PID']
    controller_index = controllers.index('PID')
    active_controller = controllers[controller_index]

    running = True
    last_time = time.time()
    start_time = time.time()
    time_to_target = None
    
    while running:
        # --- Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    controller_index = (controller_index + 1) % len(controllers)
                    active_controller = controllers[controller_index]
                    robot.reset()
                    trajectory = []
                    start_time = time.time()
                    time_to_target = None
                elif event.key == pygame.K_r:
                    robot.reset()
                    target_x, target_y = initial_target_pos
                    trajectory = []
                    start_time = time.time()
                    time_to_target = None
            elif event.type == pygame.MOUSEBUTTONDOWN:
                target_x, target_y = event.pos
                robot.integral_alpha = 0.0
                trajectory = []
                start_time = time.time()
                time_to_target = None

        # --- Physics Update ---
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        rho = robot.update(target_x, target_y, dt, active_controller)
        
        if rho > 1.0:
            trajectory.append((robot.x, robot.y))

        if rho <= 5.0 and time_to_target is None:
            time_to_target = current_time - start_time

        # --- Drawing ---
        screen.fill(BLACK)
        draw_cartesian_plane(screen, info_font)

        if len(trajectory) > 1:
            pygame.draw.lines(screen, GREEN, False, trajectory, 1)

        pygame.draw.circle(screen, RED, (target_x, target_y), 10)
        robot.draw(screen)

        # --- UI Info Display ---
        if time_to_target is not None:
            timer_text = f"Time to Target: {time_to_target:.2f}s"
        else:
            elapsed_time = current_time - start_time
            timer_text = f"Elapsed Time: {elapsed_time:.2f}s"
        
        timer_surface = timer_font.render(timer_text, True, WHITE)
        screen.blit(timer_surface, (50, 50))

        controller_text = f"Controller: {active_controller}"
        rho_text = f"Distance (rho): {rho:.2f} px"
        alpha_text = f"Angle (alpha): {math.degrees(robot.alpha):.2f} deg"
        
        info_surfaces = [
            info_font.render(controller_text, True, WHITE),
            info_font.render(rho_text, True, WHITE),
            info_font.render(alpha_text, True, WHITE)
        ]
        for i, surface in enumerate(info_surfaces):
            screen.blit(surface, (50, 100 + i * 25))

        instructions = [
            "[SPACE] Switch Controller",
            "[R] Reset Simulation",
            "[CLICK] Set New Target",
            "[ESC] Quit"
        ]
        for i, instruction in enumerate(instructions):
            inst_surface = info_font.render(instruction, True, WHITE)
            screen.blit(inst_surface, (SCREEN_WIDTH - inst_surface.get_width() - 20, 50 + i * 25))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == '__main__':
    main()