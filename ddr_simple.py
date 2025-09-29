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

        # Robot's pose (position and orientation)
        self.x = x
        self.y = y
        self.theta = theta  # Angle in radians

        # --- Controller Gains (These are the values you will tune!) ---
        # Part 1: P Controller

        self.Kp_linear = 99   # Proportional gain for linear velocity
        self.Kp_angular = 0.75 # Proportional gain for angular velocity

        # Part 3: PD Controller for distance
        self.Kd_linear = 1    # Derivative gain for linear velocity

        # Part 4: PID Controller for heading
        self.Ki_angular = 0.01  # Integral gain for angular velocity
        self.Kd_angular = 0.5   # Derivative gain for angular velocity
        
        # --- Variables for PD/PID control ---
        self.prev_rho_error = 0.0
        self.prev_alpha_error = 0.0
        self.integral_alpha = 0.0
    
    def update(self, target_x, target_y, dt, controller_type='P'):
        """
        Calculates errors, applies the selected control law, and updates the robot's pose.
        Returns the distance to the target (rho).
        """
        # ---!!!--- ROBUST dt and VELOCITY CLAMPING ---!!!---
        # Prevent division by a tiny dt on the first frame and limit max velocity
        dt = max(min(dt, 0.1), 0.001) # Clamp dt to a reasonable range
        MAX_V = 100.0  # Max linear velocity (pixels per second)
        MAX_OMEGA = 3.0 # Max angular velocity (radians per second)

        # 1. Calculate Error Terms
        dx = target_x - self.x
        dy = target_y - self.y
        
        rho = math.sqrt(dx**2 + dy**2)
        theta_target = math.atan2(dy, dx)
        alpha = (theta_target - self.theta + math.pi) % (2 * math.pi) - math.pi

        # Initialize velocities
        v = 0.0 # Linear velocity
        omega = 0.0 # Angular velocity

        # 2. Check stopping condition BEFORE calculating velocities
        if rho > 5.0: 
            # --- Apply Control Laws ---
            if controller_type == 'P':
                v = self.Kp_linear * rho
                omega = self.Kp_angular * alpha

            elif controller_type == 'PD':
                rho_derivative = (rho - self.prev_rho_error) / dt
                v = self.Kp_linear * rho + self.Kd_linear * rho_derivative
                omega = self.Kp_angular * alpha

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

        # Clamp the final velocities to their maximums
        v = max(min(v, MAX_V), -MAX_V)
        omega = max(min(omega, MAX_OMEGA), -MAX_OMEGA)

        # Store errors for the next iteration's derivative calculation
        self.prev_rho_error = rho
        self.prev_alpha_error = alpha

        # 3. Update Robot's Pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        return rho


    def draw(self, screen):
        """
        Draws the robot on the Pygame screen. 
        """
        # Draw robot body (circle)
        pygame.draw.circle(screen, BLUE, (int(self.x), int(self.y)), self.r)
        
        # Draw heading direction line
        end_x = self.x + self.r * math.cos(self.theta)
        end_y = self.y + self.r * math.sin(self.theta)
        pygame.draw.line(screen, WHITE, (int(self.x), int(self.y)), (int(end_x), int(end_y)), 2)


def draw_cartesian_plane(screen, font, grid_spacing=50):
    """
    Draws a cartesian plane with grid lines and labels.
    """
    grid_color = (80, 80, 80)
    # Vertical lines
    for x in range(0, SCREEN_WIDTH, grid_spacing):
        pygame.draw.line(screen, grid_color, (x, 0), (x, SCREEN_HEIGHT))
        if x > 0:
            label = font.render(str(x), True, WHITE)
            screen.blit(label, (x + 5, 5))
    # Horizontal lines
    for y in range(0, SCREEN_HEIGHT, grid_spacing):
        pygame.draw.line(screen, grid_color, (0, y), (SCREEN_WIDTH, y))
        if y > 0:
            label = font.render(str(y), True, WHITE)
            screen.blit(label, (5, y + 5))

    # Axes
    pygame.draw.line(screen, WHITE, (0, 0), (SCREEN_WIDTH, 0), 2) # X-axis
    pygame.draw.line(screen, WHITE, (0, 0), (0, SCREEN_HEIGHT), 2) # Y-axis


# --- Main Simulation Loop ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Differential Drive Robot Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 20)
    timer_font = pygame.font.SysFont(None, 36)

    # Create the robot
    # Starting at (100, 400) with a heading of -PI/2 (facing up)
    robot = DifferentialDriveRobot(200, 450, -math.pi / 2)
    
    # Define the target position 
    target_x, target_y = 900, 450
    
    # Store the robot's path for trajectory visualization 
    trajectory = []

    # Choose your controller: 'P', 'PD', or 'PID'
    active_controller = 'P' # <-- CHANGE THIS TO TEST DIFFERENT CONTROLLERS

    running = True
    last_time = time.time()
    start_time = time.time()
    time_to_target = None
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Calculate delta time (dt) for frame-independent physics
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Update the robot's state and get distance to target
        rho = robot.update(target_x, target_y, dt, active_controller)
        
        # Store current position for the trajectory
        trajectory.append((robot.x, robot.y))

        # Check if target is reached
        if rho <= 5.0 and time_to_target is None:
            time_to_target = current_time - start_time

        # --- Drawing ---
        screen.fill(BLACK) # Clear screen

        # Draw cartesian plane
        #draw_cartesian_plane(screen, font)

        # Draw trajectory 
        if len(trajectory) > 1:
            pygame.draw.lines(screen, GREEN, False, trajectory, 1)

        # Draw target 
        pygame.draw.circle(screen, RED, (target_x, target_y), 10)
        
        # Draw robot
        robot.draw(screen)

        # --- Timer Display ---
        # if time_to_target is not None:
        #     timer_text = f"Time: {time_to_target:.2f}s"
        # else:
        #     elapsed_time = current_time - start_time
        #     timer_text = f"Time: {elapsed_time:.2f}s"
        
        # timer_surface = timer_font.render(timer_text, True, WHITE)
        # screen.blit(timer_surface, (SCREEN_WIDTH - 150, 20))

        pygame.display.flip() # Update the display
        clock.tick(60) # Limit frame rate to 60 FPS

    pygame.quit()

if __name__ == '__main__':
    main()