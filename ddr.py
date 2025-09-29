import pygame
import math
import time

# --- Constants ---
# Screen dimensions
SCREEN_WIDTH = 720
SCREEN_HEIGHT = 480

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
        self.Kp_linear = 0.25   # Proportional gain for linear velocity
        self.Kp_angular = 0.5  # Proportional gain for angular velocity

        # Part 3: PD Controller for distance
        self.Kd_linear = 5.0    # Derivative gain for linear velocity

        # Part 4: PID Controller for heading
        self.Ki_angular = 0.01  # Integral gain for angular velocity
        self.Kd_angular = 0.5   # Derivative gain for angular velocity
        
        # --- Variables for PD/PID control ---
        self.prev_rho_error = 0.0
        self.prev_alpha_error = 0.0
        self.integral_alpha = 0.0
    
    def update(self, target_x, target_y, dt, controller_type='PID'):
        """
        Calculates errors, applies the selected control law, and updates the robot's pose.
        """
        # 1. Calculate Error Terms
        dx = target_x - self.x
        dy = target_y - self.y
        
        rho = math.sqrt(dx**2 + dy**2)
        theta_target = math.atan2(dy, dx)
        alpha = theta_target - self.theta
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # ---!!!--- DEBUGGING LINE 1 ---!!!---
        # This will print the error and gain before they are used.
        print(f"alpha: {alpha:.2f}, Kp_angular: {self.Kp_angular}, ", end="")

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

        # ---!!!--- DEBUGGING LINE 2 ---!!!---
        # This will print the final calculated angular velocity.
        print(f"omega: {omega:.2f}")

        # Store errors for the next iteration's derivative calculation
        self.prev_rho_error = rho
        self.prev_alpha_error = alpha

        # 3. Update Robot's Pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        """
        Calculates errors, applies the selected control law, and updates the robot's pose.
        """
        # 1. Calculate Error Terms
        dx = target_x - self.x
        dy = target_y - self.y
        
        rho = math.sqrt(dx**2 + dy**2)
        theta_target = math.atan2(dy, dx)
        alpha = theta_target - self.theta
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # Initialize velocities
        v = 0.0 # Linear velocity
        omega = 0.0 # Angular velocity

        # 2. Check stopping condition BEFORE calculating velocities
        if rho > 5.0: # Only calculate new velocities if we are not at the target
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
            # Target reached, reset PID accumulators
            self.integral_alpha = 0.0

        # Store errors for the next iteration's derivative calculation
        self.prev_rho_error = rho
        self.prev_alpha_error = alpha

        # 3. Update Robot's Pose (Kinematic Model) - This now runs every time
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

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


# --- Main Simulation Loop ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Differential Drive Robot Simulation")
    clock = pygame.time.Clock()

    # Create the robot
    # Starting at (100, 500) with a heading of -PI/2 (facing up)
    robot = DifferentialDriveRobot(100, 400, -math.pi / 2)
    
    # Define the target position 
    target_x, target_y = 550, 200
    
    # Store the robot's path for trajectory visualization 
    trajectory = []

    # Choose your controller: 'P', 'PD', or 'PID'
    active_controller = 'P' # <-- CHANGE THIS TO TEST DIFFERENT CONTROLLERS

    running = True
    last_time = time.time()
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Calculate delta time (dt) for frame-independent physics
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Update the robot's state
        robot.update(target_x, target_y, dt, active_controller)
        
        # Store current position for the trajectory
        trajectory.append((robot.x, robot.y))

        # --- Drawing ---
        screen.fill(BLACK) # Clear screen

        # Draw trajectory 
        if len(trajectory) > 1:
            pygame.draw.lines(screen, GREEN, False, trajectory, 1)

        # Draw target 
        pygame.draw.circle(screen, RED, (target_x, target_y), 10)
        
        # Draw robot
        robot.draw(screen)

        pygame.display.flip() # Update the display
        clock.tick(60) # Limit frame rate to 60 FPS

    pygame.quit()

if __name__ == '__main__':
    main()