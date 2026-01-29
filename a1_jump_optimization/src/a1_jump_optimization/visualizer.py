"""
visualizer.py
Trajectory Visualization and Animation

Creates plots and animations of the optimal jumping trajectory.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from pathlib import Path


class TrajectoryVisualizer:
    """
    Visualize optimal jumping trajectory
    """
    
    def __init__(self, solution, robot):
        """
        Initialize visualizer
        
        Args:
            solution: optimization solution dict
            robot: robot model
        """
        self.solution = solution
        self.robot = robot
        self.states = solution['states']
        self.forces = solution['forces']
        self.time = solution['time']
        self.contact = solution['contact_schedule']
    
    def plot_trajectory(self, save_path=None):
        """
        Plot complete trajectory overview
        
        Args:
            save_path: path to save figure
        """
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        
        t = self.time
        
        # Position
        axes[0, 0].plot(t, self.states[0, :], 'b-', linewidth=2, label='Horizontal (x)')
        axes[0, 0].plot(t, self.states[1, :], 'r-', linewidth=2, label='Vertical (z)')
        axes[0, 0].set_ylabel('Position (m)', fontsize=11)
        axes[0, 0].set_xlabel('Time (s)', fontsize=11)
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].set_title('Center of Mass Position', fontweight='bold')
        
        # Velocity
        axes[0, 1].plot(t, self.states[3, :], 'b-', linewidth=2, label='Horizontal (vx)')
        axes[0, 1].plot(t, self.states[4, :], 'r-', linewidth=2, label='Vertical (vz)')
        axes[0, 1].set_ylabel('Velocity (m/s)', fontsize=11)
        axes[0, 1].set_xlabel('Time (s)', fontsize=11)
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].set_title('Center of Mass Velocity', fontweight='bold')
        
        # Body angle
        axes[1, 0].plot(t, np.degrees(self.states[2, :]), 'g-', linewidth=2)
        axes[1, 0].set_ylabel('Pitch Angle (deg)', fontsize=11)
        axes[1, 0].set_xlabel('Time (s)', fontsize=11)
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].set_title('Body Pitch', fontweight='bold')
        
        # Angular velocity
        axes[1, 1].plot(t, np.degrees(self.states[5, :]), 'g-', linewidth=2)
        axes[1, 1].set_ylabel('Angular Velocity (deg/s)', fontsize=11)
        axes[1, 1].set_xlabel('Time (s)', fontsize=11)
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].set_title('Angular Velocity', fontweight='bold')
        
        # Ground reaction forces
        t_forces = self.time[:-1]  # forces have N elements (one less than states)
        total_Fx = self.forces[0, :] + self.forces[2, :] + self.forces[4, :] + self.forces[6, :]
        total_Fz = self.forces[1, :] + self.forces[3, :] + self.forces[5, :] + self.forces[7, :]
        
        axes[2, 0].plot(t_forces, total_Fx, 'b-', linewidth=2, label='Horizontal')
        axes[2, 0].plot(t_forces, total_Fz, 'r-', linewidth=2, label='Vertical')
        axes[2, 0].set_ylabel('Force (N)', fontsize=11)
        axes[2, 0].set_xlabel('Time (s)', fontsize=11)
        axes[2, 0].legend()
        axes[2, 0].grid(True, alpha=0.3)
        axes[2, 0].set_title('Total Ground Reaction Forces', fontweight='bold')
        
        # Contact schedule
        for i in range(4):
            axes[2, 1].plot(t_forces, self.contact[:, i] * (i + 1), 
                           linewidth=3, label=f'Leg {i+1}')
        axes[2, 1].set_ylabel('Leg Contact', fontsize=11)
        axes[2, 1].set_xlabel('Time (s)', fontsize=11)
        axes[2, 1].set_ylim(0, 5)
        axes[2, 1].legend()
        axes[2, 1].grid(True, alpha=0.3)
        axes[2, 1].set_title('Contact Schedule', fontweight='bold')
        
        plt.suptitle(f'Optimal Jump Trajectory - Distance: {self.solution["jump_distance"]:.3f} m',
                    fontsize=14, fontweight='bold', y=0.995)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Trajectory plot saved to {save_path}")
        
        return fig
    
    def plot_torque_profile(self, save_path=None):
        """
        Plot joint torque profiles
        
        Args:
            save_path: path to save figure
        """
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes = axes.flatten()
        
        t_forces = self.time[:-1]
        leg_ext = self.solution['leg_extensions']
        
        leg_names = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']
        
        for i in range(4):
            ax = axes[i]
            
            # Calculate torque for this leg
            torques = []
            for k in range(len(t_forces)):
                if self.contact[k, i] > 0.5:
                    Fx = self.forces[2*i, k]
                    Fz = self.forces[2*i + 1, k]
                    ext = leg_ext[i, k]
                    
                    lever_arm = ext * self.robot.leg_length
                    force_mag = np.sqrt(Fx**2 + Fz**2)
                    torque = (force_mag * lever_arm) / 3.0
                    torques.append(torque)
                else:
                    torques.append(0.0)
            
            torques = np.array(torques)
            
            # Plot
            ax.plot(t_forces, torques, 'b-', linewidth=2, label='Actual')
            ax.axhline(self.robot.max_torque, color='r', linestyle='--', 
                      linewidth=1.5, label=f'Limit ({self.robot.max_torque} Nm)')
            ax.fill_between(t_forces, 0, torques, alpha=0.3)
            
            ax.set_xlabel('Time (s)', fontsize=10)
            ax.set_ylabel('Joint Torque (Nm)', fontsize=10)
            ax.set_title(f'{leg_names[i]}', fontweight='bold')
            ax.legend(fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.set_ylim(0, self.robot.max_torque * 1.1)
        
        plt.suptitle('Joint Torque Profiles (Approximate)', 
                    fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Torque profile saved to {save_path}")
        
        return fig
    
    def plot_2d_trajectory(self, save_path=None):
        """
        Plot 2D trajectory (x-z plane)
        
        Args:
            save_path: path to save figure
        """
        fig, ax = plt.subplots(figsize=(12, 6))
        
        x = self.states[0, :]
        z = self.states[1, :]
        
        # Color by time
        n_points = len(x)
        colors = plt.cm.viridis(np.linspace(0, 1, n_points))
        
        # Plot trajectory
        for i in range(n_points - 1):
            ax.plot(x[i:i+2], z[i:i+2], color=colors[i], linewidth=2)
        
        # Mark start and end
        ax.plot(x[0], z[0], 'go', markersize=15, label='Start', zorder=5)
        ax.plot(x[-1], z[-1], 'ro', markersize=15, label='End', zorder=5)
        
        # Ground line
        ax.axhline(0, color='brown', linewidth=3, linestyle='-', label='Ground')
        ax.fill_between([x.min() - 0.5, x.max() + 0.5], -0.1, 0, 
                        color='brown', alpha=0.3)
        
        # Annotations
        max_height_idx = np.argmax(z)
        ax.plot(x[max_height_idx], z[max_height_idx], 'b*', 
               markersize=20, label=f'Peak ({z[max_height_idx]:.2f} m)')
        
        ax.set_xlabel('Horizontal Distance (m)', fontsize=12)
        ax.set_ylabel('Height (m)', fontsize=12)
        ax.set_title(f'Jump Trajectory (Lunar Gravity)\nTotal Distance: {x[-1] - x[0]:.3f} m', 
                    fontsize=14, fontweight='bold')
        ax.legend(fontsize=11, loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.set_ylim(-0.2, z.max() + 0.5)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"2D trajectory saved to {save_path}")
        
        return fig
    
    def create_animation(self, save_path=None):
        """
        Create animated visualization of the jump
        
        Args:
            save_path: path to save animation (as GIF)
        """
        fig, ax = plt.subplots(figsize=(12, 6))
        
        x = self.states[0, :]
        z = self.states[1, :]
        theta = self.states[2, :]
        
        # Setup plot
        ax.set_xlim(x.min() - 0.5, x.max() + 0.5)
        ax.set_ylim(-0.2, z.max() + 0.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color='brown', linewidth=3)
        
        # Initialize elements
        trajectory_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5, label='Trajectory')
        body_line, = ax.plot([], [], 'r-', linewidth=4, label='Robot Body')
        com_point, = ax.plot([], [], 'ko', markersize=10, label='Center of Mass')
        
        time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12,
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        ax.legend(loc='upper right')
        ax.set_xlabel('Horizontal Distance (m)', fontsize=12)
        ax.set_ylabel('Height (m)', fontsize=12)
        ax.set_title('Unitree A1 Jump Animation (Lunar Gravity)', 
                    fontsize=14, fontweight='bold')
        
        def init():
            trajectory_line.set_data([], [])
            body_line.set_data([], [])
            com_point.set_data([], [])
            time_text.set_text('')
            return trajectory_line, body_line, com_point, time_text
        
        def animate(frame):
            # Update trajectory
            trajectory_line.set_data(x[:frame+1], z[:frame+1])
            
            # Update body (simple rectangle representation)
            body_length = self.robot.body_length
            body_x = x[frame]
            body_z = z[frame]
            body_angle = theta[frame]
            
            # Body corners
            half_len = body_length / 2
            corners_x = [-half_len, half_len, half_len, -half_len, -half_len]
            corners_z = [0, 0, 0.1, 0.1, 0]
            
            # Rotate and translate
            cos_a = np.cos(body_angle)
            sin_a = np.sin(body_angle)
            
            rotated_x = [body_x + x*cos_a - z*sin_a for x, z in zip(corners_x, corners_z)]
            rotated_z = [body_z + x*sin_a + z*cos_a for x, z in zip(corners_x, corners_z)]
            
            body_line.set_data(rotated_x, rotated_z)
            
            # Update CoM point
            com_point.set_data([body_x], [body_z])
            
            # Update time text
            time_text.set_text(f'Time: {self.time[frame]:.2f} s\nDistance: {x[frame]:.2f} m')
            
            return trajectory_line, body_line, com_point, time_text
        
        anim = FuncAnimation(fig, animate, init_func=init, 
                           frames=len(x), interval=50, blit=True)
        
        if save_path:
            writer = PillowWriter(fps=20)
            anim.save(save_path, writer=writer)
            print(f"Animation saved to {save_path}")
        
        return anim


def create_all_visualizations(solution, robot, output_dir='results/plots'):
    """
    Create all visualizations for a solution
    
    Args:
        solution: optimization solution
        robot: robot model
        output_dir: directory to save plots
    """
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    vis = TrajectoryVisualizer(solution, robot)
    
    print("\nGenerating visualizations...")
    
    # 1. Complete trajectory plot
    vis.plot_trajectory(output_path / 'complete_trajectory.png')
    
    # 2. Torque profiles
    vis.plot_torque_profile(output_path / 'torque_profiles.png')
    
    # 3. 2D trajectory
    vis.plot_2d_trajectory(output_path / '2d_trajectory.png')
    
    # 4. Animation (optional - can be slow)
    try:
        vis.create_animation(output_path / 'jump_animation.gif')
    except:
        print("  (Animation generation skipped - install pillow if needed)")
    
    print(f"\n✓ All visualizations saved to {output_path}")
    
    return vis


if __name__ == "__main__":
    # Test with a sample solution
    from .robot_model import UnitreeA1
    from .optimizer import JumpOptimizer
    
    robot = UnitreeA1()
    opt = JumpOptimizer(robot, N=30, T=1.5, jump_mode='sequential')
    opt.set_initial_guess('aggressive')
    
    solution = opt.solve(verbose=False)
    
    if solution['success']:
        create_all_visualizations(solution, robot)
        plt.show()
