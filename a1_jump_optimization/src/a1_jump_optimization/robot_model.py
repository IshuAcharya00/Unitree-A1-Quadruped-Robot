"""
robot_model.py
Unitree A1 Robot Model with Physical Parameters
"""

import numpy as np
import yaml
from pathlib import Path


class UnitreeA1:
    """
    Unitree A1 Quadruped Robot Model
    
    Attributes:
        mass: Total robot mass (kg)
        inertia: Body moment of inertia (kg*m^2)
        gravity: Gravitational acceleration (m/s^2) - LUNAR
        max_torque: Maximum joint torque (Nm)
        leg_length: Effective leg length (m)
        friction_coef: Ground friction coefficient
    """
    
    def __init__(self, config_path=None):
        """Initialize robot with parameters from config file"""
        
        if config_path is None:
            config_path = Path(__file__).parent.parent.parent / "config" / "robot_params.yaml"
        
        with open(config_path, 'r') as f:
            params = yaml.safe_load(f)
        
        # Physical properties
        self.mass = params['mass']
        self.inertia = params['body_inertia']
        self.gravity = params['gravity']
        
        # Actuator limits
        self.max_torque = params['max_joint_torque']
        
        # Geometry
        self.leg_length = params['leg_length']
        self.body_length = params['body_length']
        self.body_width = params['body_width']
        self.body_height = params['body_height']
        
        # Contact properties
        self.friction_coef = params['friction_coefficient']
        self.restitution = params['restitution']
        
        # Joint limits
        self.joint_limits = params['joint_limits']
        
        # Initial state
        self.initial_height = params['initial_height']
        self.initial_velocity = params['initial_velocity']
        
        # Number of legs
        self.n_legs = 4
        
        # Leg positions in body frame (front-left, front-right, rear-left, rear-right)
        self.leg_positions = np.array([
            [self.body_length/2, self.body_width/2],   # FL
            [self.body_length/2, -self.body_width/2],  # FR
            [-self.body_length/2, self.body_width/2],  # RL
            [-self.body_length/2, -self.body_width/2]  # RR
        ])
    
    def get_foot_position(self, body_pos, body_angle, leg_extension):
        """
        Compute foot positions given body state and leg extensions
        
        Args:
            body_pos: [x, z] body center of mass position
            body_angle: body pitch angle (radians)
            leg_extension: array of 4 leg extensions (0=retracted, 1=fully extended)
        
        Returns:
            foot_positions: (4, 2) array of [x, z] foot positions
        """
        foot_positions = np.zeros((4, 2))
        
        for i in range(4):
            # Leg attachment point in world frame
            cos_theta = np.cos(body_angle)
            sin_theta = np.sin(body_angle)
            
            attach_x = body_pos[0] + self.leg_positions[i, 0] * cos_theta
            attach_z = body_pos[1] - self.leg_positions[i, 0] * sin_theta
            
            # Foot position (vertical extension downward)
            foot_positions[i, 0] = attach_x
            foot_positions[i, 1] = attach_z - leg_extension[i] * self.leg_length
        
        return foot_positions
    
    def compute_joint_torques_from_force(self, ground_force, leg_extension):
        """
        Estimate joint torques required to generate ground reaction force
        
        This is a simplified model: torque ≈ force * lever_arm
        
        Args:
            ground_force: [Fx, Fz] force at foot
            leg_extension: leg extension ratio (0-1)
        
        Returns:
            max_torque_required: maximum torque across joints for this leg
        """
        # Lever arm depends on leg extension
        lever_arm = leg_extension * self.leg_length
        
        # Total torque is approximately force magnitude times lever arm
        force_magnitude = np.sqrt(ground_force[0]**2 + ground_force[1]**2)
        torque_required = force_magnitude * lever_arm
        
        # Distribute across 3 joints (simplified)
        joint_torque = torque_required / 3.0
        
        return joint_torque
    
    def is_kinematically_feasible(self, body_height, leg_extensions):
        """
        Check if configuration is kinematically feasible
        
        Args:
            body_height: height of body CoM
            leg_extensions: array of 4 leg extension ratios
        
        Returns:
            feasible: True if configuration is valid
        """
        # Check height constraint
        min_height = 0.1  # minimum body height
        max_height = self.leg_length + 0.1  # maximum height
        
        if body_height < min_height or body_height > max_height:
            return False
        
        # Check leg extension limits
        if np.any(leg_extensions < 0) or np.any(leg_extensions > 1):
            return False
        
        return True
    
    def get_state_dimension(self):
        """Return dimension of state vector"""
        return 6  # [x, z, theta, vx, vz, omega]
    
    def get_control_dimension(self):
        """Return dimension of control vector"""
        return 8  # [Fx_leg1, Fz_leg1, ..., Fx_leg4, Fz_leg4]
    
    def __repr__(self):
        return f"UnitreeA1(mass={self.mass}kg, max_torque={self.max_torque}Nm, gravity={self.gravity}m/s²)"


def test_robot_model():
    """Test function to verify robot model"""
    robot = UnitreeA1()
    
    print(f"Robot: {robot}")
    print(f"Initial height: {robot.initial_height} m")
    print(f"Leg length: {robot.leg_length} m")
    print(f"Friction coefficient: {robot.friction_coef}")
    
    # Test foot position calculation
    body_pos = np.array([0.0, robot.initial_height])
    body_angle = 0.0
    leg_extensions = np.array([0.5, 0.5, 0.5, 0.5])
    
    foot_pos = robot.get_foot_position(body_pos, body_angle, leg_extensions)
    print(f"\nFoot positions:\n{foot_pos}")
    
    # Test torque calculation
    ground_force = np.array([50.0, 100.0])
    torque = robot.compute_joint_torques_from_force(ground_force, 0.5)
    print(f"\nEstimated joint torque: {torque:.2f} Nm")
    print(f"Within limits: {torque <= robot.max_torque}")


if __name__ == "__main__":
    test_robot_model()
