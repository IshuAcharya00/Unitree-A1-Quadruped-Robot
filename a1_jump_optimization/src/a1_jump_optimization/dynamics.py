"""
dynamics.py
Centroidal Dynamics for Quadruped Jumping

Implements the equations of motion for the robot's center of mass
during stance and flight phases.
"""

import numpy as np
import casadi as ca


class CentroidalDynamics:
    """
    Centroidal dynamics model for quadruped robot
    
    State vector: x = [p_x, p_z, theta, v_x, v_z, omega]
    - p_x, p_z: position of center of mass
    - theta: body pitch angle
    - v_x, v_z: velocity of center of mass
    - omega: angular velocity
    
    Control vector during stance: u = [Fx1, Fz1, Fx2, Fz2, Fx3, Fz3, Fx4, Fz4]
    - Fxi, Fzi: ground reaction force for leg i
    """
    
    def __init__(self, robot):
        """
        Initialize dynamics model
        
        Args:
            robot: UnitreeA1 robot model
        """
        self.robot = robot
        self.m = robot.mass
        self.I = robot.inertia
        self.g = robot.gravity
    
    def stance_dynamics(self, state, forces):
        """
        Dynamics during stance phase (feet on ground)
        
        Args:
            state: state vector [x, z, theta, vx, vz, omega]
            forces: ground reaction forces [Fx1, Fz1, ..., Fx4, Fz4]
        
        Returns:
            state_dot: derivative of state
        """
        # Unpack state
        x, z, theta, vx, vz, omega = state[0], state[1], state[2], state[3], state[4], state[5]
        
        # Sum all ground reaction forces
        Fx_total = forces[0] + forces[2] + forces[4] + forces[6]
        Fz_total = forces[1] + forces[3] + forces[5] + forces[7]
        
        # Linear dynamics
        ax = Fx_total / self.m
        az = Fz_total / self.m - self.g
        
        # Angular dynamics (torque from forces about CoM)
        # Simplified: assume forces create pitch torque
        # Torque = Σ (r_x * F_z - r_z * F_x) where r is position relative to CoM
        
        # Leg positions relative to CoM (simplified)
        leg_positions = self.robot.leg_positions
        
        torque = 0.0
        for i in range(4):
            rx = leg_positions[i, 0] * ca.cos(theta)
            # Vertical offset negligible for torque calculation
            torque += rx * forces[2*i + 1]  # horizontal_offset * vertical_force
        
        alpha = torque / self.I
        
        # State derivatives
        state_dot = ca.vertcat(vx, vz, omega, ax, az, alpha)
        
        return state_dot
    
    def flight_dynamics(self, state):
        """
        Dynamics during flight phase (ballistic motion)
        
        Args:
            state: state vector [x, z, theta, vx, vz, omega]
        
        Returns:
            state_dot: derivative of state
        """
        vx, vz, omega = state[3], state[4], state[5]
        
        # No forces except gravity
        ax = 0.0
        az = -self.g
        alpha = 0.0  # no torque in flight
        
        state_dot = ca.vertcat(vx, vz, omega, ax, az, alpha)
        
        return state_dot
    
    def generate_constraints(self, opti, state_vars, force_vars, contact_flags):
        """
        Generate optimization constraints for the dynamics
        
        Args:
            opti: CasADi Opti optimization object
            state_vars: list of state variables at each time step
            force_vars: list of force variables at each time step
            contact_flags: list of booleans indicating if in contact
        
        Returns:
            None (constraints added to opti)
        """
        N = len(state_vars) - 1  # number of intervals
        
        for k in range(N):
            # Current and next state
            x_k = state_vars[k]
            x_kp1 = state_vars[k+1]
            
            if contact_flags[k]:
                # Stance dynamics
                f_k = force_vars[k]
                dx = self.stance_dynamics(x_k, f_k)
            else:
                # Flight dynamics
                dx = self.flight_dynamics(x_k)
            
            # Trapezoidal integration (collocation)
            dt = 0.02  # time step (will be optimized later)
            opti.subject_to(x_kp1 == x_k + dt * dx)
    
    def friction_cone_constraint(self, Fx, Fz, mu):
        """
        Friction cone constraint: |Fx| <= mu * Fz
        
        Args:
            Fx: horizontal force
            Fz: vertical force
            mu: friction coefficient
        
        Returns:
            constraint expression
        """
        return ca.vertcat(
            Fx <= mu * Fz,
            -Fx <= mu * Fz
        )
    
    def no_penetration_constraint(self, z, z_foot):
        """
        Ground penetration constraint: foot must be above ground
        
        Args:
            z: body height
            z_foot: foot height
        
        Returns:
            constraint expression
        """
        return z_foot >= 0.0
    
    def torque_constraint_from_force(self, forces, leg_extension):
        """
        Approximate joint torque constraint from ground reaction force
        
        Args:
            forces: [Fx, Fz] for one leg
            leg_extension: extension ratio (0-1)
        
        Returns:
            torque_magnitude: estimated max joint torque
        """
        lever_arm = leg_extension * self.robot.leg_length
        force_magnitude = ca.sqrt(forces[0]**2 + forces[1]**2)
        
        # Torque per joint (simplified distribution)
        torque_per_joint = (force_magnitude * lever_arm) / 3.0
        
        return torque_per_joint


class NumPyDynamics:
    """
    NumPy version of dynamics for simulation and testing
    (Non-symbolic, for numerical integration)
    """
    
    def __init__(self, robot):
        self.robot = robot
        self.m = robot.mass
        self.I = robot.inertia
        self.g = robot.gravity
    
    def stance_dynamics_numpy(self, state, forces):
        """NumPy implementation of stance dynamics"""
        x, z, theta, vx, vz, omega = state
        
        Fx_total = forces[0] + forces[2] + forces[4] + forces[6]
        Fz_total = forces[1] + forces[3] + forces[5] + forces[7]
        
        ax = Fx_total / self.m
        az = Fz_total / self.m - self.g
        
        # Simplified torque
        leg_positions = self.robot.leg_positions
        torque = 0.0
        for i in range(4):
            rx = leg_positions[i, 0] * np.cos(theta)
            torque += rx * forces[2*i + 1]
        
        alpha = torque / self.I
        
        return np.array([vx, vz, omega, ax, az, alpha])
    
    def flight_dynamics_numpy(self, state):
        """NumPy implementation of flight dynamics"""
        vx, vz, omega = state[3], state[4], state[5]
        
        ax = 0.0
        az = -self.g
        alpha = 0.0
        
        return np.array([vx, vz, omega, ax, az, alpha])


def test_dynamics():
    """Test dynamics implementation"""
    from .robot_model import UnitreeA1
    
    robot = UnitreeA1()
    dynamics = NumPyDynamics(robot)
    
    # Initial state: standing
    state = np.array([0.0, 0.3, 0.0, 0.0, 0.0, 0.0])
    
    # Apply upward forces
    forces = np.array([0.0, 50.0,   # leg 1
                      0.0, 50.0,   # leg 2
                      0.0, 50.0,   # leg 3
                      0.0, 50.0])  # leg 4
    
    # Compute acceleration
    state_dot = dynamics.stance_dynamics_numpy(state, forces)
    
    print(f"State: {state}")
    print(f"Forces: {forces}")
    print(f"Acceleration: {state_dot}")
    print(f"Vertical acceleration: {state_dot[4]:.2f} m/s²")
    
    # Test flight dynamics
    state_flight = np.array([0.0, 1.0, 0.0, 2.0, 3.0, 0.0])
    state_dot_flight = dynamics.flight_dynamics_numpy(state_flight)
    print(f"\nFlight state dot: {state_dot_flight}")


if __name__ == "__main__":
    test_dynamics()
