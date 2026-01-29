"""
optimizer.py
Trajectory Optimization for Maximum Long Jump

Uses direct collocation with CasADi to find optimal jumping trajectory
"""

import numpy as np
import casadi as ca
from .robot_model import UnitreeA1
from .dynamics import CentroidalDynamics


class JumpOptimizer:
    """
    Trajectory optimizer for quadruped long jump
    
    Formulates the optimization problem:
        maximize: horizontal_distance
        subject to: 
            - dynamics constraints
            - torque limits
            - friction cone
            - kinematic feasibility
    """
    
    def __init__(self, robot, N=50, T=2.0, jump_mode='sequential'):
        """
        Initialize optimizer
        
        Args:
            robot: UnitreeA1 robot model
            N: number of time discretization points
            T: total trajectory time (seconds)
            jump_mode: 'simultaneous' or 'sequential' leg push strategy
        """
        self.robot = robot
        self.dynamics = CentroidalDynamics(robot)
        self.N = N  # number of time steps
        self.T = T  # total time
        self.dt = T / N  # time step
        self.jump_mode = jump_mode
        
        # Create CasADi optimization problem
        self.opti = ca.Opti()
        
        # Decision variables
        self.setup_variables()
        
        # Objective and constraints
        self.setup_objective()
        self.setup_constraints()
    
    def setup_variables(self):
        """Create optimization variables"""
        # State trajectory: [x, z, theta, vx, vz, omega] at each time step
        self.X = self.opti.variable(6, self.N + 1)
        
        # Force trajectory: [Fx1, Fz1, ..., Fx4, Fz4] at each time step
        self.F = self.opti.variable(8, self.N)
        
        # Leg extension ratios (for torque calculation)
        self.leg_ext = self.opti.variable(4, self.N)
        
        # Contact schedule (binary-ish variables indicating contact)
        # For simplicity, we'll define contact phases manually
        self.contact_schedule = self._create_contact_schedule()
    
    def _create_contact_schedule(self):
        """
        Create contact schedule based on jump mode
        
        Returns:
            contact: (N, 4) array - 1 if leg i is in contact at step k
        """
        contact = np.zeros((self.N, 4))
        
        if self.jump_mode == 'simultaneous':
            # All four legs push simultaneously, then flight
            stance_phase = int(0.3 * self.N)  # 30% of time in stance
            contact[:stance_phase, :] = 1.0
        
        elif self.jump_mode == 'sequential':
            # Front legs push first, then rear legs (cat-like)
            front_push = int(0.15 * self.N)  # 15% front leg push
            rear_push = int(0.3 * self.N)   # next 15% rear leg push
            
            # Front legs (indices 0, 1) push first
            contact[:front_push, [0, 1]] = 1.0
            
            # Rear legs (indices 2, 3) push second
            contact[front_push:rear_push, [2, 3]] = 1.0
        
        return contact
    
    def setup_objective(self):
        """Define optimization objective: maximize horizontal distance"""
        # Objective: maximize final horizontal position
        x_final = self.X[0, -1]
        x_initial = self.X[0, 0]
        
        jump_distance = x_final - x_initial
        
        # Maximize jump distance (minimize negative distance)
        self.opti.minimize(-jump_distance)
    
    def setup_constraints(self):
        """Add all optimization constraints"""
        
        # 1. Initial conditions
        self.opti.subject_to(self.X[0, 0] == 0.0)  # start at origin
        self.opti.subject_to(self.X[1, 0] == self.robot.initial_height)  # initial height
        self.opti.subject_to(self.X[2, 0] == 0.0)  # zero pitch
        self.opti.subject_to(self.X[3, 0] == 0.0)  # zero horizontal velocity
        self.opti.subject_to(self.X[4, 0] == 0.0)  # zero vertical velocity
        self.opti.subject_to(self.X[5, 0] == 0.0)  # zero angular velocity
        
        # 2. Dynamics constraints (collocation)
        for k in range(self.N):
            x_k = self.X[:, k]
            x_kp1 = self.X[:, k+1]
            
            # Check if any leg is in contact
            in_contact = np.sum(self.contact_schedule[k, :]) > 0
            
            if in_contact:
                # Stance dynamics
                f_k = self.F[:, k]
                dx = self.dynamics.stance_dynamics(x_k, f_k)
            else:
                # Flight dynamics
                dx = self.dynamics.flight_dynamics(x_k)
            
            # Trapezoidal integration
            self.opti.subject_to(x_kp1 == x_k + self.dt * dx)
        
        # 3. Torque limits
        for k in range(self.N):
            for i in range(4):
                if self.contact_schedule[k, i] > 0.5:  # leg in contact
                    Fx = self.F[2*i, k]
                    Fz = self.F[2*i + 1, k]
                    leg_ext = self.leg_ext[i, k]
                    
                    # Compute torque from force
                    lever_arm = leg_ext * self.robot.leg_length
                    force_mag = ca.sqrt(Fx**2 + Fz**2)
                    torque_per_joint = (force_mag * lever_arm) / 3.0
                    
                    # Torque constraint
                    self.opti.subject_to(torque_per_joint <= self.robot.max_torque)
        
        # 4. Friction cone constraints
        for k in range(self.N):
            for i in range(4):
                if self.contact_schedule[k, i] > 0.5:
                    Fx = self.F[2*i, k]
                    Fz = self.F[2*i + 1, k]
                    mu = self.robot.friction_coef
                    
                    # Friction cone: |Fx| <= mu * Fz
                    self.opti.subject_to(Fx <= mu * Fz)
                    self.opti.subject_to(-Fx <= mu * Fz)
                    
                    # Normal force must be positive
                    self.opti.subject_to(Fz >= 0)
                else:
                    # No contact = no force
                    self.opti.subject_to(self.F[2*i, k] == 0)
                    self.opti.subject_to(self.F[2*i + 1, k] == 0)
        
        # 5. Leg extension limits (element-wise constraints)
        for i in range(4):  # 4 legs
            for k in range(self.N):
                self.opti.subject_to(self.leg_ext[i, k] >= 0.2)  # minimum extension
                self.opti.subject_to(self.leg_ext[i, k] <= 1.0)  # maximum extension
        
        # 6. Ground penetration (body height must be positive)
        for k in range(self.N + 1):
            self.opti.subject_to(self.X[1, k] >= 0.05)  # min height 5cm
        
        # 7. Final landing constraint (return to reasonable height)
        self.opti.subject_to(self.X[1, -1] >= 0.1)  # land at least 10cm high
        self.opti.subject_to(self.X[1, -1] <= 0.5)  # but not too high
        
        # 8. Reasonable velocity bounds
        for k in range(self.N + 1):
            self.opti.subject_to(self.X[3, k] >= -2.0)  # vx >= -2 m/s
            self.opti.subject_to(self.X[3, k] <= 10.0)  # vx <= 10 m/s
            self.opti.subject_to(self.X[4, k] >= -10.0) # vz >= -10 m/s
            self.opti.subject_to(self.X[4, k] <= 10.0)  # vz <= 10 m/s
    
    def set_initial_guess(self, guess_type='crouch'):
        """
        Set initial guess for optimization
        
        Args:
            guess_type: 'crouch', 'aggressive', or 'random'
        """
        if guess_type == 'crouch':
            # Conservative crouch and jump
            x_guess = np.zeros((6, self.N + 1))
            x_guess[0, :] = np.linspace(0, 2.0, self.N + 1)  # horizontal motion
            x_guess[1, :] = 0.3  # constant height
            x_guess[3, :] = 2.0  # constant horizontal velocity
            
            self.opti.set_initial(self.X, x_guess)
            
            # Moderate forces
            f_guess = np.ones((8, self.N)) * 30.0
            self.opti.set_initial(self.F, f_guess)
            
            # Medium leg extension
            self.opti.set_initial(self.leg_ext, 0.5 * np.ones((4, self.N)))
        
        elif guess_type == 'aggressive':
            # Aggressive jump trajectory
            x_guess = np.zeros((6, self.N + 1))
            x_guess[0, :] = np.linspace(0, 3.0, self.N + 1)
            
            # Parabolic height profile
            t = np.linspace(0, 1, self.N + 1)
            x_guess[1, :] = 0.3 + 1.0 * t * (1 - t) * 4  # parabola
            
            x_guess[3, :] = 3.5
            x_guess[4, :] = np.linspace(2.0, -2.0, self.N + 1)
            
            self.opti.set_initial(self.X, x_guess)
            
            # Higher forces
            f_guess = np.ones((8, self.N)) * 50.0
            self.opti.set_initial(self.F, f_guess)
            
            # More extension
            self.opti.set_initial(self.leg_ext, 0.7 * np.ones((4, self.N)))
        
        elif guess_type == 'random':
            # Random initialization
            x_guess = np.random.randn(6, self.N + 1) * 0.5
            x_guess[1, :] = np.abs(x_guess[1, :]) + 0.3  # keep height positive
            
            self.opti.set_initial(self.X, x_guess)
            self.opti.set_initial(self.F, np.random.randn(8, self.N) * 20)
            self.opti.set_initial(self.leg_ext, np.random.rand(4, self.N) * 0.5 + 0.3)
    
    def solve(self, verbose=True):
        """
        Solve the optimization problem
        
        Returns:
            solution: dict with optimal trajectory and metadata
        """
        # Set solver options
        opts = {
            'ipopt.print_level': 5 if verbose else 0,
            'ipopt.max_iter': 2000,
            'ipopt.tol': 1e-6,
            'print_time': verbose
        }
        self.opti.solver('ipopt', opts)
        
        try:
            # Solve
            sol = self.opti.solve()
            
            # Extract solution
            X_opt = sol.value(self.X)
            F_opt = sol.value(self.F)
            leg_ext_opt = sol.value(self.leg_ext)
            
            jump_distance = X_opt[0, -1] - X_opt[0, 0]
            
            solution = {
                'success': True,
                'states': X_opt,
                'forces': F_opt,
                'leg_extensions': leg_ext_opt,
                'jump_distance': jump_distance,
                'contact_schedule': self.contact_schedule,
                'time': np.linspace(0, self.T, self.N + 1),
                'solver_stats': sol.stats()
            }
            
            return solution
        
        except Exception as e:
            print(f"Optimization failed: {e}")
            return {
                'success': False,
                'error': str(e)
            }


def test_optimizer():
    """Test the optimizer"""
    print("Testing Jump Optimizer...")
    
    # Create robot
    robot = UnitreeA1()
    
    # Test simultaneous mode
    print("\n=== Testing Simultaneous Jump ===")
    opt_sim = JumpOptimizer(robot, N=30, T=1.5, jump_mode='simultaneous')
    opt_sim.set_initial_guess('crouch')
    
    sol_sim = opt_sim.solve(verbose=False)
    
    if sol_sim['success']:
        print(f"✓ Simultaneous jump distance: {sol_sim['jump_distance']:.3f} m")
    else:
        print(f"✗ Simultaneous optimization failed")
    
    # Test sequential mode
    print("\n=== Testing Sequential Jump ===")
    opt_seq = JumpOptimizer(robot, N=30, T=1.5, jump_mode='sequential')
    opt_seq.set_initial_guess('crouch')
    
    sol_seq = opt_seq.solve(verbose=False)
    
    if sol_seq['success']:
        print(f"✓ Sequential jump distance: {sol_seq['jump_distance']:.3f} m")
    else:
        print(f"✗ Sequential optimization failed")
    
    # Compare
    if sol_sim['success'] and sol_seq['success']:
        if sol_seq['jump_distance'] > sol_sim['jump_distance']:
            print(f"\n→ Sequential strategy is better by {sol_seq['jump_distance'] - sol_sim['jump_distance']:.3f} m")
        else:
            print(f"\n→ Simultaneous strategy is better by {sol_sim['jump_distance'] - sol_seq['jump_distance']:.3f} m")


if __name__ == "__main__":
    test_optimizer()
