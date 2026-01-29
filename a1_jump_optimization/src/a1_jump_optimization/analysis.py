"""
analysis.py
Upper Bound Verification and Sensitivity Analysis

Performs various analyses to prove the solution is the true upper bound.
"""

import numpy as np
import matplotlib.pyplot as plt
from .optimizer import JumpOptimizer
from .robot_model import UnitreeA1
from pathlib import Path


class SensitivityAnalyzer:
    """
    Perform sensitivity analysis to verify upper bound
    """
    
    def __init__(self, robot):
        """
        Initialize analyzer
        
        Args:
            robot: UnitreeA1 robot model
        """
        self.robot = robot
        self.baseline_torque = robot.max_torque
    
    def torque_sensitivity(self, jump_mode='sequential', torque_range=None):
        """
        Analyze how jump distance varies with torque limit
        
        Args:
            jump_mode: 'simultaneous' or 'sequential'
            torque_range: list of torque limits to test (Nm)
        
        Returns:
            results: dict with torque values and corresponding distances
        """
        if torque_range is None:
            # Test from 20 Nm to 50 Nm
            torque_range = np.linspace(20, 50, 7)
        
        print(f"\n{'='*60}")
        print(f"TORQUE SENSITIVITY ANALYSIS ({jump_mode})")
        print(f"{'='*60}\n")
        
        distances = []
        successful_torques = []
        
        for torque_limit in torque_range:
            # Temporarily modify robot torque limit
            original_torque = self.robot.max_torque
            self.robot.max_torque = torque_limit
            
            # Create and solve optimizer
            optimizer = JumpOptimizer(
                self.robot,
                N=35,
                T=1.5,
                jump_mode=jump_mode
            )
            optimizer.set_initial_guess('crouch')
            
            solution = optimizer.solve(verbose=False)
            
            # Restore original torque
            self.robot.max_torque = original_torque
            
            if solution['success']:
                distance = solution['jump_distance']
                distances.append(distance)
                successful_torques.append(torque_limit)
                print(f"Torque: {torque_limit:5.1f} Nm  →  Distance: {distance:.3f} m")
            else:
                print(f"Torque: {torque_limit:5.1f} Nm  →  Failed to converge")
        
        # Analyze results
        print(f"\n{'='*60}")
        print("Analysis:")
        
        if len(distances) >= 2:
            # Check monotonicity
            diffs = np.diff(distances)
            if np.all(diffs >= -0.01):  # allow small numerical errors
                print("✓ Jump distance increases monotonically with torque")
                print("  This confirms torque is the limiting factor!")
            else:
                print("⚠ Non-monotonic behavior detected")
            
            # Compute sensitivity at baseline
            baseline_idx = np.argmin(np.abs(np.array(successful_torques) - self.baseline_torque))
            if baseline_idx > 0 and baseline_idx < len(distances) - 1:
                sensitivity = (distances[baseline_idx + 1] - distances[baseline_idx - 1]) / \
                             (successful_torques[baseline_idx + 1] - successful_torques[baseline_idx - 1])
                print(f"  Sensitivity at {self.baseline_torque} Nm: {sensitivity:.4f} m/Nm")
        
        print(f"{'='*60}\n")
        
        return {
            'torques': successful_torques,
            'distances': distances,
            'mode': jump_mode
        }
    
    def plot_sensitivity(self, results_list, save_path=None):
        """
        Plot torque sensitivity curves
        
        Args:
            results_list: list of sensitivity results (for different modes)
            save_path: path to save figure
        """
        plt.figure(figsize=(10, 6))
        
        for results in results_list:
            plt.plot(
                results['torques'],
                results['distances'],
                marker='o',
                linewidth=2,
                label=results['mode'].capitalize(),
                markersize=8
            )
        
        # Mark baseline torque
        plt.axvline(
            self.baseline_torque,
            color='red',
            linestyle='--',
            linewidth=1.5,
            label=f'A1 Torque Limit ({self.baseline_torque} Nm)'
        )
        
        plt.xlabel('Maximum Joint Torque (Nm)', fontsize=12)
        plt.ylabel('Jump Distance (m)', fontsize=12)
        plt.title('Jump Performance vs. Torque Limit\n(Lunar Gravity)', fontsize=14, fontweight='bold')
        plt.legend(fontsize=11)
        plt.grid(True, alpha=0.3)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Sensitivity plot saved to {save_path}")
        
        plt.tight_layout()
        return plt.gcf()


class PhysicsValidator:
    """
    Validate solution against physics principles
    """
    
    @staticmethod
    def check_energy_conservation(solution):
        """
        Verify energy conservation during flight phase
        
        Args:
            solution: optimization solution dict
        
        Returns:
            energy_error: maximum energy error during flight
        """
        states = solution['states']
        contact = solution['contact_schedule']
        time = solution['time']
        
        # Find flight phase (no contact)
        in_flight = np.sum(contact, axis=1) == 0
        
        if not np.any(in_flight):
            print("No flight phase detected!")
            return None
        
        # Get flight indices
        flight_indices = np.where(in_flight)[0]
        
        # Calculate energy at each flight point
        energies = []
        for i in flight_indices:
            if i < len(states[0]):
                vx = states[3, i]
                vz = states[4, i]
                z = states[1, i]
                
                # Kinetic + potential energy
                KE = 0.5 * 12.0 * (vx**2 + vz**2)  # mass = 12 kg
                PE = 12.0 * 1.62 * z  # lunar gravity
                
                total_energy = KE + PE
                energies.append(total_energy)
        
        energies = np.array(energies)
        
        # Energy should be conserved
        energy_variation = np.std(energies)
        energy_mean = np.mean(energies)
        relative_error = energy_variation / energy_mean if energy_mean > 0 else 0
        
        print(f"\nEnergy Conservation Check:")
        print(f"  Mean energy: {energy_mean:.2f} J")
        print(f"  Std deviation: {energy_variation:.2f} J")
        print(f"  Relative error: {relative_error*100:.2f}%")
        
        if relative_error < 0.05:
            print("  ✓ Energy well conserved during flight!")
        else:
            print("  ⚠ Significant energy variation detected")
        
        return relative_error
    
    @staticmethod
    def check_momentum_balance(solution, robot):
        """
        Verify impulse-momentum relationship at takeoff
        
        Args:
            solution: optimization solution dict
            robot: robot model
        """
        states = solution['states']
        forces = solution['forces']
        contact = solution['contact_schedule']
        dt = solution['time'][1] - solution['time'][0]
        
        # Find takeoff (transition from contact to no contact)
        contact_sum = np.sum(contact, axis=1)
        takeoff_idx = None
        
        for i in range(len(contact_sum) - 1):
            if contact_sum[i] > 0 and contact_sum[i+1] == 0:
                takeoff_idx = i
                break
        
        if takeoff_idx is None:
            print("Could not identify takeoff point!")
            return
        
        # Calculate total impulse during stance
        total_impulse_x = 0
        total_impulse_z = 0
        
        for i in range(takeoff_idx + 1):
            Fx_total = np.sum(forces[[0, 2, 4, 6], i])
            Fz_total = np.sum(forces[[1, 3, 5, 7], i])
            
            total_impulse_x += Fx_total * dt
            total_impulse_z += Fz_total * dt
        
        # Subtract gravity impulse
        total_impulse_z -= robot.mass * robot.gravity * dt * (takeoff_idx + 1)
        
        # Takeoff velocity
        vx_takeoff = states[3, takeoff_idx]
        vz_takeoff = states[4, takeoff_idx]
        
        # Expected from impulse-momentum
        expected_vx = total_impulse_x / robot.mass
        expected_vz = total_impulse_z / robot.mass
        
        print(f"\nMomentum Balance Check:")
        print(f"  Takeoff velocity (actual):   vx={vx_takeoff:.2f}, vz={vz_takeoff:.2f} m/s")
        print(f"  Takeoff velocity (expected): vx={expected_vx:.2f}, vz={expected_vz:.2f} m/s")
        print(f"  Error: vx={abs(vx_takeoff-expected_vx):.3f}, vz={abs(vz_takeoff-expected_vz):.3f} m/s")
        
        if abs(vx_takeoff - expected_vx) < 0.2 and abs(vz_takeoff - expected_vz) < 0.2:
            print("  ✓ Momentum balance satisfied!")
        else:
            print("  ⚠ Momentum mismatch detected")
    
    @staticmethod
    def check_constraint_activity(solution, robot):
        """
        Check if torque constraints are active (saturated)
        
        Args:
            solution: optimization solution dict
            robot: robot model
        """
        forces = solution['forces']
        leg_ext = solution['leg_extensions']
        contact = solution['contact_schedule']
        
        print(f"\nConstraint Activity Analysis:")
        
        max_torques = []
        
        for k in range(forces.shape[1]):
            for i in range(4):
                if contact[k, i] > 0.5:
                    Fx = forces[2*i, k]
                    Fz = forces[2*i + 1, k]
                    ext = leg_ext[i, k]
                    
                    lever_arm = ext * robot.leg_length
                    force_mag = np.sqrt(Fx**2 + Fz**2)
                    torque = (force_mag * lever_arm) / 3.0
                    
                    max_torques.append(torque)
        
        max_torques = np.array(max_torques)
        
        # Check how many are near the limit
        near_limit = np.sum(max_torques > 0.95 * robot.max_torque)
        total_contact_points = len(max_torques)
        
        print(f"  Contact points with torque > 95% of limit: {near_limit}/{total_contact_points}")
        print(f"  Maximum torque used: {np.max(max_torques):.2f} Nm (limit: {robot.max_torque} Nm)")
        print(f"  Mean torque during contact: {np.mean(max_torques):.2f} Nm")
        
        if near_limit > 0.2 * total_contact_points:
            print("  ✓ Torque limits are ACTIVE - system is hardware-limited!")
        else:
            print("  ⚠ Torque limits not saturated - may not be at true maximum")


def run_complete_analysis(jump_mode='sequential'):
    """
    Run all validation analyses
    """
    print(f"\n{'#'*60}")
    print(f"# COMPLETE ANALYSIS: {jump_mode.upper()} MODE")
    print(f"{'#'*60}\n")
    
    robot = UnitreeA1()
    
    # 1. Get optimal solution
    print("Step 1: Computing optimal trajectory...")
    optimizer = JumpOptimizer(robot, N=40, T=1.5, jump_mode=jump_mode)
    optimizer.set_initial_guess('aggressive')
    solution = optimizer.solve(verbose=False)
    
    if not solution['success']:
        print("Optimization failed!")
        return
    
    print(f"✓ Optimal jump distance: {solution['jump_distance']:.3f} m\n")
    
    # 2. Physics validation
    print("Step 2: Physics validation...")
    validator = PhysicsValidator()
    validator.check_energy_conservation(solution)
    validator.check_momentum_balance(solution, robot)
    validator.check_constraint_activity(solution, robot)
    
    # 3. Sensitivity analysis
    print("\nStep 3: Torque sensitivity analysis...")
    analyzer = SensitivityAnalyzer(robot)
    sensitivity_results = analyzer.torque_sensitivity(jump_mode)
    
    # 4. Plot results
    output_dir = Path('results/plots')
    output_dir.mkdir(parents=True, exist_ok=True)
    
    analyzer.plot_sensitivity(
        [sensitivity_results],
        save_path=output_dir / f'sensitivity_{jump_mode}.png'
    )
    
    plt.show()
    
    return solution, sensitivity_results


if __name__ == "__main__":
    solution, sensitivity = run_complete_analysis('sequential')
