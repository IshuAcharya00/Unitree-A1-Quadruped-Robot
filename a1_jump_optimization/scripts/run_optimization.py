"""
run_optimization.py
Main script to run the complete optimization and analysis
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

from a1_jump_optimization.robot_model import UnitreeA1
from a1_jump_optimization.solver import MultiStartSolver
from a1_jump_optimization.analysis import SensitivityAnalyzer, PhysicsValidator
from a1_jump_optimization.visualizer import create_all_visualizations
import numpy as np


def main():
    """
    Run complete optimization pipeline
    """
    print("\n" + "="*70)
    print(" UNITREE A1 LONG JUMP OPTIMIZATION")
    print(" Maximum Jump Distance under Lunar Gravity")
    print("="*70 + "\n")
    
    # Initialize robot
    robot = UnitreeA1()
    print(f"Robot Configuration:")
    print(f"  Mass: {robot.mass} kg")
    print(f"  Max Torque: {robot.max_torque} Nm")
    print(f"  Gravity: {robot.gravity} m/s² (Lunar)")
    print(f"  Friction: {robot.friction_coef}")
    print()
    
    # Run multi-start optimization for both modes
    results = {}
    best_solutions = {}
    
    for mode in ['simultaneous', 'sequential']:
        print(f"\n{'#'*70}")
        print(f"# OPTIMIZING: {mode.upper()} JUMP STRATEGY")
        print(f"{'#'*70}\n")
        
        solver = MultiStartSolver(robot, jump_mode=mode, n_starts=12)
        best_solution = solver.run(verbose=True)
        
        if best_solution:
            results[mode] = best_solution['jump_distance']
            best_solutions[mode] = best_solution
            
            # Save results
            solver.save_results('results/data')
            
            # Create visualizations
            create_all_visualizations(
                best_solution, 
                robot, 
                output_dir=f'results/plots/{mode}'
            )
    
    # Final comparison
    print(f"\n{'='*70}")
    print(" FINAL RESULTS")
    print(f"{'='*70}\n")
    
    if 'simultaneous' in results and 'sequential' in results:
        print(f"Simultaneous Jump Strategy: {results['simultaneous']:.3f} m")
        print(f"Sequential Jump Strategy:   {results['sequential']:.3f} m")
        print()
        
        diff = results['sequential'] - results['simultaneous']
        percent_diff = (diff / results['simultaneous']) * 100
        
        print(f"Difference: {abs(diff):.3f} m ({abs(percent_diff):.1f}%)")
        print()
        
        if abs(diff) < 0.05:
            print("→ CONCLUSION: Both strategies perform nearly identically")
            print("  The jump mode has minimal impact on maximum distance.")
        elif diff > 0:
            print("→ CONCLUSION: Sequential (cat-like) strategy is SUPERIOR")
            print("  This validates the strategy discovered by reinforcement learning!")
            print("  The front-then-rear leg push provides better performance.")
        else:
            print("→ CONCLUSION: Simultaneous strategy is SUPERIOR")
            print("  Traditional all-legs-together approach is more effective.")
        
        print()
        print(f"Comparison to RL result (2.75 m):")
        best_model_result = max(results['simultaneous'], results['sequential'])
        if best_model_result > 2.75:
            print(f"  Model predicts HIGHER maximum: {best_model_result:.3f} m")
            print(f"  RL may not have reached the true optimum yet.")
        elif best_model_result < 2.60:
            print(f"  Model predicts LOWER maximum: {best_model_result:.3f} m")
            print(f"  RL result appears exceptional - may need model refinement.")
        else:
            print(f"  Model agrees with RL: {best_model_result:.3f} m")
            print(f"  RL has likely reached near-optimal performance!")
    
    # Sensitivity analysis
    print(f"\n{'='*70}")
    print(" SENSITIVITY ANALYSIS")
    print(f"{'='*70}\n")
    
    analyzer = SensitivityAnalyzer(robot)
    
    sensitivity_results = []
    for mode in ['simultaneous', 'sequential']:
        if mode in best_solutions:
            sens = analyzer.torque_sensitivity(
                jump_mode=mode,
                torque_range=np.linspace(25, 45, 5)
            )
            sensitivity_results.append(sens)
    
    if sensitivity_results:
        analyzer.plot_sensitivity(
            sensitivity_results,
            save_path='results/plots/torque_sensitivity.png'
        )
    
    # Physics validation
    if 'sequential' in best_solutions:
        print(f"\n{'='*70}")
        print(" PHYSICS VALIDATION")
        print(f"{'='*70}\n")
        
        validator = PhysicsValidator()
        validator.check_energy_conservation(best_solutions['sequential'])
        validator.check_momentum_balance(best_solutions['sequential'], robot)
        validator.check_constraint_activity(best_solutions['sequential'], robot)
    
    print(f"\n{'='*70}")
    print(" OPTIMIZATION COMPLETE!")
    print(f"{'='*70}")
    print(f"\nResults saved to: results/")
    print(f"  - Data: results/data/")
    print(f"  - Plots: results/plots/")
    print()


if __name__ == "__main__":
    main()
