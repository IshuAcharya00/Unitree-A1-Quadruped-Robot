"""
solver.py
Multi-Start Optimization Solver

Runs trajectory optimization from multiple initial guesses
to ensure global optimum convergence.
"""

import numpy as np
from .optimizer import JumpOptimizer
from .robot_model import UnitreeA1
import json
from pathlib import Path
from tqdm import tqdm


class MultiStartSolver:
    """
    Multi-start solver for robust global optimization
    
    Runs optimization from many different initial guesses
    to verify convergence to global optimum.
    """
    
    def __init__(self, robot, jump_mode='sequential', n_starts=20):
        """
        Initialize multi-start solver
        
        Args:
            robot: UnitreeA1 robot model
            jump_mode: 'simultaneous' or 'sequential'
            n_starts: number of random starts
        """
        self.robot = robot
        self.jump_mode = jump_mode
        self.n_starts = n_starts
        self.solutions = []
    
    def run(self, verbose=True):
        """
        Run optimization from multiple initial guesses
        
        Returns:
            best_solution: solution with maximum jump distance
        """
        print(f"\n{'='*60}")
        print(f"Multi-Start Optimization: {self.jump_mode.upper()} mode")
        print(f"Number of starts: {self.n_starts}")
        print(f"{'='*60}\n")
        
        guess_types = ['crouch', 'aggressive'] + ['random'] * (self.n_starts - 2)
        
        for i in tqdm(range(self.n_starts), desc="Optimization runs"):
            # Create fresh optimizer
            optimizer = JumpOptimizer(
                self.robot,
                N=40,
                T=1.5,
                jump_mode=self.jump_mode
            )
            
            # Set initial guess
            guess_type = guess_types[i]
            optimizer.set_initial_guess(guess_type)
            
            # Solve
            solution = optimizer.solve(verbose=False)
            
            if solution['success']:
                solution['guess_type'] = guess_type
                solution['run_id'] = i
                self.solutions.append(solution)
                
                if verbose:
                    print(f"  Run {i+1}: {solution['jump_distance']:.3f} m ({guess_type})")
        
        # Analyze convergence
        self.analyze_convergence()
        
        # Return best solution
        if len(self.solutions) > 0:
            best_idx = np.argmax([s['jump_distance'] for s in self.solutions])
            return self.solutions[best_idx]
        else:
            return None
    
    def analyze_convergence(self):
        """Analyze convergence across multiple runs"""
        if len(self.solutions) == 0:
            print("No successful solutions to analyze!")
            return
        
        distances = [s['jump_distance'] for s in self.solutions]
        
        print(f"\n{'='*60}")
        print("CONVERGENCE ANALYSIS")
        print(f"{'='*60}")
        print(f"Successful runs: {len(self.solutions)} / {self.n_starts}")
        print(f"Jump distances:")
        print(f"  Mean:   {np.mean(distances):.4f} m")
        print(f"  Std:    {np.std(distances):.4f} m")
        print(f"  Min:    {np.min(distances):.4f} m")
        print(f"  Max:    {np.max(distances):.4f} m")
        print(f"  Range:  {np.max(distances) - np.min(distances):.4f} m")
        
        # Check convergence tolerance
        convergence_tol = 0.01  # 1% tolerance
        max_dist = np.max(distances)
        converged = [d for d in distances if abs(d - max_dist) / max_dist < convergence_tol]
        
        convergence_rate = len(converged) / len(distances) * 100
        print(f"\nConvergence to global optimum:")
        print(f"  {len(converged)} / {len(distances)} runs within {convergence_tol*100:.1f}% of max")
        print(f"  Convergence rate: {convergence_rate:.1f}%")
        
        if convergence_rate > 80:
            print("  ✓ EXCELLENT: Very confident this is the global optimum")
        elif convergence_rate > 50:
            print("  ✓ GOOD: Likely found the global optimum")
        else:
            print("  ⚠ WARNING: May need more runs or different initialization")
        
        print(f"{'='*60}\n")
    
    def save_results(self, output_dir):
        """
        Save all solutions to file
        
        Args:
            output_dir: directory to save results
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Save summary
        summary = {
            'jump_mode': self.jump_mode,
            'n_starts': self.n_starts,
            'n_successful': len(self.solutions),
            'jump_distances': [s['jump_distance'] for s in self.solutions],
            'statistics': {
                'mean': float(np.mean([s['jump_distance'] for s in self.solutions])),
                'std': float(np.std([s['jump_distance'] for s in self.solutions])),
                'min': float(np.min([s['jump_distance'] for s in self.solutions])),
                'max': float(np.max([s['jump_distance'] for s in self.solutions]))
            }
        }
        
        with open(output_path / f'summary_{self.jump_mode}.json', 'w') as f:
            json.dump(summary, f, indent=2)
        
        # Save best solution trajectory
        if len(self.solutions) > 0:
            best_idx = np.argmax([s['jump_distance'] for s in self.solutions])
            best_sol = self.solutions[best_idx]
            
            np.savez(
                output_path / f'best_trajectory_{self.jump_mode}.npz',
                states=best_sol['states'],
                forces=best_sol['forces'],
                leg_extensions=best_sol['leg_extensions'],
                contact_schedule=best_sol['contact_schedule'],
                time=best_sol['time'],
                jump_distance=best_sol['jump_distance']
            )
        
        print(f"Results saved to {output_path}")


def run_full_analysis():
    """
    Run complete multi-start analysis for both jump modes
    """
    robot = UnitreeA1()
    
    # Test both modes
    results = {}
    
    for mode in ['simultaneous', 'sequential']:
        solver = MultiStartSolver(robot, jump_mode=mode, n_starts=15)
        best_solution = solver.run(verbose=True)
        
        if best_solution:
            results[mode] = best_solution['jump_distance']
            
            # Save results
            solver.save_results('results/data')
    
    # Final comparison
    print(f"\n{'='*60}")
    print("FINAL COMPARISON")
    print(f"{'='*60}")
    
    if 'simultaneous' in results and 'sequential' in results:
        print(f"Simultaneous jump: {results['simultaneous']:.3f} m")
        print(f"Sequential jump:   {results['sequential']:.3f} m")
        
        diff = results['sequential'] - results['simultaneous']
        if abs(diff) < 0.05:
            print(f"\n→ Both strategies perform similarly (difference: {abs(diff):.3f} m)")
        elif diff > 0:
            print(f"\n→ Sequential (cat-like) strategy is BETTER by {diff:.3f} m")
            print("  This validates the RL-discovered strategy!")
        else:
            print(f"\n→ Simultaneous strategy is BETTER by {abs(diff):.3f} m")
            print("  Traditional approach is more effective")
    
    print(f"{'='*60}\n")
    
    return results


if __name__ == "__main__":
    results = run_full_analysis()
