"""
Unitree A1 Long Jump Optimization Package
"""

__version__ = "1.0.0"
__author__ = "Your Name"

from .robot_model import UnitreeA1

__all__ = ['UnitreeA1']

try:
    from .dynamics import CentroidalDynamics
    from .optimizer import JumpOptimizer
    from .solver import MultiStartSolver
    from .analysis import SensitivityAnalyzer, PhysicsValidator
    from .visualizer import TrajectoryVisualizer

    __all__ += [
        'CentroidalDynamics',
        'JumpOptimizer',
        'MultiStartSolver',
        'SensitivityAnalyzer',
        'PhysicsValidator',
        'TrajectoryVisualizer'
    ]
except ImportError:
    pass
