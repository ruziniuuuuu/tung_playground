"""Visualization module for Tung Playground.

This module provides tools for visualizing generated heroes, including:
- Interactive URDF visualization using viser
- 3D mesh and skeleton display
- Real-time joint manipulation
"""

try:
    from .urdf_visualizer import URDFVisualizer
    __all__ = ["URDFVisualizer"]
except ImportError:
    # viser dependencies not available
    __all__ = []