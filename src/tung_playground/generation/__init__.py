"""Image-to-3D generation module.

This module handles converting input images to 3D meshes using various AIGC approaches.
"""

from .base import Image3DGenerator, GenerationResult
from .mock_generator import MockGenerator

__all__ = [
    "Image3DGenerator",
    "GenerationResult",
    "MockGenerator",
]