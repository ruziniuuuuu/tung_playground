"""Part decomposition module for 3D meshes."""

from .base import PartDecomposer, DecompositionResult
from .mock_decomposer import MockDecomposer

__all__ = [
    "PartDecomposer",
    "DecompositionResult",
    "MockDecomposer",
]