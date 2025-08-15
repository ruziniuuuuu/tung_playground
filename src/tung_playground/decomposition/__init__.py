"""Part decomposition module for 3D meshes."""

from .base import PartDecomposer, DecompositionResult
from .partcrafter import PartCrafterDecomposer

__all__ = [
    "PartDecomposer",
    "DecompositionResult",
    "PartCrafterDecomposer",
]