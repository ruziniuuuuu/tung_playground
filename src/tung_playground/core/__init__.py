"""Core module for Tung Playground.

This module contains the fundamental classes and interfaces for the AIGC hero simulation framework.
"""

from .hero import Hero, HeroAssets, HeroStatus, AssetType
from .pipeline import PipelineStage, Pipeline, PipelineConfig, StageResult, StageStatus
from .exceptions import TungPlaygroundError, ValidationError, ProcessingError

__all__ = [
    "Hero",
    "HeroAssets",
    "HeroStatus", 
    "AssetType",
    "PipelineStage",
    "Pipeline",
    "PipelineConfig",
    "StageResult",
    "StageStatus",
    "TungPlaygroundError",
    "ValidationError",
    "ProcessingError",
]