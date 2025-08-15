"""Robot template system for template-based hero generation."""

from .base import RobotTemplate, TemplateType, PartType, TemplateLibrary
from .biped import BipedTemplate
from .quadruped import QuadrupedTemplate

__all__ = [
    "RobotTemplate",
    "TemplateType", 
    "PartType",
    "TemplateLibrary",
    "BipedTemplate",
    "QuadrupedTemplate",
]