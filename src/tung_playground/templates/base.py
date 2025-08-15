"""Base classes for robot template system."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple
from enum import Enum
from pathlib import Path
import json
import logging

logger = logging.getLogger(__name__)


class TemplateType(str, Enum):
    """Types of robot templates."""
    BIPED = "biped"
    QUADRUPED = "quadruped"
    HEXAPOD = "hexapod"
    OCTOPOD = "octopod"
    CUSTOM = "custom"


class PartType(str, Enum):
    """Types of robot parts for template matching."""
    HEAD = "head"
    TORSO = "torso"
    ARM_LEFT = "arm_left"
    ARM_RIGHT = "arm_right"
    LEG_LEFT = "leg_left"
    LEG_RIGHT = "leg_right"
    FOOT_LEFT = "foot_left"
    FOOT_RIGHT = "foot_right"
    HAND_LEFT = "hand_left"
    HAND_RIGHT = "hand_right"
    TAIL = "tail"
    WING_LEFT = "wing_left"
    WING_RIGHT = "wing_right"
    
    # Quadruped specific
    FRONT_LEG_LEFT = "front_leg_left"
    FRONT_LEG_RIGHT = "front_leg_right"
    BACK_LEG_LEFT = "back_leg_left" 
    BACK_LEG_RIGHT = "back_leg_right"
    
    # Multi-leg specific
    LEG_1 = "leg_1"
    LEG_2 = "leg_2"
    LEG_3 = "leg_3"
    LEG_4 = "leg_4"
    LEG_5 = "leg_5"
    LEG_6 = "leg_6"
    LEG_7 = "leg_7"
    LEG_8 = "leg_8"
    
    # Generic
    MISC = "misc"
    UNKNOWN = "unknown"


@dataclass
class TemplateJoint:
    """Definition of a joint in a robot template."""
    
    name: str
    joint_type: str = "revolute"  # revolute, prismatic, fixed, etc.
    parent_link: str = ""
    child_link: str = ""
    axis: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    # Joint limits
    lower_limit: Optional[float] = None
    upper_limit: Optional[float] = None
    effort_limit: Optional[float] = None
    velocity_limit: Optional[float] = None
    
    # Physics properties
    damping: float = 0.1
    friction: float = 0.1
    
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TemplateLink:
    """Definition of a link in a robot template."""
    
    name: str
    part_type: PartType
    
    # Physical properties
    mass: float = 1.0
    inertia: Dict[str, float] = field(default_factory=lambda: {
        "ixx": 1.0, "ixy": 0.0, "ixz": 0.0,
        "iyy": 1.0, "iyz": 0.0, "izz": 1.0
    })
    
    # Geometry (will be replaced with actual part mesh)
    collision_geometry: Optional[str] = None
    visual_geometry: Optional[str] = None
    
    # Transform relative to parent
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    # Scaling factors for part fitting
    scale_factor: float = 1.0
    
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PartMatchingCriteria:
    """Criteria for matching parts to template links."""
    
    # Geometric criteria
    size_weight: float = 0.3
    shape_weight: float = 0.4
    position_weight: float = 0.3
    
    # Semantic criteria (if available)
    semantic_weight: float = 0.5
    
    # Thresholds
    min_match_score: float = 0.6
    max_scale_factor: float = 3.0
    min_scale_factor: float = 0.3


class RobotTemplate(ABC):
    """Abstract base class for robot templates."""
    
    def __init__(self, name: str, template_type: TemplateType):
        """Initialize robot template.
        
        Args:
            name: Name of the template.
            template_type: Type of robot template.
        """
        self.name = name
        self.template_type = template_type
        self.links: Dict[str, TemplateLink] = {}
        self.joints: Dict[str, TemplateJoint] = {}
        self.root_link: str = "base_link"
        self.matching_criteria = PartMatchingCriteria()
        self.metadata: Dict[str, Any] = {}
        
        # Build the template structure
        self._build_template()
    
    @abstractmethod
    def _build_template(self) -> None:
        """Build the template structure with links and joints."""
        pass
    
    def add_link(self, link: TemplateLink) -> None:
        """Add a link to the template.
        
        Args:
            link: Template link to add.
        """
        self.links[link.name] = link
    
    def add_joint(self, joint: TemplateJoint) -> None:
        """Add a joint to the template.
        
        Args:
            joint: Template joint to add.
        """
        self.joints[joint.name] = joint
    
    def get_links_by_part_type(self, part_type: PartType) -> List[TemplateLink]:
        """Get all links of a specific part type.
        
        Args:
            part_type: Type of part to find.
            
        Returns:
            List of matching template links.
        """
        return [link for link in self.links.values() if link.part_type == part_type]
    
    def get_expected_part_types(self) -> List[PartType]:
        """Get list of expected part types for this template.
        
        Returns:
            List of expected part types.
        """
        return list(set(link.part_type for link in self.links.values()))
    
    def validate_structure(self) -> bool:
        """Validate the template structure.
        
        Returns:
            True if structure is valid, False otherwise.
        """
        # Check if root link exists
        if self.root_link not in self.links:
            logger.error(f"Root link '{self.root_link}' not found in template")
            return False
        
        # Check joint connectivity
        for joint in self.joints.values():
            if joint.parent_link not in self.links:
                logger.error(f"Joint '{joint.name}' parent link '{joint.parent_link}' not found")
                return False
            if joint.child_link not in self.links:
                logger.error(f"Joint '{joint.name}' child link '{joint.child_link}' not found")
                return False
        
        return True
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert template to dictionary representation.
        
        Returns:
            Dictionary representation of the template.
        """
        return {
            "name": self.name,
            "template_type": self.template_type.value,
            "root_link": self.root_link,
            "links": {name: {
                "name": link.name,
                "part_type": link.part_type.value,
                "mass": link.mass,
                "inertia": link.inertia,
                "collision_geometry": link.collision_geometry,
                "visual_geometry": link.visual_geometry,
                "origin_xyz": link.origin_xyz,
                "origin_rpy": link.origin_rpy,
                "scale_factor": link.scale_factor,
                "metadata": link.metadata
            } for name, link in self.links.items()},
            "joints": {name: {
                "name": joint.name,
                "joint_type": joint.joint_type,
                "parent_link": joint.parent_link,
                "child_link": joint.child_link,
                "axis": joint.axis,
                "origin_xyz": joint.origin_xyz,
                "origin_rpy": joint.origin_rpy,
                "lower_limit": joint.lower_limit,
                "upper_limit": joint.upper_limit,
                "effort_limit": joint.effort_limit,
                "velocity_limit": joint.velocity_limit,
                "damping": joint.damping,
                "friction": joint.friction,
                "metadata": joint.metadata
            } for name, joint in self.joints.items()},
            "matching_criteria": {
                "size_weight": self.matching_criteria.size_weight,
                "shape_weight": self.matching_criteria.shape_weight,
                "position_weight": self.matching_criteria.position_weight,
                "semantic_weight": self.matching_criteria.semantic_weight,
                "min_match_score": self.matching_criteria.min_match_score,
                "max_scale_factor": self.matching_criteria.max_scale_factor,
                "min_scale_factor": self.matching_criteria.min_scale_factor
            },
            "metadata": self.metadata
        }
    
    def save(self, path: Path) -> None:
        """Save template to JSON file.
        
        Args:
            path: Path to save template to.
        """
        with open(path, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)


class TemplateLibrary:
    """Library for managing robot templates."""
    
    def __init__(self, templates_dir: Optional[Path] = None):
        """Initialize template library.
        
        Args:
            templates_dir: Directory containing template files.
        """
        self.templates: Dict[str, RobotTemplate] = {}
        self.templates_dir = templates_dir
        
        if templates_dir and templates_dir.exists():
            self._load_templates()
    
    def add_template(self, template: RobotTemplate) -> None:
        """Add a template to the library.
        
        Args:
            template: Robot template to add.
        """
        if not template.validate_structure():
            raise ValueError(f"Template '{template.name}' has invalid structure")
        
        self.templates[template.name] = template
        logger.info(f"Added template: {template.name} ({template.template_type.value})")
    
    def get_template(self, name: str) -> Optional[RobotTemplate]:
        """Get a template by name.
        
        Args:
            name: Name of the template.
            
        Returns:
            Robot template if found, None otherwise.
        """
        return self.templates.get(name)
    
    def get_templates_by_type(self, template_type: TemplateType) -> List[RobotTemplate]:
        """Get all templates of a specific type.
        
        Args:
            template_type: Type of templates to find.
            
        Returns:
            List of matching templates.
        """
        return [
            template for template in self.templates.values()
            if template.template_type == template_type
        ]
    
    def list_templates(self) -> List[str]:
        """Get list of all template names.
        
        Returns:
            List of template names.
        """
        return list(self.templates.keys())
    
    def save_all(self, directory: Path) -> None:
        """Save all templates to a directory.
        
        Args:
            directory: Directory to save templates to.
        """
        directory.mkdir(parents=True, exist_ok=True)
        
        for template in self.templates.values():
            template_path = directory / f"{template.name}.json"
            template.save(template_path)
    
    def _load_templates(self) -> None:
        """Load templates from the templates directory."""
        if not self.templates_dir:
            return
        
        for template_file in self.templates_dir.glob("*.json"):
            try:
                self._load_template_from_file(template_file)
            except Exception as e:
                logger.error(f"Failed to load template from {template_file}: {e}")
    
    def _load_template_from_file(self, path: Path) -> None:
        """Load a template from JSON file.
        
        Args:
            path: Path to template file.
        """
        # This would need to be implemented based on the specific template format
        # For now, we'll focus on creating templates programmatically
        pass