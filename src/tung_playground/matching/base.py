"""Base classes for template matching system."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
import time

from ..core.pipeline import PipelineStage, StageResult, StageStatus
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError
from ..templates.base import RobotTemplate, TemplateLink, PartType


@dataclass
class PartGeometry:
    """Geometric properties of a part for matching."""
    
    # Bounding box
    bbox_size: Tuple[float, float, float]  # width, height, depth
    bbox_center: Tuple[float, float, float]  # x, y, z center
    
    # Volume and surface area
    volume: float
    surface_area: float
    
    # Shape descriptors
    aspect_ratios: Tuple[float, float, float]  # x/y, y/z, x/z ratios
    compactness: float  # measure of how sphere-like the part is
    
    # Optional semantic information
    semantic_label: Optional[str] = None
    confidence: float = 0.0
    
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PartMatch:
    """Represents a match between a part and a template link."""
    
    part_path: Path
    template_link: TemplateLink
    match_score: float
    scale_factor: float
    
    # Detailed scoring breakdown
    size_score: float = 0.0
    shape_score: float = 0.0
    position_score: float = 0.0
    semantic_score: float = 0.0
    
    # Transform information
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class MatchingResult:
    """Result of template matching process."""
    
    template: RobotTemplate
    matches: List[PartMatch]
    unmatched_parts: List[Path]
    unmatched_links: List[TemplateLink]
    
    # Overall matching statistics
    overall_score: float
    coverage_ratio: float  # ratio of matched links to total links
    matching_time: float
    
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def get_match_for_link(self, link_name: str) -> Optional[PartMatch]:
        """Get the match for a specific template link.
        
        Args:
            link_name: Name of the template link.
            
        Returns:
            PartMatch if found, None otherwise.
        """
        for match in self.matches:
            if match.template_link.name == link_name:
                return match
        return None
    
    def get_matches_for_part_type(self, part_type: PartType) -> List[PartMatch]:
        """Get all matches for a specific part type.
        
        Args:
            part_type: Type of part to find matches for.
            
        Returns:
            List of matching PartMatch objects.
        """
        return [
            match for match in self.matches
            if match.template_link.part_type == part_type
        ]
    
    def is_complete_match(self, min_coverage: float = 0.8) -> bool:
        """Check if the matching is reasonably complete.
        
        Args:
            min_coverage: Minimum coverage ratio required.
            
        Returns:
            True if matching meets coverage requirements.
        """
        return self.coverage_ratio >= min_coverage
    
    def get_critical_missing_parts(self) -> List[PartType]:
        """Get list of critical parts that are missing.
        
        Returns:
            List of critical part types without matches.
        """
        critical_parts = {PartType.TORSO, PartType.HEAD}
        
        # Add legs based on template type
        if self.template.template_type.value == "biped":
            critical_parts.update({PartType.LEG_LEFT, PartType.LEG_RIGHT})
        elif self.template.template_type.value == "quadruped":
            critical_parts.update({
                PartType.FRONT_LEG_LEFT, PartType.FRONT_LEG_RIGHT,
                PartType.BACK_LEG_LEFT, PartType.BACK_LEG_RIGHT
            })
        
        matched_types = {match.template_link.part_type for match in self.matches}
        return list(critical_parts - matched_types)


class PartMatcher(PipelineStage):
    """Abstract base class for template-based part matching."""
    
    PLUGIN_TYPE = "matching"
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the part matcher.
        
        Args:
            name: Name of the matcher.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.min_match_score = self.get_config_value("min_match_score", 0.6)
        self.min_coverage_ratio = self.get_config_value("min_coverage_ratio", 0.7)
        self.enable_semantic_matching = self.get_config_value("enable_semantic_matching", True)
        self.max_scale_factor = self.get_config_value("max_scale_factor", 3.0)
        self.min_scale_factor = self.get_config_value("min_scale_factor", 0.3)
    
    async def process(self, hero: Hero) -> StageResult:
        """Process hero through template matching.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Stage result with matching data.
        """
        start_time = time.time()
        
        try:
            # Perform template matching
            result = await self.match_parts_to_template(hero)
            
            # Update hero assets with matching result
            hero.assets.metadata["template_matching"] = {
                "matcher": self.name,
                "template_name": result.template.name,
                "template_type": result.template.template_type.value,
                "overall_score": result.overall_score,
                "coverage_ratio": result.coverage_ratio,
                "matched_parts": len(result.matches),
                "unmatched_parts": len(result.unmatched_parts),
                "matching_time": result.matching_time,
                **result.metadata
            }
            
            execution_time = time.time() - start_time
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.COMPLETED,
                hero=hero,
                outputs={
                    "matching_result": result,
                    "template": result.template,
                    "matches": result.matches,
                    "overall_score": result.overall_score,
                    "coverage_ratio": result.coverage_ratio
                },
                execution_time=execution_time,
                metadata=result.metadata
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"Template matching failed: {e}")
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.FAILED,
                hero=hero,
                error=e,
                execution_time=execution_time
            )
    
    @abstractmethod
    async def match_parts_to_template(self, hero: Hero) -> MatchingResult:
        """Match decomposed parts to a robot template.
        
        Args:
            hero: Hero with decomposed parts.
            
        Returns:
            Matching result with part-to-link assignments.
            
        Raises:
            ProcessingError: If matching fails.
        """
        pass
    
    @abstractmethod
    def extract_part_geometry(self, part_path: Path) -> PartGeometry:
        """Extract geometric properties from a part file.
        
        Args:
            part_path: Path to the part file (mesh).
            
        Returns:
            Geometric properties of the part.
            
        Raises:
            ProcessingError: If geometry extraction fails.
        """
        pass
    
    @abstractmethod
    def compute_match_score(
        self, 
        part_geometry: PartGeometry, 
        template_link: TemplateLink,
        matching_criteria: Any
    ) -> Tuple[float, Dict[str, float]]:
        """Compute match score between a part and template link.
        
        Args:
            part_geometry: Geometric properties of the part.
            template_link: Template link to match against.
            matching_criteria: Matching criteria from template.
            
        Returns:
            Tuple of (overall_score, score_breakdown).
        """
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required parts for matching.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if not hero.assets.has_asset(AssetType.PARTS):
            raise ValidationError("Hero must have decomposed parts for template matching")
        
        parts = hero.assets.get_asset(AssetType.PARTS)
        if not parts or len(parts) == 0:
            raise ValidationError("Hero must have at least one part for template matching")
        
        # Validate all part files exist
        for part_path in parts:
            if not part_path.exists():
                raise ValidationError(f"Part file does not exist: {part_path}")
        
        return True
    
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate template matching outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if "matching_result" not in result.outputs:
            raise ValidationError("Matching result must contain matching_result")
        
        matching_result = result.outputs["matching_result"]
        
        # Check if we have reasonable coverage
        if matching_result.coverage_ratio < self.min_coverage_ratio:
            self.logger.warning(
                f"Low coverage ratio: {matching_result.coverage_ratio:.2f} "
                f"(minimum: {self.min_coverage_ratio:.2f})"
            )
        
        # Check for critical missing parts
        critical_missing = matching_result.get_critical_missing_parts()
        if critical_missing:
            raise ValidationError(
                f"Critical parts missing: {[pt.value for pt in critical_missing]}"
            )
        
        # Validate match scores
        low_score_matches = [
            match for match in matching_result.matches 
            if match.match_score < self.min_match_score
        ]
        
        if low_score_matches:
            self.logger.warning(
                f"{len(low_score_matches)} matches have low confidence scores"
            )
        
        return True