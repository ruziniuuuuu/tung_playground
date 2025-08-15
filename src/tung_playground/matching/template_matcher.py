"""Implementation of template-based part matching."""

import numpy as np
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
import time
import logging

from .base import PartMatcher, MatchingResult, PartMatch, PartGeometry
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError
from ..templates.base import RobotTemplate, TemplateLink, TemplateLibrary, TemplateType

logger = logging.getLogger(__name__)


class TemplatePartMatcher(PartMatcher):
    """Template-based part matcher using geometric and semantic features."""
    
    def __init__(self, name: str = "template_matcher", config: Optional[Dict[str, Any]] = None):
        """Initialize template matcher.
        
        Args:
            name: Name of the matcher.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        
        # Initialize template library
        self.template_library = TemplateLibrary()
        self._load_default_templates()
        
        # Matching parameters
        self.size_weight = self.get_config_value("size_weight", 0.3)
        self.shape_weight = self.get_config_value("shape_weight", 0.4)
        self.position_weight = self.get_config_value("position_weight", 0.2)
        self.semantic_weight = self.get_config_value("semantic_weight", 0.1)
        
        # Template selection
        self.preferred_template_type = self.get_config_value("preferred_template_type", None)
        self.auto_select_template = self.get_config_value("auto_select_template", True)
    
    def _load_default_templates(self) -> None:
        """Load default robot templates."""
        from ..templates.biped import BipedTemplate
        from ..templates.quadruped import QuadrupedTemplate
        
        try:
            self.template_library.add_template(BipedTemplate())
            self.template_library.add_template(QuadrupedTemplate())
            logger.info("Loaded default robot templates")
        except Exception as e:
            logger.error(f"Failed to load default templates: {e}")
    
    async def match_parts_to_template(self, hero: Hero) -> MatchingResult:
        """Match decomposed parts to a robot template.
        
        Args:
            hero: Hero with decomposed parts.
            
        Returns:
            Matching result with part-to-link assignments.
        """
        start_time = time.time()
        
        # Get parts
        parts_paths = hero.assets.get_asset(AssetType.PARTS)
        if not parts_paths:
            raise ProcessingError("No parts found for template matching")
        
        # Extract geometry for all parts
        part_geometries = {}
        for part_path in parts_paths:
            try:
                geometry = self.extract_part_geometry(part_path)
                part_geometries[part_path] = geometry
            except Exception as e:
                logger.warning(f"Failed to extract geometry from {part_path}: {e}")
                continue
        
        if not part_geometries:
            raise ProcessingError("No valid parts found for geometry extraction")
        
        # Select best template
        template = await self._select_best_template(part_geometries, hero)
        
        # Perform matching
        matches, unmatched_parts, unmatched_links = await self._match_parts_to_links(
            part_geometries, template
        )
        
        # Calculate overall statistics
        overall_score = self._calculate_overall_score(matches)
        coverage_ratio = len(matches) / len(template.links) if template.links else 0.0
        
        matching_time = time.time() - start_time
        
        return MatchingResult(
            template=template,
            matches=matches,
            unmatched_parts=unmatched_parts,
            unmatched_links=unmatched_links,
            overall_score=overall_score,
            coverage_ratio=coverage_ratio,
            matching_time=matching_time,
            metadata={
                "total_parts": len(parts_paths),
                "valid_parts": len(part_geometries),
                "template_selection": "auto" if self.auto_select_template else "manual",
                "matching_weights": {
                    "size": self.size_weight,
                    "shape": self.shape_weight,
                    "position": self.position_weight,
                    "semantic": self.semantic_weight
                }
            }
        )
    
    async def _select_best_template(
        self, 
        part_geometries: Dict[Path, PartGeometry], 
        hero: Hero
    ) -> RobotTemplate:
        """Select the best template based on part analysis.
        
        Args:
            part_geometries: Dictionary of part paths to geometries.
            hero: Hero being processed.
            
        Returns:
            Best matching robot template.
        """
        if self.preferred_template_type:
            # Use user-specified template type
            template_type = TemplateType(self.preferred_template_type)
            templates = self.template_library.get_templates_by_type(template_type)
            if templates:
                return templates[0]  # Return first template of this type
        
        if not self.auto_select_template:
            # Default to biped if no preference and auto-selection disabled
            templates = self.template_library.get_templates_by_type(TemplateType.BIPED)
            if templates:
                return templates[0]
        
        # Auto-select based on part analysis
        return await self._analyze_parts_for_template_selection(part_geometries)
    
    async def _analyze_parts_for_template_selection(
        self, 
        part_geometries: Dict[Path, PartGeometry]
    ) -> RobotTemplate:
        """Analyze parts to determine the most suitable template type.
        
        Args:
            part_geometries: Dictionary of part paths to geometries.
            
        Returns:
            Best matching robot template.
        """
        # Simple heuristic based on part count and shapes
        num_parts = len(part_geometries)
        
        # Analyze part shapes to guess body type
        elongated_parts = 0  # Potentially legs/arms
        compact_parts = 0    # Potentially torso/head
        
        for geometry in part_geometries.values():
            max_aspect = max(geometry.aspect_ratios)
            if max_aspect > 2.0:  # Elongated part
                elongated_parts += 1
            elif geometry.compactness > 0.7:  # Compact part
                compact_parts += 1
        
        # Decision logic
        if num_parts >= 8 and elongated_parts >= 4:
            # Likely quadruped (4 legs + body parts)
            templates = self.template_library.get_templates_by_type(TemplateType.QUADRUPED)
            if templates:
                logger.info(f"Auto-selected quadruped template based on {num_parts} parts")
                return templates[0]
        
        # Default to biped
        templates = self.template_library.get_templates_by_type(TemplateType.BIPED)
        if templates:
            logger.info(f"Auto-selected biped template based on {num_parts} parts")
            return templates[0]
        
        # Fallback - should not happen with default templates loaded
        raise ProcessingError("No suitable templates available")
    
    async def _match_parts_to_links(
        self, 
        part_geometries: Dict[Path, PartGeometry], 
        template: RobotTemplate
    ) -> Tuple[List[PartMatch], List[Path], List[TemplateLink]]:
        """Match parts to template links using Hungarian algorithm approach.
        
        Args:
            part_geometries: Dictionary of part paths to geometries.
            template: Robot template to match against.
            
        Returns:
            Tuple of (matches, unmatched_parts, unmatched_links).
        """
        # Calculate match scores for all part-link pairs
        score_matrix = []
        parts_list = list(part_geometries.keys())
        links_list = list(template.links.values())
        
        for part_path in parts_list:
            part_row = []
            geometry = part_geometries[part_path]
            
            for link in links_list:
                score, score_breakdown = self.compute_match_score(
                    geometry, link, template.matching_criteria
                )
                part_row.append((score, score_breakdown, part_path, link))
            
            score_matrix.append(part_row)
        
        # Simple greedy matching (can be improved with Hungarian algorithm)
        matches = []
        matched_parts = set()
        matched_links = set()
        
        # Sort all potential matches by score
        all_matches = []
        for i, part_row in enumerate(score_matrix):
            for j, (score, breakdown, part_path, link) in enumerate(part_row):
                if score >= self.min_match_score:
                    all_matches.append((score, breakdown, part_path, link))
        
        all_matches.sort(key=lambda x: x[0], reverse=True)  # Sort by score descending
        
        # Greedily assign matches
        for score, breakdown, part_path, link in all_matches:
            if part_path not in matched_parts and link not in matched_links:
                # Calculate scale factor and transforms
                scale_factor = self._calculate_scale_factor(
                    part_geometries[part_path], link
                )
                
                if (self.min_scale_factor <= scale_factor <= self.max_scale_factor):
                    match = PartMatch(
                        part_path=part_path,
                        template_link=link,
                        match_score=score,
                        scale_factor=scale_factor,
                        size_score=breakdown.get("size", 0.0),
                        shape_score=breakdown.get("shape", 0.0),
                        position_score=breakdown.get("position", 0.0),
                        semantic_score=breakdown.get("semantic", 0.0)
                    )
                    
                    matches.append(match)
                    matched_parts.add(part_path)
                    matched_links.add(link)
        
        # Identify unmatched parts and links
        unmatched_parts = [p for p in parts_list if p not in matched_parts]
        unmatched_links = [l for l in links_list if l not in matched_links]
        
        return matches, unmatched_parts, unmatched_links
    
    def extract_part_geometry(self, part_path: Path) -> PartGeometry:
        """Extract geometric properties from a part file.
        
        Args:
            part_path: Path to the part file (mesh).
            
        Returns:
            Geometric properties of the part.
        """
        # Mock implementation - in practice, this would use mesh processing libraries
        # like Open3D, trimesh, or similar to analyze the mesh geometry
        
        try:
            # Simulate geometry extraction
            # In a real implementation, you would:
            # 1. Load the mesh file (OBJ, PLY, STL, etc.)
            # 2. Compute bounding box, volume, surface area
            # 3. Calculate shape descriptors
            # 4. Extract semantic labels if available
            
            import random
            random.seed(hash(str(part_path)))  # Consistent "random" values for testing
            
            # Mock bounding box (simulate different part sizes)
            bbox_size = (
                random.uniform(0.05, 0.3),  # width
                random.uniform(0.05, 0.3),  # height  
                random.uniform(0.05, 0.4),  # depth
            )
            
            bbox_center = (
                random.uniform(-0.1, 0.1),  # x
                random.uniform(-0.1, 0.1),  # y
                random.uniform(-0.2, 0.2),  # z
            )
            
            # Mock volume and surface area
            volume = bbox_size[0] * bbox_size[1] * bbox_size[2] * random.uniform(0.3, 0.8)
            surface_area = 2 * (
                bbox_size[0] * bbox_size[1] + 
                bbox_size[1] * bbox_size[2] + 
                bbox_size[0] * bbox_size[2]
            ) * random.uniform(0.8, 1.2)
            
            # Mock aspect ratios
            aspect_ratios = (
                bbox_size[0] / bbox_size[1] if bbox_size[1] > 0 else 1.0,
                bbox_size[1] / bbox_size[2] if bbox_size[2] > 0 else 1.0,
                bbox_size[0] / bbox_size[2] if bbox_size[2] > 0 else 1.0,
            )
            
            # Mock compactness (how sphere-like)
            ideal_radius = (volume * 3.0 / (4.0 * np.pi)) ** (1.0/3.0)
            sphere_surface = 4.0 * np.pi * ideal_radius ** 2
            compactness = sphere_surface / surface_area if surface_area > 0 else 0.0
            
            # Mock semantic label (could come from part name or ML classifier)
            semantic_label = self._guess_semantic_label(part_path)
            
            return PartGeometry(
                bbox_size=bbox_size,
                bbox_center=bbox_center,
                volume=volume,
                surface_area=surface_area,
                aspect_ratios=aspect_ratios,
                compactness=compactness,
                semantic_label=semantic_label,
                confidence=random.uniform(0.6, 0.9),
                metadata={
                    "part_path": str(part_path),
                    "extraction_method": "mock"
                }
            )
            
        except Exception as e:
            raise ProcessingError(f"Failed to extract geometry from {part_path}: {e}")
    
    def _guess_semantic_label(self, part_path: Path) -> Optional[str]:
        """Guess semantic label from part file name.
        
        Args:
            part_path: Path to part file.
            
        Returns:
            Guessed semantic label if possible.
        """
        name_lower = part_path.stem.lower()
        
        # Simple keyword matching
        if any(word in name_lower for word in ['head', 'skull', 'face']):
            return 'head'
        elif any(word in name_lower for word in ['torso', 'body', 'chest', 'trunk']):
            return 'torso'
        elif any(word in name_lower for word in ['arm', 'shoulder', 'elbow']):
            return 'arm'
        elif any(word in name_lower for word in ['leg', 'thigh', 'knee', 'shin']):
            return 'leg'
        elif any(word in name_lower for word in ['hand', 'finger', 'palm']):
            return 'hand'
        elif any(word in name_lower for word in ['foot', 'toe', 'heel']):
            return 'foot'
        elif any(word in name_lower for word in ['tail']):
            return 'tail'
        
        return None
    
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
        scores = {}
        
        # Size compatibility score
        scores["size"] = self._compute_size_score(part_geometry, template_link)
        
        # Shape compatibility score  
        scores["shape"] = self._compute_shape_score(part_geometry, template_link)
        
        # Position/location score (based on expected placement)
        scores["position"] = self._compute_position_score(part_geometry, template_link)
        
        # Semantic compatibility score
        scores["semantic"] = self._compute_semantic_score(part_geometry, template_link)
        
        # Weighted overall score
        overall_score = (
            scores["size"] * self.size_weight +
            scores["shape"] * self.shape_weight + 
            scores["position"] * self.position_weight +
            scores["semantic"] * self.semantic_weight
        )
        
        return overall_score, scores
    
    def _compute_size_score(self, geometry: PartGeometry, link: TemplateLink) -> float:
        """Compute size compatibility score."""
        # Mock implementation - compare volumes/sizes
        part_volume = geometry.volume
        expected_volume = link.mass * 0.001  # Rough volume estimate from mass
        
        if expected_volume > 0:
            ratio = min(part_volume, expected_volume) / max(part_volume, expected_volume)
            return ratio ** 0.5  # Square root to be less harsh
        
        return 0.5  # Default score if no size info
    
    def _compute_shape_score(self, geometry: PartGeometry, link: TemplateLink) -> float:
        """Compute shape compatibility score."""
        # Use aspect ratios and compactness to assess shape compatibility
        
        # Expected shape characteristics for different part types
        expected_shapes = {
            'head': {'compactness': 0.8, 'max_aspect': 1.5},
            'torso': {'compactness': 0.6, 'max_aspect': 2.0},
            'arm': {'compactness': 0.3, 'max_aspect': 4.0},
            'leg': {'compactness': 0.3, 'max_aspect': 5.0},
            'hand': {'compactness': 0.5, 'max_aspect': 2.0},
            'foot': {'compactness': 0.4, 'max_aspect': 2.5},
        }
        
        part_type_str = link.part_type.value.split('_')[0]  # Remove left/right suffix
        expected = expected_shapes.get(part_type_str, {'compactness': 0.5, 'max_aspect': 2.0})
        
        # Score based on compactness similarity
        compactness_diff = abs(geometry.compactness - expected['compactness'])
        compactness_score = max(0.0, 1.0 - compactness_diff * 2.0)
        
        # Score based on aspect ratio
        max_aspect = max(geometry.aspect_ratios)
        aspect_diff = abs(max_aspect - expected['max_aspect']) / expected['max_aspect']
        aspect_score = max(0.0, 1.0 - aspect_diff)
        
        return (compactness_score + aspect_score) / 2.0
    
    def _compute_position_score(self, geometry: PartGeometry, link: TemplateLink) -> float:
        """Compute position compatibility score."""
        # Mock implementation - in practice this would consider:
        # 1. Expected position of the part in the decomposed mesh
        # 2. Expected position of the link in the template
        # 3. Spatial relationships between parts
        
        # For now, give a base score
        return 0.7
    
    def _compute_semantic_score(self, geometry: PartGeometry, link: TemplateLink) -> float:
        """Compute semantic compatibility score."""
        if not geometry.semantic_label:
            return 0.5  # Neutral score if no semantic info
        
        # Direct label matching
        part_type_str = link.part_type.value.split('_')[0]  # Remove left/right suffix
        
        if geometry.semantic_label.lower() == part_type_str.lower():
            return geometry.confidence
        
        # Partial matching for related terms
        semantic_groups = {
            'torso': ['body', 'chest', 'trunk'],
            'arm': ['shoulder', 'elbow', 'forearm'],
            'leg': ['thigh', 'knee', 'shin'],
        }
        
        for group_name, synonyms in semantic_groups.items():
            if part_type_str.lower() == group_name:
                if geometry.semantic_label.lower() in synonyms:
                    return geometry.confidence * 0.8
        
        return 0.2  # Low score for semantic mismatch
    
    def _calculate_scale_factor(self, geometry: PartGeometry, link: TemplateLink) -> float:
        """Calculate scale factor needed to fit part to link."""
        # Simple volume-based scaling
        part_volume = geometry.volume
        expected_volume = link.mass * 0.001  # Rough estimate
        
        if expected_volume > 0 and part_volume > 0:
            volume_ratio = expected_volume / part_volume
            scale_factor = volume_ratio ** (1.0/3.0)  # Cube root for 3D scaling
            return max(self.min_scale_factor, min(self.max_scale_factor, scale_factor))
        
        return 1.0  # Default scale
    
    def _calculate_overall_score(self, matches: List[PartMatch]) -> float:
        """Calculate overall matching score."""
        if not matches:
            return 0.0
        
        # Average of all match scores, weighted by link importance
        total_weight = 0.0
        weighted_score = 0.0
        
        for match in matches:
            # Give higher weight to critical parts
            weight = self._get_part_importance_weight(match.template_link.part_type)
            weighted_score += match.match_score * weight
            total_weight += weight
        
        return weighted_score / total_weight if total_weight > 0 else 0.0
    
    def _get_part_importance_weight(self, part_type) -> float:
        """Get importance weight for different part types."""
        importance_weights = {
            'torso': 2.0,      # Most important
            'head': 1.5,       # Important for identity
            'leg': 1.2,        # Important for mobility
            'arm': 1.0,        # Standard importance
            'hand': 0.8,       # Less critical
            'foot': 0.8,       # Less critical
            'tail': 0.5,       # Optional
        }
        
        part_type_str = part_type.value.split('_')[0]
        return importance_weights.get(part_type_str, 1.0)