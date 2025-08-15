"""Base classes for automatic rigging and skeleton generation."""

from abc import abstractmethod
from pathlib import Path
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
import time

from ..core.pipeline import PipelineStage, StageResult, StageStatus
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError


@dataclass
class RiggingResult:
    """Result of rigging and skeleton generation process."""
    
    skeleton_path: Path
    joint_count: int
    bone_count: int
    rigging_time: float
    metadata: Dict[str, Any]


class AutoRigger(PipelineStage):
    """Abstract base class for automatic rigging."""
    
    PLUGIN_TYPE = "rigging"
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the rigger.
        
        Args:
            name: Name of the rigger.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.auto_detect_joints = self.get_config_value("auto_detect_joints", True)
        self.max_joints = self.get_config_value("max_joints", 50)
        self.symmetry_detection = self.get_config_value("symmetry_detection", True)
    
    async def process(self, hero: Hero) -> StageResult:
        """Process hero through rigging and skeleton generation.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Stage result with skeleton data.
        """
        start_time = time.time()
        
        try:
            # Generate skeleton
            result = await self.generate_skeleton(hero)
            
            # Update hero assets
            hero.assets.set_asset(AssetType.SKELETON, result.skeleton_path)
            hero.assets.metadata["rigging"] = {
                "rigger": self.name,
                "joint_count": result.joint_count,
                "bone_count": result.bone_count,
                "rigging_time": result.rigging_time,
                **result.metadata
            }
            
            execution_time = time.time() - start_time
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.COMPLETED,
                hero=hero,
                outputs={
                    "skeleton_path": result.skeleton_path,
                    "joint_count": result.joint_count,
                    "bone_count": result.bone_count
                },
                execution_time=execution_time,
                metadata=result.metadata
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"Rigging failed: {e}")
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.FAILED,
                hero=hero,
                error=e,
                execution_time=execution_time
            )
    
    @abstractmethod
    async def generate_skeleton(self, hero: Hero) -> RiggingResult:
        """Generate skeleton and rigging data from mesh and parts.
        
        Args:
            hero: Hero with mesh and parts.
            
        Returns:
            Rigging result with skeleton path and metadata.
            
        Raises:
            ProcessingError: If rigging fails.
        """
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required mesh and parts.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if not hero.assets.has_asset(AssetType.MESH_3D):
            raise ValidationError("Hero must have 3D mesh for rigging")
        
        if not hero.assets.has_asset(AssetType.PARTS):
            raise ValidationError("Hero must have decomposed parts for rigging")
        
        parts = hero.assets.get_asset(AssetType.PARTS)
        if not parts or len(parts) == 0:
            raise ValidationError("Hero must have at least one part for rigging")
        
        return True
    
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate rigging outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if "skeleton_path" not in result.outputs:
            raise ValidationError("Rigging result must contain skeleton_path")
        
        skeleton_path = result.outputs["skeleton_path"]
        if not skeleton_path.exists():
            raise ValidationError(f"Skeleton file does not exist: {skeleton_path}")
        
        joint_count = result.outputs.get("joint_count", 0)
        if joint_count == 0:
            raise ValidationError("Skeleton must have at least one joint")
        
        if joint_count > self.max_joints:
            raise ValidationError(f"Skeleton has {joint_count} joints, maximum is {self.max_joints}")
        
        return True