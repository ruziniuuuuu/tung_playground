"""Base classes for URDF generation."""

from abc import abstractmethod
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass
import time

from ..core.pipeline import PipelineStage, StageResult, StageStatus
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError
from ..utils.validation import validate_urdf_file


@dataclass
class URDFResult:
    """Result of URDF generation process."""
    
    urdf_path: Path
    link_count: int
    joint_count: int
    generation_time: float
    metadata: Dict[str, Any]


class URDFGenerator(PipelineStage):
    """Abstract base class for URDF generation."""
    
    PLUGIN_TYPE = "urdf"
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the URDF generator.
        
        Args:
            name: Name of the generator.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.physics_engine = self.get_config_value("physics_engine", "mujoco")
        self.auto_generate_collisions = self.get_config_value("auto_generate_collisions", True)
        self.mesh_decimation = self.get_config_value("mesh_decimation", 0.5)
    
    async def process(self, hero: Hero) -> StageResult:
        """Process hero through URDF generation.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Stage result with URDF file.
        """
        start_time = time.time()
        
        try:
            # Generate URDF
            result = await self.generate_urdf(hero)
            
            # Update hero assets
            hero.assets.set_asset(AssetType.URDF, result.urdf_path)
            hero.assets.metadata["urdf"] = {
                "generator": self.name,
                "link_count": result.link_count,
                "joint_count": result.joint_count,
                "physics_engine": self.physics_engine,
                "generation_time": result.generation_time,
                **result.metadata
            }
            
            execution_time = time.time() - start_time
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.COMPLETED,
                hero=hero,
                outputs={
                    "urdf_path": result.urdf_path,
                    "link_count": result.link_count,
                    "joint_count": result.joint_count
                },
                execution_time=execution_time,
                metadata=result.metadata
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"URDF generation failed: {e}")
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.FAILED,
                hero=hero,
                error=e,
                execution_time=execution_time
            )
    
    @abstractmethod
    async def generate_urdf(self, hero: Hero) -> URDFResult:
        """Generate URDF from mesh, parts, and skeleton.
        
        Args:
            hero: Hero with all required assets.
            
        Returns:
            URDF result with file path and metadata.
            
        Raises:
            ProcessingError: If URDF generation fails.
        """
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required assets for URDF generation.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        required_assets = [AssetType.MESH_3D, AssetType.PARTS, AssetType.SKELETON]
        
        for asset_type in required_assets:
            if not hero.assets.has_asset(asset_type):
                raise ValidationError(f"Hero must have {asset_type.value} for URDF generation")
        
        # Validate skeleton file format
        skeleton_path = hero.assets.get_asset(AssetType.SKELETON)
        if not skeleton_path.suffix.lower() == '.json':
            raise ValidationError(f"Skeleton file must be JSON format, got: {skeleton_path.suffix}")
        
        return True
    
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate URDF generation outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if "urdf_path" not in result.outputs:
            raise ValidationError("URDF result must contain urdf_path")
        
        urdf_path = result.outputs["urdf_path"]
        validate_urdf_file(urdf_path, "Generated URDF")
        
        link_count = result.outputs.get("link_count", 0)
        joint_count = result.outputs.get("joint_count", 0)
        
        if link_count == 0:
            raise ValidationError("URDF must have at least one link")
        
        if joint_count == 0:
            self.logger.warning("URDF has no joints - this may be intentional for a single rigid body")
        
        return True