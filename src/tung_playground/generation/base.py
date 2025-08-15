"""Base classes for image-to-3D generation."""

from abc import abstractmethod
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass
import time

from ..core.pipeline import PipelineStage, StageResult, StageStatus
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError
from ..utils.validation import validate_image_file


@dataclass
class GenerationResult:
    """Result of 3D generation process."""
    
    mesh_path: Path
    quality_score: float
    generation_time: float
    metadata: Dict[str, Any]


class Image3DGenerator(PipelineStage):
    """Abstract base class for image-to-3D generation."""
    
    PLUGIN_TYPE = "generation"
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the generator.
        
        Args:
            name: Name of the generator.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.output_format = self.get_config_value("output_format", "obj")
        self.quality_threshold = self.get_config_value("quality_threshold", 0.7)
    
    async def process(self, hero: Hero) -> StageResult:
        """Process hero through 3D generation.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Stage result with generated 3D mesh.
        """
        start_time = time.time()
        
        try:
            # Generate 3D mesh
            result = await self.generate_3d(hero)
            
            # Update hero assets
            hero.assets.set_asset(AssetType.MESH_3D, result.mesh_path)
            hero.assets.metadata["generation"] = {
                "generator": self.name,
                "quality_score": result.quality_score,
                "generation_time": result.generation_time,
                **result.metadata
            }
            
            execution_time = time.time() - start_time
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.COMPLETED,
                hero=hero,
                outputs={"mesh_path": result.mesh_path, "quality_score": result.quality_score},
                execution_time=execution_time,
                metadata=result.metadata
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"3D generation failed: {e}")
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.FAILED,
                hero=hero,
                error=e,
                execution_time=execution_time
            )
    
    @abstractmethod
    async def generate_3d(self, hero: Hero) -> GenerationResult:
        """Generate 3D mesh from hero's input image.
        
        Args:
            hero: Hero with input image.
            
        Returns:
            Generation result with mesh path and metadata.
            
        Raises:
            ProcessingError: If generation fails.
        """
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required input image.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if not hero.assets.has_asset(AssetType.INPUT_IMAGE):
            raise ValidationError("Hero must have input image for 3D generation")
        
        input_image = hero.assets.get_asset(AssetType.INPUT_IMAGE)
        validate_image_file(input_image, "Input image")
        
        return True
    
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate generation outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if "mesh_path" not in result.outputs:
            raise ValidationError("Generation result must contain mesh_path")
        
        mesh_path = result.outputs["mesh_path"]
        if not mesh_path.exists():
            raise ValidationError(f"Generated mesh file does not exist: {mesh_path}")
        
        if "quality_score" in result.outputs:
            quality = result.outputs["quality_score"]
            if quality < self.quality_threshold:
                raise ValidationError(f"Generated mesh quality {quality} below threshold {self.quality_threshold}")
        
        return True