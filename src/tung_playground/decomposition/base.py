"""Base classes for part decomposition."""

from abc import abstractmethod
from pathlib import Path
from typing import Optional, Dict, Any, List
from dataclasses import dataclass
import time

from ..core.pipeline import PipelineStage, StageResult, StageStatus
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError
from ..utils.validation import validate_mesh_file


@dataclass
class DecompositionResult:
    """Result of part decomposition process."""
    
    parts_paths: List[Path]
    part_names: List[str]
    decomposition_time: float
    metadata: Dict[str, Any]


class PartDecomposer(PipelineStage):
    """Abstract base class for part decomposition."""
    
    PLUGIN_TYPE = "decomposition"
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the decomposer.
        
        Args:
            name: Name of the decomposer.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.min_parts = self.get_config_value("min_parts", 2)
        self.max_parts = self.get_config_value("max_parts", 20)
        self.quality_threshold = self.get_config_value("quality_threshold", 0.8)
    
    async def process(self, hero: Hero) -> StageResult:
        """Process hero through part decomposition.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Stage result with decomposed parts.
        """
        start_time = time.time()
        
        try:
            # Decompose mesh into parts
            result = await self.decompose_parts(hero)
            
            # Update hero assets
            hero.assets.set_asset(AssetType.PARTS, result.parts_paths)
            hero.assets.metadata["decomposition"] = {
                "decomposer": self.name,
                "part_count": len(result.parts_paths),
                "part_names": result.part_names,
                "decomposition_time": result.decomposition_time,
                **result.metadata
            }
            
            execution_time = time.time() - start_time
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.COMPLETED,
                hero=hero,
                outputs={
                    "parts_paths": result.parts_paths,
                    "part_names": result.part_names,
                    "part_count": len(result.parts_paths)
                },
                execution_time=execution_time,
                metadata=result.metadata
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"Part decomposition failed: {e}")
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.FAILED,
                hero=hero,
                error=e,
                execution_time=execution_time
            )
    
    @abstractmethod
    async def decompose_parts(self, hero: Hero) -> DecompositionResult:
        """Decompose 3D mesh into semantic parts.
        
        Args:
            hero: Hero with 3D mesh.
            
        Returns:
            Decomposition result with part paths and metadata.
            
        Raises:
            ProcessingError: If decomposition fails.
        """
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required 3D mesh.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if not hero.assets.has_asset(AssetType.MESH_3D):
            raise ValidationError("Hero must have 3D mesh for part decomposition")
        
        mesh_path = hero.assets.get_asset(AssetType.MESH_3D)
        validate_mesh_file(mesh_path, "3D mesh")
        
        return True
    
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate decomposition outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if "parts_paths" not in result.outputs:
            raise ValidationError("Decomposition result must contain parts_paths")
        
        parts_paths = result.outputs["parts_paths"]
        if not parts_paths:
            raise ValidationError("Decomposition must produce at least one part")
        
        if len(parts_paths) < self.min_parts:
            raise ValidationError(f"Decomposition produced {len(parts_paths)} parts, minimum is {self.min_parts}")
        
        if len(parts_paths) > self.max_parts:
            raise ValidationError(f"Decomposition produced {len(parts_paths)} parts, maximum is {self.max_parts}")
        
        # Validate all part files exist
        for part_path in parts_paths:
            if not part_path.exists():
                raise ValidationError(f"Part file does not exist: {part_path}")
        
        return True