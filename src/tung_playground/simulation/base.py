"""Base classes for simulation integration."""

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
class SimulationResult:
    """Result of simulation setup process."""
    
    simulation_model_path: Path
    environment_type: str
    setup_time: float
    metadata: Dict[str, Any]


class SimulationAdapter(PipelineStage):
    """Abstract base class for simulation integration."""
    
    PLUGIN_TYPE = "simulation"
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the simulation adapter.
        
        Args:
            name: Name of the adapter.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.default_environment = self.get_config_value("default_environment", "mujoco")
        self.physics_timestep = self.get_config_value("physics_timestep", 0.002)
    
    async def process(self, hero: Hero) -> StageResult:
        """Process hero through simulation setup.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Stage result with simulation model.
        """
        start_time = time.time()
        
        try:
            # Setup simulation
            result = await self.setup_simulation(hero)
            
            # Update hero assets
            hero.assets.set_asset(AssetType.SIMULATION_MODEL, result.simulation_model_path)
            hero.assets.metadata["simulation"] = {
                "adapter": self.name,
                "environment_type": result.environment_type,
                "physics_timestep": self.physics_timestep,
                "setup_time": result.setup_time,
                **result.metadata
            }
            
            execution_time = time.time() - start_time
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.COMPLETED,
                hero=hero,
                outputs={
                    "simulation_model_path": result.simulation_model_path,
                    "environment_type": result.environment_type
                },
                execution_time=execution_time,
                metadata=result.metadata
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"Simulation setup failed: {e}")
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.FAILED,
                hero=hero,
                error=e,
                execution_time=execution_time
            )
    
    @abstractmethod
    async def setup_simulation(self, hero: Hero) -> SimulationResult:
        """Setup simulation environment for the hero.
        
        Args:
            hero: Hero with URDF model.
            
        Returns:
            Simulation result with model path and metadata.
            
        Raises:
            ProcessingError: If simulation setup fails.
        """
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required URDF for simulation.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if not hero.assets.has_asset(AssetType.URDF):
            raise ValidationError("Hero must have URDF for simulation setup")
        
        urdf_path = hero.assets.get_asset(AssetType.URDF)
        validate_urdf_file(urdf_path, "Hero URDF")
        
        return True
    
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate simulation setup outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if "simulation_model_path" not in result.outputs:
            raise ValidationError("Simulation result must contain simulation_model_path")
        
        model_path = result.outputs["simulation_model_path"]
        if not model_path.exists():
            raise ValidationError(f"Simulation model file does not exist: {model_path}")
        
        return True