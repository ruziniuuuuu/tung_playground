"""Base classes for RL training."""

from abc import abstractmethod
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass
import time

from ..core.pipeline import PipelineStage, StageResult, StageStatus
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError


@dataclass
class TrainingResult:
    """Result of RL training process."""
    
    policy_path: Path
    training_episodes: int
    final_reward: float
    training_time: float
    metadata: Dict[str, Any]


class PolicyTrainer(PipelineStage):
    """Abstract base class for RL policy training."""
    
    PLUGIN_TYPE = "training"
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the trainer.
        
        Args:
            name: Name of the trainer.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.algorithm = self.get_config_value("algorithm", "ppo")
        self.total_timesteps = self.get_config_value("total_timesteps", 1000000)
        self.learning_rate = self.get_config_value("learning_rate", 0.0003)
    
    async def process(self, hero: Hero) -> StageResult:
        """Process hero through RL training.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Stage result with trained policy.
        """
        start_time = time.time()
        
        try:
            # Train policy
            result = await self.train_policy(hero)
            
            # Update hero assets
            hero.assets.set_asset(AssetType.TRAINED_POLICY, result.policy_path)
            hero.assets.metadata["training"] = {
                "trainer": self.name,
                "algorithm": self.algorithm,
                "training_episodes": result.training_episodes,
                "final_reward": result.final_reward,
                "training_time": result.training_time,
                **result.metadata
            }
            
            execution_time = time.time() - start_time
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.COMPLETED,
                hero=hero,
                outputs={
                    "policy_path": result.policy_path,
                    "training_episodes": result.training_episodes,
                    "final_reward": result.final_reward
                },
                execution_time=execution_time,
                metadata=result.metadata
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            self.logger.error(f"RL training failed: {e}")
            
            return StageResult(
                stage_name=self.name,
                status=StageStatus.FAILED,
                hero=hero,
                error=e,
                execution_time=execution_time
            )
    
    @abstractmethod
    async def train_policy(self, hero: Hero) -> TrainingResult:
        """Train RL policy for the hero in simulation.
        
        Args:
            hero: Hero with simulation model.
            
        Returns:
            Training result with policy path and metrics.
            
        Raises:
            ProcessingError: If training fails.
        """
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required simulation model.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if not hero.assets.has_asset(AssetType.SIMULATION_MODEL):
            raise ValidationError("Hero must have simulation model for RL training")
        
        model_path = hero.assets.get_asset(AssetType.SIMULATION_MODEL)
        if not model_path.exists():
            raise ValidationError(f"Simulation model file does not exist: {model_path}")
        
        return True
    
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate training outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        if "policy_path" not in result.outputs:
            raise ValidationError("Training result must contain policy_path")
        
        policy_path = result.outputs["policy_path"]
        if not policy_path.exists():
            raise ValidationError(f"Policy file does not exist: {policy_path}")
        
        return True