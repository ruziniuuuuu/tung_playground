"""Mock RL trainer for pipeline demonstration."""

import asyncio
import time
import pickle
from pathlib import Path
from typing import Dict, Any
import logging
import random

from .base import PolicyTrainer, TrainingResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError

logger = logging.getLogger(__name__)


class MockTrainer(PolicyTrainer):
    """Mock RL trainer that creates a simple random policy."""
    
    PLUGIN_NAME = "mock_trainer"
    PLUGIN_TYPE = "training"
    VERSION = "1.0.0"
    AUTHOR = "Tung Playground"
    
    def __init__(self, name: str = "mock_trainer", config: Dict[str, Any] = None):
        """Initialize mock trainer.
        
        Args:
            name: Name of the trainer.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.mock_episodes = self.get_config_value("mock_episodes", 100)
    
    async def train_policy(self, hero: Hero) -> TrainingResult:
        """Train a simple random policy.
        
        Args:
            hero: Hero with simulation model.
            
        Returns:
            Training result with policy file.
            
        Raises:
            ProcessingError: If training fails.
        """
        start_time = time.time()
        
        try:
            # Simulate training time (much shorter than real training)
            await asyncio.sleep(2.0)
            
            # Create policy file
            policy_path = hero.hero_dir / "policy.pkl"
            
            # Create a simple mock policy
            mock_policy = self._create_mock_policy()
            
            # Save policy
            with open(policy_path, 'wb') as f:
                pickle.dump(mock_policy, f)
            
            training_time = time.time() - start_time
            
            # Mock training metrics
            final_reward = random.uniform(100, 500)  # Mock final reward
            
            metadata = {
                "algorithm": self.algorithm,
                "learning_rate": self.learning_rate,
                "total_timesteps": self.total_timesteps,
                "policy_type": "random",
                "convergence_achieved": True
            }
            
            self.logger.info(f"Completed training with final reward: {final_reward:.2f}")
            
            return TrainingResult(
                policy_path=policy_path,
                training_episodes=self.mock_episodes,
                final_reward=final_reward,
                training_time=training_time,
                metadata=metadata
            )
            
        except Exception as e:
            raise ProcessingError(f"Mock training failed: {e}")
    
    def _create_mock_policy(self) -> Dict[str, Any]:
        """Create a simple mock policy.
        
        Returns:
            Mock policy data structure.
        """
        return {
            "policy_type": "random",
            "algorithm": self.algorithm,
            "version": "1.0.0",
            "action_space": {
                "type": "continuous",
                "shape": [6],  # 6 DOF actions
                "low": [-1.0] * 6,
                "high": [1.0] * 6
            },
            "observation_space": {
                "type": "continuous",
                "shape": [24],  # Joint positions, velocities, etc.
                "low": [-10.0] * 24,
                "high": [10.0] * 24
            },
            "weights": {
                "layer1": [[random.uniform(-1, 1) for _ in range(24)] for _ in range(64)],
                "layer2": [[random.uniform(-1, 1) for _ in range(64)] for _ in range(32)],
                "output": [[random.uniform(-1, 1) for _ in range(32)] for _ in range(6)]
            },
            "training_stats": {
                "episodes": self.mock_episodes,
                "total_timesteps": self.total_timesteps,
                "final_reward": random.uniform(100, 500),
                "learning_rate": self.learning_rate
            }
        }