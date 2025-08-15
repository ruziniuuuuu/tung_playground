"""Pipeline orchestration and stage management."""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Type, TypeVar, Generic
from dataclasses import dataclass, field
from pathlib import Path
import asyncio
import logging
from enum import Enum

from .hero import Hero
from .exceptions import ProcessingError, ValidationError

logger = logging.getLogger(__name__)

T = TypeVar('T', bound='PipelineStage')


class StageStatus(str, Enum):
    """Status of a pipeline stage."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class StageResult:
    """Result of a pipeline stage execution."""
    
    stage_name: str
    status: StageStatus
    hero: Hero
    outputs: Dict[str, Any] = field(default_factory=dict)
    error: Optional[Exception] = None
    execution_time: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass 
class PipelineConfig:
    """Configuration for pipeline execution."""
    
    # Execution settings
    max_workers: int = 1
    timeout: Optional[float] = None
    retry_attempts: int = 0
    retry_delay: float = 1.0
    
    # Stage configuration
    stage_configs: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    
    # Validation settings
    validate_inputs: bool = True
    validate_outputs: bool = True
    
    # Logging settings
    log_level: str = "INFO"
    log_to_file: bool = False
    log_file: Optional[Path] = None
    
    def get_stage_config(self, stage_name: str) -> Dict[str, Any]:
        """Get configuration for a specific stage.
        
        Args:
            stage_name: Name of the pipeline stage.
            
        Returns:
            Configuration dictionary for the stage.
        """
        return self.stage_configs.get(stage_name, {})


class PipelineStage(ABC):
    """Abstract base class for pipeline stages."""
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the pipeline stage.
        
        Args:
            name: Name of the stage.
            config: Optional configuration dictionary.
        """
        self.name = name
        self.config = config or {}
        self.logger = logging.getLogger(f"{__name__}.{name}")
    
    @abstractmethod
    async def process(self, hero: Hero) -> StageResult:
        """Process a hero through this pipeline stage.
        
        Args:
            hero: Hero to process.
            
        Returns:
            Result of stage processing.
            
        Raises:
            ProcessingError: If processing fails.
            ValidationError: If validation fails.
        """
        pass
    
    @abstractmethod
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that the hero has required inputs for this stage.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid, False otherwise.
            
        Raises:
            ValidationError: If validation fails with details.
        """
        pass
    
    @abstractmethod
    def validate_outputs(self, result: StageResult) -> bool:
        """Validate that the stage produced expected outputs.
        
        Args:
            result: Result to validate.
            
        Returns:
            True if outputs are valid, False otherwise.
            
        Raises:
            ValidationError: If validation fails with details.
        """
        pass
    
    def get_config_value(self, key: str, default: Any = None) -> Any:
        """Get configuration value with dot notation support.
        
        Args:
            key: Configuration key.
            default: Default value if key not found.
            
        Returns:
            Configuration value or default.
        """
        keys = key.split('.')
        value = self.config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    async def _execute_with_retry(self, hero: Hero, max_attempts: int = 1, delay: float = 1.0) -> StageResult:
        """Execute stage with retry logic.
        
        Args:
            hero: Hero to process.
            max_attempts: Maximum number of attempts.
            delay: Delay between retries in seconds.
            
        Returns:
            Stage result.
            
        Raises:
            ProcessingError: If all attempts fail.
        """
        last_error = None
        
        for attempt in range(max_attempts):
            try:
                if attempt > 0:
                    self.logger.info(f"Retrying {self.name} (attempt {attempt + 1}/{max_attempts})")
                    await asyncio.sleep(delay)
                
                return await self.process(hero)
                
            except Exception as e:
                last_error = e
                self.logger.warning(f"Attempt {attempt + 1} failed for {self.name}: {e}")
                
                if attempt == max_attempts - 1:
                    break
        
        raise ProcessingError(
            f"Stage {self.name} failed after {max_attempts} attempts",
            details={"last_error": str(last_error)}
        )


class Pipeline:
    """Orchestrates the execution of multiple pipeline stages."""
    
    def __init__(self, name: str, config: Optional[PipelineConfig] = None):
        """Initialize the pipeline.
        
        Args:
            name: Name of the pipeline.
            config: Optional pipeline configuration.
        """
        self.name = name
        self.config = config or PipelineConfig()
        self.stages: List[PipelineStage] = []
        self.logger = logging.getLogger(f"{__name__}.{name}")
        
        # Set up logging
        self._setup_logging()
    
    def _setup_logging(self) -> None:
        """Set up logging configuration."""
        level = getattr(logging, self.config.log_level.upper(), logging.INFO)
        self.logger.setLevel(level)
        
        if self.config.log_to_file and self.config.log_file:
            handler = logging.FileHandler(self.config.log_file)
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
    
    def add_stage(self, stage: PipelineStage) -> 'Pipeline':
        """Add a stage to the pipeline.
        
        Args:
            stage: Pipeline stage to add.
            
        Returns:
            Self for method chaining.
        """
        # Apply stage-specific configuration
        stage_config = self.config.get_stage_config(stage.name)
        stage.config.update(stage_config)
        
        self.stages.append(stage)
        self.logger.info(f"Added stage: {stage.name}")
        return self
    
    def add_stages(self, stages: List[PipelineStage]) -> 'Pipeline':
        """Add multiple stages to the pipeline.
        
        Args:
            stages: List of pipeline stages to add.
            
        Returns:
            Self for method chaining.
        """
        for stage in stages:
            self.add_stage(stage)
        return self
    
    async def execute(self, hero: Hero) -> List[StageResult]:
        """Execute the pipeline on a hero.
        
        Args:
            hero: Hero to process through the pipeline.
            
        Returns:
            List of stage results.
            
        Raises:
            ProcessingError: If pipeline execution fails.
        """
        results: List[StageResult] = []
        
        self.logger.info(f"Starting pipeline {self.name} for hero {hero.name}")
        hero.add_processing_log("pipeline", f"Started pipeline: {self.name}")
        
        try:
            for i, stage in enumerate(self.stages):
                self.logger.info(f"Executing stage {i+1}/{len(self.stages)}: {stage.name}")
                
                # Validate inputs if configured
                if self.config.validate_inputs:
                    try:
                        stage.validate_inputs(hero)
                    except ValidationError as e:
                        result = StageResult(
                            stage_name=stage.name,
                            status=StageStatus.FAILED,
                            hero=hero,
                            error=e
                        )
                        results.append(result)
                        hero.add_processing_log(stage.name, f"Input validation failed: {e}")
                        break
                
                # Execute stage with retry logic
                try:
                    import time
                    start_time = time.time()
                    
                    result = await stage._execute_with_retry(
                        hero,
                        max_attempts=self.config.retry_attempts + 1,
                        delay=self.config.retry_delay
                    )
                    
                    result.execution_time = time.time() - start_time
                    
                    # Validate outputs if configured
                    if self.config.validate_outputs:
                        stage.validate_outputs(result)
                    
                    results.append(result)
                    hero.add_processing_log(
                        stage.name, 
                        f"Stage completed successfully in {result.execution_time:.2f}s"
                    )
                    
                    self.logger.info(f"Stage {stage.name} completed successfully")
                    
                except Exception as e:
                    self.logger.error(f"Stage {stage.name} failed: {e}")
                    result = StageResult(
                        stage_name=stage.name,
                        status=StageStatus.FAILED,
                        hero=hero,
                        error=e
                    )
                    results.append(result)
                    hero.add_processing_log(stage.name, f"Stage failed: {e}")
                    break
            
            # Check if all stages completed successfully
            if all(r.status == StageStatus.COMPLETED for r in results):
                hero.add_processing_log("pipeline", f"Pipeline {self.name} completed successfully")
                self.logger.info(f"Pipeline {self.name} completed successfully")
            else:
                hero.add_processing_log("pipeline", f"Pipeline {self.name} failed")
                self.logger.error(f"Pipeline {self.name} failed")
            
        except Exception as e:
            self.logger.error(f"Pipeline {self.name} execution failed: {e}")
            hero.add_processing_log("pipeline", f"Pipeline {self.name} execution failed: {e}")
            raise ProcessingError(f"Pipeline execution failed: {e}")
        
        return results
    
    async def execute_batch(self, heroes: List[Hero]) -> Dict[str, List[StageResult]]:
        """Execute the pipeline on multiple heroes in parallel.
        
        Args:
            heroes: List of heroes to process.
            
        Returns:
            Dictionary mapping hero IDs to their stage results.
        """
        self.logger.info(f"Starting batch execution for {len(heroes)} heroes")
        
        # Create semaphore to limit concurrent executions
        semaphore = asyncio.Semaphore(self.config.max_workers)
        
        async def execute_single(hero: Hero) -> tuple[str, List[StageResult]]:
            async with semaphore:
                results = await self.execute(hero)
                return hero.id, results
        
        # Execute all heroes concurrently
        tasks = [execute_single(hero) for hero in heroes]
        
        if self.config.timeout:
            results = await asyncio.wait_for(
                asyncio.gather(*tasks, return_exceptions=True),
                timeout=self.config.timeout
            )
        else:
            results = await asyncio.gather(*tasks, return_exceptions=True)
        
        # Process results
        batch_results = {}
        for result in results:
            if isinstance(result, Exception):
                self.logger.error(f"Batch execution error: {result}")
                continue
            
            hero_id, stage_results = result
            batch_results[hero_id] = stage_results
        
        self.logger.info(f"Batch execution completed for {len(batch_results)} heroes")
        return batch_results
    
    def get_stage(self, name: str) -> Optional[PipelineStage]:
        """Get a stage by name.
        
        Args:
            name: Name of the stage to find.
            
        Returns:
            Pipeline stage if found, None otherwise.
        """
        for stage in self.stages:
            if stage.name == name:
                return stage
        return None
    
    def remove_stage(self, name: str) -> bool:
        """Remove a stage by name.
        
        Args:
            name: Name of the stage to remove.
            
        Returns:
            True if stage was removed, False if not found.
        """
        for i, stage in enumerate(self.stages):
            if stage.name == name:
                del self.stages[i]
                self.logger.info(f"Removed stage: {name}")
                return True
        return False
    
    def clear_stages(self) -> None:
        """Remove all stages from the pipeline."""
        self.stages.clear()
        self.logger.info("Cleared all stages")
    
    @property
    def stage_count(self) -> int:
        """Get the number of stages in the pipeline."""
        return len(self.stages)
    
    @property
    def stage_names(self) -> List[str]:
        """Get list of stage names in execution order."""
        return [stage.name for stage in self.stages]