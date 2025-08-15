"""Tung Playground: AIGC Hero Simulation Framework.

A comprehensive framework for generating AI heroes from images and training them
in physics simulators using reinforcement learning.
"""

__version__ = "0.0.1"

# Core exports
from .core import (
    Hero,
    HeroAssets,
    HeroStatus,
    AssetType,
    Pipeline,
    PipelineStage,
    PipelineConfig,
    StageResult,
    StageStatus,
    TungPlaygroundError,
    ValidationError,
    ProcessingError,
)

# Utility exports
from .utils import (
    ConfigManager,
    load_config,
    setup_logging,
    get_logger,
    FileManager,
    validate_image_file,
    validate_mesh_file,
)

# Plugin system
from .plugins import (
    plugin_registry,
    register_plugin,
    get_plugin,
    list_plugins,
)

# Visualization (optional)
try:
    from .visualization import URDFVisualizer
    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False

# Main modules (import on demand to avoid circular imports)
__all__ = [
    # Core
    "Hero",
    "HeroAssets", 
    "HeroStatus",
    "AssetType",
    "Pipeline",
    "PipelineStage", 
    "PipelineConfig",
    "StageResult",
    "StageStatus",
    "TungPlaygroundError",
    "ValidationError",
    "ProcessingError",
    # Utils
    "ConfigManager",
    "load_config",
    "setup_logging",
    "get_logger", 
    "FileManager",
    "validate_image_file",
    "validate_mesh_file",
    # Plugins
    "plugin_registry",
    "register_plugin",
    "get_plugin",
    "list_plugins",
    # Visualization
    "VISUALIZATION_AVAILABLE",
    # Version
    "__version__",
]

# Add URDFVisualizer to __all__ if available
if VISUALIZATION_AVAILABLE:
    __all__.append("URDFVisualizer")


def create_hero(name: str, image_path: str, hero_dir: str = None) -> Hero:
    """Create a new hero from an input image.
    
    Args:
        name: Name for the hero.
        image_path: Path to input image.
        hero_dir: Optional directory for hero files (defaults to heroes/{name}).
        
    Returns:
        Configured Hero instance.
    """
    from pathlib import Path
    
    if hero_dir is None:
        hero_dir = Path("heroes") / name
    
    hero = Hero(
        name=name,
        hero_dir=Path(hero_dir)
    )
    
    # Set input image
    hero.assets.set_asset(AssetType.INPUT_IMAGE, Path(image_path))
    
    return hero


def create_pipeline(config_name: str = "default") -> Pipeline:
    """Create a pipeline with default configuration.
    
    Args:
        config_name: Name of configuration to load.
        
    Returns:
        Configured Pipeline instance.
    """
    config_dict = load_config(config_name)
    pipeline_config = PipelineConfig(**config_dict.get("pipeline", {}))
    
    return Pipeline("default", pipeline_config)