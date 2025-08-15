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

# Template system
from .templates import (
    RobotTemplate,
    TemplateType,
    PartType,
    TemplateLibrary,
    BipedTemplate,
    QuadrupedTemplate,
)

# Template matching
from .matching import (
    PartMatcher,
    MatchingResult,
    PartMatch,
    TemplatePartMatcher,
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
    # Templates
    "RobotTemplate",
    "TemplateType",
    "PartType", 
    "TemplateLibrary",
    "BipedTemplate",
    "QuadrupedTemplate",
    # Template Matching
    "PartMatcher",
    "MatchingResult",
    "PartMatch",
    "TemplatePartMatcher",
    # Visualization
    "VISUALIZATION_AVAILABLE",
    # Helper functions
    "create_hero",
    "create_pipeline", 
    "create_template_pipeline",
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


def create_template_pipeline(
    template_type: str = "auto",
    physics_engine: str = "mujoco"
) -> Pipeline:
    """Create a template-based generation pipeline.
    
    Args:
        template_type: Template type to use ("auto", "biped", "quadruped").
        physics_engine: Physics engine for URDF ("mujoco", "pybullet").
        
    Returns:
        Configured template-based Pipeline instance.
    """
    from .generation.mock_generator import MockGenerator
    from .decomposition.mock_decomposer import MockDecomposer  
    from .urdf.template_builder import TemplateURDFBuilder
    
    config = PipelineConfig(
        stage_configs={
            "template_matcher": {
                "preferred_template_type": template_type if template_type != "auto" else None,
                "auto_select_template": template_type == "auto",
                "min_match_score": 0.5,
                "min_coverage_ratio": 0.6
            },
            "template_urdf_builder": {
                "physics_engine": physics_engine,
                "generate_collision_meshes": True,
                "mesh_scale_factor": 1.0
            }
        }
    )
    
    pipeline = Pipeline("template_based_generation", config)
    
    # Add stages
    pipeline.add_stages([
        MockGenerator("3d_generation"),
        MockDecomposer("decomposition"),  
        TemplatePartMatcher("template_matcher"),
        TemplateURDFBuilder("template_urdf_builder"),
    ])
    
    return pipeline