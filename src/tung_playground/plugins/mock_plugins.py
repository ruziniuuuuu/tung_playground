"""Registration of all mock plugins for pipeline demonstration."""

import logging

from .registry import register_plugin

# Import all mock implementations
from ..generation.mock_generator import MockGenerator
from ..decomposition.mock_decomposer import MockDecomposer
from ..rigging.mock_rigger import MockRigger
from ..urdf.mock_builder import MockURDFBuilder
from ..simulation.mock_mujoco import MockMuJoCoAdapter
from ..training.mock_trainer import MockTrainer

logger = logging.getLogger(__name__)


def register_all_mock_plugins() -> None:
    """Register all mock plugins with the plugin system."""
    
    plugins_to_register = [
        # Generation plugins
        {
            "name": "mock_generator",
            "plugin_type": "generation", 
            "class_type": MockGenerator,
            "description": "Mock 3D generator that creates simple procedural meshes",
            "version": "1.0.0",
            "author": "Tung Playground"
        },
        
        # Decomposition plugins
        {
            "name": "mock_decomposer",
            "plugin_type": "decomposition",
            "class_type": MockDecomposer, 
            "description": "Mock part decomposer that splits meshes into semantic parts",
            "version": "1.0.0",
            "author": "Tung Playground"
        },
        
        # Rigging plugins
        {
            "name": "mock_rigger",
            "plugin_type": "rigging",
            "class_type": MockRigger,
            "description": "Mock auto rigger that creates simple biped skeletons",
            "version": "1.0.0", 
            "author": "Tung Playground"
        },
        
        # URDF plugins
        {
            "name": "mock_urdf_builder",
            "plugin_type": "urdf",
            "class_type": MockURDFBuilder,
            "description": "Mock URDF builder that creates complete robot descriptions",
            "version": "1.0.0",
            "author": "Tung Playground"
        },
        
        # Simulation plugins
        {
            "name": "mock_mujoco_adapter", 
            "plugin_type": "simulation",
            "class_type": MockMuJoCoAdapter,
            "description": "Mock MuJoCo adapter that creates basic simulation scenes",
            "version": "1.0.0",
            "author": "Tung Playground"
        },
        
        # Training plugins
        {
            "name": "mock_trainer",
            "plugin_type": "training",
            "class_type": MockTrainer,
            "description": "Mock RL trainer that creates simple random policies", 
            "version": "1.0.0",
            "author": "Tung Playground"
        }
    ]
    
    registered_count = 0
    
    for plugin_info in plugins_to_register:
        try:
            register_plugin(**plugin_info)
            registered_count += 1
            logger.info(f"Registered mock plugin: {plugin_info['plugin_type']}.{plugin_info['name']}")
        except Exception as e:
            logger.error(f"Failed to register plugin {plugin_info['name']}: {e}")
    
    logger.info(f"Successfully registered {registered_count}/{len(plugins_to_register)} mock plugins")


# Auto-register plugins when module is imported
register_all_mock_plugins()