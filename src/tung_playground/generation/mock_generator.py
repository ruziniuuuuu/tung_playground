"""Mock 3D generator for pipeline demonstration."""

import asyncio
import time
from pathlib import Path
from typing import Dict, Any
import logging

from .base import Image3DGenerator, GenerationResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError

logger = logging.getLogger(__name__)


class MockGenerator(Image3DGenerator):
    """Mock 3D generator that creates a simple procedural humanoid mesh."""
    
    PLUGIN_NAME = "mock_generator"
    PLUGIN_TYPE = "generation"
    VERSION = "1.0.0"
    AUTHOR = "Tung Playground"
    
    def __init__(self, name: str = "mock_generator", config: Dict[str, Any] = None):
        """Initialize mock generator.
        
        Args:
            name: Name of the generator.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.mesh_scale = self.get_config_value("mesh_scale", 1.0)
        self.add_texture = self.get_config_value("add_texture", True)
    
    async def generate_3d(self, hero: Hero) -> GenerationResult:
        """Generate a simple procedural 3D mesh.
        
        Args:
            hero: Hero with input image.
            
        Returns:
            Generation result with mesh path.
            
        Raises:
            ProcessingError: If generation fails.
        """
        start_time = time.time()
        
        try:
            # Simulate processing time
            await asyncio.sleep(1.0)
            
            # Create output mesh file
            mesh_path = hero.hero_dir / "generated_mesh.obj"
            
            # Generate simple humanoid OBJ file
            self._create_simple_humanoid_mesh(mesh_path)
            
            generation_time = time.time() - start_time
            quality_score = 0.85  # Mock quality score
            
            metadata = {
                "method": "procedural_humanoid",
                "scale": self.mesh_scale,
                "vertices": 156,  # Mock vertex count
                "faces": 280,     # Mock face count
                "input_image_size": self._get_image_size(hero.assets.get_asset(AssetType.INPUT_IMAGE))
            }
            
            self.logger.info(f"Generated 3D mesh: {mesh_path}")
            
            return GenerationResult(
                mesh_path=mesh_path,
                quality_score=quality_score,
                generation_time=generation_time,
                metadata=metadata
            )
            
        except Exception as e:
            raise ProcessingError(f"Mock 3D generation failed: {e}")
    
    def _create_simple_humanoid_mesh(self, output_path: Path) -> None:
        """Create a simple humanoid mesh in OBJ format.
        
        Args:
            output_path: Path to save the OBJ file.
        """
        # Simple humanoid mesh data (vertices and faces)
        # Generate clean OBJ without inline comments to avoid parsing issues
        obj_content = '''v 0.0 1.8 0.0
v 0.0 1.6 0.0
v 0.15 1.7 0.0
v -0.15 1.7 0.0
v 0.0 1.4 0.0
v 0.2 1.2 0.0
v -0.2 1.2 0.0
v 0.2 0.8 0.0
v -0.2 0.8 0.0
v 0.0 0.6 0.0
v 0.4 1.1 0.0
v 0.4 0.7 0.0
v 0.4 0.4 0.0
v -0.4 1.1 0.0
v -0.4 0.7 0.0
v -0.4 0.4 0.0
v 0.1 0.6 0.0
v 0.1 0.2 0.0
v 0.1 -0.2 0.0
v 0.1 -0.3 0.0
v -0.1 0.6 0.0
v -0.1 0.2 0.0
v -0.1 -0.2 0.0
v -0.1 -0.3 0.0
f 1 2 3
f 1 3 4
f 2 4 3
f 2 1 4
f 5 6 7
f 6 8 9
f 7 9 8
f 8 10 9
f 6 11 12
f 11 12 13
f 7 14 15
f 14 15 16
f 17 18 19
f 18 19 20
f 21 22 23
f 22 23 24
'''
        
        # Write OBJ file
        with open(output_path, 'w') as f:
            f.write(obj_content)
    
    def _get_image_size(self, image_path: Path) -> tuple[int, int]:
        """Get image dimensions.
        
        Args:
            image_path: Path to image file.
            
        Returns:
            Image size as (width, height).
        """
        try:
            from PIL import Image
            with Image.open(image_path) as img:
                return img.size
        except Exception:
            # Return default size if PIL not available
            return (512, 512)