"""Mock MuJoCo adapter for pipeline demonstration."""

import asyncio
import time
from pathlib import Path
from typing import Dict, Any
import logging

from .base import SimulationAdapter, SimulationResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError

logger = logging.getLogger(__name__)


class MockMuJoCoAdapter(SimulationAdapter):
    """Mock MuJoCo adapter that creates a basic simulation scene."""
    
    PLUGIN_NAME = "mock_mujoco_adapter"
    PLUGIN_TYPE = "simulation"
    VERSION = "1.0.0"
    AUTHOR = "Tung Playground"
    
    def __init__(self, name: str = "mock_mujoco_adapter", config: Dict[str, Any] = None):
        """Initialize mock MuJoCo adapter.
        
        Args:
            name: Name of the adapter.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.scene_type = self.get_config_value("scene_type", "basic_ground")
    
    async def setup_simulation(self, hero: Hero) -> SimulationResult:
        """Create a MuJoCo scene with the hero.
        
        Args:
            hero: Hero with URDF model.
            
        Returns:
            Simulation result with scene file.
            
        Raises:
            ProcessingError: If simulation setup fails.
        """
        start_time = time.time()
        
        try:
            # Simulate processing time
            await asyncio.sleep(0.4)
            
            # Create MuJoCo scene file
            scene_path = hero.hero_dir / "scene.xml"
            
            # Generate MuJoCo XML scene
            scene_content = self._create_mujoco_scene(hero)
            
            # Write scene file
            with open(scene_path, 'w') as f:
                f.write(scene_content)
            
            setup_time = time.time() - start_time
            
            metadata = {
                "physics_engine": "mujoco",
                "scene_type": self.scene_type,
                "timestep": self.physics_timestep,
                "urdf_included": True
            }
            
            self.logger.info(f"Created MuJoCo scene: {scene_path}")
            
            return SimulationResult(
                simulation_model_path=scene_path,
                environment_type="mujoco",
                setup_time=setup_time,
                metadata=metadata
            )
            
        except Exception as e:
            raise ProcessingError(f"Mock MuJoCo setup failed: {e}")
    
    def _create_mujoco_scene(self, hero: Hero) -> str:
        """Create MuJoCo XML scene content.
        
        Args:
            hero: Hero with assets.
            
        Returns:
            Complete MuJoCo XML scene.
        """
        urdf_path = hero.assets.get_asset(AssetType.URDF)
        urdf_rel_path = urdf_path.name if urdf_path else "robot.urdf"
        
        scene_xml = f'''<mujoco model="{hero.name}_scene">
  
  <!-- Compiler settings -->
  <compiler angle="radian" coordinate="local" inertiafromgeom="true"/>
  
  <!-- Size and timestep -->
  <size nconmax="50" njmax="100" nstack="300000"/>
  <option timestep="{self.physics_timestep}" iterations="50"/>
  
  <!-- Visual settings -->
  <visual>
    <rgba haze="0.3 0.3 0.3 1"/>
    <quality shadowsize="2048"/>
    <map force="0.1" zfar="30"/>
  </visual>
  
  <!-- Default classes -->
  <default>
    <light cutoff="100" exponent="1"/>
    <geom rgba="0.8 0.6 .4 1" margin="0.001" condim="3" conaffinity="1" friction="1 0.1 0.1"/>
    <motor ctrlrange="-1 1" ctrllimited="true"/>
  </default>
  
  <!-- Assets -->
  <asset>
    <texture builtin="gradient" height="100" rgb1=".4 .5 .6" rgb2="0 0 0" type="skybox" width="100"/>
    <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" type="cube" width="127"/>
    <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
    <material name="MatPlane" reflectance="0.5" shininess="1" specular="1" texrepeat="60 60" texture="texplane"/>
    <material name="geom" texture="texgeom" texuniform="true"/>
  </asset>
  
  <!-- World body -->
  <worldbody>
    <!-- Lighting -->
    <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
    
    <!-- Ground plane -->
    <geom conaffinity="1" condim="3" material="MatPlane" name="floor" pos="0 0 0" rgba="0.8 0.9 0.8 1" size="40 40 40" type="plane"/>
    
    <!-- Include hero robot -->
    <body name="{hero.name}" pos="0 0 0.5">
      <!-- Include URDF here or define robot geometry -->
      <geom name="{hero.name}_base" type="box" size="0.1 0.1 0.2" rgba="0.8 0.6 0.2 1"/>
      <joint name="{hero.name}_free" type="free"/>
    </body>
    
    <!-- Additional scene objects -->
    <body name="target" pos="2 0 0.1">
      <geom name="target_geom" type="sphere" size="0.1" rgba="1 0 0 1"/>
    </body>
  </worldbody>
  
  <!-- Actuators (if needed) -->
  <actuator>
    <!-- Add motors/actuators for joints here -->
  </actuator>
  
</mujoco>'''
        
        return scene_xml