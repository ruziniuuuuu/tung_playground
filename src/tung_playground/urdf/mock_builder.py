"""Mock URDF builder for pipeline demonstration."""

import asyncio
import time
import json
from pathlib import Path
from typing import Dict, Any, List
import logging

from .base import URDFGenerator, URDFResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError

logger = logging.getLogger(__name__)


class MockURDFBuilder(URDFGenerator):
    """Mock URDF builder that creates a complete robot description."""
    
    PLUGIN_NAME = "mock_urdf_builder"
    PLUGIN_TYPE = "urdf"
    VERSION = "1.0.0"
    AUTHOR = "Tung Playground"
    
    def __init__(self, name: str = "mock_urdf_builder", config: Dict[str, Any] = None):
        """Initialize mock URDF builder.
        
        Args:
            name: Name of the builder.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.robot_name = self.get_config_value("robot_name", "tung_hero")
    
    async def generate_urdf(self, hero: Hero) -> URDFResult:
        """Generate a complete URDF file.
        
        Args:
            hero: Hero with all required assets.
            
        Returns:
            URDF result with file path and metadata.
            
        Raises:
            ProcessingError: If URDF generation fails.
        """
        start_time = time.time()
        
        try:
            # Simulate processing time
            await asyncio.sleep(0.5)
            
            # Load skeleton data
            skeleton_path = hero.assets.get_asset(AssetType.SKELETON)
            with open(skeleton_path, 'r') as f:
                skeleton_data = json.load(f)
            
            # Create URDF file
            urdf_path = hero.hero_dir / "robot.urdf"
            
            # Generate URDF content
            urdf_content = self._create_urdf_content(hero, skeleton_data)
            
            # Write URDF file
            with open(urdf_path, 'w') as f:
                f.write(urdf_content)
            
            generation_time = time.time() - start_time
            
            # Count links and joints
            link_count = skeleton_data.get("joint_count", 0)  # Each joint becomes a link
            joint_count = len(skeleton_data.get("bones", []))  # Each bone becomes a joint
            
            metadata = {
                "physics_engine": self.physics_engine,
                "robot_name": self.robot_name,
                "collision_generated": self.auto_generate_collisions,
                "mesh_decimation": self.mesh_decimation,
                "skeleton_type": skeleton_data.get("skeleton_type", "unknown")
            }
            
            self.logger.info(f"Generated URDF with {link_count} links and {joint_count} joints")
            
            return URDFResult(
                urdf_path=urdf_path,
                link_count=link_count,
                joint_count=joint_count,
                generation_time=generation_time,
                metadata=metadata
            )
            
        except Exception as e:
            raise ProcessingError(f"Mock URDF generation failed: {e}")
    
    def _create_urdf_content(self, hero: Hero, skeleton_data: Dict[str, Any]) -> str:
        """Create URDF XML content.
        
        Args:
            hero: Hero with assets.
            skeleton_data: Skeleton structure data.
            
        Returns:
            Complete URDF XML content.
        """
        # Get relative paths for mesh files
        mesh_path = hero.assets.get_asset(AssetType.MESH_3D)
        parts_paths = hero.assets.get_asset(AssetType.PARTS) or []
        
        # Mesh file relative to URDF location
        mesh_rel_path = mesh_path.name if mesh_path else "generated_mesh.obj"
        
        # Start URDF content
        urdf_lines = [
            '<?xml version="1.0"?>',
            f'<robot name="{self.robot_name}">',
            ''
        ]
        
        # Add base link (root of the robot)
        urdf_lines.extend([
            '  <!-- Base Link -->',
            '  <link name="base_link">',
            '    <visual>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            '      <geometry>',
            f'        <mesh filename="{mesh_rel_path}" scale="1 1 1"/>',
            '      </geometry>',
            '      <material name="hero_material">',
            '        <color rgba="0.8 0.6 0.2 1.0"/>',
            '      </material>',
            '    </visual>',
            '    <collision>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            '      <geometry>',
            f'        <mesh filename="{mesh_rel_path}" scale="1 1 1"/>',
            '      </geometry>',
            '    </collision>',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            '      <mass value="1.0"/>',
            '      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>',
            '    </inertial>',
            '  </link>',
            ''
        ])
        
        # Process skeleton joints to create links and joints
        joints = skeleton_data.get("joints", {})
        bones = skeleton_data.get("bones", [])
        
        # Create links for each joint (except root)
        for joint_name, joint_data in joints.items():
            if joint_name == "root":
                continue
                
            urdf_lines.extend([
                f'  <!-- Link for {joint_name} -->',
                f'  <link name="{joint_name}_link">',
                '    <visual>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                '      <geometry>',
                '        <box size="0.05 0.05 0.05"/>',
                '      </geometry>',
                '      <material name="joint_material">',
                '        <color rgba="0.2 0.2 0.8 1.0"/>',
                '      </material>',
                '    </visual>',
                '    <collision>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                '      <geometry>',
                '        <box size="0.05 0.05 0.05"/>',
                '      </geometry>',
                '    </collision>',
                '    <inertial>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                '      <mass value="0.1"/>',
                '      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>',
                '    </inertial>',
                '  </link>',
                ''
            ])
        
        # Create joints (connections between links)
        for bone in bones:
            start_joint = bone["start_joint"]
            end_joint = bone["end_joint"]
            joint_name = bone["name"]
            
            # Determine parent and child links
            parent_link = "base_link" if start_joint == "root" else f"{start_joint}_link"
            child_link = f"{end_joint}_link"
            
            # Get joint position
            joint_pos = joints.get(end_joint, {}).get("position", [0, 0, 0])
            
            urdf_lines.extend([
                f'  <!-- Joint: {joint_name} -->',
                f'  <joint name="{joint_name}" type="revolute">',
                f'    <parent link="{parent_link}"/>',
                f'    <child link="{child_link}"/>',
                f'    <origin xyz="{joint_pos[0]:.3f} {joint_pos[1]:.3f} {joint_pos[2]:.3f}" rpy="0 0 0"/>',
                '    <axis xyz="0 0 1"/>',
                '    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="10"/>',
                '    <dynamics damping="0.1" friction="0.1"/>',
                '  </joint>',
                ''
            ])
        
        # Close robot tag
        urdf_lines.append('</robot>')
        
        return '\n'.join(urdf_lines)
    
    def _calculate_inertia_matrix(self, mass: float, dimensions: List[float]) -> Dict[str, float]:
        """Calculate inertia matrix for a box.
        
        Args:
            mass: Mass of the object.
            dimensions: [width, height, depth] dimensions.
            
        Returns:
            Inertia matrix components.
        """
        w, h, d = dimensions
        
        ixx = (mass / 12.0) * (h*h + d*d)
        iyy = (mass / 12.0) * (w*w + d*d)
        izz = (mass / 12.0) * (w*w + h*h)
        
        return {
            "ixx": ixx,
            "iyy": iyy,
            "izz": izz,
            "ixy": 0.0,
            "ixz": 0.0,
            "iyz": 0.0
        }