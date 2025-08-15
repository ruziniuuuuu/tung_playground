"""Mock auto rigger for pipeline demonstration."""

import asyncio
import time
import json
from pathlib import Path
from typing import Dict, Any, List
import logging

from .base import AutoRigger, RiggingResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError

logger = logging.getLogger(__name__)


class MockRigger(AutoRigger):
    """Mock auto rigger that creates a simple biped skeleton."""
    
    PLUGIN_NAME = "mock_rigger"
    PLUGIN_TYPE = "rigging"
    VERSION = "1.0.0"
    AUTHOR = "Tung Playground"
    
    def __init__(self, name: str = "mock_rigger", config: Dict[str, Any] = None):
        """Initialize mock rigger.
        
        Args:
            name: Name of the rigger.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.skeleton_type = self.get_config_value("skeleton_type", "biped")
    
    async def generate_skeleton(self, hero: Hero) -> RiggingResult:
        """Generate a simple biped skeleton.
        
        Args:
            hero: Hero with mesh and parts.
            
        Returns:
            Rigging result with skeleton data.
            
        Raises:
            ProcessingError: If rigging fails.
        """
        start_time = time.time()
        
        try:
            # Simulate processing time
            await asyncio.sleep(0.6)
            
            # Create skeleton file
            skeleton_path = hero.hero_dir / "skeleton.json"
            
            # Generate skeleton data
            skeleton_data = self._create_biped_skeleton()
            
            # Save skeleton to JSON
            with open(skeleton_path, 'w') as f:
                json.dump(skeleton_data, f, indent=2)
            
            rigging_time = time.time() - start_time
            
            joint_count = len(skeleton_data["joints"])
            bone_count = len(skeleton_data["bones"])
            
            metadata = {
                "skeleton_type": self.skeleton_type,
                "auto_detected": self.auto_detect_joints,
                "symmetry_applied": self.symmetry_detection,
                "parts_analyzed": len(hero.assets.get_asset(AssetType.PARTS) or [])
            }
            
            self.logger.info(f"Generated skeleton with {joint_count} joints and {bone_count} bones")
            
            return RiggingResult(
                skeleton_path=skeleton_path,
                joint_count=joint_count,
                bone_count=bone_count,
                rigging_time=rigging_time,
                metadata=metadata
            )
            
        except Exception as e:
            raise ProcessingError(f"Mock rigging failed: {e}")
    
    def _create_biped_skeleton(self) -> Dict[str, Any]:
        """Create a simple biped skeleton structure.
        
        Returns:
            Skeleton data dictionary.
        """
        # Define joint hierarchy and positions
        joints = {
            "root": {
                "position": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": None,
                "children": ["pelvis"]
            },
            "pelvis": {
                "position": [0.0, 0.8, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "root",
                "children": ["spine", "left_hip", "right_hip"]
            },
            "spine": {
                "position": [0.0, 1.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "pelvis",
                "children": ["chest"]
            },
            "chest": {
                "position": [0.0, 1.2, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "spine",
                "children": ["neck", "left_shoulder", "right_shoulder"]
            },
            "neck": {
                "position": [0.0, 1.5, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "chest",
                "children": ["head"]
            },
            "head": {
                "position": [0.0, 1.7, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "neck",
                "children": []
            },
            # Left arm
            "left_shoulder": {
                "position": [-0.2, 1.3, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "chest",
                "children": ["left_upper_arm"]
            },
            "left_upper_arm": {
                "position": [-0.3, 1.1, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "left_shoulder",
                "children": ["left_lower_arm"]
            },
            "left_lower_arm": {
                "position": [-0.3, 0.7, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "left_upper_arm",
                "children": ["left_hand"]
            },
            "left_hand": {
                "position": [-0.3, 0.4, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "left_lower_arm",
                "children": []
            },
            # Right arm (mirrored)
            "right_shoulder": {
                "position": [0.2, 1.3, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "chest",
                "children": ["right_upper_arm"]
            },
            "right_upper_arm": {
                "position": [0.3, 1.1, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "right_shoulder",
                "children": ["right_lower_arm"]
            },
            "right_lower_arm": {
                "position": [0.3, 0.7, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "right_upper_arm",
                "children": ["right_hand"]
            },
            "right_hand": {
                "position": [0.3, 0.4, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "right_lower_arm",
                "children": []
            },
            # Left leg
            "left_hip": {
                "position": [-0.1, 0.7, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "pelvis",
                "children": ["left_upper_leg"]
            },
            "left_upper_leg": {
                "position": [-0.1, 0.4, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "left_hip",
                "children": ["left_lower_leg"]
            },
            "left_lower_leg": {
                "position": [-0.1, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "left_upper_leg",
                "children": ["left_foot"]
            },
            "left_foot": {
                "position": [-0.1, -0.3, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "left_lower_leg",
                "children": []
            },
            # Right leg (mirrored)
            "right_hip": {
                "position": [0.1, 0.7, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "pelvis",
                "children": ["right_upper_leg"]
            },
            "right_upper_leg": {
                "position": [0.1, 0.4, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "right_hip",
                "children": ["right_lower_leg"]
            },
            "right_lower_leg": {
                "position": [0.1, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "right_upper_leg",
                "children": ["right_foot"]
            },
            "right_foot": {
                "position": [0.1, -0.3, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "parent": "right_lower_leg",
                "children": []
            }
        }
        
        # Create bones (connections between joints)
        bones = []
        for joint_name, joint_data in joints.items():
            if joint_data["parent"]:
                bones.append({
                    "name": f"{joint_data['parent']}_to_{joint_name}",
                    "start_joint": joint_data["parent"],
                    "end_joint": joint_name,
                    "length": self._calculate_bone_length(
                        joints[joint_data["parent"]]["position"],
                        joint_data["position"]
                    )
                })
        
        return {
            "skeleton_type": "biped",
            "joint_count": len(joints),
            "bone_count": len(bones),
            "joints": joints,
            "bones": bones,
            "metadata": {
                "generated_by": "MockRigger",
                "version": "1.0.0",
                "coordinate_system": "Y-up",
                "units": "meters"
            }
        }
    
    def _calculate_bone_length(self, start_pos: List[float], end_pos: List[float]) -> float:
        """Calculate distance between two 3D points.
        
        Args:
            start_pos: Start position [x, y, z].
            end_pos: End position [x, y, z].
            
        Returns:
            Distance between points.
        """
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        dz = end_pos[2] - start_pos[2]
        
        return (dx*dx + dy*dy + dz*dz) ** 0.5