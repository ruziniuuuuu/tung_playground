"""Template-based URDF generator that assembles robots from matched parts."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
import time
import logging
import shutil

from .base import URDFGenerator, URDFResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError
from ..matching.base import MatchingResult, PartMatch
from ..templates.base import RobotTemplate, TemplateLink, TemplateJoint

logger = logging.getLogger(__name__)


class TemplateURDFBuilder(URDFGenerator):
    """URDF generator that assembles robots from template matching results."""
    
    def __init__(self, name: str = "template_urdf_builder", config: Optional[Dict[str, Any]] = None):
        """Initialize template URDF builder.
        
        Args:
            name: Name of the builder.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        
        # URDF generation settings
        self.generate_collision_meshes = self.get_config_value("generate_collision_meshes", True)
        self.simplify_collision_meshes = self.get_config_value("simplify_collision_meshes", True)
        self.mesh_scale_factor = self.get_config_value("mesh_scale_factor", 1.0)
        self.default_material_color = self.get_config_value("default_material_color", [0.8, 0.8, 0.8, 1.0])
        
        # Physics settings
        self.default_damping = self.get_config_value("default_damping", 0.1)
        self.default_friction = self.get_config_value("default_friction", 0.1)
        self.gravity_compensation = self.get_config_value("gravity_compensation", 0.0)
    
    async def generate_urdf(self, hero: Hero) -> URDFResult:
        """Generate URDF from template matching results.
        
        Args:
            hero: Hero with template matching results.
            
        Returns:
            URDF result with file path and metadata.
        """
        start_time = time.time()
        
        # Get matching result from hero metadata
        matching_result = self._extract_matching_result(hero)
        
        # Create URDF directory
        urdf_dir = hero.hero_dir / "urdf"
        urdf_dir.mkdir(exist_ok=True)
        
        # Copy and prepare mesh files
        meshes_dir = urdf_dir / "meshes"
        meshes_dir.mkdir(exist_ok=True)
        mesh_mapping = await self._prepare_mesh_files(matching_result, meshes_dir)
        
        # Generate URDF XML
        urdf_path = urdf_dir / f"{hero.name}.urdf"
        urdf_root = self._build_urdf_xml(matching_result, mesh_mapping, hero.name)
        
        # Write URDF file
        self._write_urdf_file(urdf_root, urdf_path)
        
        # Generate additional files (materials, config, etc.)
        await self._generate_additional_files(urdf_dir, matching_result, hero)
        
        generation_time = time.time() - start_time
        
        # Count links and joints
        link_count = len(matching_result.matches)
        joint_count = len(matching_result.template.joints)
        
        return URDFResult(
            urdf_path=urdf_path,
            link_count=link_count,
            joint_count=joint_count,
            generation_time=generation_time,
            metadata={
                "template_name": matching_result.template.name,
                "template_type": matching_result.template.template_type.value,
                "matching_score": matching_result.overall_score,
                "coverage_ratio": matching_result.coverage_ratio,
                "mesh_files": len(mesh_mapping),
                "generation_method": "template_based",
                "physics_engine": self.physics_engine
            }
        )
    
    def _extract_matching_result(self, hero: Hero) -> MatchingResult:
        """Extract matching result from hero processing history.
        
        Args:
            hero: Hero with matching results.
            
        Returns:
            Template matching result.
        """
        # Check if matching result is stored in assets metadata
        if "template_matching" not in hero.assets.metadata:
            raise ProcessingError("Hero must have template matching results for URDF generation")
        
        # In a real implementation, the matching result would be stored and retrieved
        # For now, we'll create a mock result based on available information
        raise ProcessingError(
            "Template matching result extraction not implemented. "
            "The matching result should be passed through the pipeline context."
        )
    
    async def _prepare_mesh_files(
        self, 
        matching_result: MatchingResult, 
        meshes_dir: Path
    ) -> Dict[str, str]:
        """Prepare mesh files for URDF inclusion.
        
        Args:
            matching_result: Template matching result.
            meshes_dir: Directory to store mesh files.
            
        Returns:
            Dictionary mapping template links to mesh file paths.
        """
        mesh_mapping = {}
        
        for match in matching_result.matches:
            # Copy original mesh file
            original_mesh = match.part_path
            mesh_name = f"{match.template_link.name}.{original_mesh.suffix[1:]}"
            target_mesh = meshes_dir / mesh_name
            
            try:
                shutil.copy2(original_mesh, target_mesh)
                
                # Apply scaling if needed
                if abs(match.scale_factor - 1.0) > 0.01:
                    await self._scale_mesh_file(target_mesh, match.scale_factor)
                
                # Generate collision mesh if requested
                collision_mesh = None
                if self.generate_collision_meshes:
                    collision_mesh = await self._generate_collision_mesh(
                        target_mesh, match.template_link.name
                    )
                
                mesh_mapping[match.template_link.name] = {
                    "visual": f"meshes/{mesh_name}",
                    "collision": f"meshes/{collision_mesh.name}" if collision_mesh else f"meshes/{mesh_name}",
                    "scale_factor": match.scale_factor
                }
                
            except Exception as e:
                logger.warning(f"Failed to prepare mesh for {match.template_link.name}: {e}")
                # Use basic geometry as fallback
                mesh_mapping[match.template_link.name] = self._generate_fallback_geometry(
                    match.template_link
                )
        
        return mesh_mapping
    
    async def _scale_mesh_file(self, mesh_path: Path, scale_factor: float) -> None:
        """Scale mesh file by the given factor.
        
        Args:
            mesh_path: Path to mesh file.
            scale_factor: Scaling factor to apply.
        """
        # Mock implementation - in practice, use mesh processing library
        # like Open3D, trimesh, or meshlab to scale the mesh
        logger.info(f"Scaling mesh {mesh_path.name} by factor {scale_factor}")
        pass
    
    async def _generate_collision_mesh(self, visual_mesh: Path, link_name: str) -> Optional[Path]:
        """Generate simplified collision mesh from visual mesh.
        
        Args:
            visual_mesh: Path to visual mesh file.
            link_name: Name of the template link.
            
        Returns:
            Path to collision mesh file if generated.
        """
        if not self.simplify_collision_meshes:
            return None
        
        # Mock implementation - in practice, use mesh decimation/convex hull
        collision_name = f"{link_name}_collision.{visual_mesh.suffix[1:]}"
        collision_path = visual_mesh.parent / collision_name
        
        try:
            # Copy as simplified mesh (would be replaced with actual simplification)
            shutil.copy2(visual_mesh, collision_path)
            logger.info(f"Generated collision mesh: {collision_name}")
            return collision_path
        except Exception as e:
            logger.warning(f"Failed to generate collision mesh for {link_name}: {e}")
            return None
    
    def _generate_fallback_geometry(self, link: TemplateLink) -> Dict[str, str]:
        """Generate fallback geometry for links without proper meshes.
        
        Args:
            link: Template link needing fallback geometry.
            
        Returns:
            Geometry specification for the link.
        """
        # Estimate dimensions based on link properties and part type
        part_type = link.part_type.value.split('_')[0]
        
        geometry_specs = {
            "head": {"type": "sphere", "radius": "0.1"},
            "torso": {"type": "box", "size": "0.3 0.2 0.4"},
            "arm": {"type": "cylinder", "radius": "0.05", "length": "0.25"},
            "leg": {"type": "cylinder", "radius": "0.06", "length": "0.35"},
            "hand": {"type": "box", "size": "0.08 0.04 0.12"},
            "foot": {"type": "box", "size": "0.12 0.06 0.08"},
        }
        
        spec = geometry_specs.get(part_type, {"type": "box", "size": "0.1 0.1 0.1"})
        
        return {
            "visual": spec,
            "collision": spec,
            "scale_factor": 1.0
        }
    
    def _build_urdf_xml(
        self, 
        matching_result: MatchingResult, 
        mesh_mapping: Dict[str, Any],
        robot_name: str
    ) -> ET.Element:
        """Build URDF XML from template and matching results.
        
        Args:
            matching_result: Template matching result.
            mesh_mapping: Dictionary mapping links to mesh files.
            robot_name: Name of the robot.
            
        Returns:
            Root XML element of the URDF.
        """
        # Create root robot element
        robot = ET.Element("robot", name=robot_name)
        
        # Add materials
        self._add_materials(robot)
        
        # Add links
        for match in matching_result.matches:
            link_elem = self._create_link_element(match, mesh_mapping)
            robot.append(link_elem)
        
        # Add joints
        template = matching_result.template
        for joint in template.joints.values():
            # Only add joint if both parent and child links are matched
            if (self._has_matched_link(joint.parent_link, matching_result) and 
                self._has_matched_link(joint.child_link, matching_result)):
                joint_elem = self._create_joint_element(joint)
                robot.append(joint_elem)
        
        return robot
    
    def _add_materials(self, robot: ET.Element) -> None:
        """Add material definitions to URDF.
        
        Args:
            robot: Root robot XML element.
        """
        # Default material
        material = ET.SubElement(robot, "material", name="default_material")
        color = ET.SubElement(material, "color")
        color.set("rgba", " ".join(map(str, self.default_material_color)))
        
        # Part-specific materials
        part_colors = {
            "head": [0.9, 0.8, 0.7, 1.0],      # Skin tone
            "torso": [0.6, 0.6, 0.8, 1.0],     # Blue-ish
            "arm": [0.7, 0.9, 0.7, 1.0],       # Green-ish
            "leg": [0.8, 0.7, 0.6, 1.0],       # Brown-ish
            "hand": [0.9, 0.8, 0.7, 1.0],      # Skin tone
            "foot": [0.4, 0.4, 0.4, 1.0],      # Dark gray
        }
        
        for part_type, color_rgba in part_colors.items():
            material = ET.SubElement(robot, "material", name=f"{part_type}_material")
            color = ET.SubElement(material, "color")
            color.set("rgba", " ".join(map(str, color_rgba)))
    
    def _create_link_element(self, match: PartMatch, mesh_mapping: Dict[str, Any]) -> ET.Element:
        """Create URDF link element from part match.
        
        Args:
            match: Part match with template link.
            mesh_mapping: Mesh file mapping.
            
        Returns:
            XML link element.
        """
        link = ET.Element("link", name=match.template_link.name)
        
        # Visual geometry
        visual = ET.SubElement(link, "visual")
        visual_geom = ET.SubElement(visual, "geometry")
        
        mesh_info = mesh_mapping.get(match.template_link.name, {})
        
        if "visual" in mesh_info and isinstance(mesh_info["visual"], str):
            # Use mesh file
            mesh_elem = ET.SubElement(visual_geom, "mesh")
            mesh_elem.set("filename", mesh_info["visual"])
            if "scale_factor" in mesh_info:
                scale = mesh_info["scale_factor"]
                mesh_elem.set("scale", f"{scale} {scale} {scale}")
        else:
            # Use basic geometry
            self._add_basic_geometry(visual_geom, mesh_info.get("visual", {}))
        
        # Visual material
        visual_material = ET.SubElement(visual, "material")
        part_type = match.template_link.part_type.value.split('_')[0]
        visual_material.set("name", f"{part_type}_material")
        
        # Collision geometry
        collision = ET.SubElement(link, "collision")
        collision_geom = ET.SubElement(collision, "geometry")
        
        if "collision" in mesh_info and isinstance(mesh_info["collision"], str):
            # Use mesh file
            mesh_elem = ET.SubElement(collision_geom, "mesh")
            mesh_elem.set("filename", mesh_info["collision"])
            if "scale_factor" in mesh_info:
                scale = mesh_info["scale_factor"]
                mesh_elem.set("scale", f"{scale} {scale} {scale}")
        else:
            # Use basic geometry
            self._add_basic_geometry(collision_geom, mesh_info.get("collision", {}))
        
        # Inertial properties
        inertial = ET.SubElement(link, "inertial")
        
        # Mass
        mass_elem = ET.SubElement(inertial, "mass")
        mass_elem.set("value", str(match.template_link.mass))
        
        # Center of mass (origin)
        origin_elem = ET.SubElement(inertial, "origin")
        origin_elem.set("xyz", "0 0 0")  # Assume centered for now
        origin_elem.set("rpy", "0 0 0")
        
        # Inertia tensor
        inertia_elem = ET.SubElement(inertial, "inertia")
        inertia = match.template_link.inertia
        for key, value in inertia.items():
            inertia_elem.set(key, str(value))
        
        return link
    
    def _add_basic_geometry(self, geom_parent: ET.Element, geom_spec: Dict[str, str]) -> None:
        """Add basic geometry element to parent.
        
        Args:
            geom_parent: Parent geometry XML element.
            geom_spec: Geometry specification dictionary.
        """
        geom_type = geom_spec.get("type", "box")
        
        if geom_type == "box":
            box_elem = ET.SubElement(geom_parent, "box")
            box_elem.set("size", geom_spec.get("size", "0.1 0.1 0.1"))
        elif geom_type == "sphere":
            sphere_elem = ET.SubElement(geom_parent, "sphere")
            sphere_elem.set("radius", geom_spec.get("radius", "0.05"))
        elif geom_type == "cylinder":
            cylinder_elem = ET.SubElement(geom_parent, "cylinder")
            cylinder_elem.set("radius", geom_spec.get("radius", "0.05"))
            cylinder_elem.set("length", geom_spec.get("length", "0.1"))
    
    def _create_joint_element(self, joint: TemplateJoint) -> ET.Element:
        """Create URDF joint element from template joint.
        
        Args:
            joint: Template joint definition.
            
        Returns:
            XML joint element.
        """
        joint_elem = ET.Element("joint", name=joint.name, type=joint.joint_type)
        
        # Parent and child links
        parent_elem = ET.SubElement(joint_elem, "parent")
        parent_elem.set("link", joint.parent_link)
        
        child_elem = ET.SubElement(joint_elem, "child")
        child_elem.set("link", joint.child_link)
        
        # Origin (transform)
        origin_elem = ET.SubElement(joint_elem, "origin")
        origin_elem.set("xyz", f"{joint.origin_xyz[0]} {joint.origin_xyz[1]} {joint.origin_xyz[2]}")
        origin_elem.set("rpy", f"{joint.origin_rpy[0]} {joint.origin_rpy[1]} {joint.origin_rpy[2]}")
        
        # Axis (for revolute/prismatic joints)
        if joint.joint_type in ["revolute", "prismatic", "continuous"]:
            axis_elem = ET.SubElement(joint_elem, "axis")
            axis_elem.set("xyz", f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}")
        
        # Joint limits
        if joint.lower_limit is not None or joint.upper_limit is not None:
            limit_elem = ET.SubElement(joint_elem, "limit")
            if joint.lower_limit is not None:
                limit_elem.set("lower", str(joint.lower_limit))
            if joint.upper_limit is not None:
                limit_elem.set("upper", str(joint.upper_limit))
            if joint.effort_limit is not None:
                limit_elem.set("effort", str(joint.effort_limit))
            if joint.velocity_limit is not None:
                limit_elem.set("velocity", str(joint.velocity_limit))
        
        # Joint dynamics
        dynamics_elem = ET.SubElement(joint_elem, "dynamics")
        dynamics_elem.set("damping", str(joint.damping))
        dynamics_elem.set("friction", str(joint.friction))
        
        return joint_elem
    
    def _has_matched_link(self, link_name: str, matching_result: MatchingResult) -> bool:
        """Check if a template link has been matched to a part.
        
        Args:
            link_name: Name of the template link.
            matching_result: Template matching result.
            
        Returns:
            True if link is matched, False otherwise.
        """
        return any(match.template_link.name == link_name for match in matching_result.matches)
    
    def _write_urdf_file(self, urdf_root: ET.Element, urdf_path: Path) -> None:
        """Write URDF XML to file with proper formatting.
        
        Args:
            urdf_root: Root URDF XML element.
            urdf_path: Path to write URDF file.
        """
        # Format XML with proper indentation
        self._indent_xml(urdf_root)
        
        # Create XML tree and write to file
        tree = ET.ElementTree(urdf_root)
        
        with open(urdf_path, 'wb') as f:
            # Write XML declaration
            f.write(b'<?xml version="1.0"?>\\n')
            tree.write(f, encoding='utf-8', xml_declaration=False)
    
    def _indent_xml(self, elem: ET.Element, level: int = 0) -> None:
        """Add proper indentation to XML elements.
        
        Args:
            elem: XML element to indent.
            level: Current indentation level.
        """
        indent = "\\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = indent + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = indent
            for child in elem:
                self._indent_xml(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = indent
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = indent
    
    async def _generate_additional_files(
        self, 
        urdf_dir: Path, 
        matching_result: MatchingResult, 
        hero: Hero
    ) -> None:
        """Generate additional files for the URDF package.
        
        Args:
            urdf_dir: URDF directory.
            matching_result: Template matching result.
            hero: Hero being processed.
        """
        # Generate launch file for easy loading
        await self._generate_launch_file(urdf_dir, hero.name)
        
        # Generate configuration file with joint limits and parameters
        await self._generate_config_file(urdf_dir, matching_result, hero)
        
        # Generate README with instructions
        await self._generate_readme_file(urdf_dir, matching_result, hero)
    
    async def _generate_launch_file(self, urdf_dir: Path, robot_name: str) -> None:
        """Generate ROS launch file for the robot.
        
        Args:
            urdf_dir: URDF directory.
            robot_name: Name of the robot.
        """
        launch_content = f'''<?xml version="1.0"?>
<launch>
  <!-- Load robot description -->
  <param name="robot_description" 
         command="$(find xacro)/xacro $(find {robot_name}_description)/urdf/{robot_name}.urdf" />
  
  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="30.0" />
  </node>
  
  <!-- Joint state publisher GUI (for manual control) -->
  <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" 
        unless="$(arg use_gui)" />
  
  <!-- RViz visualization -->
  <node pkg="rviz" type="rviz" name="rviz" 
        args="-d $(find {robot_name}_description)/rviz/{robot_name}.rviz" />
</launch>'''
        
        launch_path = urdf_dir / f"{robot_name}.launch"
        with open(launch_path, 'w') as f:
            f.write(launch_content)
    
    async def _generate_config_file(
        self, 
        urdf_dir: Path, 
        matching_result: MatchingResult, 
        hero: Hero
    ) -> None:
        """Generate configuration file with robot parameters.
        
        Args:
            urdf_dir: URDF directory.
            matching_result: Template matching result.
            hero: Hero being processed.
        """
        config = {
            "robot_info": {
                "name": hero.name,
                "template_type": matching_result.template.template_type.value,
                "template_name": matching_result.template.name,
                "generation_date": time.strftime("%Y-%m-%d %H:%M:%S"),
            },
            "matching_statistics": {
                "overall_score": matching_result.overall_score,
                "coverage_ratio": matching_result.coverage_ratio,
                "matched_parts": len(matching_result.matches),
                "total_template_links": len(matching_result.template.links),
            },
            "joint_limits": {},
            "physics_parameters": {
                "gravity": [0, 0, -9.81],
                "default_damping": self.default_damping,
                "default_friction": self.default_friction,
            }
        }
        
        # Add joint limits
        for joint in matching_result.template.joints.values():
            if joint.lower_limit is not None or joint.upper_limit is not None:
                config["joint_limits"][joint.name] = {
                    "lower": joint.lower_limit,
                    "upper": joint.upper_limit,
                    "effort": joint.effort_limit,
                    "velocity": joint.velocity_limit,
                }
        
        import json
        config_path = urdf_dir / f"{hero.name}_config.json"
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
    
    async def _generate_readme_file(
        self, 
        urdf_dir: Path, 
        matching_result: MatchingResult, 
        hero: Hero
    ) -> None:
        """Generate README file with usage instructions.
        
        Args:
            urdf_dir: URDF directory.
            matching_result: Template matching result.
            hero: Hero being processed.
        """
        readme_content = f'''# {hero.name} Robot Description

This URDF was generated automatically from a {matching_result.template.template_type.value} template
using the Tung Playground template-based robot generation system.

## Robot Information

- **Name**: {hero.name}
- **Template**: {matching_result.template.name}
- **Type**: {matching_result.template.template_type.value}
- **Generated**: {time.strftime("%Y-%m-%d %H:%M:%S")}

## Matching Statistics

- **Overall Score**: {matching_result.overall_score:.2f}
- **Coverage Ratio**: {matching_result.coverage_ratio:.2f}
- **Matched Parts**: {len(matching_result.matches)}/{len(matching_result.template.links)}

## Files

- `{hero.name}.urdf` - Main URDF description file
- `{hero.name}.launch` - ROS launch file for visualization
- `{hero.name}_config.json` - Robot configuration and parameters
- `meshes/` - Directory containing part mesh files

## Usage

### With ROS:
```bash
roslaunch {hero.name}.launch
```

### With MuJoCo:
```python
import mujoco
model = mujoco.MjModel.from_xml_path("{hero.name}.urdf")
data = mujoco.MjData(model)
```

### With PyBullet:
```python
import pybullet as p
robot_id = p.loadURDF("{hero.name}.urdf")
```

## Notes

This robot was generated using template matching. The quality depends on:
1. Quality of the original 3D mesh and part decomposition
2. Accuracy of the template matching algorithm
3. Appropriateness of the selected template type

For best results, verify joint limits and physical parameters before use.
'''
        
        readme_path = urdf_dir / "README.md"
        with open(readme_path, 'w') as f:
            f.write(readme_content)
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate that hero has required template matching results.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        # Check for template matching results
        if "template_matching" not in hero.assets.metadata:
            raise ValidationError("Hero must have template matching results for URDF generation")
        
        # Check that parts exist
        if not hero.assets.has_asset(AssetType.PARTS):
            raise ValidationError("Hero must have decomposed parts for URDF generation")
        
        parts = hero.assets.get_asset(AssetType.PARTS)
        if not parts or len(parts) == 0:
            raise ValidationError("Hero must have at least one part for URDF generation")
        
        return True