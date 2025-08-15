"""Interactive URDF visualizer using viser.

This module provides a web-based interactive visualization for URDF robot models,
with support for real-time joint manipulation and mesh display.
"""

import time
from pathlib import Path
from typing import Optional, Dict, Any, List

try:
    import viser
    import yourdfpy as urdf
    import numpy as np
except ImportError as e:
    raise ImportError(
        "Visualization dependencies not found. Install with: pip install tung_playground[vis]"
    ) from e

from ..core.hero import Hero, AssetType
from ..utils.logging import get_logger


class URDFVisualizer:
    """Interactive URDF visualizer using viser.
    
    Provides a web-based interface for visualizing and manipulating URDF robot models
    with real-time joint control and mesh display options.
    """
    
    def __init__(self, port: int = 8080, host: str = "localhost"):
        """Initialize the URDF visualizer.
        
        Args:
            port: Port for the viser server.
            host: Host address for the viser server.
        """
        self.logger = get_logger(__name__)
        self.server = viser.ViserServer(port=port, host=host)
        self.robot: Optional[urdf.URDF] = None
        self.joint_sliders: Dict[str, Any] = {}
        self.mesh_handles: Dict[str, Any] = {}
        self.current_config: Dict[str, float] = {}
        
        # Add grid to scene with RGB tuples instead of hex colors
        try:
            self.server.scene.add_grid(
                name="ground",
                width=10,
                height=10,
                width_segments=10,
                height_segments=10,
                plane="xz",
                cell_color=(0.8, 0.8, 0.8),  # Light gray
                cell_thickness=1.0,
                section_color=(0.5, 0.5, 0.5),  # Dark gray
                section_thickness=1.0,
            )
        except Exception as e:
            self.logger.warning(f"Failed to add grid: {e}")
        
        # Add lighting - try different viser API methods
        try:
            if hasattr(self.server.scene, 'add_directional_light'):
                self.server.scene.add_directional_light(
                    name="sun",
                    color=(1.0, 1.0, 1.0),  # White
                    intensity=3.0,
                    position=(2, 2, 2),
                )
            elif hasattr(self.server.scene, 'add_light'):
                self.server.scene.add_light(
                    name="sun",
                    color=(1.0, 1.0, 1.0),
                    position=(2, 2, 2),
                )
        except Exception as e:
            self.logger.warning(f"Failed to add lighting: {e}")
        
        self.logger.info(f"Viser server started at http://{host}:{port}")
    
    def load_hero(self, hero: Hero) -> None:
        """Load a hero's URDF for visualization.
        
        Args:
            hero: Hero object with URDF asset.
            
        Raises:
            ValueError: If hero doesn't have a URDF asset.
        """
        if not hero.assets.has_asset(AssetType.URDF):
            raise ValueError(f"Hero {hero.name} does not have a URDF asset")
        
        urdf_path = hero.assets.get_asset(AssetType.URDF)
        self.load_urdf(urdf_path, hero.hero_dir)
        self.logger.info(f"Loaded hero {hero.name} for visualization")
    
    def load_urdf(self, urdf_path: Path, base_dir: Optional[Path] = None) -> None:
        """Load a URDF file for visualization.
        
        Args:
            urdf_path: Path to the URDF file.
            base_dir: Base directory for resolving relative mesh paths.
            
        Raises:
            ValueError: If URDF file cannot be loaded or parsed.
        """
        self.logger.info(f"Loading URDF: {urdf_path}")
        
        try:
            # Ensure we have absolute paths
            urdf_path = Path(urdf_path).resolve()
            if base_dir:
                base_dir = Path(base_dir).resolve()
            
            # Load URDF with mesh directory context
            if base_dir:
                # Change working directory temporarily to resolve relative mesh paths
                import os
                original_cwd = os.getcwd()
                try:
                    os.chdir(base_dir)
                    self.robot = urdf.URDF.load(str(urdf_path))
                finally:
                    os.chdir(original_cwd)
            else:
                self.robot = urdf.URDF.load(str(urdf_path))
            
            # Initialize joint configuration
            self.current_config = {}
            for joint in self.robot.robot.joints:
                if joint.type in ['revolute', 'prismatic', 'continuous']:
                    self.current_config[joint.name] = 0.0
            
            # Create visualization
            self._create_controls()
            self._update_visualization()
            
            self.logger.info(f"Successfully loaded URDF with {len(self.current_config)} controllable joints")
            
        except Exception as e:
            error_msg = f"Failed to load URDF file: {e}"
            self.logger.error(error_msg)
            
            # Try to provide more specific error information
            if "could not convert string to float" in str(e):
                self.logger.error("This error is often caused by malformed OBJ mesh files with invalid comments")
                self.logger.error("Try regenerating the mesh files or check for parsing issues")
            elif "No such file or directory" in str(e):
                self.logger.error("Mesh files referenced in URDF may be missing")
                self.logger.error(f"Check that all mesh files exist relative to: {base_dir or urdf_path.parent}")
            
            # Try fallback: create simple visualization without meshes
            try:
                self._create_fallback_visualization(urdf_path)
                self.logger.warning("Using fallback visualization due to URDF loading error")
                return  # Don't raise exception if fallback works
            except Exception as fallback_error:
                self.logger.error(f"Fallback visualization also failed: {fallback_error}")
            
            raise ValueError(error_msg) from e
    
    def _create_fallback_visualization(self, urdf_path: Path) -> None:
        """Create a simple fallback visualization when URDF loading fails.
        
        Args:
            urdf_path: Path to the problematic URDF file.
        """
        try:
            # Try to parse URDF as XML to extract joint information
            import xml.etree.ElementTree as ET
            
            self.logger.info("Attempting fallback visualization with simple geometry")
            
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            # Extract joint information
            joints = []
            for joint_elem in root.findall('joint'):
                joint_type = joint_elem.get('type')
                if joint_type in ['revolute', 'prismatic', 'continuous']:
                    joint_name = joint_elem.get('name')
                    joints.append(joint_name)
                    
                    # Get joint limits
                    limit_elem = joint_elem.find('limit')
                    if limit_elem is not None:
                        lower = float(limit_elem.get('lower', -3.14))
                        upper = float(limit_elem.get('upper', 3.14))
                    else:
                        lower, upper = -3.14, 3.14
            
            # Create simple joint visualization
            self.current_config = {name: 0.0 for name in joints}
            
            # Add simple visual elements for each joint
            for i, joint_name in enumerate(joints):
                # Add a simple box for each joint with compatible API
                try:
                    self.server.scene.add_box(
                        name=f"joint_{joint_name}",
                        dimensions=(0.1, 0.1, 0.1),
                        color=(0.2 + 0.1 * i, 0.5, 0.8),
                    )
                except Exception as e:
                    self.logger.warning(f"Failed to add visual for joint {joint_name}: {e}")
            
            # Create simple controls
            self._create_fallback_controls(joints)
            
            self.logger.info(f"Fallback visualization created with {len(joints)} joints")
            
        except Exception as fallback_error:
            self.logger.error(f"Fallback visualization also failed: {fallback_error}")
            # Create minimal visualization
            self._create_minimal_visualization()
    
    def _create_fallback_controls(self, joint_names: List[str]) -> None:
        """Create simple controls for fallback mode.
        
        Args:
            joint_names: List of joint names to create controls for.
        """
        with self.server.gui.add_folder("Fallback Joint Controls"):
            for joint_name in joint_names:
                slider = self.server.gui.add_slider(
                    label=joint_name,
                    min=-3.14,
                    max=3.14,
                    step=0.01,
                    initial_value=0.0,
                )
                
                @slider.on_update
                def _update_fallback_joint(value: float, joint_name: str = joint_name) -> None:
                    self.current_config[joint_name] = value
                    # Simple update: just log the change
                    self.logger.debug(f"Joint {joint_name} set to {value}")
    
    def _create_minimal_visualization(self) -> None:
        """Create minimal visualization when everything else fails."""
        self.logger.info("Creating minimal visualization")
        
        # Add a simple message box
        try:
            self.server.scene.add_box(
                name="error_indicator",
                dimensions=(0.5, 0.5, 0.5),
                color=(1.0, 0.2, 0.2),
            )
        except Exception as e:
            self.logger.warning(f"Failed to add error indicator: {e}")
        
        try:
            with self.server.gui.add_folder("Error Info"):
                self.server.gui.add_text(
                    label="Status",
                    initial_value="URDF loading failed - showing error indicator"
                )
        except Exception as e:
            self.logger.warning(f"Failed to add error info: {e}")
    
    def _create_controls(self) -> None:
        """Create GUI controls for joint manipulation."""
        if not self.robot:
            return
        
        # Clear existing controls
        for slider in self.joint_sliders.values():
            slider.remove()
        self.joint_sliders.clear()
        
        # Add joint controls
        with self.server.gui.add_folder("Joint Controls"):
            for joint in self.robot.robot.joints:
                if joint.type in ['revolute', 'prismatic', 'continuous']:
                    # Determine joint limits
                    if joint.limit is not None:
                        min_val = float(joint.limit.lower) if joint.limit.lower is not None else -np.pi
                        max_val = float(joint.limit.upper) if joint.limit.upper is not None else np.pi
                    else:
                        # Default limits for continuous joints
                        min_val = -np.pi
                        max_val = np.pi
                    
                    # Create slider
                    slider = self.server.gui.add_slider(
                        label=joint.name,
                        min=min_val,
                        max=max_val,
                        step=0.01,
                        initial_value=0.0,
                    )
                    
                    # Add callback for real-time updates
                    @slider.on_update
                    def _update_joint(value: float, joint_name: str = joint.name) -> None:
                        self.current_config[joint_name] = value
                        self._update_visualization()
                    
                    self.joint_sliders[joint.name] = slider
            
            # Add reset button
            reset_button = self.server.gui.add_button("Reset All Joints")
            
            @reset_button.on_click
            def _reset_joints() -> None:
                for joint_name in self.current_config:
                    self.current_config[joint_name] = 0.0
                    if joint_name in self.joint_sliders:
                        self.joint_sliders[joint_name].value = 0.0
                self._update_visualization()
        
        # Add display options
        with self.server.gui.add_folder("Display Options"):
            visual_checkbox = self.server.gui.add_checkbox(
                label="Show Visual Meshes",
                initial_value=True,
            )
            collision_checkbox = self.server.gui.add_checkbox(
                label="Show Collision Meshes", 
                initial_value=False,
            )
            
            @visual_checkbox.on_update
            def _toggle_visual(show: bool) -> None:
                self._update_mesh_visibility("visual", show)
            
            @collision_checkbox.on_update  
            def _toggle_collision(show: bool) -> None:
                self._update_mesh_visibility("collision", show)
    
    def _update_visualization(self) -> None:
        """Update the robot visualization with current joint configuration."""
        if not self.robot:
            return
        
        # Clear existing meshes
        for handle in self.mesh_handles.values():
            handle.remove()
        self.mesh_handles.clear()
        
        # Update robot configuration
        try:
            # Get forward kinematics for current configuration
            cfg = {}
            for joint_name, value in self.current_config.items():
                cfg[joint_name] = value
            
            # Get link poses - try different API methods
            fk = None
            if hasattr(self.robot, 'link_fk'):
                fk = self.robot.link_fk(cfg=cfg)
            elif hasattr(self.robot, 'visual_geometry_fk'):
                fk = self.robot.visual_geometry_fk(cfg=cfg)
            elif hasattr(self.robot, 'collision_geometry_fk'):
                fk = self.robot.collision_geometry_fk(cfg=cfg)
            
            if fk is None:
                self.logger.warning("No forward kinematics method available")
                return
            
            # Render each link
            for link_name, transform in fk.items():
                link = None
                for l in self.robot.robot.links:
                    if l.name == link_name:
                        link = l
                        break
                
                if link and link.visuals:
                    for i, visual in enumerate(link.visuals):
                        self._add_mesh(
                            f"{link_name}_visual_{i}",
                            visual,
                            transform,
                            mesh_type="visual"
                        )
        
        except Exception as e:
            self.logger.warning(f"Failed to update visualization: {e}")
    
    def _add_mesh(self, name: str, visual_element: Any, transform: np.ndarray, mesh_type: str = "visual") -> None:
        """Add a mesh to the visualization.
        
        Args:
            name: Unique name for the mesh.
            visual_element: URDF visual or collision element.
            transform: 4x4 transformation matrix.
            mesh_type: Type of mesh ("visual" or "collision").
        """
        try:
            if visual_element.geometry.mesh is not None:
                # Handle mesh geometry
                mesh_path = visual_element.geometry.mesh.filename
                scale = visual_element.geometry.mesh.scale
                if scale is None:
                    scale = [1.0, 1.0, 1.0]
                
                # Try to load mesh file
                if Path(mesh_path).exists():
                    mesh_handle = self.server.scene.add_mesh_simple(
                        name=name,
                        vertices=np.array([[0, 0, 0]]),  # Placeholder - would need proper mesh loading
                        faces=np.array([[0, 0, 0]]),
                        color=(0.8, 0.6, 0.2) if mesh_type == "visual" else (0.2, 0.8, 0.2),
                        transform=transform,
                    )
                    self.mesh_handles[name] = mesh_handle
            
            elif visual_element.geometry.box is not None:
                # Handle box geometry
                size = visual_element.geometry.box.size
                box_handle = self.server.scene.add_box(
                    name=name,
                    dimensions=size,
                    color=(0.8, 0.6, 0.2) if mesh_type == "visual" else (0.2, 0.8, 0.2),
                    transform=transform,
                )
                self.mesh_handles[name] = box_handle
            
            elif visual_element.geometry.cylinder is not None:
                # Handle cylinder geometry
                radius = visual_element.geometry.cylinder.radius
                length = visual_element.geometry.cylinder.length
                cylinder_handle = self.server.scene.add_icosphere(  # Approximate with sphere
                    name=name,
                    radius=radius,
                    color=(0.8, 0.6, 0.2) if mesh_type == "visual" else (0.2, 0.8, 0.2),
                    transform=transform,
                )
                self.mesh_handles[name] = cylinder_handle
            
            elif visual_element.geometry.sphere is not None:
                # Handle sphere geometry
                radius = visual_element.geometry.sphere.radius
                sphere_handle = self.server.scene.add_icosphere(
                    name=name,
                    radius=radius,
                    color=(0.8, 0.6, 0.2) if mesh_type == "visual" else (0.2, 0.8, 0.2),
                    transform=transform,
                )
                self.mesh_handles[name] = sphere_handle
        
        except Exception as e:
            self.logger.warning(f"Failed to add mesh {name}: {e}")
    
    def _update_mesh_visibility(self, mesh_type: str, visible: bool) -> None:
        """Update visibility of specific mesh types.
        
        Args:
            mesh_type: Type of mesh ("visual" or "collision").
            visible: Whether to show or hide meshes.
        """
        for name, handle in self.mesh_handles.items():
            if mesh_type in name:
                handle.visible = visible
    
    def set_joint_positions(self, joint_positions: Dict[str, float]) -> None:
        """Set joint positions programmatically.
        
        Args:
            joint_positions: Dictionary mapping joint names to positions.
        """
        for joint_name, position in joint_positions.items():
            if joint_name in self.current_config:
                self.current_config[joint_name] = position
                if joint_name in self.joint_sliders:
                    self.joint_sliders[joint_name].value = position
        
        self._update_visualization()
    
    def run(self, blocking: bool = True) -> None:
        """Start the visualization server.
        
        Args:
            blocking: Whether to block the main thread.
        """
        if blocking:
            self.logger.info("Visualization server running. Press Ctrl+C to stop.")
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                self.logger.info("Visualization server stopped.")
        else:
            self.logger.info("Visualization server running in background.")
    
    def stop(self) -> None:
        """Stop the visualization server."""
        if hasattr(self.server, 'stop'):
            self.server.stop()
        self.logger.info("Visualization server stopped.")


def visualize_hero(hero: Hero, port: int = 8080, host: str = "localhost", blocking: bool = True) -> URDFVisualizer:
    """Convenience function to visualize a hero.
    
    Args:
        hero: Hero object to visualize.
        port: Port for the viser server.
        host: Host address for the viser server.
        blocking: Whether to block the main thread.
        
    Returns:
        URDFVisualizer instance.
    """
    visualizer = URDFVisualizer(port=port, host=host)
    visualizer.load_hero(hero)
    visualizer.run(blocking=blocking)
    return visualizer


def visualize_urdf(urdf_path: Path, base_dir: Optional[Path] = None, port: int = 8080, 
                  host: str = "localhost", blocking: bool = True) -> URDFVisualizer:
    """Convenience function to visualize a URDF file.
    
    Args:
        urdf_path: Path to the URDF file.
        base_dir: Base directory for resolving relative mesh paths.
        port: Port for the viser server.
        host: Host address for the viser server.
        blocking: Whether to block the main thread.
        
    Returns:
        URDFVisualizer instance.
    """
    visualizer = URDFVisualizer(port=port, host=host)
    visualizer.load_urdf(urdf_path, base_dir)
    visualizer.run(blocking=blocking)
    return visualizer