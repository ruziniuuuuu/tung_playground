"""Mock part decomposer for development and testing purposes."""

import asyncio
import time
import random
from pathlib import Path
from typing import Dict, Any, List, Optional
import logging

from .base import PartDecomposer, DecompositionResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError

logger = logging.getLogger(__name__)


class MockDecomposer(PartDecomposer):
    """Mock part decomposer that simulates mesh segmentation.
    
    This decomposer provides realistic simulation of mesh segmentation
    with configurable parameters for testing the complete pipeline.
    """
    
    PLUGIN_NAME = "mock_decomposer"
    PLUGIN_TYPE = "decomposition"
    VERSION = "2.0.0"
    AUTHOR = "Tung Playground"
    
    def __init__(self, name: str = "mock_decomposer", config: Dict[str, Any] = None):
        """Initialize mock decomposer.
        
        Args:
            name: Name of the decomposer.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        
        # Part generation configuration
        self.parts_to_create = self.get_config_value("parts_to_create", None)  # Auto if None
        self.num_parts = self.get_config_value("num_parts", None)  # Auto if None
        self.min_parts = self.get_config_value("min_parts", 3)
        self.max_parts = self.get_config_value("max_parts", 8)
        
        # Simulation configuration
        self.simulate_processing_time = self.get_config_value("simulate_processing_time", True)
        self.min_processing_time = self.get_config_value("min_processing_time", 1.0)  # seconds
        self.max_processing_time = self.get_config_value("max_processing_time", 5.0)  # seconds
        self.failure_rate = self.get_config_value("failure_rate", 0.0)  # 0.0-1.0
        self.quality_variation = self.get_config_value("quality_variation", True)
        
        # Output configuration
        self.output_format = self.get_config_value("output_format", "obj")  # obj, ply
        self.generate_realistic_parts = self.get_config_value("generate_realistic_parts", True)
        
        # Template part names for different model types
        self.template_parts = {
            "humanoid": ["head", "torso", "left_arm", "right_arm", "left_leg", "right_leg"],
            "quadruped": ["head", "body", "front_left_leg", "front_right_leg", "rear_left_leg", "rear_right_leg", "tail"],
            "vehicle": ["body", "wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right"],
            "generic": ["part_01", "part_02", "part_03", "part_04", "part_05", "part_06", "part_07", "part_08"]
        }
        
        # Model type detection keywords
        self.model_type_keywords = {
            "humanoid": ["human", "person", "character", "man", "woman", "robot", "android"],
            "quadruped": ["dog", "cat", "horse", "animal", "creature", "beast"],
            "vehicle": ["car", "truck", "vehicle", "bike", "motorcycle"]
        }
    
    async def decompose_parts(self, hero: Hero) -> DecompositionResult:
        """Mock decompose 3D mesh into parts.
        
        Args:
            hero: Hero with 3D mesh.
            
        Returns:
            Mock decomposition result with generated part files.
            
        Raises:
            ProcessingError: If decomposition fails.
        """
        start_time = time.time()
        
        # Get input mesh
        mesh_path = hero.assets.get_asset(AssetType.MESH_3D)
        self.logger.info(f"Starting mock mesh decomposition for {hero.name} with mesh {mesh_path}")
        
        try:
            # Simulate random failure if configured
            if self.failure_rate > 0 and random.random() < self.failure_rate:
                raise ProcessingError(f"Mock decomposer simulated failure (failure rate: {self.failure_rate})")
            
            # Simulate processing time
            if self.simulate_processing_time:
                processing_time = random.uniform(self.min_processing_time, self.max_processing_time)
                self.logger.info(f"Simulating processing time: {processing_time:.1f}s")
                await asyncio.sleep(processing_time)
            
            # Determine model type and parts
            model_type = self._detect_model_type(hero)
            part_names = self._determine_part_names(model_type)
            
            # Create parts directory
            parts_dir = hero.hero_dir / "parts"
            parts_dir.mkdir(exist_ok=True)
            
            # Generate part files
            if self.generate_realistic_parts:
                # Read original mesh and create realistic parts
                original_mesh = self._read_obj_file(mesh_path)
                parts_paths = await self._create_realistic_parts(parts_dir, part_names, original_mesh)
            else:
                # Create simple geometric parts
                parts_paths = await self._create_simple_parts(parts_dir, part_names)
            
            decomposition_time = time.time() - start_time
            
            # Prepare metadata with simulation details
            metadata = {
                "decomposition_method": "mock_simulation",
                "model_type_detected": model_type,
                "num_parts": len(parts_paths),
                "output_format": self.output_format,
                "simulated": True,
                "processing_time_simulated": self.simulate_processing_time,
                "realistic_parts": self.generate_realistic_parts,
                "original_vertices": len(original_mesh.get("vertices", [])) if self.generate_realistic_parts else 0,
                "parts_dir": str(parts_dir)
            }
            
            self.logger.info(f"Mock decomposition completed: {len(parts_paths)} parts generated in {decomposition_time:.1f}s")
            
            return DecompositionResult(
                parts_paths=parts_paths,
                part_names=part_names,
                decomposition_time=decomposition_time,
                metadata=metadata
            )
            
        except Exception as e:
            raise ProcessingError(f"Mock decomposition failed: {e}")
    
    def _detect_model_type(self, hero: Hero) -> str:
        """Detect model type from hero name and description.
        
        Args:
            hero: Hero to analyze.
            
        Returns:
            Detected model type.
        """
        text_to_check = f"{hero.name} {hero.description}".lower()
        
        for model_type, keywords in self.model_type_keywords.items():
            if any(keyword in text_to_check for keyword in keywords):
                self.logger.info(f"Detected model type: {model_type}")
                return model_type
        
        self.logger.info("Using generic model type")
        return "generic"
    
    def _determine_part_names(self, model_type: str) -> List[str]:
        """Determine part names based on configuration and model type.
        
        Args:
            model_type: Detected or specified model type.
            
        Returns:
            List of part names to generate.
        """
        if self.parts_to_create:
            # Use explicitly configured parts
            return self.parts_to_create
        
        # Determine number of parts
        if self.num_parts is not None:
            num_parts = self.num_parts
        else:
            num_parts = random.randint(self.min_parts, self.max_parts)
        
        # Get template parts for the model type
        template_names = self.template_parts.get(model_type, self.template_parts["generic"])
        
        if len(template_names) >= num_parts:
            return template_names[:num_parts]
        else:
            # Extend with numbered parts if needed
            names = template_names.copy()
            for i in range(len(template_names) + 1, num_parts + 1):
                names.append(f"additional_part_{i:02d}")
            return names
    
    async def _create_realistic_parts(self, parts_dir: Path, part_names: List[str], original_mesh: Dict[str, List]) -> List[Path]:
        """Create realistic part files by segmenting the original mesh.
        
        Args:
            parts_dir: Directory to save parts.
            part_names: Names of parts to create.
            original_mesh: Original mesh data.
            
        Returns:
            List of generated part file paths.
        """
        parts_paths = []
        
        for i, part_name in enumerate(part_names):
            part_filename = f"{part_name}.{self.output_format}"
            part_path = parts_dir / part_filename
            
            # Create part mesh by filtering original mesh
            self._create_part_mesh(part_path, part_name, original_mesh, i)
            parts_paths.append(part_path)
            
            self.logger.debug(f"Generated realistic part: {part_path} (size: {part_path.stat().st_size} bytes)")
        
        return parts_paths
    
    async def _create_simple_parts(self, parts_dir: Path, part_names: List[str]) -> List[Path]:
        """Create simple geometric part files.
        
        Args:
            parts_dir: Directory to save parts.
            part_names: Names of parts to create.
            
        Returns:
            List of generated part file paths.
        """
        parts_paths = []
        
        for i, part_name in enumerate(part_names):
            part_filename = f"{part_name}.{self.output_format}"
            part_path = parts_dir / part_filename
            
            # Generate simple geometric content
            if self.output_format == "obj":
                content = self._generate_mock_obj_content(part_name, i)
            elif self.output_format == "ply":
                content = self._generate_mock_ply_content(part_name, i)
            else:
                content = f"# Mock part: {part_name}\n"
            
            # Write part file
            with open(part_path, 'w') as f:
                f.write(content)
            
            parts_paths.append(part_path)
            self.logger.debug(f"Generated simple part: {part_path} (size: {part_path.stat().st_size} bytes)")
        
        return parts_paths
    
    def _generate_mock_obj_content(self, part_name: str, index: int) -> str:
        """Generate mock OBJ file content for a part.
        
        Args:
            part_name: Name of the part.
            index: Part index for variation.
            
        Returns:
            Mock OBJ file content.
        """
        # Generate a simple geometric shape based on part name and index
        offset_x = (index % 3) * 2.0
        offset_y = ((index // 3) % 3) * 2.0
        offset_z = 0.0
        
        # Add some randomness if quality variation is enabled
        if self.quality_variation:
            offset_x += random.uniform(-0.2, 0.2)
            offset_y += random.uniform(-0.2, 0.2)
            offset_z += random.uniform(-0.1, 0.1)
        
        # Different shapes for different part types
        if "head" in part_name.lower():
            # Sphere-like shape
            return f"""# Mock part: {part_name}
# Simple sphere representation
v {0.0 + offset_x} {1.0 + offset_y} {0.0 + offset_z}
v {0.5 + offset_x} {0.5 + offset_y} {0.5 + offset_z}
v {-0.5 + offset_x} {0.5 + offset_y} {0.5 + offset_z}
v {0.5 + offset_x} {0.5 + offset_y} {-0.5 + offset_z}
v {-0.5 + offset_x} {0.5 + offset_y} {-0.5 + offset_z}
v {0.0 + offset_x} {0.0 + offset_y} {0.0 + offset_z}
f 1 2 3
f 1 3 5
f 1 5 4
f 1 4 2
f 6 3 2
f 6 5 3
f 6 4 5
f 6 2 4
"""
        elif any(keyword in part_name.lower() for keyword in ["leg", "arm", "limb"]):
            # Cylinder-like shape
            return f"""# Mock part: {part_name}
# Simple cylinder representation
v {0.0 + offset_x} {0.0 + offset_y} {0.0 + offset_z}
v {0.3 + offset_x} {0.0 + offset_y} {0.0 + offset_z}
v {0.0 + offset_x} {0.3 + offset_y} {0.0 + offset_z}
v {-0.3 + offset_x} {0.0 + offset_y} {0.0 + offset_z}
v {0.0 + offset_x} {-0.3 + offset_y} {0.0 + offset_z}
v {0.0 + offset_x} {0.0 + offset_y} {1.0 + offset_z}
v {0.3 + offset_x} {0.0 + offset_y} {1.0 + offset_z}
v {0.0 + offset_x} {0.3 + offset_y} {1.0 + offset_z}
v {-0.3 + offset_x} {0.0 + offset_y} {1.0 + offset_z}
v {0.0 + offset_x} {-0.3 + offset_y} {1.0 + offset_z}
f 1 2 3
f 1 3 4
f 1 4 5
f 1 5 2
f 6 7 8
f 6 8 9
f 6 9 10
f 6 10 7
f 2 7 8
f 2 8 3
f 3 8 9
f 3 9 4
f 4 9 10
f 4 10 5
f 5 10 7
f 5 7 2
"""
        else:
            # Box-like shape for torso, body, etc.
            return f"""# Mock part: {part_name}
# Simple box representation
v {-0.5 + offset_x} {-0.5 + offset_y} {-0.5 + offset_z}
v {0.5 + offset_x} {-0.5 + offset_y} {-0.5 + offset_z}
v {0.5 + offset_x} {0.5 + offset_y} {-0.5 + offset_z}
v {-0.5 + offset_x} {0.5 + offset_y} {-0.5 + offset_z}
v {-0.5 + offset_x} {-0.5 + offset_y} {0.5 + offset_z}
v {0.5 + offset_x} {-0.5 + offset_y} {0.5 + offset_z}
v {0.5 + offset_x} {0.5 + offset_y} {0.5 + offset_z}
v {-0.5 + offset_x} {0.5 + offset_y} {0.5 + offset_z}
f 1 2 3
f 1 3 4
f 5 8 7
f 5 7 6
f 1 5 6
f 1 6 2
f 2 6 7
f 2 7 3
f 3 7 8
f 3 8 4
f 4 8 5
f 4 5 1
"""
    
    def _generate_mock_ply_content(self, part_name: str, index: int) -> str:
        """Generate mock PLY file content for a part.
        
        Args:
            part_name: Name of the part.
            index: Part index for variation.
            
        Returns:
            Mock PLY file content.
        """
        return f"""ply
format ascii 1.0
comment Mock part: {part_name}
element vertex 8
property float x
property float y
property float z
element face 12
property list uchar int vertex_indices
end_header
-0.5 -0.5 -0.5
0.5 -0.5 -0.5
0.5 0.5 -0.5
-0.5 0.5 -0.5
-0.5 -0.5 0.5
0.5 -0.5 0.5
0.5 0.5 0.5
-0.5 0.5 0.5
3 0 1 2
3 0 2 3
3 4 7 6
3 4 6 5
3 0 4 5
3 0 5 1
3 1 5 6
3 1 6 2
3 2 6 7
3 2 7 3
3 3 7 4
3 3 4 0
"""
    
    def _read_obj_file(self, obj_path: Path) -> Dict[str, List]:
        """Read vertices and faces from OBJ file.
        
        Args:
            obj_path: Path to OBJ file.
            
        Returns:
            Dictionary with vertices and faces.
        """
        vertices = []
        faces = []
        
        try:
            with open(obj_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('v '):
                        # Vertex line
                        coords = line.split()[1:4]
                        vertices.append([float(c) for c in coords])
                    elif line.startswith('f '):
                        # Face line
                        face_indices = line.split()[1:]
                        faces.append([int(idx.split('/')[0]) for idx in face_indices])
        except Exception as e:
            self.logger.warning(f"Error reading OBJ file {obj_path}: {e}")
        
        return {"vertices": vertices, "faces": faces}
    
    def _create_part_mesh(self, output_path: Path, part_name: str, original_mesh: Dict[str, List], part_index: int = 0) -> None:
        """Create a part mesh by filtering the original mesh.
        
        Args:
            output_path: Path to save the part OBJ file.
            part_name: Name of the part (head, torso, arms, legs).
            original_mesh: Original mesh data.
            part_index: Index of the part for variation.
        """
        vertices = original_mesh.get("vertices", [])
        faces = original_mesh.get("faces", [])
        
        # Define part regions based on Y coordinate (height)
        part_regions = {
            "head": (1.5, 2.0),      # Upper part
            "torso": (0.5, 1.5),     # Middle upper
            "arms": (0.3, 1.3),      # Arms span multiple heights
            "legs": (-0.5, 0.7)      # Lower part
        }
        
        y_min, y_max = part_regions.get(part_name, (0.0, 2.0))
        
        # Filter vertices based on part region
        part_vertices = []
        vertex_mapping = {}
        new_vertex_index = 1
        
        for i, vertex in enumerate(vertices):
            if len(vertex) >= 2 and y_min <= vertex[1] <= y_max:
                # Include vertex in this part
                part_vertices.append(vertex)
                vertex_mapping[i + 1] = new_vertex_index  # OBJ indices start at 1
                new_vertex_index += 1
        
        # Filter faces that use the included vertices
        part_faces = []
        for face in faces:
            # Check if all vertices of this face are in the part
            mapped_face = []
            valid_face = True
            
            for vertex_idx in face:
                if vertex_idx in vertex_mapping:
                    mapped_face.append(vertex_mapping[vertex_idx])
                else:
                    valid_face = False
                    break
            
            if valid_face and len(mapped_face) >= 3:
                part_faces.append(mapped_face)
        
        # If no vertices found for this part, create a simple placeholder
        if not part_vertices:
            part_vertices = self._create_placeholder_part(part_name)
            part_faces = [[1, 2, 3], [1, 3, 4]]  # Simple quad as two triangles
        
        # Write part OBJ file
        with open(output_path, 'w') as f:
            f.write(f"# Part: {part_name}\n")
            f.write(f"# Generated by Tung Playground Mock Decomposer\n\n")
            
            # Write vertices
            for vertex in part_vertices:
                f.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
            
            f.write("\n")
            
            # Write faces
            for face in part_faces:
                face_str = " ".join(str(idx) for idx in face)
                f.write(f"f {face_str}\n")
    
    def _create_placeholder_part(self, part_name: str) -> List[List[float]]:
        """Create a simple placeholder geometry for a part.
        
        Args:
            part_name: Name of the part.
            
        Returns:
            List of vertices for the placeholder.
        """
        # Simple box/cylinder for each part type
        if part_name == "head":
            return [
                [0.0, 1.7, 0.1], [0.1, 1.7, 0.0], [0.0, 1.7, -0.1], [-0.1, 1.7, 0.0]
            ]
        elif part_name == "torso":
            return [
                [0.2, 1.0, 0.1], [0.2, 1.0, -0.1], [-0.2, 1.0, -0.1], [-0.2, 1.0, 0.1]
            ]
        elif part_name == "arms":
            return [
                [0.4, 0.8, 0.0], [0.4, 0.6, 0.0], [-0.4, 0.6, 0.0], [-0.4, 0.8, 0.0]
            ]
        elif part_name == "legs":
            return [
                [0.1, 0.0, 0.1], [0.1, 0.0, -0.1], [-0.1, 0.0, -0.1], [-0.1, 0.0, 0.1]
            ]
        else:
            # Default placeholder
            return [
                [0.1, 0.1, 0.1], [0.1, 0.1, -0.1], [-0.1, 0.1, -0.1], [-0.1, 0.1, 0.1]
            ]
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate inputs for mock decomposition.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
            
        Raises:
            ValidationError: If validation fails.
        """
        # Call parent validation first
        super().validate_inputs(hero)
        
        # Mock decomposer is very permissive - accepts any mesh format
        mesh_path = hero.assets.get_asset(AssetType.MESH_3D)
        
        # Just check that file exists and is readable
        if not mesh_path.exists():
            raise ValidationError(f"Mesh file does not exist: {mesh_path}")
        
        if not mesh_path.is_file():
            raise ValidationError(f"Mesh path is not a file: {mesh_path}")
        
        # Check if file is readable
        try:
            with open(mesh_path, 'rb') as f:
                f.read(1)  # Try to read first byte
        except Exception as e:
            raise ValidationError(f"Cannot read mesh file {mesh_path}: {e}")
        
        return True