"""Mock part decomposer for pipeline demonstration."""

import asyncio
import time
from pathlib import Path
from typing import Dict, Any, List
import logging

from .base import PartDecomposer, DecompositionResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError

logger = logging.getLogger(__name__)


class MockDecomposer(PartDecomposer):
    """Mock part decomposer that splits mesh into logical body parts."""
    
    PLUGIN_NAME = "mock_decomposer"
    PLUGIN_TYPE = "decomposition"
    VERSION = "1.0.0"
    AUTHOR = "Tung Playground"
    
    def __init__(self, name: str = "mock_decomposer", config: Dict[str, Any] = None):
        """Initialize mock decomposer.
        
        Args:
            name: Name of the decomposer.
            config: Optional configuration dictionary.
        """
        super().__init__(name, config)
        self.parts_to_create = self.get_config_value("parts_to_create", ["head", "torso", "arms", "legs"])
    
    async def decompose_parts(self, hero: Hero) -> DecompositionResult:
        """Decompose mesh into semantic body parts.
        
        Args:
            hero: Hero with 3D mesh.
            
        Returns:
            Decomposition result with part files.
            
        Raises:
            ProcessingError: If decomposition fails.
        """
        start_time = time.time()
        
        try:
            # Simulate processing time
            await asyncio.sleep(0.8)
            
            # Create parts directory
            parts_dir = hero.hero_dir / "parts"
            parts_dir.mkdir(exist_ok=True)
            
            # Read original mesh
            mesh_path = hero.assets.get_asset(AssetType.MESH_3D)
            original_mesh = self._read_obj_file(mesh_path)
            
            # Create parts
            parts_paths = []
            part_names = []
            
            for part_name in self.parts_to_create:
                part_path = parts_dir / f"{part_name}.obj"
                self._create_part_mesh(part_path, part_name, original_mesh)
                parts_paths.append(part_path)
                part_names.append(part_name)
            
            decomposition_time = time.time() - start_time
            
            metadata = {
                "method": "semantic_splitting",
                "original_vertices": len(original_mesh.get("vertices", [])),
                "parts_created": len(parts_paths),
                "parts_dir": str(parts_dir)
            }
            
            self.logger.info(f"Decomposed mesh into {len(parts_paths)} parts: {part_names}")
            
            return DecompositionResult(
                parts_paths=parts_paths,
                part_names=part_names,
                decomposition_time=decomposition_time,
                metadata=metadata
            )
            
        except Exception as e:
            raise ProcessingError(f"Mock decomposition failed: {e}")
    
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
    
    def _create_part_mesh(self, output_path: Path, part_name: str, original_mesh: Dict[str, List]) -> None:
        """Create a part mesh by filtering the original mesh.
        
        Args:
            output_path: Path to save the part OBJ file.
            part_name: Name of the part (head, torso, arms, legs).
            original_mesh: Original mesh data.
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