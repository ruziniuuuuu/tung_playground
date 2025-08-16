"""Mesh format conversion utilities."""

import os
from pathlib import Path
from typing import Optional, Union, List
import logging

logger = logging.getLogger(__name__)


class MeshConverter:
    """Utility class for converting between different mesh formats."""
    
    def __init__(self):
        """Initialize mesh converter with available backends."""
        self.available_backends = self._detect_backends()
        if not self.available_backends:
            logger.warning("No mesh conversion backends available. Install trimesh, open3d, or pymeshlab for conversion support.")
    
    def _detect_backends(self) -> List[str]:
        """Detect available mesh processing backends."""
        backends = []
        
        try:
            import trimesh
            backends.append("trimesh")
        except ImportError:
            pass
        
        try:
            import open3d
            backends.append("open3d")
        except ImportError:
            pass
        
        try:
            import pymeshlab
            backends.append("pymeshlab")
        except ImportError:
            pass
        
        return backends
    
    def convert(
        self, 
        input_path: Union[str, Path], 
        output_path: Union[str, Path],
        backend: Optional[str] = None
    ) -> Path:
        """Convert mesh from one format to another.
        
        Args:
            input_path: Path to input mesh file.
            output_path: Path to output mesh file.
            backend: Preferred backend ("trimesh", "open3d", "pymeshlab"). Auto-select if None.
            
        Returns:
            Path to converted mesh file.
            
        Raises:
            ValueError: If conversion is not supported.
            ImportError: If required backend is not available.
            Exception: If conversion fails.
        """
        input_path = Path(input_path)
        output_path = Path(output_path)
        
        if not input_path.exists():
            raise FileNotFoundError(f"Input mesh file not found: {input_path}")
        
        # Create output directory
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Auto-select backend if not specified
        if backend is None:
            if "trimesh" in self.available_backends:
                backend = "trimesh"
            elif "open3d" in self.available_backends:
                backend = "open3d"
            elif "pymeshlab" in self.available_backends:
                backend = "pymeshlab"
            else:
                raise ImportError(
                    "No mesh conversion backend available. Install one of: "
                    "trimesh, open3d, pymeshlab"
                )
        
        if backend not in self.available_backends:
            raise ImportError(f"Backend '{backend}' not available. Available: {self.available_backends}")
        
        # Get file extensions
        input_ext = input_path.suffix.lower()
        output_ext = output_path.suffix.lower()
        
        logger.info(f"Converting {input_ext} → {output_ext} using {backend}")
        
        # Perform conversion
        if backend == "trimesh":
            return self._convert_with_trimesh(input_path, output_path)
        elif backend == "open3d":
            return self._convert_with_open3d(input_path, output_path)
        elif backend == "pymeshlab":
            return self._convert_with_pymeshlab(input_path, output_path)
        else:
            raise ValueError(f"Unknown backend: {backend}")
    
    def _convert_with_trimesh(self, input_path: Path, output_path: Path) -> Path:
        """Convert mesh using trimesh backend."""
        try:
            import trimesh
            
            # Load mesh
            mesh = trimesh.load(str(input_path))
            
            # Handle different mesh types
            if isinstance(mesh, trimesh.Scene):
                # Extract the first mesh from scene
                if len(mesh.geometry) > 0:
                    mesh = list(mesh.geometry.values())[0]
                else:
                    raise ValueError("No geometry found in scene")
            
            # Export mesh
            mesh.export(str(output_path))
            
            logger.info(f"Converted mesh using trimesh: {output_path}")
            return output_path
            
        except Exception as e:
            raise Exception(f"Trimesh conversion failed: {e}")
    
    def _convert_with_open3d(self, input_path: Path, output_path: Path) -> Path:
        """Convert mesh using Open3D backend."""
        try:
            import open3d as o3d
            
            # Load mesh
            mesh = o3d.io.read_triangle_mesh(str(input_path))
            
            if len(mesh.vertices) == 0:
                raise ValueError("No vertices found in mesh")
            
            # Export mesh
            success = o3d.io.write_triangle_mesh(str(output_path), mesh)
            
            if not success:
                raise Exception("Failed to write mesh")
            
            logger.info(f"Converted mesh using Open3D: {output_path}")
            return output_path
            
        except Exception as e:
            raise Exception(f"Open3D conversion failed: {e}")
    
    def _convert_with_pymeshlab(self, input_path: Path, output_path: Path) -> Path:
        """Convert mesh using PyMeshLab backend."""
        try:
            import pymeshlab
            
            # Create MeshSet and load mesh
            ms = pymeshlab.MeshSet()
            ms.load_new_mesh(str(input_path))
            
            if ms.number_meshes() == 0:
                raise ValueError("No meshes loaded")
            
            # Export mesh
            ms.save_current_mesh(str(output_path))
            
            logger.info(f"Converted mesh using PyMeshLab: {output_path}")
            return output_path
            
        except Exception as e:
            raise Exception(f"PyMeshLab conversion failed: {e}")
    
    def glb_to_obj(self, glb_path: Union[str, Path], obj_path: Optional[Union[str, Path]] = None) -> Path:
        """Convert GLB file to OBJ format.
        
        Args:
            glb_path: Path to input GLB file.
            obj_path: Path to output OBJ file. Auto-generated if None.
            
        Returns:
            Path to converted OBJ file.
        """
        glb_path = Path(glb_path)
        
        if obj_path is None:
            obj_path = glb_path.with_suffix('.obj')
        else:
            obj_path = Path(obj_path)
        
        return self.convert(glb_path, obj_path)
    
    def obj_to_glb(self, obj_path: Union[str, Path], glb_path: Optional[Union[str, Path]] = None) -> Path:
        """Convert OBJ file to GLB format.
        
        Args:
            obj_path: Path to input OBJ file.
            glb_path: Path to output GLB file. Auto-generated if None.
            
        Returns:
            Path to converted GLB file.
        """
        obj_path = Path(obj_path)
        
        if glb_path is None:
            glb_path = obj_path.with_suffix('.glb')
        else:
            glb_path = Path(glb_path)
        
        return self.convert(obj_path, glb_path)
    
    def get_supported_formats(self, backend: Optional[str] = None) -> dict:
        """Get supported input and output formats for a backend.
        
        Args:
            backend: Backend to check. Uses available backend if None.
            
        Returns:
            Dictionary with supported input and output formats.
        """
        if backend is None:
            backend = self.available_backends[0] if self.available_backends else None
        
        if backend == "trimesh":
            return {
                "input": [".obj", ".ply", ".stl", ".off", ".glb", ".gltf", ".dae", ".3mf"],
                "output": [".obj", ".ply", ".stl", ".off", ".glb", ".gltf", ".dae"]
            }
        elif backend == "open3d":
            return {
                "input": [".obj", ".ply", ".stl", ".off", ".gltf", ".glb"],
                "output": [".obj", ".ply", ".stl", ".off"]
            }
        elif backend == "pymeshlab":
            return {
                "input": [".obj", ".ply", ".stl", ".off", ".3ds", ".dae", ".x3d"],
                "output": [".obj", ".ply", ".stl", ".off", ".x3d"]
            }
        else:
            return {"input": [], "output": []}
    
    def is_conversion_supported(self, input_ext: str, output_ext: str, backend: Optional[str] = None) -> bool:
        """Check if a conversion is supported.
        
        Args:
            input_ext: Input file extension (e.g., ".glb").
            output_ext: Output file extension (e.g., ".obj").
            backend: Backend to check. Uses available backend if None.
            
        Returns:
            True if conversion is supported, False otherwise.
        """
        formats = self.get_supported_formats(backend)
        return (input_ext.lower() in formats["input"] and 
                output_ext.lower() in formats["output"])


# Global converter instance
_converter = None


def get_converter() -> MeshConverter:
    """Get global mesh converter instance."""
    global _converter
    if _converter is None:
        _converter = MeshConverter()
    return _converter


def convert_mesh(
    input_path: Union[str, Path], 
    output_path: Union[str, Path],
    backend: Optional[str] = None
) -> Path:
    """Convert mesh from one format to another.
    
    Args:
        input_path: Path to input mesh file.
        output_path: Path to output mesh file.
        backend: Preferred backend. Auto-select if None.
        
    Returns:
        Path to converted mesh file.
    """
    converter = get_converter()
    return converter.convert(input_path, output_path, backend)


def glb_to_obj(glb_path: Union[str, Path], obj_path: Optional[Union[str, Path]] = None) -> Path:
    """Convert GLB file to OBJ format.
    
    Args:
        glb_path: Path to input GLB file.
        obj_path: Path to output OBJ file. Auto-generated if None.
        
    Returns:
        Path to converted OBJ file.
    """
    converter = get_converter()
    return converter.glb_to_obj(glb_path, obj_path)


def obj_to_glb(obj_path: Union[str, Path], glb_path: Optional[Union[str, Path]] = None) -> Path:
    """Convert OBJ file to GLB format.
    
    Args:
        obj_path: Path to input OBJ file.
        glb_path: Path to output GLB file. Auto-generated if None.
        
    Returns:
        Path to converted GLB file.
    """
    converter = get_converter()
    return converter.obj_to_glb(obj_path, glb_path)


def get_supported_formats(backend: Optional[str] = None) -> dict:
    """Get supported mesh formats.
    
    Args:
        backend: Backend to check. Auto-select if None.
        
    Returns:
        Dictionary with supported input and output formats.
    """
    converter = get_converter()
    return converter.get_supported_formats(backend)


def list_available_backends() -> List[str]:
    """List available mesh conversion backends.
    
    Returns:
        List of available backend names.
    """
    converter = get_converter()
    return converter.available_backends.copy()


# Convenience function for the specific use case
def prepare_mesh_for_segmentation(mesh_path: Union[str, Path], preferred_format: str = "obj") -> Path:
    """Prepare mesh file for segmentation by converting to preferred format if needed.
    
    Args:
        mesh_path: Path to input mesh file.
        preferred_format: Preferred format for segmentation ("obj", "ply", etc.).
        
    Returns:
        Path to mesh file in preferred format.
    """
    mesh_path = Path(mesh_path)
    current_ext = mesh_path.suffix.lower()
    preferred_ext = f".{preferred_format.lower().lstrip('.')}"
    
    # If already in preferred format, return as-is
    if current_ext == preferred_ext:
        return mesh_path
    
    # Convert to preferred format
    output_path = mesh_path.with_suffix(preferred_ext)
    converter = get_converter()
    
    if not converter.is_conversion_supported(current_ext, preferred_ext):
        logger.warning(f"Conversion from {current_ext} to {preferred_ext} may not be supported")
    
    return converter.convert(mesh_path, output_path)