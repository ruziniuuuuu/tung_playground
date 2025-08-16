#!/usr/bin/env python3
"""Mesh format conversion example and utility script.

This script demonstrates the mesh conversion utilities and provides a 
command-line interface for converting between different mesh formats.

Supported conversions depend on available backends:
- trimesh: GLB, GLTF, OBJ, PLY, STL, OFF, DAE, 3MF
- open3d: GLB, GLTF, OBJ, PLY, STL, OFF  
- pymeshlab: OBJ, PLY, STL, OFF, 3DS, DAE, X3D

Requirements:
Install at least one backend:
    pip install trimesh
    pip install open3d  
    pip install pymeshlab
"""

import sys
import argparse
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tung_playground.utils.mesh_converter import (
    MeshConverter, 
    glb_to_obj,
    list_available_backends,
    get_supported_formats
)


def convert_glb_to_obj_for_hero(hero_path: str):
    """Convert GLB mesh to OBJ for a specific hero.
    
    Args:
        hero_path: Path to hero directory.
    """
    hero_dir = Path(hero_path)
    glb_file = hero_dir / "generated_mesh.glb"
    
    if not glb_file.exists():
        print(f"❌ GLB file not found: {glb_file}")
        return
    
    print(f"🔄 Converting GLB to OBJ for hero: {hero_dir.name}")
    print(f"   Input: {glb_file}")
    
    try:
        obj_file = glb_to_obj(glb_file)
        print(f"✅ Conversion successful!")
        print(f"   Output: {obj_file}")
        print(f"   Size: {obj_file.stat().st_size:,} bytes")
        
        # Check if MTL file was also created
        mtl_file = obj_file.with_suffix('.mtl')
        if mtl_file.exists():
            print(f"   Materials: {mtl_file}")
        
        return obj_file
        
    except Exception as e:
        print(f"❌ Conversion failed: {e}")
        return None


def show_backend_info():
    """Display information about available backends."""
    print("🔧 Mesh Conversion Backend Information")
    print("=" * 50)
    
    backends = list_available_backends()
    
    if not backends:
        print("❌ No conversion backends available!")
        print("\nInstall at least one backend:")
        print("   pip install trimesh      # Recommended")
        print("   pip install open3d       # Good for scientific use")
        print("   pip install pymeshlab    # Full-featured")
        return
    
    print(f"✅ Available backends: {', '.join(backends)}")
    
    for backend in backends:
        print(f"\n📦 {backend.upper()} Backend:")
        formats = get_supported_formats(backend)
        print(f"   Input formats:  {', '.join(formats['input'])}")
        print(f"   Output formats: {', '.join(formats['output'])}")


def batch_convert_meshes(input_dir: str, output_format: str):
    """Batch convert all meshes in a directory.
    
    Args:
        input_dir: Directory containing mesh files.
        output_format: Target format (e.g., "obj", "ply").
    """
    input_path = Path(input_dir)
    
    if not input_path.exists():
        print(f"❌ Input directory not found: {input_path}")
        return
    
    # Find mesh files
    mesh_extensions = ['.glb', '.gltf', '.obj', '.ply', '.stl', '.off', '.dae']
    mesh_files = []
    
    for ext in mesh_extensions:
        mesh_files.extend(input_path.glob(f"**/*{ext}"))
    
    if not mesh_files:
        print(f"❌ No mesh files found in: {input_path}")
        return
    
    print(f"🔄 Batch converting {len(mesh_files)} mesh files to {output_format.upper()}")
    
    converter = MeshConverter()
    successful = 0
    
    for mesh_file in mesh_files:
        try:
            output_file = mesh_file.with_suffix(f'.{output_format.lower()}')
            
            # Skip if already in target format
            if mesh_file.suffix.lower() == f'.{output_format.lower()}':
                print(f"⏭️  Skipping {mesh_file.name} (already {output_format.upper()})")
                continue
            
            print(f"🔄 Converting: {mesh_file.name}")
            result = converter.convert(mesh_file, output_file)
            print(f"✅ → {result.name}")
            successful += 1
            
        except Exception as e:
            print(f"❌ Failed to convert {mesh_file.name}: {e}")
    
    print(f"\n📊 Batch conversion completed: {successful}/{len(mesh_files)} successful")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Mesh format conversion utility")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")
    
    # Convert command
    convert_parser = subparsers.add_parser("convert", help="Convert single mesh file")
    convert_parser.add_argument("input", help="Input mesh file")
    convert_parser.add_argument("output", help="Output mesh file")
    convert_parser.add_argument("--backend", choices=["trimesh", "open3d", "pymeshlab"], 
                               help="Conversion backend to use")
    
    # GLB to OBJ command
    glb_parser = subparsers.add_parser("glb-to-obj", help="Convert GLB to OBJ")
    glb_parser.add_argument("input", help="Input GLB file")
    glb_parser.add_argument("--output", help="Output OBJ file (optional)")
    
    # Hero conversion command
    hero_parser = subparsers.add_parser("hero", help="Convert hero's GLB to OBJ")
    hero_parser.add_argument("hero_path", help="Path to hero directory")
    
    # Batch conversion command
    batch_parser = subparsers.add_parser("batch", help="Batch convert meshes in directory")
    batch_parser.add_argument("input_dir", help="Input directory")
    batch_parser.add_argument("format", help="Target format (obj, ply, stl, etc.)")
    
    # Info command
    subparsers.add_parser("info", help="Show backend information")
    
    # Test command for the specific hero
    subparsers.add_parser("test", help="Test conversion with the specified hero")
    
    args = parser.parse_args()
    
    if args.command == "convert":
        try:
            converter = MeshConverter()
            result = converter.convert(args.input, args.output, args.backend)
            print(f"✅ Conversion successful: {result}")
        except Exception as e:
            print(f"❌ Conversion failed: {e}")
    
    elif args.command == "glb-to-obj":
        try:
            result = glb_to_obj(args.input, args.output)
            print(f"✅ GLB to OBJ conversion successful: {result}")
        except Exception as e:
            print(f"❌ Conversion failed: {e}")
    
    elif args.command == "hero":
        convert_glb_to_obj_for_hero(args.hero_path)
    
    elif args.command == "batch":
        batch_convert_meshes(args.input_dir, args.format)
    
    elif args.command == "info":
        show_backend_info()
    
    elif args.command == "test":
        # Test with the specified hero
        hero_path = "heroes/tungtungtungtungtungtungtungtungtung_sahur"
        print(f"🧪 Testing conversion with hero: {hero_path}")
        obj_file = convert_glb_to_obj_for_hero(hero_path)
        
        if obj_file:
            print(f"\n🎯 Ready for segmentation testing:")
            print(f"python examples/tripo_segmentation_example.py --mesh {obj_file} --test-only")
    
    else:
        parser.print_help()


if __name__ == "__main__":
    main()