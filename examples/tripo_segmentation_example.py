#!/usr/bin/env python3
"""Tripo3D mesh segmentation example using the complete template-based pipeline.

This example demonstrates the full pipeline:
1. Image → 3D mesh (Tripo3D generation)
2. 3D mesh → segmented parts (Tripo3D segmentation) 
3. Parts → template matching
4. Template + parts → URDF assembly

Requirements:
- Tripo3D API key with segmentation access
- TRIPO_API_KEY environment variable
"""

import asyncio
import sys
from pathlib import Path
import argparse

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tung_playground.core.hero import Hero
from tung_playground.core.pipeline import Pipeline, PipelineConfig
from tung_playground.generation.tripo_generator import TripoGenerator
from tung_playground.decomposition.tripo_decomposer import TripoDecomposer
from tung_playground.decomposition.mock_decomposer import MockDecomposer
from tung_playground.matching.template_matcher import TemplatePartMatcher
from tung_playground.urdf.template_builder import TemplateURDFBuilder


async def create_hero_with_tripo_segmentation(
    hero_name: str,
    image_path: Path,
    template_type: str = "auto",
    quality: str = "standard",
    output_dir: Path = None
) -> Hero:
    """Create a hero using complete Tripo3D pipeline with segmentation.
    
    Args:
        hero_name: Name for the hero.
        image_path: Path to input image.
        template_type: Template type ("auto", "biped", "quadruped").
        quality: Generation quality ("standard", "high").
        output_dir: Directory to save hero files.
        
    Returns:
        Generated hero with URDF.
    """
    # Set up output directory
    if output_dir is None:
        output_dir = Path("heroes") / hero_name
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Create hero
    hero = Hero(
        name=hero_name,
        description=f"Hero generated with Tripo3D segmentation from {image_path.name}",
        hero_dir=output_dir
    )
    
    # Set input image
    hero.assets.input_image = image_path
    
    # Configure quality settings
    if quality == "high":
        generation_config = {
            "texture_quality": "HD",
            "face_limit": 20000,
            "enable_pbr": True
        }
        segmentation_config = {
            "detail_level": "high",
            "smoothness": 0.3,
            "generate_labels": True
        }
    else:
        generation_config = {
            "texture_quality": "standard", 
            "face_limit": 10000,
            "enable_pbr": False
        }
        segmentation_config = {
            "detail_level": "medium",
            "smoothness": 0.5,
            "generate_labels": True
        }
    
    # Create pipeline configuration
    config = PipelineConfig(
        max_workers=1,
        validate_inputs=True,
        validate_outputs=True,
        stage_configs={
            "tripo_generator": {
                "tripo": generation_config
            },
            "tripo_decomposer": {
                "tripo": segmentation_config
            },
            "template_matcher": {
                "preferred_template_type": template_type if template_type != "auto" else None,
                "auto_select_template": template_type == "auto",
                "min_match_score": 0.5,
                "min_coverage_ratio": 0.6
            },
            "template_urdf_builder": {
                "physics_engine": "mujoco",
                "generate_collision_meshes": True,
                "mesh_scale_factor": 1.0
            }
        }
    )
    
    pipeline = Pipeline("tripo_segmentation_pipeline", config)
    
    # Add pipeline stages
    pipeline.add_stages([
        TripoGenerator("tripo_generator"),           # Image → 3D mesh
        TripoDecomposer("tripo_decomposer"),         # 3D mesh → segmented parts  
        TemplatePartMatcher("template_matcher"),      # Parts → template matching
        TemplateURDFBuilder("template_urdf_builder"), # Template + parts → URDF
    ])
    
    print(f"🚀 Starting Tripo3D segmentation pipeline for {hero_name}")
    print(f"   Image: {image_path}")
    print(f"   Template: {template_type}")
    print(f"   Quality: {quality}")
    print(f"   Output: {output_dir}")
    
    # Execute pipeline
    try:
        results = await pipeline.execute(hero)
        
        # Check results
        if all(result.status.value == "completed" for result in results):
            print("\\n✅ Tripo3D segmentation pipeline completed successfully!")
            
            # Print detailed summary
            print(f"\\n📊 Pipeline Results:")
            print(f"   Hero: {hero.name}")
            print(f"   Status: {hero.status.value}")
            
            # Generation info
            if "generation" in hero.assets.metadata:
                gen_info = hero.assets.metadata["generation"]
                print(f"   3D Generation: {gen_info.get('generator', 'unknown')}")
                print(f"   Quality Score: {gen_info.get('quality_score', 'N/A'):.2f}")
                print(f"   Generation Time: {gen_info.get('generation_time', 0):.1f}s")
            
            # Decomposition info  
            if "decomposition" in hero.assets.metadata:
                decomp_info = hero.assets.metadata["decomposition"]
                print(f"   Segmentation: {decomp_info.get('decomposer', 'unknown')}")
                print(f"   Parts Found: {decomp_info.get('part_count', 0)}")
                print(f"   Segmentation Time: {decomp_info.get('decomposition_time', 0):.1f}s")
                
                # Show part names if available
                part_names = decomp_info.get('part_names', [])
                if part_names:
                    print(f"   Part Names: {', '.join(part_names[:5])}{'...' if len(part_names) > 5 else ''}")
            
            # Template matching info
            if "template_matching" in hero.assets.metadata:
                tm_info = hero.assets.metadata["template_matching"]
                print(f"   Template: {tm_info.get('template_name', 'Unknown')} ({tm_info.get('template_type', 'Unknown')})")
                print(f"   Match Score: {tm_info.get('overall_score', 0.0):.2f}")
                print(f"   Coverage: {tm_info.get('coverage_ratio', 0.0):.1%}")
                print(f"   Matched Parts: {tm_info.get('matched_parts', 0)}")
            
            # URDF info
            if "urdf" in hero.assets.metadata:
                urdf_info = hero.assets.metadata["urdf"]
                print(f"   URDF Links: {urdf_info.get('link_count', 0)}")
                print(f"   URDF Joints: {urdf_info.get('joint_count', 0)}")
                print(f"   URDF Path: {hero.assets.urdf}")
            
            # Save hero metadata
            hero.save()
            print(f"\\n💾 Hero metadata saved to: {hero.hero_dir}/hero.json")
            
            # Show file structure
            print(f"\\n📁 Generated Files:")
            if hero.assets.mesh_3d:
                print(f"   3D Mesh: {hero.assets.mesh_3d}")
            if hero.assets.parts:
                print(f"   Parts: {len(hero.assets.parts)} files in parts/")
            if hero.assets.urdf:
                print(f"   URDF: {hero.assets.urdf}")
                print(f"   Launch: {hero.assets.urdf.parent / f'{hero.name}.launch'}")
            
        else:
            print("\\n❌ Pipeline execution failed!")
            for result in results:
                if result.status.value == "failed":
                    print(f"   Stage '{result.stage_name}' failed: {result.error}")
    
    except Exception as e:
        print(f"\\n❌ Pipeline execution error: {e}")
        hero.update_status("failed", str(e))
    
    return hero


async def test_segmentation_only(mesh_path: Path, hero_name: str = "test_segmentation", use_mock: bool = False):
    """Test only the segmentation functionality with an existing mesh.
    
    Args:
        mesh_path: Path to existing 3D mesh file.
        hero_name: Name for test hero.
        use_mock: Whether to use mock decomposer instead of Tripo.
    """
    decomposer_type = "Mock" if use_mock else "Tripo3D"
    print(f"🧪 Testing {decomposer_type} segmentation with existing mesh: {mesh_path}")
    
    # Create test hero
    hero_dir = Path("heroes") / hero_name
    hero_dir.mkdir(parents=True, exist_ok=True)
    
    hero = Hero(
        name=hero_name,
        description=f"Test {decomposer_type} segmentation of {mesh_path.name}",
        hero_dir=hero_dir
    )
    
    # Set mesh asset
    hero.assets.mesh_3d = mesh_path
    
    # Create decomposer
    if use_mock:
        decomposer = MockDecomposer("test_mock_decomposer", {
            "num_parts": 5,
            "simulate_processing_time": True,
            "min_processing_time": 1.0,
            "max_processing_time": 3.0,
            "failure_rate": 0.0,
            "output_format": "obj",
            "generate_realistic_parts": True
        })
    else:
        decomposer = TripoDecomposer("test_tripo_decomposer", {
            "tripo": {
                "detail_level": "medium",
                "smoothness": 0.5,
                "generate_labels": True,
                "output_format": "obj"
            }
        })
    
    try:
        # Run segmentation
        result = await decomposer.process(hero)
        
        if result.status.value == "completed":
            print(f"✅ {decomposer_type} segmentation completed successfully!")
            parts_paths = result.outputs.get("parts_paths", [])
            part_names = result.outputs.get("part_names", [])
            
            print(f"   Parts generated: {len(parts_paths)}")
            for i, (path, name) in enumerate(zip(parts_paths, part_names)):
                print(f"   Part {i+1}: {name} → {path}")
        else:
            print(f"❌ {decomposer_type} segmentation failed: {result.error}")
    
    except Exception as e:
        print(f"❌ {decomposer_type} segmentation error: {e}")


async def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Tripo3D mesh segmentation pipeline example")
    parser.add_argument("--image", help="Path to input image for full pipeline")
    parser.add_argument("--mesh", help="Path to existing mesh for segmentation test")
    parser.add_argument("--name", default="tripo_hero", help="Hero name")
    parser.add_argument("--template", choices=["auto", "biped", "quadruped"], default="auto", help="Template type")
    parser.add_argument("--quality", choices=["standard", "high"], default="standard", help="Generation quality")
    parser.add_argument("--output", help="Output directory")
    parser.add_argument("--test-only", action="store_true", help="Test segmentation with existing mesh")
    parser.add_argument("--use-mock", action="store_true", help="Use mock decomposer instead of Tripo3D")
    
    args = parser.parse_args()
    
    print("🎯 Tripo3D Mesh Segmentation Pipeline")
    print("=" * 50)
    
    if args.test_only:
        if not args.mesh:
            print("❌ --mesh required for --test-only mode")
            return
        
        mesh_path = Path(args.mesh)
        if not mesh_path.exists():
            print(f"❌ Mesh file not found: {mesh_path}")
            return
        
        await test_segmentation_only(mesh_path, args.name, args.use_mock)
    
    else:
        # Full pipeline
        if not args.image:
            print("❌ --image required for full pipeline")
            return
        
        image_path = Path(args.image)
        if not image_path.exists():
            print(f"❌ Image file not found: {image_path}")
            return
        
        output_dir = Path(args.output) if args.output else None
        
        hero = await create_hero_with_tripo_segmentation(
            hero_name=args.name,
            image_path=image_path,
            template_type=args.template,
            quality=args.quality,
            output_dir=output_dir
        )
        
        if hero.assets.has_asset("urdf"):
            print(f"\\n🎉 Pipeline completed! URDF available at: {hero.assets.urdf}")
            print("\\n💡 Next steps:")
            print("   1. Load URDF in physics simulator (MuJoCo, PyBullet)")
            print("   2. Visualize with RViz or similar tools")
            print("   3. Train RL policies for locomotion")
        
    print("\\n✅ Example completed!")


if __name__ == "__main__":
    asyncio.run(main())