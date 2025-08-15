#!/usr/bin/env python3
"""Example of template-based hero generation using the new architecture."""

import asyncio
import sys
from pathlib import Path
import argparse

# Add src to path to import tung_playground
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tung_playground.core.hero import Hero
from tung_playground.core.pipeline import Pipeline, PipelineConfig
from tung_playground.templates.biped import BipedTemplate
from tung_playground.templates.quadruped import QuadrupedTemplate
from tung_playground.templates.base import TemplateLibrary
from tung_playground.matching.template_matcher import TemplatePartMatcher
from tung_playground.urdf.template_builder import TemplateURDFBuilder
from tung_playground.generation.mock_generator import MockGenerator
from tung_playground.decomposition.mock_decomposer import MockDecomposer


async def create_template_based_hero(
    hero_name: str,
    image_path: Path,
    template_type: str = "auto",
    output_dir: Path = None
) -> Hero:
    """Create a hero using the template-based approach.
    
    Args:
        hero_name: Name for the hero.
        image_path: Path to input image.
        template_type: Template type to use ("biped", "quadruped", or "auto").
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
        description=f"Template-based hero generated from {image_path.name}",
        hero_dir=output_dir
    )
    
    # Set up input image
    hero.assets.input_image = image_path
    
    # Create pipeline with template-based stages
    config = PipelineConfig(
        max_workers=1,
        validate_inputs=True,
        validate_outputs=True,
        stage_configs={
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
    
    pipeline = Pipeline("template_based_generation", config)
    
    # Add pipeline stages
    pipeline.add_stages([
        MockGenerator("mock_3d_generator"),           # Image → 3D mesh
        MockDecomposer("mock_decomposer"),            # 3D mesh → parts
        TemplatePartMatcher("template_matcher"),       # Parts → template matching
        TemplateURDFBuilder("template_urdf_builder"),  # Template + parts → URDF
    ])
    
    print(f"Starting template-based generation for {hero_name}...")
    print(f"Template type: {template_type}")
    print(f"Output directory: {output_dir}")
    
    # Execute pipeline
    try:
        results = await pipeline.execute(hero)
        
        # Check results
        if all(result.status.value == "completed" for result in results):
            print("\\n✅ Hero generation completed successfully!")
            
            # Print summary
            print(f"\\n📊 Generation Summary:")
            print(f"   Hero: {hero.name}")
            print(f"   Status: {hero.status.value}")
            
            # Template matching info
            if "template_matching" in hero.assets.metadata:
                tm_info = hero.assets.metadata["template_matching"]
                print(f"   Template: {tm_info.get('template_name', 'Unknown')}")
                print(f"   Template Type: {tm_info.get('template_type', 'Unknown')}")
                print(f"   Match Score: {tm_info.get('overall_score', 0.0):.2f}")
                print(f"   Coverage: {tm_info.get('coverage_ratio', 0.0):.1%}")
                print(f"   Matched Parts: {tm_info.get('matched_parts', 0)}")
            
            # URDF info
            if "urdf" in hero.assets.metadata:
                urdf_info = hero.assets.metadata["urdf"]
                print(f"   Links: {urdf_info.get('link_count', 0)}")
                print(f"   Joints: {urdf_info.get('joint_count', 0)}")
                print(f"   URDF Path: {hero.assets.urdf}")
            
            # Save hero metadata
            hero.save()
            print(f"\\n💾 Hero saved to: {hero.hero_dir}/hero.json")
            
        else:
            print("\\n❌ Hero generation failed!")
            for result in results:
                if result.status.value == "failed":
                    print(f"   Stage '{result.stage_name}' failed: {result.error}")
    
    except Exception as e:
        print(f"\\n❌ Pipeline execution failed: {e}")
        hero.update_status("failed", str(e))
    
    return hero


async def demo_template_library():
    """Demonstrate the template library functionality."""
    print("\\n🏗️  Template Library Demo")
    print("=" * 50)
    
    # Create template library
    library = TemplateLibrary()
    
    # Add default templates
    biped = BipedTemplate()
    quadruped = QuadrupedTemplate()
    
    library.add_template(biped)
    library.add_template(quadruped)
    
    # Show library contents
    print(f"Available templates: {library.list_templates()}")
    
    # Demonstrate template properties
    for template_name in library.list_templates():
        template = library.get_template(template_name)
        print(f"\\n📋 Template: {template.name}")
        print(f"   Type: {template.template_type.value}")
        print(f"   Links: {len(template.links)}")
        print(f"   Joints: {len(template.joints)}")
        print(f"   Root link: {template.root_link}")
        print(f"   Expected parts: {template.get_expected_part_types()}")
        
        # Show some links
        print("   Sample links:")
        for i, (name, link) in enumerate(template.links.items()):
            if i >= 3:  # Show only first 3
                print("   ...")
                break
            print(f"     - {name}: {link.part_type.value} (mass: {link.mass}kg)")


async def compare_templates():
    """Compare biped and quadruped templates."""
    print("\\n🔍 Template Comparison")
    print("=" * 50)
    
    biped = BipedTemplate()
    quadruped = QuadrupedTemplate()
    
    print(f"{'Aspect':<20} {'Biped':<15} {'Quadruped':<15}")
    print("-" * 50)
    print(f"{'Links':<20} {len(biped.links):<15} {len(quadruped.links):<15}")
    print(f"{'Joints':<20} {len(biped.joints):<15} {len(quadruped.joints):<15}")
    print(f"{'Total Mass':<20} {sum(l.mass for l in biped.links.values()):<15.1f} {sum(l.mass for l in quadruped.links.values()):<15.1f}")
    
    # Compare part types
    biped_parts = set(link.part_type for link in biped.links.values())
    quad_parts = set(link.part_type for link in quadruped.links.values())
    
    print(f"\\nPart Types:")
    print(f"  Biped: {[pt.value for pt in biped_parts]}")
    print(f"  Quadruped: {[pt.value for pt in quad_parts]}")
    print(f"  Common: {[pt.value for pt in biped_parts & quad_parts]}")
    print(f"  Unique to Biped: {[pt.value for pt in biped_parts - quad_parts]}")
    print(f"  Unique to Quadruped: {[pt.value for pt in quad_parts - biped_parts]}")


async def main():
    """Main demonstration function."""
    parser = argparse.ArgumentParser(description="Template-based hero generation example")
    parser.add_argument("--demo-only", action="store_true", help="Run demo without generating hero")
    parser.add_argument("--name", default="template_hero", help="Hero name")
    parser.add_argument("--template", choices=["auto", "biped", "quadruped"], default="auto", help="Template type")
    parser.add_argument("--image", help="Path to input image")
    parser.add_argument("--output", help="Output directory")
    
    args = parser.parse_args()
    
    print("🚀 Tung Playground - Template-Based Hero Generation")
    print("=" * 60)
    
    # Run template demonstrations
    await demo_template_library()
    await compare_templates()
    
    if not args.demo_only:
        # Create a hero using template matching
        print("\\n🎯 Creating Template-Based Hero")
        print("=" * 50)
        
        # Use provided image or create a mock one
        if args.image:
            image_path = Path(args.image)
            if not image_path.exists():
                print(f"❌ Image not found: {image_path}")
                return
        else:
            # Create mock image path (the mock generator doesn't actually need the file)
            image_path = Path("mock_image.png")
            print(f"ℹ️  Using mock image path: {image_path}")
        
        output_dir = Path(args.output) if args.output else None
        
        hero = await create_template_based_hero(
            hero_name=args.name,
            image_path=image_path,
            template_type=args.template,
            output_dir=output_dir
        )
        
        # Show final results
        if hero.assets.has_asset("urdf"):
            print(f"\\n🎉 Generated URDF available at: {hero.assets.urdf}")
            print("\\n💡 Next steps:")
            print("   1. Load the URDF in a physics simulator (MuJoCo, PyBullet, etc.)")
            print("   2. Visualize with RViz or similar tools")
            print("   3. Train RL policies for locomotion")
            print("   4. Customize joint limits and physics parameters")
    
    print("\\n✅ Demo completed!")


if __name__ == "__main__":
    asyncio.run(main())