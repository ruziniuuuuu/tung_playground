#!/usr/bin/env python3
"""Meshy AI 3D Asset Generation Example

This example demonstrates how to use Meshy AI to generate high-quality 3D assets
from character images in the Tung Playground pipeline.

Features:
- Image-to-3D generation with Meshy AI
- Quality level selection (low/medium/high)
- Progress tracking and status updates
- Asset management and file organization
- Integration with the complete pipeline

Requirements:
- Meshy AI Pro subscription (free plan no longer supports task creation)
- MESHY_API_KEY environment variable
- Input image file (PNG, JPG, JPEG)
"""

import asyncio
import os
import sys
from pathlib import Path
from typing import Optional
import time

# Add project source to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / "src"))

# Import Tung Playground components
import tung_playground as tp
from tung_playground.generation import MeshyGenerator
from tung_playground.core.exceptions import ProcessingError, ConfigurationError


# Quality presets for different use cases
QUALITY_PRESETS = {
    "low": {
        "description": "Fast generation, low polygon count",
        "target_polycount": 5000,
        "should_texture": False,
        "enable_pbr": False,
        "estimated_time": "1-2 minutes",
        "estimated_credits": 10
    },
    "medium": {
        "description": "Balanced quality and speed",
        "target_polycount": 10000,
        "should_texture": True,
        "enable_pbr": False,
        "estimated_time": "2-4 minutes",
        "estimated_credits": 15
    },
    "high": {
        "description": "High quality with PBR materials",
        "target_polycount": 20000,
        "should_texture": True,
        "enable_pbr": True,
        "estimated_time": "3-5 minutes",
        "estimated_credits": 20
    }
}


async def generate_3d_asset(
    hero_name: str,
    image_path: str,
    quality: str = "medium",
    texture_prompt: Optional[str] = None
) -> bool:
    """Generate a 3D asset from an image using Meshy AI.
    
    Args:
        hero_name: Name for the generated hero
        image_path: Path to the input character image
        quality: Quality level - "low", "medium", or "high"
        texture_prompt: Optional text prompt to guide texture generation
        
    Returns:
        True if generation was successful, False otherwise
    """
    print(f"🎨 Meshy AI 3D Asset Generation")
    print(f"=" * 40)
    
    # Validate inputs
    if quality not in QUALITY_PRESETS:
        print(f"❌ Invalid quality level: {quality}")
        print(f"Available options: {', '.join(QUALITY_PRESETS.keys())}")
        return False
    
    image_file = Path(image_path)
    if not image_file.exists():
        print(f"❌ Image file not found: {image_path}")
        return False
    
    # Check API key
    api_key = os.getenv("MESHY_API_KEY")
    if not api_key:
        print(f"❌ MESHY_API_KEY environment variable not set")
        print(f"Please set your Meshy AI API key:")
        print(f"  export MESHY_API_KEY='your_api_key_here'")
        print(f"Get your API key at: https://www.meshy.ai/settings/api")
        return False
    
    preset = QUALITY_PRESETS[quality]
    
    print(f"📋 Generation Settings:")
    print(f"   Hero name: {hero_name}")
    print(f"   Image: {image_file.name}")
    print(f"   Quality: {quality} ({preset['description']})")
    print(f"   Polygons: {preset['target_polycount']:,}")
    print(f"   Texturing: {preset['should_texture']}")
    print(f"   PBR materials: {preset['enable_pbr']}")
    print(f"   Estimated time: {preset['estimated_time']}")
    print(f"   Estimated credits: {preset['estimated_credits']}")
    if texture_prompt:
        print(f"   Texture prompt: {texture_prompt}")
    
    try:
        # Create hero
        print(f"\n🦸 Creating hero...")
        hero = tp.create_hero(hero_name, str(image_file))
        print(f"✅ Hero created: {hero.name}")
        print(f"   Output directory: {hero.hero_dir}")
        
        # Configure Meshy generator
        print(f"\n⚙️  Configuring Meshy AI generator...")
        config = {
            "meshy": {
                "model": "meshy-5",
                "target_polycount": preset["target_polycount"],
                "should_texture": preset["should_texture"],
                "enable_pbr": preset["enable_pbr"],
                "poll_interval": 5,  # Check status every 5 seconds
                "max_wait_time": 900,  # 15 minute timeout
            }
        }
        
        # Add texture prompt if provided
        if texture_prompt:
            config["meshy"]["texture_prompt"] = texture_prompt
        
        generator = MeshyGenerator("meshy_generator", config)
        print(f"✅ Generator configured")
        
        # Validate inputs
        print(f"\n🔍 Validating inputs...")
        generator.validate_inputs(hero)
        print(f"✅ Input validation passed")
        
        # Start generation
        print(f"\n🚀 Starting 3D generation...")
        print(f"   This will consume approximately {preset['estimated_credits']} credits")
        print(f"   Progress updates will appear below:")
        print(f"")
        
        start_time = time.time()
        
        # Process the hero through generation
        result = await generator.process(hero)
        
        total_time = time.time() - start_time
        
        if result.status == tp.StageStatus.COMPLETED:
            print(f"\n🎉 Generation successful!")
            print(f"✅ Generated mesh: {result.outputs['mesh_path']}")
            print(f"✅ Quality score: {result.outputs['quality_score']:.2f}")
            print(f"✅ Generation time: {result.execution_time:.1f} seconds")
            print(f"✅ Total time: {total_time:.1f} seconds")
            
            # Check generated file
            mesh_path = result.outputs['mesh_path']
            if mesh_path.exists():
                file_size = mesh_path.stat().st_size
                print(f"✅ File size: {file_size:,} bytes")
                
                # Display generation details
                if result.metadata:
                    print(f"\n📊 Generation Details:")
                    task_id = result.metadata.get('task_id', 'N/A')
                    actual_polycount = result.metadata.get('polycount', 'N/A')
                    textured = result.metadata.get('textured', False)
                    
                    print(f"   Task ID: {task_id}")
                    print(f"   Polygon count: {actual_polycount}")
                    print(f"   Textured: {textured}")
                    print(f"   Model: {result.metadata.get('model', 'N/A')}")
                
                # Save hero metadata
                hero_file = hero.save()
                print(f"✅ Hero metadata saved: {hero_file}")
                
                print(f"\n🎯 Generated Assets:")
                print(f"   3D Mesh: {mesh_path}")
                print(f"   Hero Data: {hero_file}")
                print(f"   Directory: {hero.hero_dir}")
                
                print(f"\n🔄 Next Steps:")
                print(f"1. View the 3D mesh in your preferred 3D software")
                print(f"2. Continue with full pipeline:")
                print(f"   python examples/configurable_pipeline_demo.py run {hero_name} {image_path} --generator meshy")
                print(f"3. Generate URDF and visualize:")
                print(f"   python examples/hero_visualization_demo.py {hero_name}")
                
                return True
            else:
                print(f"❌ Generated mesh file not found: {mesh_path}")
                return False
                
        else:
            print(f"\n❌ Generation failed!")
            print(f"   Status: {result.status}")
            if result.error:
                print(f"   Error: {result.error}")
                
                # Provide helpful error guidance
                error_str = str(result.error)
                if "402" in error_str or "NoMorePendingTasks" in error_str:
                    print(f"\n💡 This error indicates you need a Meshy AI Pro subscription.")
                    print(f"   Visit: https://www.meshy.ai/settings/subscription")
                elif "401" in error_str:
                    print(f"\n💡 API key issue. Please check your MESHY_API_KEY.")
                elif "credits" in error_str.lower():
                    print(f"\n💡 Insufficient credits. Add more at: https://www.meshy.ai/settings/billing")
                    
            return False
            
    except ConfigurationError as e:
        print(f"❌ Configuration error: {e}")
        return False
    except ProcessingError as e:
        print(f"❌ Processing error: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


async def batch_generate(
    image_directory: str,
    quality: str = "low"
) -> None:
    """Generate 3D assets for multiple images in a directory.
    
    Args:
        image_directory: Directory containing input images
        quality: Quality level for all generations
    """
    print(f"🔄 Batch 3D Asset Generation")
    print(f"=" * 35)
    
    image_dir = Path(image_directory)
    if not image_dir.exists():
        print(f"❌ Image directory not found: {image_directory}")
        return
    
    # Find image files
    image_files = []
    for ext in ['.png', '.jpg', '.jpeg']:
        image_files.extend(image_dir.glob(f"*{ext}"))
        image_files.extend(image_dir.glob(f"*{ext.upper()}"))
    
    if not image_files:
        print(f"❌ No image files found in {image_directory}")
        return
    
    print(f"📁 Found {len(image_files)} images to process")
    
    successful = 0
    failed = 0
    
    for i, image_file in enumerate(image_files, 1):
        hero_name = f"batch_{image_file.stem}"
        
        print(f"\n{'='*50}")
        print(f"Processing {i}/{len(image_files)}: {image_file.name}")
        print(f"{'='*50}")
        
        success = await generate_3d_asset(
            hero_name=hero_name,
            image_path=str(image_file),
            quality=quality
        )
        
        if success:
            successful += 1
        else:
            failed += 1
            
        # Small delay between requests to be respectful to the API
        if i < len(image_files):
            print(f"\n⏱️  Waiting 10 seconds before next generation...")
            await asyncio.sleep(10)
    
    print(f"\n📊 Batch Generation Complete")
    print(f"   Successful: {successful}")
    print(f"   Failed: {failed}")
    print(f"   Total: {len(image_files)}")


async def main():
    """Main example function."""
    print("🎨 Meshy AI 3D Asset Generation Example")
    print("=" * 45)
    
    # Example 1: Generate a single 3D asset
    print("\n📝 Example 1: Single Asset Generation")
    print("-" * 35)
    
    # Check for existing test image
    test_images = [
        "heroes/tungtungtungtungtungtungtungtungtung_sahur/image.png",
        "heroes/tung_sahur_demo/input_image.png"
    ]
    
    example_image = None
    for img in test_images:
        if Path(img).exists():
            example_image = img
            break
    
    if example_image:
        print(f"Using existing test image: {example_image}")
        
        # Generate with medium quality
        success = await generate_3d_asset(
            hero_name="meshy_example_hero",
            image_path=example_image,
            quality="medium",
            texture_prompt="clean, detailed character textures"
        )
        
        if success:
            print(f"\n✅ Example completed successfully!")
        else:
            print(f"\n❌ Example failed - check error messages above")
    else:
        print(f"❌ No test images found")
        print(f"Place an image file in the project directory and update the path below:")
        print(f"""
# Example usage with your own image:
success = await generate_3d_asset(
    hero_name="my_character",
    image_path="path/to/your/character.png",
    quality="high",
    texture_prompt="fantasy character with detailed armor"
)
""")
    
    # Example 2: Different quality levels
    print(f"\n📝 Quality Level Reference")
    print(f"-" * 25)
    
    for quality, preset in QUALITY_PRESETS.items():
        print(f"{quality.upper()}:")
        print(f"  Description: {preset['description']}")
        print(f"  Polygons: {preset['target_polycount']:,}")
        print(f"  Time: {preset['estimated_time']}")
        print(f"  Credits: {preset['estimated_credits']}")
        print()


if __name__ == "__main__":
    # Check for API key before running
    if not os.getenv("MESHY_API_KEY"):
        print("❌ MESHY_API_KEY environment variable not set")
        print("\nTo run this example:")
        print("1. Get your API key from: https://www.meshy.ai/settings/api")
        print("2. Set the environment variable:")
        print("   export MESHY_API_KEY='your_api_key_here'")
        print("3. Ensure you have a Meshy AI Pro subscription")
        print("4. Run this script again")
        sys.exit(1)
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹️  Generation cancelled by user")
    except Exception as e:
        print(f"\n❌ Example failed: {e}")
        sys.exit(1)