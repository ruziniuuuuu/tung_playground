#!/usr/bin/env python3
"""Complete pipeline demonstration using the sample hero image."""

import asyncio
import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import tung_playground as tp
from tung_playground.plugins import mock_plugins  # Auto-registers mock plugins


async def main():
    """Demonstrate the complete AIGC hero pipeline."""
    
    # Setup logging
    tp.setup_logging(level="INFO", use_rich=True)
    logger = tp.get_logger(__name__)
    
    logger.info("🚀 Starting Tung Playground Full Pipeline Demo")
    logger.info("=" * 60)
    
    # Check sample image exists
    image_path = Path("heroes/tungtungtungtungtungtungtungtungtung_sahur/image.png")
    if not image_path.exists():
        logger.error(f"❌ Sample image not found: {image_path}")
        logger.info("Please ensure you're running from the project root directory")
        return
    
    # Create hero
    hero_name = "tung_sahur_demo"
    logger.info(f"📸 Creating hero: {hero_name}")
    
    hero = tp.create_hero(
        name=hero_name,
        image_path=str(image_path),
        hero_dir=f"heroes/{hero_name}"
    )
    
    logger.info(f"✅ Created hero: {hero.name} (ID: {hero.id[:8]}...)")
    logger.info(f"📁 Hero directory: {hero.hero_dir}")
    
    # Load configuration
    config = tp.load_config("default")
    logger.info(f"⚙️  Loaded configuration with {len(config)} sections")
    
    # Create pipeline with all stages
    logger.info("🔧 Building complete pipeline...")
    pipeline = tp.create_pipeline()
    
    # Add all mock pipeline stages
    try:
        # Import mock implementations
        from tung_playground.generation import MockGenerator
        from tung_playground.decomposition import MockDecomposer  
        from tung_playground.rigging import MockRigger
        from tung_playground.urdf import MockURDFBuilder
        from tung_playground.simulation import MockMuJoCoAdapter
        from tung_playground.training import MockTrainer
        
        # Configure and add stages
        pipeline.add_stage(MockGenerator("generation", config.get("stages", {}).get("generation", {})))
        pipeline.add_stage(MockDecomposer("decomposition", config.get("stages", {}).get("decomposition", {})))
        pipeline.add_stage(MockRigger("rigging", config.get("stages", {}).get("rigging", {})))
        pipeline.add_stage(MockURDFBuilder("urdf", config.get("stages", {}).get("urdf", {})))
        pipeline.add_stage(MockMuJoCoAdapter("simulation", config.get("stages", {}).get("simulation", {})))
        pipeline.add_stage(MockTrainer("training", config.get("stages", {}).get("training", {})))
        
        logger.info(f"✅ Pipeline configured with {pipeline.stage_count} stages:")
        for i, stage_name in enumerate(pipeline.stage_names, 1):
            logger.info(f"   {i}. {stage_name}")
        
    except ImportError as e:
        logger.error(f"❌ Failed to import pipeline stages: {e}")
        return
    
    logger.info("")
    logger.info("🎬 Executing complete pipeline...")
    logger.info("=" * 60)
    
    # Execute pipeline
    try:
        results = await pipeline.execute(hero)
        
        logger.info("")
        logger.info("📊 Pipeline Results:")
        logger.info("=" * 60)
        
        total_time = sum(r.execution_time for r in results)
        successful_stages = sum(1 for r in results if r.status == tp.StageStatus.COMPLETED)
        
        for i, result in enumerate(results, 1):
            status_emoji = "✅" if result.status == tp.StageStatus.COMPLETED else "❌"
            logger.info(f"{status_emoji} Stage {i}: {result.stage_name}")
            logger.info(f"   Status: {result.status}")
            logger.info(f"   Time: {result.execution_time:.2f}s")
            
            if result.error:
                logger.info(f"   Error: {result.error}")
            elif result.outputs:
                key_outputs = {k: v for k, v in result.outputs.items() 
                             if not str(k).endswith('_time')}
                if key_outputs:
                    logger.info(f"   Outputs: {list(key_outputs.keys())}")
            logger.info("")
        
        logger.info(f"🏁 Pipeline Summary:")
        logger.info(f"   Total stages: {len(results)}")
        logger.info(f"   Successful: {successful_stages}")
        logger.info(f"   Failed: {len(results) - successful_stages}")
        logger.info(f"   Total time: {total_time:.2f}s")
        logger.info("")
        
        # Show final hero status
        logger.info("🎭 Final Hero Status:")
        logger.info("=" * 60)
        logger.info(f"Hero: {hero.name}")
        logger.info(f"Status: {hero.status}")
        logger.info(f"Processing log entries: {len(hero.processing_log)}")
        
        # Show generated assets
        logger.info("")
        logger.info("📦 Generated Assets:")
        asset_types = [
            ("Input Image", tp.AssetType.INPUT_IMAGE),
            ("3D Mesh", tp.AssetType.MESH_3D),
            ("Parts", tp.AssetType.PARTS),
            ("Skeleton", tp.AssetType.SKELETON),
            ("URDF", tp.AssetType.URDF),
            ("Simulation Model", tp.AssetType.SIMULATION_MODEL),
            ("Trained Policy", tp.AssetType.TRAINED_POLICY)
        ]
        
        for asset_name, asset_type in asset_types:
            if hero.assets.has_asset(asset_type):
                asset_path = hero.assets.get_asset(asset_type)
                if isinstance(asset_path, list):
                    logger.info(f"✅ {asset_name}: {len(asset_path)} files")
                    for i, path in enumerate(asset_path[:3]):  # Show first 3
                        logger.info(f"   - {path.name}")
                    if len(asset_path) > 3:
                        logger.info(f"   - ... and {len(asset_path) - 3} more")
                else:
                    logger.info(f"✅ {asset_name}: {asset_path.name}")
            else:
                logger.info(f"❌ {asset_name}: Not generated")
        
        # Save hero metadata
        hero_file = hero.save()
        logger.info(f"💾 Saved hero metadata: {hero_file}")
        
        logger.info("")
        logger.info("🎉 Pipeline demo completed successfully!")
        logger.info(f"📁 Check the results in: {hero.hero_dir}")
        
    except Exception as e:
        logger.error(f"❌ Pipeline execution failed: {e}")
        import traceback
        logger.error(traceback.format_exc())


if __name__ == "__main__":
    asyncio.run(main())