#!/usr/bin/env python3
"""Basic usage example for Tung Playground framework."""

import asyncio
from pathlib import Path

import tung_playground as tp


async def main():
    """Demonstrate basic framework usage."""
    
    # Setup logging
    tp.setup_logging(level="INFO", use_rich=True)
    logger = tp.get_logger(__name__)
    
    logger.info("Starting Tung Playground example")
    
    # Create a hero from an image
    hero_name = "example_hero"
    image_path = "heroes/tungtungtungtungtungtungtungtungtung_sahur/image.png"
    
    if not Path(image_path).exists():
        logger.error(f"Example image not found: {image_path}")
        return
    
    # Create hero
    hero = tp.create_hero(
        name=hero_name,
        image_path=image_path
    )
    
    logger.info(f"Created hero: {hero.name} (ID: {hero.id})")
    
    # Load configuration
    config = tp.load_config("default")
    logger.info(f"Loaded configuration: {list(config.keys())}")
    
    # Create pipeline
    pipeline = tp.create_pipeline()
    
    # Note: In a real implementation, you would add actual pipeline stages here
    # For now, we just demonstrate the framework structure
    
    logger.info(f"Created pipeline with {pipeline.stage_count} stages")
    
    # List available plugins
    plugins = tp.list_plugins()
    logger.info(f"Available plugin types: {list(plugins.keys())}")
    
    # Save hero metadata
    hero_file = hero.save()
    logger.info(f"Saved hero metadata to: {hero_file}")
    
    # Load hero back
    loaded_hero = tp.Hero.load(hero_file)
    logger.info(f"Loaded hero: {loaded_hero.name}")
    
    logger.info("Example completed successfully!")


if __name__ == "__main__":
    asyncio.run(main())