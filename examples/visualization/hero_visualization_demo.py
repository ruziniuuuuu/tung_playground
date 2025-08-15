#!/usr/bin/env python3
"""Comprehensive hero visualization demo.

This script demonstrates the complete visualization workflow:
1. Generate a hero using the pipeline
2. Visualize the result interactively
3. Show different visualization options

Usage:
    python examples/hero_visualization_demo.py
"""

import sys
import asyncio
from pathlib import Path

# Add src to path for development
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import tung_playground as tp
from rich.console import Console

console = Console()


async def main():
    """Main demo function."""
    console.print("[bold blue]🎬 Hero Visualization Demo[/bold blue]")
    console.print("=" * 50)
    
    # Check if visualization is available
    if not tp.VISUALIZATION_AVAILABLE:
        console.print(
            "[red]❌ Visualization dependencies not found![/red]\n"
            "Install with: [bold]pip install tung_playground[vis][/bold]"
        )
        return
    
    # Step 1: Check for existing demo hero
    demo_hero_name = "tung_sahur_demo"
    heroes_dir = Path("heroes")
    demo_hero_dir = heroes_dir / demo_hero_name
    
    if demo_hero_dir.exists() and (demo_hero_dir / "robot.urdf").exists():
        console.print(f"[green]✅ Found existing demo hero: {demo_hero_name}[/green]")
        
        # Load existing hero
        metadata_file = demo_hero_dir / "hero.json"
        if metadata_file.exists():
            hero = tp.Hero.load(metadata_file)
        else:
            # Create hero object manually
            hero = tp.Hero(name=demo_hero_name, hero_dir=demo_hero_dir)
            # Set URDF asset
            hero.assets.set_asset(tp.AssetType.URDF, demo_hero_dir / "robot.urdf")
    else:
        console.print("[yellow]No existing demo hero found. Generating new one...[/yellow]")
        
        # Step 2: Generate a new hero
        console.print("[cyan]🎨 Creating demo hero...[/cyan]")
        
        # Use the sample image
        sample_image = Path("heroes/tungtungtungtungtungtungtungtungtung_sahur/image.png")
        if not sample_image.exists():
            console.print("[red]❌ Sample image not found. Please run the full pipeline demo first.[/red]")
            return
        
        # Create hero
        hero = tp.create_hero(
            name=demo_hero_name,
            image_path=str(sample_image)
        )
        
        console.print(f"[green]✅ Created hero: {hero.name}[/green]")
        
        # Step 3: Generate URDF using pipeline
        console.print("[cyan]🔧 Running pipeline to generate URDF...[/cyan]")
        
        # Load configuration and create pipeline
        config = tp.load_config("default")
        pipeline = tp.create_pipeline("default")
        
        # Add required stages for URDF generation
        from tung_playground.generation.mock_generator import MockGenerator
        from tung_playground.decomposition.mock_decomposer import MockDecomposer
        from tung_playground.rigging.mock_rigger import MockRigger
        from tung_playground.urdf.mock_builder import MockURDFBuilder
        
        pipeline.add_stage(MockGenerator("generation"))
        pipeline.add_stage(MockDecomposer("decomposition"))
        pipeline.add_stage(MockRigger("rigging"))
        pipeline.add_stage(MockURDFBuilder("urdf"))
        
        # Execute pipeline up to URDF generation
        console.print("[yellow]⚙️  Executing pipeline stages...[/yellow]")
        try:
            results = await pipeline.execute(hero)
            console.print("[green]✅ Pipeline completed successfully![/green]")
        except Exception as e:
            console.print(f"[red]❌ Pipeline failed: {e}[/red]")
            return
    
    # Step 4: Display hero information
    console.print("\n[bold]📋 Hero Information:[/bold]")
    console.print(f"  Name: {hero.name}")
    console.print(f"  Directory: {hero.hero_dir}")
    console.print(f"  Status: {hero.status.value}")
    
    # List available assets
    console.print("\n[bold]📦 Available Assets:[/bold]")
    for asset_type in tp.AssetType:
        if hero.assets.has_asset(asset_type):
            asset_path = hero.assets.get_asset(asset_type)
            if isinstance(asset_path, list):
                console.print(f"  ✅ {asset_type.value}: {len(asset_path)} files")
            else:
                console.print(f"  ✅ {asset_type.value}: {asset_path.name}")
        else:
            console.print(f"  ❌ {asset_type.value}: Not available")
    
    # Step 5: Start visualization
    console.print("\n[bold green]🎯 Starting Interactive Visualization[/bold green]")
    console.print("=" * 50)
    console.print("[blue]📡 Server URL: http://localhost:8080[/blue]")
    console.print("[yellow]💡 Features available in the web interface:[/yellow]")
    console.print("  • Interactive joint control sliders")
    console.print("  • Toggle visual/collision meshes")
    console.print("  • Reset robot to initial pose")
    console.print("  • Real-time robot manipulation")
    console.print("\n[dim]Press Ctrl+C to stop the visualization server[/dim]")
    
    try:
        # Start the visualization
        hero.visualize(port=8080, blocking=True)
    except KeyboardInterrupt:
        console.print("\n[yellow]👋 Visualization stopped[/yellow]")
    except Exception as e:
        console.print(f"[red]❌ Visualization error: {e}[/red]")


if __name__ == "__main__":
    asyncio.run(main())