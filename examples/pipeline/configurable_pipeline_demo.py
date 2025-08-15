#!/usr/bin/env python3
"""Configurable pipeline demonstration supporting both mock and real generators.

This script allows you to run the complete pipeline with either:
- Mock generators (fast, for testing)
- Real generators like Meshy AI (requires API keys)
"""

import asyncio
import os
import sys
from pathlib import Path
import typer
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TaskProgressColumn

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import tung_playground as tp
from tung_playground.plugins import mock_plugins  # Auto-registers mock plugins

console = Console()
app = typer.Typer()


@app.command()
def run(
    hero_name: str = typer.Argument(..., help="Name of the hero to create"),
    image_path: str = typer.Argument(..., help="Path to the input image"),
    generator: str = typer.Option("mock", "--generator", "-g", help="Generator type: mock, meshy, tripo"),
    meshy_api_key: str = typer.Option(None, "--meshy-key", help="Meshy API key (or set MESHY_API_KEY env var)"),
    tripo_api_key: str = typer.Option(None, "--tripo-key", help="Tripo3D API key (or set FAL_KEY/TRIPO_API_KEY env var)"),
    stages: str = typer.Option("all", "--stages", "-s", help="Stages to run: all, generation, decomposition, etc."),
    quality: str = typer.Option("medium", "--quality", "-q", help="Quality level for real generators: low, medium, high"),
    output_dir: str = typer.Option("heroes", "--output-dir", "-o", help="Output directory"),
    config_file: str = typer.Option("default", "--config", "-c", help="Configuration file to use"),
):
    """Run the hero generation pipeline with configurable generators."""
    
    # Setup API keys if needed
    if generator == "meshy":
        if meshy_api_key:
            os.environ["MESHY_API_KEY"] = meshy_api_key
        
        if not os.getenv("MESHY_API_KEY"):
            console.print("[red]Error: Meshy API key is required for Meshy generator.[/red]")
            console.print("Set MESHY_API_KEY environment variable or use --meshy-key option.")
            console.print("Get your API key at: https://www.meshy.ai/api")
            raise typer.Exit(1)
    
    elif generator == "tripo":
        if tripo_api_key:
            os.environ["FAL_KEY"] = tripo_api_key
        
        if not (os.getenv("FAL_KEY") or os.getenv("TRIPO_API_KEY")):
            console.print("[red]Error: Tripo3D API key is required for Tripo generator.[/red]")
            console.print("Set FAL_KEY or TRIPO_API_KEY environment variable or use --tripo-key option.")
            console.print("Get your API key at: https://platform.tripo3d.ai/api-keys")
            raise typer.Exit(1)
    
    # Validate image path
    image_path = Path(image_path)
    if not image_path.exists():
        console.print(f"[red]Error: Image file not found: {image_path}[/red]")
        raise typer.Exit(1)
    
    # Parse stages
    if stages.lower() == "all":
        stage_list = ["generation", "decomposition", "rigging", "urdf", "simulation", "training"]
    else:
        stage_list = [s.strip() for s in stages.split(",")]
    
    console.print(f"[cyan]Starting pipeline for hero '{hero_name}'[/cyan]")
    console.print(f"[cyan]Generator: {generator}[/cyan]")
    console.print(f"[cyan]Stages: {', '.join(stage_list)}[/cyan]")
    console.print()
    
    async def run_pipeline():
        try:
            # Setup logging
            tp.setup_logging(level="INFO", use_rich=True)
            logger = tp.get_logger(__name__)
            
            # Create hero
            hero = tp.create_hero(hero_name, str(image_path), base_dir=output_dir)
            logger.info(f"Created hero: {hero.name}")
            
            # Load configuration
            config = tp.load_config(config_file)
            
            # Quality settings for real generators
            meshy_quality_settings = {
                "low": {"target_polycount": 5000, "should_texture": False},
                "medium": {"target_polycount": 10000, "should_texture": True},
                "high": {"target_polycount": 20000, "should_texture": True, "enable_pbr": True}
            }
            
            tripo_quality_settings = {
                "low": {"texture_quality": "standard", "face_limit": 5000, "enable_pbr": False},
                "medium": {"texture_quality": "standard", "face_limit": 10000, "enable_pbr": False},
                "high": {"texture_quality": "HD", "face_limit": 20000, "enable_pbr": True, "enable_quad": True}
            }
            
            # Create pipeline
            pipeline = tp.create_pipeline()
            
            # Configure stages based on generator type
            stage_configs = config.get("stages", {})
            
            with Progress(
                SpinnerColumn(),
                TextColumn("[progress.description]{task.description}"),
                BarColumn(),
                TaskProgressColumn(),
                console=console,
            ) as progress:
                
                setup_task = progress.add_task("Setting up pipeline...", total=len(stage_list))
                
                for stage_name in stage_list:
                    stage_config = stage_configs.get(stage_name, {}).copy()
                    
                    if stage_name == "generation":
                        if generator == "mock":
                            from tung_playground.generation import MockGenerator
                            stage = MockGenerator("generation", stage_config)
                        elif generator == "meshy":
                            from tung_playground.generation import MeshyGenerator
                            # Apply quality settings to meshy config
                            if "meshy" not in stage_config:
                                stage_config["meshy"] = {}
                            stage_config["meshy"].update(meshy_quality_settings[quality])
                            stage = MeshyGenerator("generation", stage_config)
                        elif generator == "tripo":
                            from tung_playground.generation import TripoGenerator
                            # Apply quality settings to tripo config
                            if "tripo" not in stage_config:
                                stage_config["tripo"] = {}
                            stage_config["tripo"].update(tripo_quality_settings[quality])
                            stage = TripoGenerator("generation", stage_config)
                        else:
                            console.print(f"[red]Error: Unknown generator: {generator}[/red]")
                            return False
                    
                    elif stage_name == "decomposition":
                        from tung_playground.decomposition import MockDecomposer
                        stage = MockDecomposer("decomposition", stage_config)
                    
                    elif stage_name == "rigging":
                        from tung_playground.rigging import MockRigger
                        stage = MockRigger("rigging", stage_config)
                    
                    elif stage_name == "urdf":
                        from tung_playground.urdf import MockURDFBuilder
                        stage = MockURDFBuilder("urdf", stage_config)
                    
                    elif stage_name == "simulation":
                        from tung_playground.simulation import MockMuJoCoAdapter
                        stage = MockMuJoCoAdapter("simulation", stage_config)
                    
                    elif stage_name == "training":
                        from tung_playground.training import MockTrainer
                        stage = MockTrainer("training", stage_config)
                    
                    else:
                        console.print(f"[red]Error: Unknown stage: {stage_name}[/red]")
                        return False
                    
                    pipeline.add_stage(stage)
                    progress.advance(setup_task)
                
                progress.update(setup_task, description="[green]Pipeline setup complete[/green]")
                
                # Execute pipeline
                execution_task = progress.add_task("Executing pipeline...", total=len(stage_list))
                
                results = []
                for i, stage in enumerate(pipeline.stages):
                    progress.update(execution_task, description=f"Running {stage.name}...")
                    
                    try:
                        result = await stage.process(hero)
                        results.append(result)
                        
                        if result.status == tp.StageStatus.COMPLETED:
                            progress.update(execution_task, description=f"[green]✅ {stage.name} completed[/green]")
                        else:
                            progress.update(execution_task, description=f"[red]❌ {stage.name} failed[/red]")
                            console.print(f"[red]Stage {stage.name} failed: {result.error}[/red]")
                            
                    except Exception as e:
                        console.print(f"[red]Error in stage {stage.name}: {e}[/red]")
                        return False
                    
                    progress.advance(execution_task)
                
                progress.update(execution_task, description="[green]Pipeline execution complete[/green]")
            
            # Display results
            console.print("\n[green]🎉 Pipeline completed![/green]")
            console.print(f"[cyan]Hero directory:[/cyan] {hero.directory}")
            
            # Show results summary
            successful = sum(1 for r in results if r.status == tp.StageStatus.COMPLETED)
            total_time = sum(r.execution_time for r in results)
            
            console.print(f"\n[cyan]Results Summary:[/cyan]")
            console.print(f"  Stages completed: {successful}/{len(results)}")
            console.print(f"  Total time: {total_time:.1f}s")
            
            # Show generated assets
            console.print(f"\n[cyan]Generated Assets:[/cyan]")
            asset_types = [
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
                        console.print(f"  ✅ {asset_name}: {len(asset_path)} files")
                    else:
                        console.print(f"  ✅ {asset_name}: {asset_path.name}")
                else:
                    console.print(f"  ❌ {asset_name}: Not generated")
            
            # Save hero
            hero_file = hero.save()
            console.print(f"\n[cyan]Hero saved to:[/cyan] {hero_file}")
            
            # Suggest next steps
            console.print(f"\n[yellow]Next steps:[/yellow]")
            if hero.assets.has_asset(tp.AssetType.URDF):
                console.print(f"1. Visualize URDF: python examples/visualize_hero.py {hero_name}")
            console.print(f"2. Check hero directory: {hero.directory}")
            
            return True
            
        except Exception as e:
            console.print(f"[red]❌ Pipeline error: {e}[/red]")
            import traceback
            console.print(traceback.format_exc())
            return False
    
    # Run the pipeline
    success = asyncio.run(run_pipeline())
    
    if not success:
        raise typer.Exit(1)


@app.command()
def generators():
    """List available generators and their requirements."""
    console.print("[cyan]Available Generators:[/cyan]\n")
    
    console.print("[green]mock[/green] - Mock generator (fast, for testing)")
    console.print("  • No API key required")
    console.print("  • Generates simple procedural meshes")
    console.print("  • Good for testing pipeline\n")
    
    console.print("[green]meshy[/green] - Meshy AI generator")
    console.print("  • Requires MESHY_API_KEY environment variable")
    console.print("  • High-quality AI-generated 3D models")
    console.print("  • Supports texturing and PBR materials")
    console.print("  • Get API key at: https://www.meshy.ai/api\n")
    
    console.print("[green]tripo[/green] - Tripo3D AI generator")
    console.print("  • Requires FAL_KEY or TRIPO_API_KEY environment variable")
    console.print("  • Fast AI-generated 3D models")
    console.print("  • Supports HD textures and PBR materials")
    console.print("  • Get API key at: https://platform.tripo3d.ai/api-keys\n")


@app.command()
def stages():
    """List available pipeline stages."""
    console.print("[cyan]Available Pipeline Stages:[/cyan]\n")
    
    stages_info = [
        ("generation", "Convert 2D image to 3D mesh"),
        ("decomposition", "Split mesh into body parts"),
        ("rigging", "Create skeleton with joints"),
        ("urdf", "Generate robot description file"),
        ("simulation", "Setup simulation environment"),
        ("training", "Train RL locomotion policy"),
    ]
    
    for stage_name, description in stages_info:
        console.print(f"[green]{stage_name}[/green] - {description}")
    
    console.print(f"\n[yellow]Usage examples:[/yellow]")
    console.print("  --stages all                    # Run all stages")
    console.print("  --stages generation,urdf        # Run only generation and URDF")
    console.print("  --stages generation             # Run only generation")


if __name__ == "__main__":
    app()