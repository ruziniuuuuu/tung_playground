#!/usr/bin/env python3
"""Demo of using Meshy AI for image-to-3D generation.

This example shows how to use the MeshyGenerator to convert a hero image
into a 3D model using the Meshy AI API.

Requirements:
- Set MESHY_API_KEY environment variable
- Install tung_playground with: uv pip install -e .
"""

import asyncio
import os
import sys
from pathlib import Path
import typer
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn

# Add the project root to the path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import tung_playground as tp
from tung_playground.generation import MeshyGenerator
from tung_playground.core.hero import Hero
from tung_playground.core.pipeline import Pipeline

console = Console()
app = typer.Typer()


@app.command()
def generate(
    hero_name: str = typer.Argument(..., help="Name of the hero to generate"),
    image_path: str = typer.Argument(..., help="Path to the input image"),
    api_key: str = typer.Option(None, "--api-key", help="Meshy API key (or set MESHY_API_KEY env var)"),
    model: str = typer.Option("meshy-5", "--model", help="Meshy model to use"),
    quality: str = typer.Option("medium", "--quality", help="Quality level: low, medium, high"),
    textured: bool = typer.Option(True, "--textured/--no-texture", help="Generate textures"),
    output_dir: str = typer.Option("heroes", "--output-dir", help="Output directory for heroes"),
):
    """Generate a 3D model from an image using Meshy AI."""
    
    # Set API key if provided
    if api_key:
        os.environ["MESHY_API_KEY"] = api_key
    
    # Check API key
    if not os.getenv("MESHY_API_KEY"):
        console.print("[red]Error: Meshy API key is required.[/red]")
        console.print("Set the MESHY_API_KEY environment variable or use --api-key option.")
        console.print("Get your API key at: https://www.meshy.ai/api")
        raise typer.Exit(1)
    
    # Validate image path
    image_path = Path(image_path)
    if not image_path.exists():
        console.print(f"[red]Error: Image file not found: {image_path}[/red]")
        raise typer.Exit(1)
    
    # Set quality parameters
    quality_settings = {
        "low": {"target_polycount": 5000, "should_texture": False},
        "medium": {"target_polycount": 10000, "should_texture": textured},
        "high": {"target_polycount": 20000, "should_texture": textured, "enable_pbr": True}
    }
    
    if quality not in quality_settings:
        console.print(f"[red]Error: Invalid quality level: {quality}[/red]")
        console.print("Valid options: low, medium, high")
        raise typer.Exit(1)
    
    async def run_generation():
        try:
            # Create hero
            console.print(f"[cyan]Creating hero '{hero_name}' from {image_path}[/cyan]")
            hero = tp.create_hero(hero_name, str(image_path), base_dir=output_dir)
            
            # Configure Meshy generator
            meshy_config = {
                "meshy": {
                    "model": model,
                    **quality_settings[quality]
                }
            }
            
            # Create generator
            generator = MeshyGenerator("meshy_generator", meshy_config)
            
            # Run generation with progress
            with Progress(
                SpinnerColumn(),
                TextColumn("[progress.description]{task.description}"),
                console=console,
            ) as progress:
                task = progress.add_task("Generating 3D model with Meshy AI...", total=None)
                
                try:
                    result = await generator.process(hero)
                    
                    if result.status.name == "COMPLETED":
                        progress.update(task, description="[green]Generation completed![/green]")
                        
                        # Display results
                        console.print("\n[green]✅ 3D generation successful![/green]")
                        console.print(f"[cyan]Hero directory:[/cyan] {hero.directory}")
                        console.print(f"[cyan]Generated mesh:[/cyan] {result.outputs['mesh_path']}")
                        console.print(f"[cyan]Quality score:[/cyan] {result.outputs['quality_score']:.2f}")
                        console.print(f"[cyan]Generation time:[/cyan] {result.execution_time:.1f}s")
                        
                        # Show metadata
                        if result.metadata:
                            console.print("\n[cyan]Generation details:[/cyan]")
                            for key, value in result.metadata.items():
                                console.print(f"  {key}: {value}")
                        
                        # Suggest next steps
                        console.print("\n[yellow]Next steps:[/yellow]")
                        console.print("1. Run the complete pipeline:")
                        console.print(f"   python examples/hero_pipeline_demo.py {hero_name}")
                        console.print("2. Visualize the URDF:")
                        console.print(f"   python examples/visualize_hero.py {hero_name}")
                        
                        return True
                    else:
                        progress.update(task, description="[red]Generation failed[/red]")
                        console.print(f"\n[red]❌ Generation failed: {result.error}[/red]")
                        return False
                        
                except Exception as e:
                    progress.update(task, description="[red]Generation error[/red]")
                    console.print(f"\n[red]❌ Error during generation: {e}[/red]")
                    return False
        
        except Exception as e:
            console.print(f"[red]❌ Setup error: {e}[/red]")
            return False
    
    # Run the async generation
    success = asyncio.run(run_generation())
    
    if not success:
        raise typer.Exit(1)


@app.command()
def list_models():
    """List available Meshy AI models."""
    console.print("[cyan]Available Meshy AI models:[/cyan]")
    console.print("• meshy-5 (latest, best quality)")
    console.print("• meshy-4 (older version)")
    console.print("\n[yellow]Note:[/yellow] Use meshy-5 for best results.")


@app.command()
def check_credits():
    """Check your Meshy AI account credits."""
    api_key = os.getenv("MESHY_API_KEY")
    if not api_key:
        console.print("[red]Error: MESHY_API_KEY environment variable not set[/red]")
        raise typer.Exit(1)
    
    async def check_balance():
        import httpx
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    "https://api.meshy.ai/openapi/v1/user/balance",
                    headers={"Authorization": f"Bearer {api_key}"}
                )
                
                if response.status_code == 200:
                    data = response.json()
                    console.print(f"[green]Credits remaining: {data.get('credits', 'Unknown')}[/green]")
                else:
                    console.print(f"[red]Error checking credits: {response.status_code}[/red]")
                    console.print(response.text)
        
        except Exception as e:
            console.print(f"[red]Error: {e}[/red]")
    
    asyncio.run(check_balance())


if __name__ == "__main__":
    app()