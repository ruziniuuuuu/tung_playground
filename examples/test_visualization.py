#!/usr/bin/env python3
"""Test script for visualization functionality.

This script tests the visualization setup without actually starting the server.
"""

import sys
from pathlib import Path

# Add src to path for development
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import tung_playground as tp
from rich.console import Console

console = Console()


def test_visualization_setup():
    """Test that visualization components are properly set up."""
    console.print("[bold blue]🧪 Testing Visualization Setup[/bold blue]")
    console.print("=" * 40)
    
    # Test 1: Check if visualization is available
    console.print(f"[cyan]📦 Visualization available: {tp.VISUALIZATION_AVAILABLE}[/cyan]")
    
    if not tp.VISUALIZATION_AVAILABLE:
        console.print("[red]❌ Visualization not available![/red]")
        return False
    
    # Test 2: Import visualization components
    try:
        from tung_playground.visualization import URDFVisualizer
        console.print("[green]✅ URDFVisualizer import successful[/green]")
    except ImportError as e:
        console.print(f"[red]❌ URDFVisualizer import failed: {e}[/red]")
        return False
    
    # Test 3: Load existing demo hero
    heroes_dir = Path("heroes")
    demo_hero_dir = heroes_dir / "tung_sahur_demo"
    
    if not demo_hero_dir.exists():
        console.print("[yellow]⚠️  Demo hero directory not found[/yellow]")
        return False
    
    urdf_file = demo_hero_dir / "robot.urdf"
    if not urdf_file.exists():
        console.print("[yellow]⚠️  URDF file not found[/yellow]")
        return False
    
    console.print(f"[green]✅ Found demo hero at: {demo_hero_dir}[/green]")
    
    # Test 4: Load hero
    try:
        metadata_file = demo_hero_dir / "hero.json"
        if metadata_file.exists():
            hero = tp.Hero.load(metadata_file)
        else:
            hero = tp.Hero(name="tung_sahur_demo", hero_dir=demo_hero_dir)
            hero.assets.set_asset(tp.AssetType.URDF, urdf_file)
        
        console.print(f"[green]✅ Hero loaded: {hero.name}[/green]")
    except Exception as e:
        console.print(f"[red]❌ Hero loading failed: {e}[/red]")
        return False
    
    # Test 5: Check hero has required assets
    if not hero.assets.has_asset(tp.AssetType.URDF):
        console.print("[red]❌ Hero missing URDF asset[/red]")
        return False
    
    console.print("[green]✅ Hero has URDF asset[/green]")
    
    # Test 6: Test visualization method (without starting server)
    try:
        # We can't actually start the server in this test environment,
        # but we can test that the method exists and imports work
        visualize_method = getattr(hero, 'visualize', None)
        if visualize_method is None:
            console.print("[red]❌ Hero.visualize method not found[/red]")
            return False
        
        console.print("[green]✅ Hero.visualize method available[/green]")
    except Exception as e:
        console.print(f"[red]❌ Visualization method test failed: {e}[/red]")
        return False
    
    # Test 7: Test that URDFVisualizer class is importable and has required methods
    try:
        # Test that the class has required methods
        required_methods = ['load_hero', 'load_urdf', 'run', 'stop']
        missing_methods = []
        
        for method in required_methods:
            if not hasattr(URDFVisualizer, method):
                missing_methods.append(method)
        
        if missing_methods:
            console.print(f"[red]❌ URDFVisualizer missing methods: {missing_methods}[/red]")
            return False
        
        console.print("[green]✅ URDFVisualizer has all required methods[/green]")
        
    except Exception as e:
        console.print(f"[red]❌ URDFVisualizer method check failed: {e}[/red]")
        return False
    
    console.print("\n[bold green]🎉 All visualization tests passed![/bold green]")
    console.print("\n[yellow]📋 To start the actual visualization, run:[/yellow]")
    console.print("[cyan]python examples/visualize_hero.py tung_sahur_demo[/cyan]")
    console.print("[cyan]python examples/hero_visualization_demo.py[/cyan]")
    
    return True


if __name__ == "__main__":
    success = test_visualization_setup()
    sys.exit(0 if success else 1)