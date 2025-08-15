#!/usr/bin/env python3
"""Simple test to verify the pipeline structure without external dependencies."""

import sys
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

def test_imports():
    """Test that all modules can be imported structurally."""
    try:
        # Test core imports (these should work without pydantic)
        from tung_playground.core import exceptions
        from tung_playground.utils import file_manager
        print("✅ Core modules import successfully")
        
        # Test pipeline module structure
        from tung_playground.generation import base
        from tung_playground.decomposition import base as decomp_base
        from tung_playground.rigging import base as rig_base
        from tung_playground.urdf import base as urdf_base
        from tung_playground.simulation import base as sim_base
        from tung_playground.training import base as train_base
        print("✅ Pipeline base modules import successfully")
        
        # Test mock implementations
        from tung_playground.generation import mock_generator
        from tung_playground.decomposition import mock_decomposer
        from tung_playground.rigging import mock_rigger
        from tung_playground.urdf import mock_builder
        from tung_playground.simulation import mock_mujoco
        from tung_playground.training import mock_trainer
        print("✅ Mock implementations import successfully")
        
        # Test plugin system
        from tung_playground.plugins import registry
        print("✅ Plugin system imports successfully")
        
        return True
        
    except ImportError as e:
        print(f"❌ Import failed: {e}")
        return False

def test_file_structure():
    """Test that all expected files exist."""
    base_path = Path(__file__).parent.parent
    
    expected_files = [
        "src/tung_playground/__init__.py",
        "src/tung_playground/core/hero.py", 
        "src/tung_playground/core/pipeline.py",
        "src/tung_playground/generation/mock_generator.py",
        "src/tung_playground/decomposition/mock_decomposer.py",
        "src/tung_playground/rigging/mock_rigger.py",
        "src/tung_playground/urdf/mock_builder.py",
        "src/tung_playground/simulation/mock_mujoco.py",
        "src/tung_playground/training/mock_trainer.py",
        "src/tung_playground/plugins/mock_plugins.py",
        "config/default.yaml",
        "heroes/tungtungtungtungtungtungtungtungtung_sahur/image.png"
    ]
    
    missing_files = []
    for file_path in expected_files:
        full_path = base_path / file_path
        if not full_path.exists():
            missing_files.append(file_path)
    
    if missing_files:
        print(f"❌ Missing files: {missing_files}")
        return False
    else:
        print(f"✅ All {len(expected_files)} expected files exist")
        return True

def test_mock_content():
    """Test that mock implementations have expected content."""
    base_path = Path(__file__).parent.parent
    
    # Test that mock generator creates OBJ content
    mock_gen_path = base_path / "src/tung_playground/generation/mock_generator.py"
    with open(mock_gen_path, 'r') as f:
        content = f.read()
        if "def _create_simple_humanoid_mesh" in content and "# Simple Humanoid Mesh" in content:
            print("✅ Mock generator has mesh creation logic")
        else:
            print("❌ Mock generator missing expected content")
            return False
    
    # Test that mock decomposer has part splitting logic
    mock_decomp_path = base_path / "src/tung_playground/decomposition/mock_decomposer.py"
    with open(mock_decomp_path, 'r') as f:
        content = f.read()
        if "head" in content and "torso" in content and "arms" in content and "legs" in content:
            print("✅ Mock decomposer has part splitting logic")
        else:
            print("❌ Mock decomposer missing expected content")
            return False
    
    # Test that mock rigger has skeleton creation
    mock_rig_path = base_path / "src/tung_playground/rigging/mock_rigger.py"
    with open(mock_rig_path, 'r') as f:
        content = f.read()
        if "_create_biped_skeleton" in content and "pelvis" in content and "left_hip" in content:
            print("✅ Mock rigger has skeleton creation logic")
        else:
            print("❌ Mock rigger missing expected content")
            return False
    
    return True

def verify_expected_pipeline_outputs():
    """Verify what outputs each stage should produce."""
    print("\n📋 Expected Pipeline Outputs:")
    print("=" * 50)
    
    stages = [
        ("1. Generation", "generated_mesh.obj"),
        ("2. Decomposition", "parts/ directory with head.obj, torso.obj, arms.obj, legs.obj"),
        ("3. Rigging", "skeleton.json with biped joint hierarchy"),
        ("4. URDF", "robot.urdf with links and joints"),
        ("5. Simulation", "scene.xml with MuJoCo scene"),
        ("6. Training", "policy.pkl with trained policy")
    ]
    
    for stage, output in stages:
        print(f"   {stage} → {output}")
    
    print(f"\n📁 Final hero directory structure should contain:")
    print("   heroes/tung_sahur_demo/")
    print("   ├── hero.json (metadata)")
    print("   ├── generated_mesh.obj")
    print("   ├── parts/")
    print("   │   ├── head.obj")
    print("   │   ├── torso.obj") 
    print("   │   ├── arms.obj")
    print("   │   └── legs.obj")
    print("   ├── skeleton.json")
    print("   ├── robot.urdf")
    print("   ├── scene.xml")
    print("   └── policy.pkl")

def main():
    """Run all tests."""
    print("🧪 Testing Tung Playground Pipeline Implementation")
    print("=" * 60)
    
    tests = [
        ("File Structure", test_file_structure),
        ("Module Imports", test_imports),
        ("Mock Content", test_mock_content)
    ]
    
    passed = 0
    for test_name, test_func in tests:
        print(f"\n🔍 Testing {test_name}...")
        if test_func():
            passed += 1
        else:
            print(f"❌ {test_name} test failed")
    
    print(f"\n📊 Test Results: {passed}/{len(tests)} passed")
    
    if passed == len(tests):
        print("🎉 All tests passed! Pipeline implementation is ready.")
        verify_expected_pipeline_outputs()
        print(f"\n🚀 To run the full pipeline:")
        print(f"   1. Install dependencies: uv sync")
        print(f"   2. Run demo: python examples/full_pipeline_demo.py")
    else:
        print("❌ Some tests failed. Please check the implementation.")

if __name__ == "__main__":
    main()