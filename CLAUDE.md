# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Tung Playground is a Python project that enables AI-generated heroes to live in a physics simulator. The project uses MuJoCo for robotics simulation and is structured as a Python package.

## Development Commands

### Package Management
- **Install dependencies**: `uv sync` (uses uv.lock for dependency management)
- **Install in development mode**: `uv pip install -e .`

### Running Code
- **Import the package**: The main package is `tung_playground` located in `src/tung_playground/`
- **Current version**: Defined in `src/tung_playground/__init__.py` (currently 0.0.1)

## Project Structure

- `src/tung_playground/`: Main Python package source code
- `heroes/`: Directory containing hero definitions and assets
  - Each hero has its own subdirectory with placeholder files and images
- `docs/`: Documentation and image assets
- `pyproject.toml`: Project configuration using Hatchling build system
- `uv.lock`: Dependency lock file managed by uv package manager

## Key Dependencies

- **MuJoCo**: Primary dependency for physics simulation
- **Python**: Requires Python >=3.10
- **Build system**: Uses Hatchling for packaging

## Architecture Notes

The project implements a **template-based** AIGC hero simulation pipeline: **Image → 3D Mesh → Part Decomposition → Template Matching → URDF Assembly → MuJoCo/Isaac Lab → RL Policy**

This architecture replaces complex skeleton generation with a practical template matching approach, making the system more robust and maintainable.

### Framework Components
- **Core System**: Hero data model, pipeline orchestration, plugin registry
- **Generation Module**: Image-to-3D conversion (Wonder3D, Tripo3D, Meshy AI)
- **Decomposition Module**: 3D mesh part decomposition (semantic segmentation)
- **Template System**: Robot templates (biped, quadruped, multi-legged) with predefined kinematic structures
- **Matching Module**: Intelligent part-to-template matching using geometric and semantic features
- **URDF Assembly**: Template-based robot description generation with real part meshes
- **Simulation Module**: MuJoCo and Isaac Lab integration adapters
- **Training Module**: Reinforcement learning policy training framework
- **Utils Module**: Configuration management, logging, validation, file operations

### Template-Based Approach

The new template-based architecture offers several advantages over skeleton generation:

**Template System:**
- **Predefined Robots**: Biped, quadruped, and extensible multi-legged templates
- **Kinematic Structure**: Each template defines joints, links, and their relationships
- **Physical Properties**: Mass, inertia, joint limits, and actuator specifications
- **Extensibility**: Easy to add new template types (hexapod, snake-like, etc.)

**Smart Matching:**
- **Geometric Analysis**: Part size, shape, aspect ratios, and spatial relationships
- **Semantic Understanding**: Optional part labeling and classification
- **Scoring System**: Multi-criteria matching with configurable weights
- **Quality Assurance**: Coverage ratio and confidence scoring

**Benefits:**
- **Robustness**: No complex skeleton estimation - use proven kinematic structures
- **Quality**: Consistent joint placement and realistic physical properties
- **Speed**: Much faster than skeleton generation and rigging
- **Maintainability**: Easy to debug, customize, and extend templates

### Key Design Principles
- **Modular Pipeline**: Each stage is independently replaceable via plugin system
- **Configuration-Driven**: YAML-based hierarchical configuration with stage-specific settings
- **Async Execution**: Full async support with retry logic and batch processing
- **Type Safety**: Comprehensive type hints and Pydantic data validation
- **Extensibility**: Plugin architecture allows easy addition of new algorithms
- **Template-First Design**: Prioritize proven kinematic structures over generative approaches

## Code Style Guidelines

### Documentation Standards
- **Use Google-style docstrings** for all functions, classes, and modules
- **All comments and documentation must be in English**
- **All code comments must be in English** and follow Google-style formatting
- **Ensure code readability and extensibility** through clear, descriptive comments
- Include type information in docstrings when not using type hints
- Document parameters, return values, and exceptions
- Add inline comments for complex logic and important implementation details

Example:
```python
def create_hero(name: str, abilities: list[str]) -> Hero:
    """Creates a new hero with specified abilities.
    
    Args:
        name: The hero's name, must be unique.
        abilities: List of special abilities the hero possesses.
        
    Returns:
        A configured Hero instance ready for simulation.
        
    Raises:
        ValueError: If name is empty or abilities list is invalid.
    """
```

### Code Readability
- Use descriptive variable and function names that explain intent
- Keep functions focused on single responsibilities
- Limit function length to ~20-30 lines when possible
- Use meaningful intermediate variables instead of complex nested expressions
- Group related functionality into classes and modules

### Extensibility Design
- Design with composition over inheritance
- Use abstract base classes for defining interfaces
- Implement plugin-style architecture for hero types
- Keep configuration separate from implementation
- Use dependency injection for external dependencies

### Concise Implementation
- Avoid over-engineering simple solutions
- Prefer built-in Python features over custom implementations
- Use list/dict comprehensions when they improve clarity
- Eliminate unnecessary abstractions
- Follow the principle: "Make it work, make it right, make it fast"

## Python Standards

### Type Hints
- Use type hints for all function signatures
- Import types from `typing` module when needed
- Use `Union` or `|` for multiple possible types
- Document complex type structures

### Error Handling
- Use specific exception types, not bare `except:`
- Fail fast with clear error messages
- Use context managers for resource management
- Log errors with sufficient context

### Import Organization
```python
# Standard library imports
import os
from typing import Optional, Union

# Third-party imports
import mujoco
import numpy as np

# Local imports
from tung_playground.core import Hero
from tung_playground.utils import validation
```

## Template-Based Hero Generation

### Quick Start

The system now uses a template-based approach for robust robot generation:

```python
# Simple usage
from tung_playground import create_hero, create_template_pipeline

hero = create_hero("my_robot", "image.png")
pipeline = create_template_pipeline(template_type="biped")
await pipeline.execute(hero)
print(f"URDF generated: {hero.assets.urdf}")
```

### Available Templates

- **Biped Template**: Humanoid robots (16 links, 14 DOF)
- **Quadruped Template**: Four-legged animals (17 links, 12+ DOF)
- **Extensible**: Easy to add hexapod, snake-like, etc.

### Configuration

```python
config = {
    "template_matcher": {
        "preferred_template_type": "quadruped",
        "auto_select_template": False,
        "min_match_score": 0.6,
        "size_weight": 0.3,
        "shape_weight": 0.4,
        "semantic_weight": 0.2
    }
}
```

### Usage Example

```bash
python examples/template_matching_example.py --name test_hero --template auto
```

## Development Workflow

### Project Rules and Guidelines

1. **Template-First Design**: Always use template matching instead of skeleton generation
2. **No Standalone Documentation**: Avoid creating separate markdown files - use docs/ directory or CLAUDE.md
3. **Configuration-Driven**: Make systems configurable rather than hardcoded
4. **Async by Default**: Use async/await for all I/O operations
5. **Type Safety**: Always include type hints and use Pydantic for data validation
6. **Plugin Architecture**: Design components to be pluggable and replaceable
7. **Mock for Development**: Provide mock implementations for testing and development
8. **Pipeline-Based**: Structure processing as composable pipeline stages

### Git Commit Strategy
- **Feature-based commits**: Group related changes into logical commits
- **Descriptive messages**: Use clear, concise English commit messages
- **Atomic commits**: Each commit should be a complete, working unit
- **No monolithic commits**: Split large changes into focused commits

### Code Review Checklist
- [ ] Google-style docstrings present and complete
- [ ] Type hints on all function signatures
- [ ] Clear, descriptive naming throughout
- [ ] Single responsibility principle followed
- [ ] Error handling appropriate for context
- [ ] No hardcoded values (use configuration)
- [ ] Extensible design that allows future modifications
- [ ] Plugin registration for new pipeline stages
- [ ] Configuration schema updates for new features
- [ ] Input/output validation implemented

### Development Preferences
- **Prefer composition over inheritance** for flexibility
- **Use dependency injection** for external services and components
- **Implement comprehensive logging** at appropriate levels
- **Write defensive code** with proper validation and error handling
- **Design for testability** with clear interfaces and mockable dependencies
- **Follow async/await patterns** for I/O bound operations
- **Use dataclasses/Pydantic models** for structured data
- **Implement plugin patterns** for extensible architectures