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

The project follows a standard Python package structure with:
- Package metadata and dependencies managed via `pyproject.toml`
- Source code organized under `src/` layout
- Heroes stored as separate entities in the `heroes/` directory
- Version management through `__init__.py`

## Code Style Guidelines

### Documentation Standards
- **Use Google-style docstrings** for all functions, classes, and modules
- **All comments and documentation must be in English**
- Include type information in docstrings when not using type hints
- Document parameters, return values, and exceptions

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

## Development Workflow

### Code Review Checklist
- [ ] Google-style docstrings present and complete
- [ ] Type hints on all function signatures
- [ ] Clear, descriptive naming throughout
- [ ] Single responsibility principle followed
- [ ] Error handling appropriate for context
- [ ] No hardcoded values (use configuration)
- [ ] Extensible design that allows future modifications