# Tung Playground Framework

A comprehensive, modular framework for generating AI heroes from images and training them in physics simulators using reinforcement learning.

## Architecture Overview

The framework implements a complete pipeline: **Image → 3D Mesh → Part Decomposition → Skeleton → URDF → MuJoCo/Isaac Lab → RL Policy**

## Project Structure

```
src/tung_playground/
├── core/                    # Core framework components
│   ├── hero.py             # Hero data model and asset management
│   ├── pipeline.py         # Pipeline orchestration and stages
│   └── exceptions.py       # Custom exception classes
├── generation/             # Image-to-3D generation
│   ├── base.py            # Abstract base classes
│   ├── wonder3d.py        # Wonder3D integration (to implement)
│   └── commercial.py      # Commercial API wrappers (to implement)
├── decomposition/         # Part decomposition
│   ├── base.py           # Abstract base classes
│   └── partcrafter.py    # PartCrafter integration (to implement)
├── rigging/              # Skeleton generation
│   ├── base.py          # Abstract base classes
│   └── auto_rigger.py   # Automatic rigging (to implement)
├── urdf/                # URDF generation
│   ├── base.py         # Abstract base classes
│   └── builder.py      # URDF construction (to implement)
├── simulation/          # Simulation adapters
│   ├── base.py         # Abstract base classes
│   ├── mujoco_adapter.py   # MuJoCo integration (to implement)
│   └── isaac_adapter.py    # Isaac Lab integration (to implement)
├── training/            # RL training
│   ├── base.py         # Abstract base classes
│   └── trainers.py     # Training implementations (to implement)
├── utils/               # Utility modules
│   ├── config.py       # Configuration management
│   ├── logging.py      # Logging utilities
│   ├── validation.py   # Validation utilities
│   └── file_manager.py # File operations
└── plugins/             # Plugin system
    └── registry.py      # Plugin registration and discovery
```

## Key Features

### 1. Modular Architecture
- Each pipeline stage is independently replaceable
- Plugin-based system for easy extension
- Clean separation of concerns

### 2. Configuration Management
- YAML-based configuration with hierarchical merging
- Stage-specific configuration support
- User configuration overrides

### 3. Hero Asset Management
- Centralized asset tracking throughout pipeline
- Automatic directory structure creation
- Metadata persistence and retrieval

### 4. Pipeline Orchestration
- Async execution with retry logic
- Input/output validation
- Comprehensive logging and progress tracking
- Batch processing support

### 5. Plugin System
- Dynamic plugin discovery and registration
- Type-safe plugin interfaces
- Dependency management

## Quick Start

### Installation

```bash
# Install in development mode
uv sync
uv pip install -e .

# Install with ML dependencies
uv pip install -e ".[ml]"

# Install with Isaac Lab support
uv pip install -e ".[isaac]"
```

### Basic Usage

```python
import tung_playground as tp

# Create a hero from an image
hero = tp.create_hero(
    name="my_hero",
    image_path="path/to/image.png"
)

# Create and configure pipeline
pipeline = tp.create_pipeline("default")

# Add stages (when implemented)
# pipeline.add_stage(Wonder3DGenerator("generation"))
# pipeline.add_stage(PartCrafterDecomposer("decomposition"))
# ...

# Execute pipeline
results = await pipeline.execute(hero)
```

### Configuration

Create `config/custom.yaml`:

```yaml
pipeline:
  retry_attempts: 3
  timeout: 3600

stages:
  generation:
    quality_threshold: 0.8
    output_format: "ply"
  
  training:
    total_timesteps: 2000000
    learning_rate: 0.0001
```

Load with: `tp.load_config("custom")`

## Development Workflow

### 1. Implementing Pipeline Stages

Each stage inherits from `PipelineStage`:

```python
from tung_playground.core.pipeline import PipelineStage, StageResult

class MyGenerator(PipelineStage):
    PLUGIN_TYPE = "generation"
    
    async def process(self, hero: Hero) -> StageResult:
        # Implementation here
        pass
    
    def validate_inputs(self, hero: Hero) -> bool:
        # Input validation
        pass
    
    def validate_outputs(self, result: StageResult) -> bool:
        # Output validation
        pass
```

### 2. Plugin Registration

```python
from tung_playground.plugins import register_plugin

register_plugin(
    name="my_generator",
    plugin_type="generation", 
    class_type=MyGenerator,
    description="My custom generator"
)
```

### 3. Testing

```bash
# Run example
python examples/basic_usage.py

# Run tests (when added)
pytest tests/
```

## Next Steps

The framework is now ready for implementation of the actual pipeline stages:

1. **Wonder3D Integration** - Image-to-3D generation
2. **PartCrafter Integration** - Part decomposition
3. **Auto-rigging System** - Skeleton generation
4. **URDF Builder** - Robot description generation
5. **MuJoCo/Isaac Lab Adapters** - Simulation integration
6. **RL Training** - Policy learning implementation

Each module has abstract base classes defining the required interfaces, making implementation straightforward and type-safe.

## Configuration Reference

See `config/default.yaml` for the complete configuration schema with all available options and defaults.

## Contributing

1. Implement pipeline stages following the established patterns
2. Add comprehensive tests for new functionality
3. Update documentation and examples
4. Follow the coding standards in `CLAUDE.md`