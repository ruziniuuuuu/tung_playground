# Quick Start

Get up and running with Tung Playground in just a few minutes! This guide will walk you through creating your first AI hero from a character image.

## Prerequisites

- Python 3.10 or higher
- Git
- Basic familiarity with command line

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/ruziniuuuuu/tung_playground.git
cd tung_playground
```

### 2. Install Dependencies

Using UV (recommended):

```bash
# Install UV if you don't have it
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install project dependencies
uv sync

# Install in development mode
uv pip install -e .
```

Using pip:

```bash
pip install -e .
```

### 3. Verify Installation

```bash
python -c "import tung_playground; print('✅ Installation successful!')"
```

## Your First Hero

Let's create your first AI hero using the included sample image!

### 1. Run the Complete Demo

```bash
python examples/full_pipeline_demo.py
```

This will:
- 📸 Load the sample character image
- 🎯 Process it through all 6 pipeline stages
- 📊 Show detailed progress and results
- 📁 Generate all output files

### 2. Check the Results

After completion, you'll find your hero in:

```
heroes/tung_sahur_demo/
├── input_image.png          # Original character image
├── generated_mesh.obj       # Generated 3D mesh
├── parts/                   # Decomposed body parts
│   ├── head.obj
│   ├── torso.obj
│   ├── arms.obj
│   └── legs.obj
├── skeleton.json           # Biped skeleton structure
├── robot.urdf              # Physics-ready robot description
├── scene.xml               # MuJoCo simulation scene
├── policy.pkl              # Trained locomotion policy
└── hero.json               # Complete processing metadata
```

### 3. Understanding the Output

Each file represents a stage in the pipeline:

- **🎨 3D Mesh**: Your character converted to 3D geometry
- **🧩 Parts**: Intelligently segmented body components
- **🦴 Skeleton**: Articulated bone structure for movement
- **🤖 URDF**: Robot description with physics properties
- **🌍 Scene**: Complete simulation environment
- **🧠 Policy**: Trained AI brain for locomotion

## Next Steps

### Custom Character

Try with your own character image:

```python
import tung_playground as tp

# Create hero from your image
hero = tp.create_hero("my_character", "path/to/your/image.png")

# Run the pipeline
pipeline = tp.create_pipeline()
results = await pipeline.execute(hero)
```

### Explore the Framework

- **📖 [Architecture Overview](../architecture/overview.md)**: Understand the system design
- **🔧 [Development Guide](../development/setup.md)**: Set up for development
- **🚀 [Examples](../examples/complete-demo.md)**: More detailed examples
- **⚙️ [Configuration](../architecture/configuration.md)**: Customize the pipeline

### Common Use Cases

- **🎮 Game Development**: Create NPCs and characters
- **🎬 Animation**: Generate rigged characters for films
- **🤖 Robotics Research**: Test algorithms with diverse morphologies
- **📚 Education**: Learn about AI, graphics, and simulation

## Troubleshooting

### Installation Issues

**Problem**: `ModuleNotFoundError: No module named 'pydantic'`

**Solution**: Install dependencies:
```bash
uv sync  # or pip install -r requirements.txt
```

**Problem**: `ImportError: No module named 'tung_playground'`

**Solution**: Install in development mode:
```bash
uv pip install -e .  # or pip install -e .
```

### Runtime Issues

**Problem**: Pipeline fails at generation stage

**Solution**: Check that your image is valid:
```bash
python -c "from PIL import Image; Image.open('your_image.png').verify()"
```

**Problem**: Out of memory errors

**Solution**: Reduce batch size in configuration:
```yaml
# config/default.yaml
pipeline:
  max_workers: 1  # Reduce parallelism
```

## Getting Help

- **📚 [Documentation](../introduction.md)**: Complete guides and references
- **🐛 [Issues](https://github.com/ruziniuuuuu/tung_playground/issues)**: Report bugs
- **💬 [Discussions](https://github.com/ruziniuuuuu/tung_playground/discussions)**: Ask questions
- **📧 [Email](mailto:ruziniuuuuu@gmail.com)**: Direct support

## What's Next?

Now that you have your first hero, explore:

1. **[Basic Usage](./basic-usage.md)**: Learn the fundamental concepts
2. **[Pipeline Details](../pipeline/generation.md)**: Deep dive into each stage
3. **[Custom Plugins](../development/plugins.md)**: Extend the framework
4. **[Production Deployment](../deployment/production.md)**: Scale your setup

Welcome to the Tung Playground community! 🎉