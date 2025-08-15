# Installation

This guide covers multiple installation methods for Tung Playground, from quick setup for experimentation to production deployment.

## System Requirements

### Minimum Requirements
- **Python**: 3.10 or higher
- **Memory**: 4GB RAM
- **Storage**: 2GB free space
- **OS**: Linux, macOS, or Windows

### Recommended for Production
- **Python**: 3.11+
- **Memory**: 16GB+ RAM
- **Storage**: 50GB+ SSD
- **GPU**: NVIDIA GPU with CUDA support (optional but recommended)
- **OS**: Ubuntu 20.04+ or similar Linux distribution

## Installation Methods

### Method 1: Quick Start (Recommended)

Using UV package manager for fastest setup:

```bash
# Install UV if you don't have it
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.cargo/env

# Clone and setup
git clone https://github.com/ruziniuuuuu/tung_playground.git
cd tung_playground

# Install dependencies and package
uv sync
uv pip install -e .
```

### Method 2: Traditional pip

```bash
# Clone repository
git clone https://github.com/ruziniuuuuu/tung_playground.git
cd tung_playground

# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -e .
```

### Method 3: Development Setup

For contributors and advanced users:

```bash
# Clone with development dependencies
git clone https://github.com/ruziniuuuuu/tung_playground.git
cd tung_playground

# Install with all optional dependencies
uv sync --extra dev --extra ml --extra isaac

# Install pre-commit hooks
pre-commit install
```

### Method 4: Docker (Production)

```bash
# Pull the official image
docker pull tungplayground/tung_playground:latest

# Or build from source
git clone https://github.com/ruziniuuuuu/tung_playground.git
cd tung_playground
docker build -t tung_playground .
```

## Dependency Groups

Tung Playground uses optional dependency groups for different use cases:

### Core Dependencies (Always Installed)
```toml
dependencies = [
    "mujoco",
    "numpy", 
    "opencv-python",
    "pillow",
    "pyyaml",
    "trimesh",
    "open3d",
    "scipy",
    "pydantic>=2.0",
    "rich",
    "typer",
    "requests",
    "aiohttp"
]
```

### Development Dependencies
```bash
uv sync --extra dev
# or
pip install -e ".[dev]"
```

Includes:
- `pytest` - Testing framework
- `black` - Code formatting
- `isort` - Import sorting
- `mypy` - Type checking
- `pre-commit` - Git hooks

### Machine Learning Dependencies
```bash
uv sync --extra ml
# or  
pip install -e ".[ml]"
```

Includes:
- `torch` - PyTorch for deep learning
- `torchvision` - Computer vision utilities
- `transformers` - Hugging Face transformers
- `diffusers` - Diffusion models
- `accelerate` - Training acceleration

### Isaac Lab Integration
```bash
uv sync --extra isaac
# or
pip install -e ".[isaac]"
```

Includes:
- `omni-isaac-lab` - NVIDIA Isaac Lab

## Verification

### Basic Installation Check

```bash
python -c "import tung_playground; print('✅ Installation successful!')"
```

### Full System Check

```bash
python examples/simple_test.py
```

Expected output:
```
🧪 Testing Tung Playground Pipeline Implementation
============================================================

🔍 Testing File Structure...
✅ All 12 expected files exist

🔍 Testing Module Imports...
✅ Core modules import successfully
✅ Pipeline base modules import successfully  
✅ Mock implementations import successfully
✅ Plugin system imports successfully

🔍 Testing Mock Content...
✅ Mock generator has mesh creation logic
✅ Mock decomposer has part splitting logic
✅ Mock rigger has skeleton creation logic

📊 Test Results: 3/3 passed
🎉 All tests passed! Pipeline implementation is ready.
```

### Run Demo Pipeline

```bash
python examples/full_pipeline_demo.py
```

This should complete without errors and generate files in `heroes/tung_sahur_demo/`.

## Platform-Specific Instructions

### Ubuntu/Debian

```bash
# Install system dependencies
sudo apt update
sudo apt install -y python3.10 python3.10-venv git build-essential

# Install UV
curl -LsSf https://astral.sh/uv/install.sh | sh

# Continue with standard installation
git clone https://github.com/ruziniuuuuu/tung_playground.git
cd tung_playground
uv sync
```

### macOS

```bash
# Install Homebrew if needed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Python and dependencies
brew install python@3.11 git

# Install UV
curl -LsSf https://astral.sh/uv/install.sh | sh

# Continue with standard installation
git clone https://github.com/ruziniuuuuu/tung_playground.git
cd tung_playground
uv sync
```

### Windows

1. **Install Python 3.10+** from [python.org](https://python.org)
2. **Install Git** from [git-scm.com](https://git-scm.com)
3. **Install UV**:
   ```powershell
   powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
   ```
4. **Continue with standard installation**:
   ```cmd
   git clone https://github.com/ruziniuuuuu/tung_playground.git
   cd tung_playground
   uv sync
   ```

## GPU Support

### NVIDIA CUDA Setup

For GPU acceleration:

```bash
# Check CUDA availability
nvidia-smi

# Install CUDA-enabled PyTorch
uv add torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Verify GPU support
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### AMD ROCm (Linux)

```bash
# Install ROCm
sudo apt install rocm-dev

# Install ROCm PyTorch
uv add torch torchvision --index-url https://download.pytorch.org/whl/rocm5.4.2
```

## Configuration

### Environment Variables

Set these environment variables for optimal performance:

```bash
# Add to ~/.bashrc or ~/.zshrc
export TUNG_PLAYGROUND_HOME="$HOME/.tung_playground"
export TUNG_PLAYGROUND_CACHE="$HOME/.cache/tung_playground"
export PYTHONPATH="$PYTHONPATH:$(pwd)/src"

# For GPU users
export CUDA_VISIBLE_DEVICES=0  # Use first GPU only
```

### Configuration Directory

Create configuration directory:

```bash
mkdir -p ~/.tung_playground
cp config/default.yaml ~/.tung_playground/config.yaml
```

Edit `~/.tung_playground/config.yaml` for custom settings.

## Troubleshooting

### Common Issues

#### `ModuleNotFoundError: No module named 'tung_playground'`

**Cause**: Package not installed in editable mode  
**Solution**:
```bash
cd /path/to/tung_playground
uv pip install -e .
```

#### `ImportError: No module named 'mujoco'`

**Cause**: MuJoCo not properly installed  
**Solution**:
```bash
uv add mujoco
# Or for specific version
uv add "mujoco>=2.3.0"
```

#### `CUDA out of memory`

**Cause**: GPU memory insufficient  
**Solution**:
```bash
# Reduce batch size in config
echo "pipeline:\n  max_workers: 1" > ~/.tung_playground/config.yaml
```

#### `Permission denied` errors on Linux

**Cause**: Installation directory permissions  
**Solution**:
```bash
# Install to user directory
uv pip install --user -e .
```

### Performance Issues

#### Slow pipeline execution

1. **Check CPU usage**: `htop` or `top`
2. **Monitor memory**: `free -h`
3. **Enable GPU acceleration**: See GPU Support section
4. **Reduce parallelism**:
   ```yaml
   # In config/default.yaml
   pipeline:
     max_workers: 1
   ```

#### Large memory usage

1. **Monitor per-stage memory**: Add logging
2. **Clear intermediate files**: Enable cleanup
3. **Use smaller models**: Reduce quality settings

### Getting Help

If you encounter issues not covered here:

1. **Check the logs**: Enable debug logging
   ```python
   import tung_playground as tp
   tp.setup_logging(level="DEBUG")
   ```

2. **Search existing issues**: [GitHub Issues](https://github.com/ruziniuuuuu/tung_playground/issues)

3. **Create new issue**: Include:
   - Operating system and version
   - Python version (`python --version`)
   - Installation method used
   - Complete error traceback
   - Steps to reproduce

4. **Join discussions**: [GitHub Discussions](https://github.com/ruziniuuuuu/tung_playground/discussions)

## Next Steps

After successful installation:

1. **[Quick Start](./quick-start.md)**: Run your first pipeline
2. **[Basic Usage](./basic-usage.md)**: Learn core concepts
3. **[Your First Hero](./first-hero.md)**: Create a custom character
4. **[Development Setup](../development/setup.md)**: Set up for development

## Updating

### Update to Latest Version

```bash
cd tung_playground
git pull origin main
uv sync  # Update dependencies
```

### Version Management

Check current version:
```bash
python -c "import tung_playground; print(tung_playground.__version__)"
```

Switch to specific version:
```bash
git checkout v1.0.0
uv sync
```

### Migration Guide

When updating between major versions, check the [Changelog](../appendices/changelog.md) for breaking changes and migration instructions.