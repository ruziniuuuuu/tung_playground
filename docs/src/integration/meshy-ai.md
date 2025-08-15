# Meshy AI Integration

This guide covers integrating [Meshy AI](https://www.meshy.ai/) for high-quality image-to-3D generation in your Tung Playground pipeline.

## Overview

Meshy AI provides state-of-the-art AI models for converting 2D images into 3D models with textures. The integration supports:

- **High-quality 3D generation** from single images
- **Automatic texturing** with PBR material generation
- **Multiple output formats** (OBJ, GLB, FBX, etc.)
- **Configurable quality levels** and polygon counts
- **Asynchronous processing** with progress tracking

## Setup

### 1. Get API Key

1. Sign up at [meshy.ai](https://www.meshy.ai/)
2. Navigate to your API settings
3. Generate an API key
4. Set the environment variable:

```bash
export MESHY_API_KEY="your_api_key_here"
```

### 2. Install Dependencies

The Meshy integration uses `httpx` for async HTTP requests, which is included in the base dependencies:

```bash
uv sync
```

### 3. Configuration

Add Meshy configuration to your config file:

```yaml
# config/meshy.yaml
stages:
  generation:
    meshy:
      api_key: null  # Use environment variable MESHY_API_KEY
      base_url: "https://api.meshy.ai"
      model: "meshy-5"
      topology: "triangle"
      target_polycount: 10000
      should_texture: true
      enable_pbr: false
      texture_prompt: ""
      content_safety: true
      poll_interval: 10  # seconds
      max_wait_time: 1800  # 30 minutes
```

## Usage

### Basic Usage

```python
import tung_playground as tp
from tung_playground.generation import MeshyGenerator

# Create hero
hero = tp.create_hero("my_hero", "character.png")

# Configure Meshy generator
config = {
    "meshy": {
        "model": "meshy-5",
        "target_polycount": 15000,
        "should_texture": True,
        "enable_pbr": True
    }
}

generator = MeshyGenerator("meshy_gen", config)

# Generate 3D model
result = await generator.process(hero)

if result.status == tp.StageStatus.COMPLETED:
    print(f"Generated mesh: {result.outputs['mesh_path']}")
    print(f"Quality score: {result.outputs['quality_score']:.2f}")
```

### Pipeline Integration

```python
# Create pipeline with Meshy generator
pipeline = tp.create_pipeline()

# Add Meshy generation stage
meshy_config = {
    "meshy": {
        "model": "meshy-5",
        "quality": "high",
        "should_texture": True
    }
}

pipeline.add_stage(MeshyGenerator("generation", meshy_config))

# Add other stages...
# pipeline.add_stage(MockDecomposer("decomposition"))
# etc.

# Execute pipeline
results = await pipeline.execute(hero)
```

### Using the Demo Scripts

#### Simple Generation

```bash
# Generate with default settings
python examples/meshy_generation_demo.py generate "my_hero" "path/to/image.png"

# High quality with textures
python examples/meshy_generation_demo.py generate "my_hero" "image.png" \
    --quality high --textured --model meshy-5

# Check your credits
python examples/meshy_generation_demo.py check-credits
```

#### Full Pipeline

```bash
# Run complete pipeline with Meshy
python examples/configurable_pipeline_demo.py run "my_hero" "image.png" \
    --generator meshy --quality high

# Run only generation + URDF stages
python examples/configurable_pipeline_demo.py run "my_hero" "image.png" \
    --generator meshy --stages "generation,urdf"
```

## Configuration Options

### Quality Presets

The integration includes quality presets for common use cases:

```python
quality_settings = {
    "low": {
        "target_polycount": 5000,
        "should_texture": False
    },
    "medium": {
        "target_polycount": 10000, 
        "should_texture": True
    },
    "high": {
        "target_polycount": 20000,
        "should_texture": True,
        "enable_pbr": True
    }
}
```

### Advanced Configuration

```yaml
stages:
  generation:
    meshy:
      # Model selection
      model: "meshy-5"  # Latest model
      
      # Mesh configuration
      topology: "triangle"  # or "quad"
      target_polycount: 15000
      
      # Texturing options
      should_texture: true
      enable_pbr: true  # Generate metallic/roughness/normal maps
      texture_prompt: "clean, high-resolution textures"
      
      # Safety and processing
      content_safety: true  # Screen for harmful content
      poll_interval: 5  # Check status every 5 seconds
      max_wait_time: 3600  # Wait up to 1 hour
```

## Output Formats

Meshy AI generates multiple formats. The integration automatically selects the best available:

1. **OBJ** (preferred) - Widely compatible
2. **GLB** - Compact binary format
3. **FBX** - Animation-ready format

Generated assets include:

- **3D Mesh** - Main geometry file
- **Textures** - Diffuse, normal, metallic, roughness maps (if PBR enabled)
- **Thumbnail** - Preview image
- **Metadata** - Generation parameters and statistics

## Error Handling

The integration includes robust error handling:

```python
try:
    result = await generator.process(hero)
    
    if result.status == tp.StageStatus.FAILED:
        print(f"Generation failed: {result.error}")
        
except tp.ConfigurationError as e:
    print(f"Configuration error: {e}")
    
except tp.ProcessingError as e:
    print(f"Processing error: {e}")
```

Common error scenarios:

- **Missing API key** - Set `MESHY_API_KEY` environment variable
- **Invalid image format** - Use PNG, JPG, or JPEG
- **File too large** - Images must be under 10MB
- **API rate limits** - Reduce request frequency
- **Insufficient credits** - Add credits to your Meshy account

## Performance Tips

### Optimization

1. **Image preparation**:
   - Use high-resolution images (512x512 or higher)
   - Ensure good lighting and contrast
   - Remove background for better results

2. **Quality vs Speed**:
   - Use lower polygon counts for faster generation
   - Disable texturing for quick prototypes
   - Enable PBR only when needed

3. **Batch processing**:
   - Generate multiple heroes in parallel
   - Use async/await properly for concurrent requests

### Monitoring

```python
# Monitor generation progress
async def monitor_generation(generator, hero):
    with Progress() as progress:
        task = progress.add_task("Generating...", total=None)
        
        result = await generator.process(hero)
        
        progress.update(task, description="Complete!")
    
    return result
```

## Cost Management

### Credit Usage

- **Basic generation**: ~10 credits per model
- **With texturing**: ~15 credits per model  
- **PBR materials**: ~20 credits per model

### Best Practices

1. **Test with mock generator** first to validate pipeline
2. **Use appropriate quality** levels for your use case
3. **Monitor credit usage** with the API
4. **Cache results** to avoid regenerating

```bash
# Check credits before generation
python examples/meshy_generation_demo.py check-credits
```

## Troubleshooting

### Common Issues

**API Key Issues**:
```bash
# Check if API key is set
echo $MESHY_API_KEY

# Set temporarily
export MESHY_API_KEY="your_key"

# Set permanently (Linux/Mac)
echo 'export MESHY_API_KEY="your_key"' >> ~/.bashrc
source ~/.bashrc
```

**Network/Timeout Issues**:
```yaml
# Increase timeouts in config
stages:
  generation:
    meshy:
      max_wait_time: 3600  # 1 hour
      poll_interval: 30   # Check every 30 seconds
```

**Quality Issues**:
- Use higher resolution input images
- Ensure good lighting and contrast
- Try different texture prompts
- Enable PBR for better materials

### Debug Mode

Enable detailed logging:

```python
import logging
logging.getLogger('tung_playground.generation.meshy_generator').setLevel(logging.DEBUG)
```

## API Reference

### MeshyGenerator Class

```python
class MeshyGenerator(Image3DGenerator):
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None)
    async def generate_3d(self, hero: Hero) -> GenerationResult
    def validate_inputs(self, hero: Hero) -> bool
```

### Configuration Schema

```python
{
    "meshy": {
        "api_key": str,              # API key (use env var)
        "base_url": str,             # API base URL
        "model": str,                # Model version
        "topology": str,             # "triangle" or "quad"  
        "target_polycount": int,     # Target polygon count
        "should_texture": bool,      # Generate textures
        "enable_pbr": bool,          # Generate PBR maps
        "texture_prompt": str,       # Texture guidance
        "content_safety": bool,      # Content screening
        "poll_interval": int,        # Status check interval (seconds)
        "max_wait_time": int         # Maximum wait time (seconds)
    }
}
```

## Examples

See the following example scripts:

- `examples/meshy_generation_demo.py` - Single generation with Meshy
- `examples/configurable_pipeline_demo.py` - Full pipeline with generator selection
- `examples/full_pipeline_demo.py` - Mock pipeline for comparison

## Further Reading

- [Meshy AI Documentation](https://docs.meshy.ai/)
- [Meshy AI API Reference](https://docs.meshy.ai/api-reference)
- [Pipeline Architecture](../architecture/pipeline.md)
- [Configuration Guide](../architecture/configuration.md)