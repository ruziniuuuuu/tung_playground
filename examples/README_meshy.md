# Meshy AI 3D Generation Examples

This directory contains examples demonstrating how to use Meshy AI for high-quality 3D asset generation in Tung Playground.

## Files

- `meshy_3d_generation_example.py` - Comprehensive example with single and batch generation
- `meshy_generation_demo.py` - CLI tool for interactive generation
- `configurable_pipeline_demo.py` - Pipeline demo with Meshy AI option

## Prerequisites

### 1. Meshy AI Account
- Sign up at [meshy.ai](https://www.meshy.ai/)
- **Upgrade to Pro subscription** (required - free plan no longer supports API task creation)
- Get your API key from [API settings](https://www.meshy.ai/settings/api)

### 2. Environment Setup
```bash
# Set your API key
export MESHY_API_KEY='your_api_key_here'

# Install dependencies
uv sync
```

### 3. Credits
Ensure you have sufficient credits for generation:
- Low quality: ~10 credits
- Medium quality: ~15 credits  
- High quality: ~20 credits

## Quick Start

### Single Asset Generation

```python
import asyncio
from examples.meshy_3d_generation_example import generate_3d_asset

# Generate a 3D asset
success = await generate_3d_asset(
    hero_name="my_character",
    image_path="path/to/character.png", 
    quality="medium",
    texture_prompt="fantasy character with detailed textures"
)
```

### Command Line Usage

```bash
# Run the comprehensive example
python examples/meshy_3d_generation_example.py

# Use the CLI demo tool
python examples/meshy_generation_demo.py generate my_hero image.png --quality high

# Full pipeline with Meshy
python examples/configurable_pipeline_demo.py run my_hero image.png --generator meshy
```

## Quality Levels

| Quality | Polygons | Textures | PBR | Time | Credits |
|---------|----------|----------|-----|------|---------|
| Low     | 5,000    | No       | No  | 1-2 min | ~10 |
| Medium  | 10,000   | Yes      | No  | 2-4 min | ~15 |
| High    | 20,000   | Yes      | Yes | 3-5 min | ~20 |

## Examples

### Example 1: Basic Generation
```python
import asyncio
from examples.meshy_3d_generation_example import generate_3d_asset

async def basic_example():
    success = await generate_3d_asset(
        hero_name="warrior_character",
        image_path="heroes/my_warrior/image.png",
        quality="medium"
    )
    print(f"Generation {'successful' if success else 'failed'}")

asyncio.run(basic_example())
```

### Example 2: High Quality with Custom Textures
```python
async def high_quality_example():
    success = await generate_3d_asset(
        hero_name="mage_character", 
        image_path="heroes/my_mage/image.png",
        quality="high",
        texture_prompt="magical robes with glowing runes and metallic details"
    )

asyncio.run(high_quality_example())
```

### Example 3: Batch Processing
```python
from examples.meshy_3d_generation_example import batch_generate

async def batch_example():
    await batch_generate(
        image_directory="input_images/",
        quality="low",  # Use low quality for batch to save credits
        output_dir="generated_heroes"
    )

asyncio.run(batch_example())
```

## Output Structure

After generation, you'll get:

```
heroes/my_character/
├── input_image.png         # Original image
├── generated_mesh.obj      # Generated 3D mesh (or .glb/.fbx)
├── hero.json              # Hero metadata
└── textures/              # Texture files (if enabled)
    ├── diffuse.png
    ├── normal.png         # If PBR enabled
    ├── metallic.png       # If PBR enabled
    └── roughness.png      # If PBR enabled
```

## Integration with Pipeline

The generated 3D assets can be used in the complete Tung Playground pipeline:

```bash
# Continue with full pipeline after Meshy generation
python examples/configurable_pipeline_demo.py run my_character image.png \
  --generator meshy \
  --stages generation,decomposition,rigging,urdf,simulation,training
```

## Error Handling

Common issues and solutions:

### 402 Error (No More Pending Tasks)
```
Error: NoMorePendingTasks: Task creation on the free plan is no longer supported
```
**Solution**: Upgrade to Meshy AI Pro subscription at [meshy.ai/settings/subscription](https://www.meshy.ai/settings/subscription)

### 401 Error (Unauthorized)
```
Error: Unauthorized
```
**Solution**: Check your API key is correctly set in `MESHY_API_KEY` environment variable

### Insufficient Credits
```
Error: Insufficient credits
```
**Solution**: Add more credits at [meshy.ai/settings/billing](https://www.meshy.ai/settings/billing)

### Proxy Issues
If you encounter proxy-related errors, ensure `api.meshy.ai` is bypassed in your proxy configuration.

## Tips for Best Results

### Image Requirements
- **Format**: PNG, JPG, or JPEG
- **Size**: Maximum 10MB
- **Resolution**: 512x512 or higher recommended
- **Content**: Clear, well-lit character images work best

### Quality Selection
- **Low**: Good for testing and prototyping
- **Medium**: Balanced quality for most use cases
- **High**: Best for final production assets

### Texture Prompts
- Be specific: "metallic armor with weathered bronze finish"
- Include material types: "leather, metal, fabric, fur"
- Mention style: "realistic", "stylized", "fantasy", "sci-fi"

### Credit Management
- Test with low quality first
- Use batch processing carefully (can consume many credits)
- Monitor your credit balance regularly

## Support

- Meshy AI Documentation: [docs.meshy.ai](https://docs.meshy.ai/)
- Tung Playground Issues: [GitHub Issues](https://github.com/your-repo/issues)
- Meshy AI Support: [help.meshy.ai](https://help.meshy.ai/)