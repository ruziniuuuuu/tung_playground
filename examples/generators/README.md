# 3D模型生成器示例

这个目录包含各种3D模型生成器的使用示例。

## 文件说明

### Tripo3D AI
- **`tripo_3d_generation_example.py`** - Tripo3D官方API集成示例
  - 支持官方API密钥(tsk_开头)
  - 自动使用Tung Sahur英雄图像
  - 包含质量等级选择和详细诊断

### Meshy AI  
- **`meshy_3d_generation_example.py`** - Meshy AI完整功能示例
- **`meshy_generation_demo.py`** - Meshy AI基础演示
- **`README_meshy.md`** - Meshy AI集成详细说明

## 使用方法

### Tripo3D 快速开始
```bash
# 设置API密钥
export TRIPO_API_KEY='tsk_your_key_here'

# 自动使用现有英雄图像
python tripo_3d_generation_example.py --quality low

# 或指定图像和英雄名称
python tripo_3d_generation_example.py path/to/image.png hero_name --quality high
```

### Meshy AI 快速开始
```bash
# 设置API密钥
export MESHY_API_KEY='your_meshy_key'

# 生成3D模型
python meshy_3d_generation_example.py path/to/image.png hero_name
```

## 质量等级

### Tripo3D
- **low**: 快速生成，标准纹理质量 (1-2分钟)
- **medium**: 平衡质量和速度 (2-3分钟)  
- **high**: 高质量HD纹理和PBR (3-5分钟)

### Meshy AI
- 支持多种艺术风格
- 自动质量优化
- 专业级模型输出

## API密钥获取

- **Tripo3D**: https://platform.tripo3d.ai/api-keys
- **Meshy AI**: https://www.meshy.ai/api

## 输出格式

生成的3D模型会保存到 `heroes/{hero_name}/` 目录：
- `generated_mesh.glb` - 3D模型文件
- `hero.json` - 英雄元数据

## 故障排除

常见问题和解决方案请查看各示例文件中的详细诊断功能。