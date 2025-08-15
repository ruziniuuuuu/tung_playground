# 流程管道示例

这个目录包含完整的AI英雄生成流程管道示例。

## 文件说明

### 完整流程演示
- **`configurable_pipeline_demo.py`** - 可配置的完整流程演示
  - 支持多种生成器 (Tripo3D, Meshy AI)
  - 灵活的配置选项
  - 完整的 Image → 3D → URDF → Simulation 流程

- **`full_pipeline_demo.py`** - 标准完整流程演示
  - 预设流程配置
  - 适合快速演示

- **`simulate_pipeline_output.py`** - 流程输出模拟
  - 测试和验证流程组件
  - 调试工具

## 使用方法

### 可配置流程演示
```bash
# 使用Tripo3D生成器
python configurable_pipeline_demo.py run hero_name path/to/image.png --generator tripo

# 使用Meshy AI生成器  
python configurable_pipeline_demo.py run hero_name path/to/image.png --generator meshy

# 查看可用选项
python configurable_pipeline_demo.py --help
```

### 完整流程演示
```bash
# 运行标准流程
python full_pipeline_demo.py

# 使用现有英雄
python full_pipeline_demo.py existing_hero_name
```

## 流程阶段

完整的AI英雄生成包含以下阶段：

1. **图像输入** - 提供英雄原始图像
2. **3D生成** - 使用AI生成3D模型
3. **部件分解** - 将模型分解为身体部位
4. **骨骼绑定** - 生成骨骼和关节
5. **URDF生成** - 创建机器人描述文件
6. **物理仿真** - 在MuJoCo中模拟
7. **强化学习** - 训练运动策略

## 配置选项

### 生成器选择
- `tripo` - Tripo3D AI (推荐)
- `meshy` - Meshy AI
- `mock` - 模拟生成器 (测试用)

### 质量设置
- `low` - 快速生成
- `medium` - 平衡质量
- `high` - 最高质量

### 输出格式
- GLB - 标准3D格式
- OBJ - 传统网格格式
- URDF - 机器人描述

## 环境要求

```bash
# 安装依赖
uv sync

# API密钥 (根据使用的生成器)
export TRIPO_API_KEY='your_tripo_key'
export MESHY_API_KEY='your_meshy_key'
```

## 输出结构

完成的英雄会在 `heroes/{hero_name}/` 目录包含：
```
heroes/hero_name/
├── input_image.png      # 原始图像
├── generated_mesh.glb   # 3D模型
├── parts/              # 分解的部件
├── skeleton.json       # 骨骼结构
├── robot.urdf          # URDF文件
├── scene.xml           # MuJoCo场景
├── policy.pkl          # 训练的策略
└── hero.json           # 元数据
```