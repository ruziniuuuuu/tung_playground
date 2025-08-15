# Tung Playground Examples

这个目录包含了Tung Playground的各种使用示例，按功能分类组织。

## 目录结构

### 📁 generators/
3D模型生成器相关示例
- `meshy_3d_generation_example.py` - Meshy AI 3D生成完整示例
- `meshy_generation_demo.py` - Meshy AI生成演示
- `tripo_3d_generation_example.py` - Tripo3D AI生成示例
- `README_meshy.md` - Meshy AI集成说明

### 📁 pipeline/
完整流程管道示例
- `configurable_pipeline_demo.py` - 可配置的完整流程演示
- `full_pipeline_demo.py` - 完整流程演示
- `simulate_pipeline_output.py` - 模拟流程输出

### 📁 visualization/
可视化相关示例
- `hero_visualization_demo.py` - 英雄可视化演示
- `test_visualization.py` - 可视化测试
- `visualize_hero.py` - 英雄可视化工具

### 📁 basic/
基础使用示例
- `basic_usage.py` - 基本使用方法
- `simple_test.py` - 简单测试示例

## 快速开始

### 3D模型生成
```bash
# Tripo3D 生成
python generators/tripo_3d_generation_example.py --quality low

# Meshy AI 生成
python generators/meshy_3d_generation_example.py path/to/image.png hero_name
```

### 完整流程
```bash
# 可配置流程
python pipeline/configurable_pipeline_demo.py run hero_name path/to/image.png

# 完整演示
python pipeline/full_pipeline_demo.py
```

### 可视化
```bash
# 英雄可视化
python visualization/hero_visualization_demo.py hero_name

# 可视化工具
python visualization/visualize_hero.py
```

### 基础使用
```bash
# 基本用法
python basic/basic_usage.py

# 简单测试
python basic/simple_test.py
```

## 环境变量

某些示例需要API密钥：

```bash
# Tripo3D API
export TRIPO_API_KEY='your_tripo_key'

# Meshy AI API  
export MESHY_API_KEY='your_meshy_key'
```

## 依赖

确保已安装所有依赖：
```bash
uv sync
```

## 更多信息

查看各子目录中的具体文档和示例说明。