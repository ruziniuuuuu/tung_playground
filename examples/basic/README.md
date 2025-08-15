# 基础使用示例

这个目录包含Tung Playground的基础使用方法和简单示例。

## 文件说明

### 基础使用
- **`basic_usage.py`** - 基本使用方法演示
  - 英雄创建和管理
  - 基础API使用
  - 简单流程演示

- **`simple_test.py`** - 简单测试示例
  - 功能验证
  - 快速测试
  - 错误检查

## 使用方法

### 基础使用演示
```bash
# 运行基础使用示例
python basic_usage.py

# 查看帮助
python basic_usage.py --help
```

### 简单测试
```bash
# 运行简单测试
python simple_test.py

# 详细输出
python simple_test.py --verbose
```

## 学习路径

### 1. 从基础开始
```bash
# 第一步：了解基本概念
python basic_usage.py
```

### 2. 创建第一个英雄
```python
import tung_playground as tp

# 创建英雄
hero = tp.create_hero("my_hero", "path/to/image.png")
print(f"创建英雄: {hero.name}")
```

### 3. 探索高级功能
```bash
# 进入流程管道示例
cd ../pipeline/
python configurable_pipeline_demo.py

# 尝试可视化
cd ../visualization/  
python hero_visualization_demo.py
```

## 核心概念

### 英雄 (Hero)
- 每个英雄代表一个AI生成的角色
- 包含图像、3D模型、动画等资产
- 存储在 `heroes/` 目录

### 流程管道 (Pipeline)  
- Image → 3D → Parts → Skeleton → URDF → Simulation
- 每个阶段可以独立配置
- 支持插件扩展

### 配置系统
- YAML配置文件
- 环境变量支持
- 灵活的参数设置

## API快速参考

### 英雄管理
```python
# 创建英雄
hero = tp.create_hero(name, image_path)

# 加载英雄
hero = tp.load_hero(name)

# 保存英雄
hero.save()
```

### 生成器使用
```python
# Tripo3D生成器
generator = tp.TripoGenerator("tripo", config)
result = await generator.generate_3d(hero)

# Meshy AI生成器  
generator = tp.MeshyGenerator("meshy", config)
result = await generator.generate_3d(hero)
```

### 配置管理
```python
# 加载配置
config = tp.load_config("config/default.yaml")

# 设置日志
tp.setup_logging(level="INFO")
```

## 环境设置

### 最小安装
```bash
# 克隆仓库
git clone https://github.com/your-repo/tung_playground
cd tung_playground

# 安装依赖
uv sync

# 开发模式安装
uv pip install -e .
```

### 验证安装
```bash
# 测试导入
python -c "import tung_playground; print('安装成功!')"

# 运行基础测试
python basic/simple_test.py
```

## 下一步

从这里开始，您可以：

1. **探索生成器** - 查看 `../generators/` 了解3D生成
2. **试用完整流程** - 查看 `../pipeline/` 体验端到端流程  
3. **学习可视化** - 查看 `../visualization/` 查看结果
4. **阅读文档** - 查看项目根目录的 `docs/` 文件夹

## 获得帮助

- 📖 **文档**: `docs/` 目录
- 🐛 **问题**: GitHub Issues
- 💬 **讨论**: GitHub Discussions