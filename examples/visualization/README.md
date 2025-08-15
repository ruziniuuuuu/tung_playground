# 可视化示例

这个目录包含各种可视化工具和演示。

## 文件说明

### 英雄可视化
- **`hero_visualization_demo.py`** - 英雄可视化演示
  - URDF模型可视化
  - 3D网格显示
  - 交互式查看

- **`visualize_hero.py`** - 英雄可视化工具
  - 命令行可视化工具
  - 支持多种格式

- **`test_visualization.py`** - 可视化功能测试
  - 测试可视化组件
  - 验证渲染功能

## 使用方法

### 英雄可视化演示
```bash
# 可视化指定英雄
python hero_visualization_demo.py hero_name

# 可视化Tung Sahur
python hero_visualization_demo.py tungtung_sahur_tripo
```

### 可视化工具
```bash
# 启动可视化工具
python visualize_hero.py

# 指定英雄名称
python visualize_hero.py --hero hero_name
```

### 测试可视化
```bash
# 运行可视化测试
python test_visualization.py
```

## 功能特性

### URDF可视化
- 关节和链接显示
- 运动学结构展示
- 交互式操作

### 3D网格显示
- GLB/OBJ格式支持
- 纹理和材质显示
- 多角度查看

### 交互功能
- 鼠标控制视角
- 关节运动控制
- 实时渲染

## 依赖要求

可视化功能需要额外的依赖：

```bash
# 安装可视化依赖
pip install viser
pip install trimesh
pip install pybullet  # 可选，用于物理仿真
```

## 支持格式

### 输入格式
- URDF - 机器人描述文件
- GLB - 3D模型格式
- OBJ - 网格格式
- STL - 几何格式

### 可视化类型
- **静态显示** - 查看模型结构
- **关节运动** - 演示运动学
- **物理仿真** - 模拟物理行为

## 控制说明

### 鼠标控制
- **左键拖拽** - 旋转视角
- **右键拖拽** - 平移视图
- **滚轮** - 缩放视图

### 键盘控制
- **R** - 重置视角
- **S** - 保存截图
- **ESC** - 退出程序

## 输出选项

可视化结果可以保存为：
- PNG/JPG 截图
- MP4 动画视频
- GIF 动图

## 故障排除

### 常见问题
1. **无法显示3D模型** - 检查文件路径和格式
2. **性能问题** - 降低模型复杂度
3. **依赖错误** - 确保安装所有可视化库

### 调试模式
```bash
# 启用详细日志
python visualization_tool.py --verbose

# 调试模式
python visualization_tool.py --debug
```