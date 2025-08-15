#!/usr/bin/env python3
"""Tripo3D AI 3D模型生成示例

这个示例展示如何使用Tripo3D AI从图像生成高质量的3D模型。

功能特性:
- 图像到3D模型生成
- 质量等级选择 (低/中/高)
- 进度跟踪和状态更新
- 资源管理和文件组织
- 完整的pipeline集成
- 命令行参数支持

使用要求:
- Tripo3D API密钥 (官方API，推荐Pro订阅)
- TRIPO_API_KEY 环境变量 (以tsk_开头的官方API密钥)
- 输入图像文件 (PNG, JPG, JPEG)

快速使用:
    python examples/tripo_3d_generation_example.py path/to/image.png my_hero
    python examples/tripo_3d_generation_example.py path/to/image.png my_hero --quality high
"""

import asyncio
import os
import sys
from pathlib import Path
from typing import Optional
import time
import argparse
import httpx
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TaskProgressColumn
from rich.table import Table

# Add project source to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / "src"))

# Import Tung Playground components
import tung_playground as tp
from tung_playground.generation import TripoGenerator
from tung_playground.core.exceptions import ProcessingError, ConfigurationError

console = Console()


# Quality presets for different use cases
QUALITY_PRESETS = {
    "low": {
        "description": "Fast generation, standard texture quality",
        "texture_quality": "standard",
        "face_limit": 5000,
        "enable_pbr": False,
        "enable_quad": False,
        "estimated_time": "1-2 minutes"
    },
    "medium": {
        "description": "Balanced quality and speed",
        "texture_quality": "standard",
        "face_limit": 10000,
        "enable_pbr": False,
        "enable_quad": False,
        "estimated_time": "2-3 minutes"
    },
    "high": {
        "description": "High quality with HD textures and PBR",
        "texture_quality": "HD",
        "face_limit": 20000,
        "enable_pbr": True,
        "enable_quad": True,
        "estimated_time": "3-5 minutes"
    }
}


def check_requirements() -> bool:
    """检查运行环境和配置"""
    
    console.print("[cyan]🔍 检查运行环境...[/cyan]")
    
    # 检查API密钥
    api_key = os.getenv("TRIPO_API_KEY") or os.getenv("FAL_KEY")
    if not api_key:
        console.print("[red]❌ 未找到API密钥![/red]")
        console.print("\n[yellow]设置方法:[/yellow]")
        console.print("export TRIPO_API_KEY='your_api_key_here'")
        console.print("\n[yellow]获取API密钥:[/yellow] https://platform.tripo3d.ai/api-keys")
        return False
    
    if not api_key.startswith("tsk_"):
        console.print("[red]❌ 只支持官方Tripo3D API密钥 (以tsk_开头)![/red]")
        console.print("\n[yellow]请到以下地址获取官方API密钥:[/yellow]")
        console.print("https://platform.tripo3d.ai/api-keys")
        return False
    
    console.print(f"[green]✅ 官方Tripo3D API密钥已配置[/green]")
    
    # 检查输出目录
    output_dir = Path("heroes")
    output_dir.mkdir(exist_ok=True)
    console.print(f"[green]✅ 输出目录: {output_dir.absolute()}[/green]")
    
    return True

def get_default_hero_image():
    """获取默认的英雄图像（Tung Sahur）"""
    
    # 首先检查是否有现有的英雄图像
    tung_sahur_image = Path("heroes/tungtungtungtungtungtungtungtungtung_sahur/image.png")
    if tung_sahur_image.exists():
        return tung_sahur_image, "tungtung_sahur"
    
    # 备选：查找其他英雄目录中的图像
    heroes_dir = Path("heroes")
    if heroes_dir.exists():
        for hero_dir in heroes_dir.iterdir():
            if hero_dir.is_dir():
                for image_file in ["image.png", "input_image.png"]:
                    image_path = hero_dir / image_file
                    if image_path.exists():
                        return image_path, hero_dir.name
    
    # 如果没有找到现有图像，创建测试图像
    test_image_path = Path("test_image.png")
    if test_image_path.exists():
        return test_image_path, "test_hero"
    
    try:
        from PIL import Image, ImageDraw, ImageFont
        
        # 创建简单的测试图像
        img = Image.new('RGB', (512, 512), color=(255, 255, 255))
        draw = ImageDraw.Draw(img)
        
        # 绘制一个立方体
        color = (100, 150, 200)
        margin = 100
        draw.rectangle([margin, margin, 512-margin, 512-margin], 
                      outline=color, width=8, fill=color)
        
        # 添加3D效果
        offset = 30
        draw.rectangle([margin+offset, margin-offset, 512-margin+offset, 512-margin-offset],
                      outline=(50, 100, 150), width=6)
        
        # 添加文字
        try:
            font = ImageFont.truetype("arial.ttf", 36)
        except:
            font = ImageFont.load_default()
        
        text = "TEST CUBE"
        bbox = draw.textbbox((0, 0), text, font=font)
        text_width = bbox[2] - bbox[0]
        text_x = (512 - text_width) // 2
        text_y = 450
        
        draw.text((text_x + 2, text_y + 2), text, fill=(0, 0, 0), font=font)
        draw.text((text_x, text_y), text, fill=(255, 255, 255), font=font)
        
        img.save(test_image_path)
        console.print(f"[green]✅ 创建测试图像: {test_image_path}[/green]")
        return test_image_path
        
    except ImportError:
        console.print("[yellow]⚠️  无法创建测试图像 (需要 Pillow: pip install Pillow)[/yellow]")
        return None

async def generate_3d_asset(
    hero_name: str,
    image_path: str,
    quality: str = "medium",
    style: Optional[str] = None
) -> bool:
    """从图像生成3D模型（专注测试Tripo3D API）
    
    Args:
        hero_name: 英雄名称
        image_path: 输入图像路径
        quality: 质量等级 - "low", "medium", 或 "high"
        style: 可选样式 (例如: "object:clay", "person:person2cartoon")
        
    Returns:
        生成成功返回True，否则返回False
    """
    console.print(f"[bold cyan]🎨 Tripo3D AI 3D模型生成[/bold cyan]")
    console.print("=" * 50)
    
    # 验证输入
    if quality not in QUALITY_PRESETS:
        console.print(f"[red]❌ 无效的质量等级: {quality}[/red]")
        console.print(f"[yellow]可选项: {', '.join(QUALITY_PRESETS.keys())}[/yellow]")
        return False
    
    image_file = Path(image_path)
    if not image_file.exists():
        console.print(f"[red]❌ 图像文件不存在: {image_path}[/red]")
        return False
    
    preset = QUALITY_PRESETS[quality]
    
    # 创建设置表格
    settings_table = Table(show_header=True, header_style="bold magenta")
    settings_table.add_column("设置项", style="cyan")
    settings_table.add_column("值", style="green")
    
    settings_table.add_row("英雄名称", hero_name)
    settings_table.add_row("图像文件", image_file.name)
    settings_table.add_row("质量等级", f"{quality} ({preset['description']})")
    settings_table.add_row("纹理质量", preset['texture_quality'])
    settings_table.add_row("面数限制", f"{preset['face_limit']:,}")
    settings_table.add_row("PBR材质", str(preset['enable_pbr']))
    settings_table.add_row("四边形网格", str(preset['enable_quad']))
    settings_table.add_row("预计时间", preset['estimated_time'])
    if style:
        settings_table.add_row("样式", style)
    
    console.print(settings_table)
    
    try:
        # 设置日志
        tp.setup_logging(level="INFO", use_rich=True)
        
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TaskProgressColumn(),
            console=console,
        ) as progress:
            
            # 创建英雄
            task = progress.add_task("创建英雄...", total=5)
            hero = tp.create_hero(hero_name, str(image_file))
            console.print(f"[green]✅ 英雄创建: {hero.name}[/green]")
            console.print(f"[cyan]   输出目录: {hero.hero_dir}[/cyan]")
            progress.advance(task)
            
            # 配置Tripo生成器
            progress.update(task, description="配置生成器...")
            config = {
                "tripo": {
                    "texture_quality": preset["texture_quality"],
                    "face_limit": preset["face_limit"],
                    "enable_pbr": preset["enable_pbr"],
                    "enable_quad": preset["enable_quad"],
                    "auto_size": True,
                    "poll_interval": 5,  # 每5秒检查状态
                    "max_wait_time": 600,  # 10分钟超时
                }
            }
            
            # 添加样式（如果提供）
            if style:
                config["tripo"]["style"] = style
            
            generator = TripoGenerator("tripo_generator", config)
            console.print(f"[green]✅ 生成器配置完成[/green]")
            progress.advance(task)
            
            # 验证输入
            progress.update(task, description="验证输入...")
            generator.validate_inputs(hero)
            console.print(f"[green]✅ 输入验证通过[/green]")
            progress.advance(task)
            
            # 开始生成
            progress.update(task, description="生成3D模型...")
            console.print(f"\n[cyan]🚀 开始3D模型生成...[/cyan]")
            console.print(f"[cyan]预计时间: {preset['estimated_time']}[/cyan]\n")
            
            start_time = time.time()
            
            # 通过生成器处理英雄
            result = await generator.generate_3d(hero)
            
            total_time = time.time() - start_time
            progress.advance(task)
            
            progress.update(task, description="完成!")
            
            # 显示生成结果
            console.print(f"\n[green]🎉 生成成功![/green]")
            
            # 创建结果表格
            result_table = Table(show_header=True, header_style="bold green")
            result_table.add_column("项目", style="cyan")
            result_table.add_column("值", style="green")
            
            result_table.add_row("生成时间", f"{result.generation_time:.1f}秒")
            result_table.add_row("总时间", f"{total_time:.1f}秒")
            result_table.add_row("质量评分", f"{result.quality_score:.2f}")
            result_table.add_row("模型文件", result.mesh_path.name)
            
            # 检查生成文件
            if result.mesh_path.exists():
                file_size = result.mesh_path.stat().st_size
                result_table.add_row("文件大小", f"{file_size:,} 字节")
            
            console.print(result_table)
            
            # 显示生成详情
            if result.metadata:
                console.print(f"\n[cyan]📊 生成详情:[/cyan]")
                details_table = Table(show_header=True, header_style="bold cyan")
                details_table.add_column("参数", style="cyan")
                details_table.add_column("值", style="yellow")
                
                details_table.add_row("纹理质量", result.metadata.get('texture_quality', 'N/A'))
                details_table.add_row("面数限制", str(result.metadata.get('face_limit', 'N/A')))
                details_table.add_row("PBR启用", str(result.metadata.get('pbr_enabled', False)))
                details_table.add_row("四边形网格", str(result.metadata.get('quad_mesh', False)))
                if result.metadata.get('style'):
                    details_table.add_row("样式", result.metadata.get('style'))
                
                console.print(details_table)
            
            # 保存英雄元数据
            hero_file = hero.save()
            console.print(f"[green]✅ 英雄数据已保存: {hero_file}[/green]")
            
            # 生成的资产信息
            console.print(f"\n[cyan]🎯 生成的资产:[/cyan]")
            console.print(f"   3D网格: {result.mesh_path}")
            console.print(f"   英雄数据: {hero_file}")
            console.print(f"   目录: {hero.hero_dir}")
            
            # 下一步建议
            console.print(f"\n[yellow]🔄 下一步建议:[/yellow]")
            console.print(f"1. 在3D软件中查看生成的网格")
            console.print(f"2. 继续完整流程:")
            console.print(f"   [cyan]python examples/configurable_pipeline_demo.py run {hero_name} {image_path} --generator tripo[/cyan]")
            console.print(f"3. 生成URDF并可视化:")
            console.print(f"   [cyan]python examples/hero_visualization_demo.py {hero_name}[/cyan]")
            
            return True
                
    except ProcessingError as e:
        console.print(f"\n[red]❌ Tripo3D API错误: {e}[/red]")
        
        # 详细的API问题分析
        error_str = str(e).lower()
        console.print(f"\n[cyan]🔍 API问题诊断:[/cyan]")
        
        if "parameter" in error_str and "invalid" in error_str:
            console.print("[yellow]🚨 参数格式问题[/yellow]")
            console.print("• 这通常表示API请求的参数格式不正确")
            console.print("• 可能的原因:")
            console.print("  - 账户没有image_to_model权限")
            console.print("  - API需要额外的必需参数")
            console.print("  - 请求格式与API文档不匹配")
            console.print("  - 账户类型不支持此功能")
            
            console.print(f"\n[cyan]🔧 建议的解决步骤:[/cyan]")
            console.print("1. 检查账户状态: https://platform.tripo3d.ai")
            console.print("2. 验证订阅是否包含image_to_model功能")
            console.print("3. 确认API密钥对应的账户类型")
            console.print("4. 查看官方API文档获取最新参数格式")
            
        elif "credit" in error_str or "quota" in error_str or "402" in error_str:
            console.print("[yellow]💰 账户配额问题[/yellow]")
            console.print("• 账户余额不足或配额已用完")
            console.print("• 解决方案:")
            console.print("  - 检查账户余额")
            console.print("  - 充值或升级订阅")
            console.print("  - 查看配额使用情况")
            
        elif "auth" in error_str or "401" in error_str:
            console.print("[yellow]🔑 认证问题[/yellow]")
            console.print("• API密钥认证失败")
            console.print("• 可能的原因:")
            console.print("  - API密钥无效或过期")
            console.print("  - TRIPO_API_KEY环境变量设置错误")
            console.print("  - 需要官方API密钥(tsk_开头)")
            
        elif "429" in error_str:
            console.print("[yellow]⏰ 频率限制[/yellow]")
            console.print("• API请求频率过高")
            console.print("• 等待一段时间后重试")
            
        elif "404" in error_str:
            console.print("[yellow]🔗 API端点问题[/yellow]")
            console.print("• API端点不存在或URL错误")
            console.print("• 可能API版本已更改")
            
        elif "500" in error_str or "502" in error_str or "503" in error_str:
            console.print("[yellow]🖥️ 服务器问题[/yellow]")
            console.print("• Tripo3D服务器内部错误")
            console.print("• 这是API提供商的问题，请稍后重试")
            
        else:
            console.print("[yellow]❓ 未知错误类型[/yellow]")
            console.print("• 这可能是新的API问题")
            console.print("• 建议联系Tripo3D技术支持")
        
        # API测试建议
        console.print(f"\n[cyan]🧪 进一步测试建议:[/cyan]")
        
        # 自动测试text_to_model来验证账户权限
        console.print("正在测试text_to_model以验证账户权限...")
        try:
            async with httpx.AsyncClient(timeout=30.0, trust_env=False) as test_client:
                test_payload = {
                    "type": "text_to_model",
                    "prompt": "a simple red cube"
                }
                test_headers = {
                    "Authorization": f"Bearer {os.getenv('TRIPO_API_KEY') or os.getenv('FAL_KEY')}",
                    "Content-Type": "application/json"
                }
                test_response = await test_client.post(
                    "https://api.tripo3d.ai/v2/openapi/task",
                    headers=test_headers,
                    json=test_payload
                )
                
                if test_response.status_code == 403:
                    console.print("[yellow]✅ API认证正常，但账户余额不足[/yellow]")
                    console.print("[yellow]🔍 这表明image_to_model可能需要特殊权限[/yellow]")
                elif test_response.status_code == 200:
                    console.print("[green]✅ text_to_model功能正常[/green]")
                    console.print("[yellow]🔍 问题可能是image_to_model权限或格式不同[/yellow]")
                else:
                    console.print(f"[red]❌ text_to_model也失败: {test_response.status_code}[/red]")
                    console.print(f"[red]原因: {test_response.text[:100]}[/red]")
                    
        except Exception as test_error:
            console.print(f"[red]❌ 无法测试text_to_model: {test_error}[/red]")
        
        console.print("\n其他建议:")
        console.print("1. 确认使用官方API密钥(tsk_开头)")
        console.print("2. 检查账户是否包含image_to_model权限")
        console.print("3. 查看订阅计划详情")
        console.print("4. 联系Tripo3D确认API格式和权限")
        
        console.print(f"\n[yellow]🔗 有用链接:[/yellow]")
        console.print("• 账户管理: https://platform.tripo3d.ai")
        console.print("• API文档: https://platform.tripo3d.ai/docs")
        console.print("• 技术支持: 通过平台联系")
        
        return False
            
    except ConfigurationError as e:
        console.print(f"[red]❌ 配置错误: {e}[/red]")
        return False
    except Exception as e:
        console.print(f"[red]❌ 意外错误: {e}[/red]")
        import traceback
        console.print(f"[red]{traceback.format_exc()}[/red]")
        return False


async def batch_generate(
    image_directory: str,
    quality: str = "low"
) -> None:
    """批量生成多个图像的3D模型
    
    Args:
        image_directory: 包含输入图像的目录
        quality: 所有生成的质量等级
    """
    console.print(f"[bold cyan]🔄 批量3D模型生成[/bold cyan]")
    console.print("=" * 40)
    
    image_dir = Path(image_directory)
    if not image_dir.exists():
        console.print(f"[red]❌ 图像目录不存在: {image_directory}[/red]")
        return
    
    # 查找图像文件
    image_files = []
    supported_exts = ['.png', '.jpg', '.jpeg']
    for ext in supported_exts:
        image_files.extend(image_dir.glob(f"*{ext}"))
        image_files.extend(image_dir.glob(f"*{ext.upper()}"))
    
    if not image_files:
        console.print(f"[red]❌ 在 {image_directory} 中未找到图像文件[/red]")
        console.print(f"[yellow]支持格式: {', '.join(supported_exts)}[/yellow]")
        return
    
    console.print(f"[cyan]📁 找到 {len(image_files)} 个图像文件进行处理[/cyan]")
    console.print(f"[cyan]使用质量等级: {quality}[/cyan]\n")
    
    successful = 0
    failed = 0
    
    for i, image_file in enumerate(image_files, 1):
        hero_name = f"batch_{image_file.stem}"
        
        console.print(f"\n[bold]{'='*60}[/bold]")
        console.print(f"[bold cyan]处理 {i}/{len(image_files)}: {image_file.name}[/bold cyan]")
        console.print(f"[bold]{'='*60}[/bold]")
        
        success = await generate_3d_asset(
            hero_name=hero_name,
            image_path=str(image_file),
            quality=quality
        )
        
        if success:
            successful += 1
            console.print(f"[green]✅ {image_file.name} 处理成功[/green]")
        else:
            failed += 1
            console.print(f"[red]❌ {image_file.name} 处理失败[/red]")
            
        # API请求间隔
        if i < len(image_files):
            console.print(f"[yellow]⏱️  等待10秒后处理下一个文件...[/yellow]")
            await asyncio.sleep(10)
    
    # 批量处理结果统计
    console.print(f"\n[bold cyan]📊 批量生成完成[/bold cyan]")
    
    result_table = Table(show_header=True, header_style="bold magenta")
    result_table.add_column("结果", style="cyan")
    result_table.add_column("数量", style="green")
    
    result_table.add_row("成功", str(successful))
    result_table.add_row("失败", str(failed))
    result_table.add_row("总计", str(len(image_files)))
    
    console.print(result_table)


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="Tripo3D AI 3D模型生成示例",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
    # 使用指定图像和英雄名称
    python examples/tripo_3d_generation_example.py path/to/image.png my_hero
    
    # 指定质量等级
    python examples/tripo_3d_generation_example.py path/to/image.png my_hero --quality high
    
    # 添加样式
    python examples/tripo_3d_generation_example.py path/to/image.png my_hero --style "object:clay"
    
    # 批量处理
    python examples/tripo_3d_generation_example.py --batch images_folder
    
    # 显示帮助和质量等级信息
    python examples/tripo_3d_generation_example.py --info

专注Tripo3D官方API测试:
    此脚本专门用于测试Tripo3D官方API功能，使用file_token上传机制。
    只支持官方API密钥(tsk_开头)，如果遇到API问题会提供详细诊断。
        """
    )
    
    parser.add_argument(
        'image_path',
        nargs='?',
        help='输入图像文件路径'
    )
    
    parser.add_argument(
        'hero_name', 
        nargs='?',
        help='英雄名称'
    )
    
    parser.add_argument(
        '--quality', '-q',
        choices=['low', 'medium', 'high'],
        default='medium',
        help='质量等级 (默认: medium)'
    )
    
    parser.add_argument(
        '--style', '-s',
        help='样式提示 (例如: "object:clay", "person:person2cartoon")'
    )
    
    parser.add_argument(
        '--batch', '-b',
        metavar='DIR',
        help='批量处理指定目录中的所有图像'
    )
    
    parser.add_argument(
        '--info', '-i',
        action='store_true',
        help='显示质量等级信息和系统状态'
    )
    
    
    return parser.parse_args()

def show_info():
    """显示系统信息和质量等级"""
    console.print(f"[bold cyan]Tripo3D AI 3D模型生成 - 系统信息[/bold cyan]\n")
    
    # API配置信息
    api_key = os.getenv("TRIPO_API_KEY") or os.getenv("FAL_KEY")
    
    if api_key and api_key.startswith("tsk_"):
        console.print(f"[green]✅ Tripo3D API密钥: 已配置 (官方API)[/green]")
        console.print(f"[cyan]📡 API端点: https://api.tripo3d.ai/v2/openapi/task[/cyan]")
    elif api_key:
        console.print(f"[red]❌ 只支持官方Tripo3D API密钥 (以tsk_开头)[/red]")
        console.print(f"[yellow]💡 请获取官方API密钥:[/yellow]")
        console.print("https://platform.tripo3d.ai/api-keys")
    else:
        console.print(f"[red]❌ Tripo3D API密钥: 未配置[/red]")
        console.print(f"[yellow]💡 设置方法:[/yellow]")
        console.print("export TRIPO_API_KEY='your_api_key_here'")
    
    # 质量等级表格
    console.print(f"\n[cyan]质量等级配置:[/cyan]")
    quality_table = Table(show_header=True, header_style="bold magenta")
    quality_table.add_column("等级", style="cyan")
    quality_table.add_column("描述", style="green")
    quality_table.add_column("纹理", style="yellow") 
    quality_table.add_column("面数", style="yellow")
    quality_table.add_column("PBR", style="yellow")
    quality_table.add_column("时间", style="red")
    
    for quality, preset in QUALITY_PRESETS.items():
        quality_table.add_row(
            quality.upper(),
            preset['description'],
            preset['texture_quality'],
            f"{preset['face_limit']:,}",
            str(preset['enable_pbr']),
            preset['estimated_time']
        )
    
    console.print(quality_table)
    
    # API测试建议
    console.print(f"\n[cyan]API测试建议:[/cyan]")
    console.print("• 首次使用建议从low质量开始测试")
    console.print("• 确认账户有足够的配额")
    console.print("• 检查image_to_model权限")
    
    # 使用示例
    console.print(f"\n[cyan]使用示例:[/cyan]")
    examples = [
        "python examples/tripo_3d_generation_example.py image.png my_hero",
        "python examples/tripo_3d_generation_example.py image.png my_hero --quality high",
        "python examples/tripo_3d_generation_example.py image.png my_hero --style 'object:clay'",
        "python examples/tripo_3d_generation_example.py --batch images_folder",
    ]
    
    for example in examples:
        console.print(f"  [green]{example}[/green]")

async def main():
    """主函数"""
    args = parse_arguments()
    
    console.print(f"[bold cyan]🎨 Tripo3D AI 3D模型生成[/bold cyan]\n")
    
    # 显示信息模式
    if args.info:
        show_info()
        return
    
    # 检查运行要求
    if not check_requirements():
        sys.exit(1)
    
    console.print()
    
    # 批量处理模式
    if args.batch:
        await batch_generate(args.batch, args.quality)
        return
    
    # 单个文件处理模式
    if not args.image_path or not args.hero_name:
        # 交互式输入或使用现有英雄图像
        if not args.image_path:
            # 尝试使用现有的英雄图像
            default_image_result = get_default_hero_image()
            if default_image_result:
                default_image, suggested_name = default_image_result
                args.image_path = str(default_image)
                console.print(f"[yellow]使用现有英雄图像: {args.image_path}[/yellow]")
                if not args.hero_name:
                    args.hero_name = f"{suggested_name}_tripo"
                    console.print(f"[yellow]建议英雄名称: {args.hero_name}[/yellow]")
            else:
                console.print("[red]❌ 请提供图像文件路径[/red]")
                console.print("使用方法: python examples/tripo_3d_generation_example.py <image_path> <hero_name>")
                console.print("或者在heroes目录中放置英雄图像")
                sys.exit(1)
        
        if not args.hero_name:
            args.hero_name = "test_hero_tripo"
            console.print(f"[yellow]使用默认英雄名称: {args.hero_name}[/yellow]")
    
    # 生成3D模型
    success = await generate_3d_asset(
        hero_name=args.hero_name,
        image_path=args.image_path,
        quality=args.quality,
        style=args.style
    )
    
    if success:
        console.print(f"\n[green]🎉 生成完成![/green]")
    else:
        console.print(f"\n[red]❌ 生成失败[/red]")
        sys.exit(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        console.print("\n[yellow]⏹️  用户取消操作[/yellow]")
    except Exception as e:
        console.print(f"\n[red]❌ 程序错误: {e}[/red]")
        sys.exit(1)