#!/usr/bin/env python3
"""
通用跨机械臂主从控制系统主程序
支持多种机械臂品牌和型号的主从协调控制
"""

import sys
import os
import argparse
import yaml
import logging
from pathlib import Path
from typing import Dict, Any, Optional

# 添加项目根目录到Python路径
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from src.robot_models.robot_interface import RobotType
from demo_master_slave import UniversalMasterSlaveController


def load_config(config_path: str) -> Dict[str, Any]:
    """
    加载配置文件
    
    Args:
        config_path: 配置文件路径
        
    Returns:
        配置字典
    """
    if not Path(config_path).exists():
        raise FileNotFoundError(f"配置文件不存在: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    return config


def setup_logging(config: Dict[str, Any]):
    """
    设置日志系统
    
    Args:
        config: 配置字典
    """
    log_config = config.get('logging', {})
    log_level = getattr(logging, log_config.get('level', 'INFO'))
    log_file = log_config.get('file', 'logs/master_slave.log')
    
    # 创建日志目录
    log_dir = Path(log_file).parent
    log_dir.mkdir(parents=True, exist_ok=True)
    
    # 配置日志格式
    log_format = log_config.get('format', 
                               '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    # 配置日志处理器
    logging.basicConfig(
        level=log_level,
        format=log_format,
        handlers=[
            logging.FileHandler(log_file, encoding='utf-8'),
            logging.StreamHandler(sys.stdout)
        ]
    )


def get_robot_config(config: Dict[str, Any], robot_type: str, 
                    robot_role: str, mode: str) -> Dict[str, Any]:
    """
    获取机械臂配置
    
    Args:
        config: 配置字典
        robot_type: 机械臂类型
        robot_role: 机械臂角色 ("master" 或 "slave")
        mode: 运行模式 ("simulation" 或 "hardware")
        
    Returns:
        机械臂配置字典
    """
    robot_configs = config.get(f'{robot_role}_robot', {})
    robot_config = robot_configs.get(robot_type, {})
    
    if not robot_config:
        raise ValueError(f"未找到机械臂配置: {robot_role}_{robot_type}")
    
    mode_config = robot_config.get(mode, {})
    if not mode_config:
        raise ValueError(f"未找到模式配置: {robot_type}_{mode}")
    
    return mode_config


def validate_robot_type(robot_type: str) -> bool:
    """
    验证机械臂类型是否支持
    
    Args:
        robot_type: 机械臂类型
        
    Returns:
        是否支持
    """
    supported_types = [
        "unitree_d1", "ur3e", "ur5", "ur10", "franka_panda"
    ]
    return robot_type in supported_types


def print_system_info(config: Dict[str, Any]):
    """打印系统信息"""
    print("=" * 60)
    print("通用跨机械臂主从控制系统")
    print("=" * 60)
    print("功能特性:")
    print("1. 支持多种机械臂品牌和型号")
    print("2. 统一的主从控制接口")
    print("3. 实时主从控制，支持位置、速度、力控制")
    print("4. 坐标变换，支持主从机械臂的相对位置调整")
    print("5. 安全约束，确保机械臂在安全范围内运行")
    print("6. 跨品牌兼容性")
    print("7. 配置文件支持")
    print("8. 日志记录和调试功能")
    print("=" * 60)


def print_robot_status(controller: UniversalMasterSlaveController):
    """打印机械臂状态"""
    status = controller.get_system_status()
    print("\n系统状态:")
    for key, value in status.items():
        print(f"  {key}: {value}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="通用跨机械臂主从控制系统",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 使用默认配置
  python main.py
  
  # 指定机械臂类型
  python main.py --master-type unitree_d1 --slave-type ur3e
  
  # 使用配置文件
  python main.py --config config/universal_robot_config.yaml
  
  # 仿真模式
  python main.py --master-mode simulation --slave-mode simulation
  
  # 自定义变换参数
  python main.py --transform 0.2 0.1 0.0 0.0 0.0 0.0
        """
    )
    
    # 基本参数
    parser.add_argument("--config", type=str, default="config/universal_robot_config.yaml",
                       help="配置文件路径")
    parser.add_argument("--master-type", type=str, default="unitree_d1",
                       choices=["unitree_d1", "ur3e", "ur5", "ur10", "franka_panda"],
                       help="主机械臂类型")
    parser.add_argument("--slave-type", type=str, default="ur3e",
                       choices=["unitree_d1", "ur3e", "ur5", "ur10", "franka_panda"],
                       help="从机械臂类型")
    parser.add_argument("--master-mode", type=str, default="simulation",
                       choices=["simulation", "hardware"],
                       help="主机械臂模式")
    parser.add_argument("--slave-mode", type=str, default="hardware",
                       choices=["simulation", "hardware"],
                       help="从机械臂模式")
    
    # 控制参数
    parser.add_argument("--duration", type=float, default=10.0,
                       help="运行时长(秒)")
    parser.add_argument("--transform", type=float, nargs=6, 
                       default=[0.1, 0.1, 0.0, 0.0, 0.0, 0.0],
                       help="空间变换参数 [x, y, z, rx, ry, rz]")
    parser.add_argument("--control-mode", type=str, default="position",
                       choices=["position", "velocity", "force"],
                       help="控制模式")
    
    # 调试参数
    parser.add_argument("--verbose", "-v", action="store_true",
                       help="详细输出")
    parser.add_argument("--debug", action="store_true",
                       help="调试模式")
    parser.add_argument("--dry-run", action="store_true",
                       help="试运行模式，不实际控制机械臂")
    
    args = parser.parse_args()
    
    try:
        # 加载配置文件
        print("正在加载配置文件...")
        config = load_config(args.config)
        
        # 设置日志
        setup_logging(config)
        logger = logging.getLogger(__name__)
        
        if args.verbose:
            logging.getLogger().setLevel(logging.DEBUG)
        
        if args.debug:
            config['debug']['enabled'] = True
            config['debug']['verbose'] = True
        
        # 打印系统信息
        print_system_info(config)
        
        # 验证机械臂类型
        if not validate_robot_type(args.master_type):
            raise ValueError(f"不支持的主机械臂类型: {args.master_type}")
        if not validate_robot_type(args.slave_type):
            raise ValueError(f"不支持的从机械臂类型: {args.slave_type}")
        
        # 获取机械臂配置
        master_config = get_robot_config(config, args.master_type, "master", args.master_mode)
        slave_config = get_robot_config(config, args.slave_type, "slave", args.slave_mode)
        
        logger.info(f"主机械臂配置: {master_config}")
        logger.info(f"从机械臂配置: {slave_config}")
        
        # 转换机械臂类型
        master_type = RobotType[args.master_type.upper()]
        slave_type = RobotType[args.slave_type.upper()]
        
        # 创建主从控制器
        print(f"\n正在初始化主从控制器...")
        print(f"主端: {args.master_type} ({args.master_mode})")
        print(f"从端: {args.slave_type} ({args.slave_mode})")
        
        controller = UniversalMasterSlaveController(
            master_type=master_type,
            slave_type=slave_type,
            master_mode=args.master_mode,
            slave_mode=args.slave_mode
        )
        
        # 设置空间变换
        transform = config['master_slave_control']['transform'].get(
            f"{args.master_type}_to_{args.slave_type}",
            args.transform
        )
        
        transform_matrix = [[1, 0, 0, transform[0]],
                          [0, 1, 0, transform[1]],
                          [0, 0, 1, transform[2]],
                          [0, 0, 0, 1]]
        
        controller.set_master_to_slave_transform(transform_matrix)
        
        # 设置控制模式
        controller.set_control_mode(args.control_mode)
        
        # 打印系统状态
        print_robot_status(controller)
        
        if args.dry_run:
            print("\n⚠️ 试运行模式，不会实际控制机械臂")
            print("系统配置验证完成")
            return
        
        # 运行演示
        print(f"\n开始主从控制演示 (时长: {args.duration}秒)...")
        
        # 基本控制演示
        controller.demo_basic_control()
        
        # 实时控制演示
        controller.demo_realtime_control()
        
        print("\n✅ 主从控制演示完成!")
        
    except FileNotFoundError as e:
        print(f"❌ 文件错误: {e}")
        sys.exit(1)
    except ValueError as e:
        print(f"❌ 参数错误: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ 系统错误: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
        sys.exit(1)
    
    finally:
        # 清理资源
        if 'controller' in locals():
            print("\n正在清理资源...")
            controller.cleanup()


if __name__ == "__main__":
    main() 