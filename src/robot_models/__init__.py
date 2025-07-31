"""
机械臂模型模块
"""

from .d1arm_robot import D1ArmModel, D1ArmSimulator
from .robot_interface import RobotInterface, SimulatedRobotInterface, RobotType, RobotState, EndEffectorType
from .d1arm_robot import D1ArmSimRobot, D1ArmHardware
from .ur_robot import URRobot  # 只导入ROS控制的UR机械臂
from .franka_robot import FrankaSimRobot, FrankaHardware
from .robot_factory import RobotFactory, RobotManager

__all__ = [
    # 传统模型
    'D1ArmModel',
    'D1ArmSimulator',
    
    # 抽象接口
    'RobotInterface',
    'SimulatedRobotInterface',
    'RobotType',
    'RobotState', 
    'EndEffectorType',
    
    # 具体实现
    'D1ArmSimRobot',
    'D1ArmHardware',
    'URRobot',  # ROS控制的UR机械臂
    'FrankaSimRobot',
    'FrankaHardware',
    
    # 工厂和管理器
    'RobotFactory',
    'RobotManager'
] 