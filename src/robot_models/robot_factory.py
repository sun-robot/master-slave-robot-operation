"""
机械臂工厂类
用于统一创建和管理不同品牌的机械臂
"""

from typing import Dict, Optional, Type, Any
from .robot_interface import RobotInterface, RobotType, SimulatedRobotInterface
from .d1arm_robot import D1ArmSimRobot, D1ArmHardware
from .ur_robot import URRobot  # 只导入ROS控制的UR机械臂
from .franka_robot import FrankaSimRobot, FrankaHardware

# 尝试导入ROS版本
try:
    from .ur_robot import URRobot
    ROS_UR_AVAILABLE = True
except ImportError:
    ROS_UR_AVAILABLE = False


class RobotFactory:
    """机械臂工厂类"""
    
    # 注册的机械臂类型
    _robot_classes: Dict[RobotType, Dict[str, Type[RobotInterface]]] = {
        RobotType.UNITREE_D1: {
            'simulation': D1ArmSimRobot,
            'hardware': D1ArmHardware
        },
        RobotType.UR3E: {
            'simulation': SimulatedRobotInterface,  # 使用通用的仿真接口
            'hardware': URRobot  # 真机使用ROS控制
        },
        RobotType.UR5: {
            'simulation': SimulatedRobotInterface,  # 使用通用的仿真接口
            'hardware': URRobot  # 真机使用ROS控制
        },
        RobotType.UR10: {
            'simulation': SimulatedRobotInterface,  # 使用通用的仿真接口
            'hardware': URRobot  # 真机使用ROS控制
        },
        RobotType.FRANKA_PANDA: {
            'simulation': FrankaSimRobot,
            'hardware': FrankaHardware
        }
    }
    
    # 动态添加ROS支持 - 现在所有UR真机都使用ROS控制
    if ROS_UR_AVAILABLE:
        # UR机械臂的真机控制已经统一使用ROS
        pass
    
    @classmethod
    def create_robot(cls, robot_type: RobotType, 
                    mode: str = "simulation",
                    robot_id: Optional[str] = None,
                    **kwargs) -> RobotInterface:
        """
        创建机械臂实例
        
        Args:
            robot_type: 机械臂类型
            mode: 运行模式 ("simulation" 或 "hardware")
            robot_id: 机械臂唯一标识符
            **kwargs: 其他参数
            
        Returns:
            机械臂实例
            
        Raises:
            ValueError: 不支持的机械臂类型或模式
        """
        if robot_type not in cls._robot_classes:
            raise ValueError(f"不支持的机械臂类型: {robot_type}")
        
        if mode not in cls._robot_classes[robot_type]:
            raise ValueError(f"机械臂 {robot_type} 不支持模式: {mode}")
        
        robot_class = cls._robot_classes[robot_type][mode]
        
        # 设置默认robot_id
        if robot_id is None:
            robot_id = f"{robot_type.value}_{mode}"
        
        # 根据机械臂类型传递特定参数
        if robot_type == RobotType.UNITREE_D1:
            return robot_class(robot_id=robot_id, **kwargs)
        elif robot_type in [RobotType.UR3E, RobotType.UR5, RobotType.UR10]:
            return robot_class(robot_type=robot_type, robot_id=robot_id, **kwargs)
        elif robot_type == RobotType.FRANKA_PANDA:
            return robot_class(robot_id=robot_id, **kwargs)
        else:
            return robot_class(robot_type=robot_type, robot_id=robot_id, **kwargs)
    
    @classmethod
    def get_supported_robots(cls) -> Dict[RobotType, list]:
        """
        获取支持的机械臂类型和模式
        
        Returns:
            支持的机械臂类型字典
        """
        return {robot_type: list(modes.keys()) 
                for robot_type, modes in cls._robot_classes.items()}
    
    @classmethod
    def register_robot(cls, robot_type: RobotType, mode: str, 
                      robot_class: Type[RobotInterface]):
        """
        注册新的机械臂类型
        
        Args:
            robot_type: 机械臂类型
            mode: 运行模式
            robot_class: 机械臂类
        """
        if robot_type not in cls._robot_classes:
            cls._robot_classes[robot_type] = {}
        
        cls._robot_classes[robot_type][mode] = robot_class
    
    @classmethod
    def create_d1arm(cls, mode: str = "simulation", 
                    robot_id: str = "d1arm",
                    use_pinocchio: bool = False,
                    urdf_path: Optional[str] = None,
                    **kwargs) -> RobotInterface:
        """
        快速创建D1Arm机械臂
        
        Args:
            mode: 运行模式
            robot_id: 机械臂ID
            use_pinocchio: 是否使用Pinocchio
            urdf_path: URDF文件路径
            **kwargs: 其他参数
            
        Returns:
            D1Arm机械臂实例
        """
        return cls.create_robot(
            RobotType.UNITREE_D1, mode, robot_id,
            use_pinocchio=use_pinocchio,
            urdf_path=urdf_path,
            **kwargs
        )
    
    @classmethod
    def create_ur3e(cls, mode: str = "simulation",
                   robot_id: str = "ur3e",
                   use_pinocchio: bool = False,
                   urdf_path: Optional[str] = None,
                   node_name: str = "ur3e_controller",
                   **kwargs) -> RobotInterface:
        """
        快速创建UR3e机械臂
        
        Args:
            mode: 运行模式 ("simulation", "hardware")
            robot_id: 机械臂ID
            use_pinocchio: 是否使用Pinocchio
            urdf_path: URDF文件路径
            node_name: ROS节点名称 (硬件模式使用ROS控制)
            **kwargs: 其他参数
            
        Returns:
            UR3e机械臂实例
        """
        if mode == "hardware":
            # 硬件模式使用ROS控制，不需要IP地址
            kwargs['node_name'] = node_name
        elif mode == "simulation":
            # 仿真模式使用SimulatedRobotInterface
            kwargs['robot_type'] = RobotType.UR3E
            if urdf_path is None:
                urdf_path = "assets/ur3e/urdf/ur3e.urdf"  # 默认URDF路径
        
        if mode == "hardware":
            # 硬件模式使用ROS控制，不需要use_pinocchio参数
            return cls.create_robot(
                RobotType.UR3E, mode, robot_id,
                urdf_path=urdf_path,
                **kwargs
            )
        else:
            # 仿真模式需要use_pinocchio参数
            return cls.create_robot(
                RobotType.UR3E, mode, robot_id,
                use_pinocchio=use_pinocchio,
                urdf_path=urdf_path,
                **kwargs
            )
    
    @classmethod
    def create_ur5(cls, mode: str = "simulation",
                  robot_id: str = "ur5",
                  use_pinocchio: bool = False,
                  urdf_path: Optional[str] = None,
                  node_name: str = "ur5_controller",
                  **kwargs) -> RobotInterface:
        """
        快速创建UR5机械臂
        
        Args:
            mode: 运行模式 ("simulation", "hardware")
            robot_id: 机械臂ID
            use_pinocchio: 是否使用Pinocchio
            urdf_path: URDF文件路径
            node_name: ROS节点名称 (硬件模式使用ROS控制)
            **kwargs: 其他参数
            
        Returns:
            UR5机械臂实例
        """
        if mode == "hardware":
            # 硬件模式使用ROS控制，不需要IP地址
            kwargs['node_name'] = node_name
        elif mode == "simulation":
            # 仿真模式使用SimulatedRobotInterface
            kwargs['robot_type'] = RobotType.UR5
            if urdf_path is None:
                urdf_path = "assets/ur5/urdf/ur5.urdf"  # 默认URDF路径
        
        if mode == "hardware":
            # 硬件模式使用ROS控制，不需要use_pinocchio参数
            return cls.create_robot(
                RobotType.UR5, mode, robot_id,
                urdf_path=urdf_path,
                **kwargs
            )
        else:
            # 仿真模式需要use_pinocchio参数
            return cls.create_robot(
                RobotType.UR5, mode, robot_id,
                use_pinocchio=use_pinocchio,
                urdf_path=urdf_path,
                **kwargs
            )
    
    @classmethod
    def create_ur10(cls, mode: str = "simulation",
                   robot_id: str = "ur10",
                   use_pinocchio: bool = False,
                   urdf_path: Optional[str] = None,
                   node_name: str = "ur10_controller",
                   **kwargs) -> RobotInterface:
        """
        快速创建UR10机械臂
        
        Args:
            mode: 运行模式 ("simulation", "hardware")
            robot_id: 机械臂ID
            use_pinocchio: 是否使用Pinocchio
            urdf_path: URDF文件路径
            node_name: ROS节点名称 (硬件模式使用ROS控制)
            **kwargs: 其他参数
            
        Returns:
            UR10机械臂实例
        """
        if mode == "hardware":
            # 硬件模式使用ROS控制，不需要IP地址
            kwargs['node_name'] = node_name
        elif mode == "simulation":
            # 仿真模式使用SimulatedRobotInterface
            kwargs['robot_type'] = RobotType.UR10
            if urdf_path is None:
                urdf_path = "assets/ur10/urdf/ur10.urdf"  # 默认URDF路径
        
        if mode == "hardware":
            # 硬件模式使用ROS控制，不需要use_pinocchio参数
            return cls.create_robot(
                RobotType.UR10, mode, robot_id,
                urdf_path=urdf_path,
                **kwargs
            )
        else:
            # 仿真模式需要use_pinocchio参数
            return cls.create_robot(
                RobotType.UR10, mode, robot_id,
                use_pinocchio=use_pinocchio,
                urdf_path=urdf_path,
                **kwargs
            )
    
    @classmethod
    def create_franka(cls, mode: str = "simulation",
                     robot_id: str = "franka_panda",
                     use_pinocchio: bool = False,
                     urdf_path: Optional[str] = None,
                     franka_ip: str = "172.16.0.2",
                     **kwargs) -> RobotInterface:
        """
        快速创建Franka Panda机械臂
        
        Args:
            mode: 运行模式
            robot_id: 机械臂ID
            use_pinocchio: 是否使用Pinocchio
            urdf_path: URDF文件路径
            franka_ip: Franka控制器IP (硬件模式)
            **kwargs: 其他参数
            
        Returns:
            Franka机械臂实例
        """
        if mode == "hardware":
            kwargs['franka_ip'] = franka_ip
        
        return cls.create_robot(
            RobotType.FRANKA_PANDA, mode, robot_id,
            use_pinocchio=use_pinocchio,
            urdf_path=urdf_path,
            **kwargs
        )


class RobotManager:
    """机械臂管理器"""
    
    def __init__(self):
        self._robots: Dict[str, RobotInterface] = {}
        self._active_robot: Optional[str] = None
    
    def add_robot(self, robot: RobotInterface, robot_id: Optional[str] = None) -> str:
        """
        添加机械臂到管理器
        
        Args:
            robot: 机械臂实例
            robot_id: 机械臂ID (如果为None则使用robot.robot_id)
            
        Returns:
            机械臂ID
        """
        if robot_id is None:
            robot_id = robot.robot_id
        
        self._robots[robot_id] = robot
        
        # 如果这是第一个机械臂，设为活动机械臂
        if self._active_robot is None:
            self._active_robot = robot_id
        
        return robot_id
    
    def remove_robot(self, robot_id: str) -> bool:
        """
        移除机械臂
        
        Args:
            robot_id: 机械臂ID
            
        Returns:
            是否成功移除
        """
        if robot_id not in self._robots:
            return False
        
        # 停止机械臂并断开连接
        robot = self._robots[robot_id]
        robot.stop_motion(emergency=False)
        robot.disable()
        
        del self._robots[robot_id]
        
        # 如果移除的是活动机械臂，选择新的活动机械臂
        if self._active_robot == robot_id:
            self._active_robot = next(iter(self._robots.keys())) if self._robots else None
        
        return True
    
    def get_robot(self, robot_id: str) -> Optional[RobotInterface]:
        """
        获取机械臂实例
        
        Args:
            robot_id: 机械臂ID
            
        Returns:
            机械臂实例或None
        """
        return self._robots.get(robot_id)
    
    def get_active_robot(self) -> Optional[RobotInterface]:
        """
        获取活动机械臂
        
        Returns:
            活动机械臂实例或None
        """
        if self._active_robot:
            return self._robots.get(self._active_robot)
        return None
    
    def set_active_robot(self, robot_id: str) -> bool:
        """
        设置活动机械臂
        
        Args:
            robot_id: 机械臂ID
            
        Returns:
            是否成功设置
        """
        if robot_id in self._robots:
            self._active_robot = robot_id
            return True
        return False
    
    def list_robots(self) -> Dict[str, Dict]:
        """
        列出所有机械臂
        
        Returns:
            机械臂信息字典
        """
        result = {}
        for robot_id, robot in self._robots.items():
            result[robot_id] = {
                "info": robot.get_robot_info(),
                "is_active": robot_id == self._active_robot,
                "is_ready": robot.is_ready(),
                "is_moving": robot.is_moving()
            }
        return result
    
    def enable_all(self) -> Dict[str, bool]:
        """
        使能所有机械臂
        
        Returns:
            每个机械臂的使能结果
        """
        results = {}
        for robot_id, robot in self._robots.items():
            results[robot_id] = robot.enable()
        return results
    
    def disable_all(self) -> Dict[str, bool]:
        """
        失能所有机械臂
        
        Returns:
            每个机械臂的失能结果
        """
        results = {}
        for robot_id, robot in self._robots.items():
            results[robot_id] = robot.disable()
        return results
    
    def stop_all(self, emergency: bool = False) -> Dict[str, bool]:
        """
        停止所有机械臂
        
        Args:
            emergency: 是否紧急停止
            
        Returns:
            每个机械臂的停止结果
        """
        results = {}
        for robot_id, robot in self._robots.items():
            results[robot_id] = robot.stop_motion(emergency=emergency)
        return results
    
    def get_system_status(self) -> Dict:
        """
        获取系统状态
        
        Returns:
            系统状态字典
        """
        total_robots = len(self._robots)
        ready_robots = sum(1 for robot in self._robots.values() if robot.is_ready())
        moving_robots = sum(1 for robot in self._robots.values() if robot.is_moving())
        error_robots = sum(1 for robot in self._robots.values() 
                          if robot.state.value == "error")
        
        return {
            "total_robots": total_robots,
            "ready_robots": ready_robots,
            "moving_robots": moving_robots,
            "error_robots": error_robots,
            "active_robot": self._active_robot,
            "supported_types": RobotFactory.get_supported_robots()
        } 