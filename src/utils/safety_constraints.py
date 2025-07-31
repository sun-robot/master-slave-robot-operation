"""
安全约束管理器
用于管理机械臂的安全约束，包括工作空间边界、Z轴高度等
"""

import numpy as np
import yaml
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import logging


class SafetyConstraintManager:
    """安全约束管理器"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化安全约束管理器
        
        Args:
            config_path: 配置文件路径，如果为None则使用默认配置
        """
        # 先设置日志
        self._setup_logging()
        
        # 然后加载配置
        self.config = self._load_config(config_path)
        
        # 更新日志级别
        self._update_logging_level()
        
        # 工作空间边界
        self.workspace_limits = self.config['workspace_limits']
        
        # 约束开关
        self.safety_enabled = self.config['safety_enabled']
        self.constraints = self.config['constraints']
        
        # 警告阈值
        self.warnings = self.config['warnings']
        
        # 紧急停止设置
        self.emergency_stop = self.config['emergency_stop']
        
        self.logger.info("安全约束管理器初始化完成")
    
    def _load_config(self, config_path: Optional[str]) -> Dict:
        """加载配置文件"""
        if config_path is None:
            config_path = "config/safety_constraints.yaml"
        
        config_file = Path(config_path)
        if config_file.exists():
            with open(config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self.logger.info(f"从文件加载配置: {config_path}")
        else:
            # 使用默认配置
            config = {
                'workspace_limits': {
                    'x': [-0.8, 0.8],
                    'y': [-0.8, 0.8],
                    'z': [0.0, 1.2]
                },
                'safety_enabled': True,
                'constraints': {
                    'min_z_height': True,
                    'workspace_limits': True,
                    'joint_limits': True,
                    'velocity_limits': True
                },
                'warnings': {
                    'boundary_warning_threshold': 0.9,
                    'ground_warning_threshold': 0.1
                },
                'emergency_stop': {
                    'enabled': True,
                    'stop_on_boundary_violation': True,
                    'stop_on_ground_approach': False
                },
                'logging': {
                    'enabled': True,
                    'level': 'WARNING'
                }
            }
            self.logger.warning(f"配置文件不存在: {config_path}，使用默认配置")
        
        return config
    
    def _setup_logging(self):
        """设置日志"""
        self.logger = logging.getLogger('SafetyConstraints')
        
        # 设置默认日志级别
        self.logger.setLevel(logging.WARNING)
        
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
    
    def _update_logging_level(self):
        """更新日志级别（在配置加载后调用）"""
        if hasattr(self, 'config') and 'logging' in self.config:
            level = self.config['logging']['level']
            self.logger.setLevel(getattr(logging, level))
    
    def apply_safety_constraints(self, position: np.ndarray, 
                                joint_positions: Optional[np.ndarray] = None,
                                joint_velocities: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        应用安全约束
        
        Args:
            position: 末端位置 [x, y, z]
            joint_positions: 关节位置（可选）
            joint_velocities: 关节速度（可选）
            
        Returns:
            constrained_position: 约束后的位置
            is_safe: 是否安全
        """
        if not self.safety_enabled:
            return position, True
        
        constrained_position = position.copy()
        is_safe = True
        violations = []
        
        # 约束1: Z轴不允许小于0
        if self.constraints['min_z_height']:
            if constrained_position[2] < 0:
                self.logger.warning(f"Z轴位置 {constrained_position[2]:.3f} 小于0，调整为0")
                constrained_position[2] = 0.0
                violations.append("Z轴高度约束")
                is_safe = False
        
        # 约束2: 工作空间边界约束
        if self.constraints['workspace_limits']:
            # 检查X轴
            if constrained_position[0] < self.workspace_limits['x'][0]:
                self.logger.warning(f"X轴位置 {constrained_position[0]:.3f} 超出下限 {self.workspace_limits['x'][0]}")
                constrained_position[0] = self.workspace_limits['x'][0]
                violations.append("X轴下限")
                is_safe = False
            elif constrained_position[0] > self.workspace_limits['x'][1]:
                self.logger.warning(f"X轴位置 {constrained_position[0]:.3f} 超出上限 {self.workspace_limits['x'][1]}")
                constrained_position[0] = self.workspace_limits['x'][1]
                violations.append("X轴上限")
                is_safe = False
            
            # 检查Y轴
            if constrained_position[1] < self.workspace_limits['y'][0]:
                self.logger.warning(f"Y轴位置 {constrained_position[1]:.3f} 超出下限 {self.workspace_limits['y'][0]}")
                constrained_position[1] = self.workspace_limits['y'][0]
                violations.append("Y轴下限")
                is_safe = False
            elif constrained_position[1] > self.workspace_limits['y'][1]:
                self.logger.warning(f"Y轴位置 {constrained_position[1]:.3f} 超出上限 {self.workspace_limits['y'][1]}")
                constrained_position[1] = self.workspace_limits['y'][1]
                violations.append("Y轴上限")
                is_safe = False
            
            # 检查Z轴上限
            if constrained_position[2] > self.workspace_limits['z'][1]:
                self.logger.warning(f"Z轴位置 {constrained_position[2]:.3f} 超出上限 {self.workspace_limits['z'][1]}")
                constrained_position[2] = self.workspace_limits['z'][1]
                violations.append("Z轴上限")
                is_safe = False
        
        # 约束3: 关节限位约束
        if self.constraints['joint_limits'] and joint_positions is not None:
            # 这里可以添加关节限位检查
            pass
        
        # 约束4: 速度限制约束
        if self.constraints['velocity_limits'] and joint_velocities is not None:
            # 这里可以添加速度限制检查
            pass
        
        # 检查是否需要紧急停止
        if self.emergency_stop['enabled'] and violations:
            if self.emergency_stop['stop_on_boundary_violation']:
                self.logger.error(f"检测到边界违规: {violations}，建议紧急停止")
        
        return constrained_position, is_safe
    
    def check_boundary_warning(self, position: np.ndarray) -> List[str]:
        """
        检查是否接近边界并发出警告
        
        Args:
            position: 位置 [x, y, z]
            
        Returns:
            warnings: 警告列表
        """
        warnings = []
        
        # 检查X轴
        x_range = self.workspace_limits['x'][1] - self.workspace_limits['x'][0]
        x_center = (self.workspace_limits['x'][1] + self.workspace_limits['x'][0]) / 2
        x_distance = abs(position[0] - x_center) / (x_range / 2)
        
        if x_distance > self.warnings['boundary_warning_threshold']:
            warnings.append(f"接近X轴边界: {position[0]:.3f}")
        
        # 检查Y轴
        y_range = self.workspace_limits['y'][1] - self.workspace_limits['y'][0]
        y_center = (self.workspace_limits['y'][1] + self.workspace_limits['y'][0]) / 2
        y_distance = abs(position[1] - y_center) / (y_range / 2)
        
        if y_distance > self.warnings['boundary_warning_threshold']:
            warnings.append(f"接近Y轴边界: {position[1]:.3f}")
        
        # 检查Z轴（接近地面）
        if position[2] < self.warnings['ground_warning_threshold']:
            warnings.append(f"接近地面: {position[2]:.3f}")
        
        return warnings
    
    def get_workspace_bounds(self) -> Dict[str, List[float]]:
        """获取工作空间边界"""
        return self.workspace_limits.copy()
    
    def set_workspace_bounds(self, bounds: Dict[str, List[float]]):
        """设置工作空间边界"""
        self.workspace_limits = bounds.copy()
        self.logger.info(f"工作空间边界已更新: {bounds}")
    
    def enable_safety(self, enabled: bool = True):
        """启用或禁用安全约束"""
        self.safety_enabled = enabled
        status = "启用" if enabled else "禁用"
        self.logger.info(f"安全约束已{status}")
    
    def get_safety_status(self) -> Dict:
        """获取安全状态"""
        return {
            'safety_enabled': self.safety_enabled,
            'workspace_limits': self.workspace_limits,
            'constraints': self.constraints,
            'warnings': self.warnings,
            'emergency_stop': self.emergency_stop
        } 