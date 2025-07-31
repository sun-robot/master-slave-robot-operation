#!/usr/bin/env python3
"""
MuJoCo D1主端 + UR3e从端主从控制演示
使用MuJoCo D1机械臂作为主端，真实UR3e机械臂作为从端
"""

import numpy as np
import time
import threading
from pathlib import Path
from typing import Optional, Tuple

# 导入必要的模块
from src.robot_models.mujoco_robot_interface import MuJoCoRobotInterface
from src.robot_models.ur_robot import URRobot
from src.robot_models.robot_interface import RobotType, RobotState
from src.kinematics.forward_kinematics import ForwardKinematics
from src.kinematics.inverse_kinematics import InverseKinematics
from src.utils.math_utils import normalize_quaternion
from src.utils.safety_constraints import SafetyConstraintManager


class MuJoCoURMasterSlaveController:
    """MuJoCo D1主端 + UR3e从端主从控制器"""
    
    def __init__(self, mujoco_xml_path: str, ur_robot_type: RobotType = RobotType.UR3E):
        """
        初始化主从控制器
        
        Args:
            mujoco_xml_path: MuJoCo D1 XML文件路径
            ur_robot_type: UR机械臂类型
        """
        print("=== 初始化MuJoCo D1 + UR3e主从控制系统 ===")
        
        # 检查XML文件
        if not Path(mujoco_xml_path).exists():
            raise FileNotFoundError(f"MuJoCo XML文件不存在: {mujoco_xml_path}")
        
        # 初始化MuJoCo D1主端
        print("正在初始化MuJoCo D1主端...")
        self.master_robot = MuJoCoRobotInterface(
            robot_type=RobotType.UNITREE_D1,
            xml_path=mujoco_xml_path,
            headless=False,  # 显示可视化界面
            render_mode="human"
        )
        
        # 确保主端正确初始化
        if not self.master_robot.data:
            raise RuntimeError("MuJoCo主端数据未正确初始化")
        
        # 初始化UR3e从端
        print("正在初始化UR3e从端...")
        try:
            self.slave_robot = URRobot(
                robot_type=ur_robot_type,
                robot_id="ur3e_slave",
                node_name='ur3e_slave_controller'
            )
            print("✅ UR3e从端初始化成功")
        except Exception as e:
            print(f"⚠️ UR3e从端初始化失败: {e}")
            print("将使用仿真模式进行演示")
            self.slave_robot = None
        
        # 控制参数
        self.control_frequency = 50.0  # Hz
        self.control_dt = 1.0 / self.control_frequency
        self.is_running = False
        
        # 主从变换矩阵（从机械臂相对主机械臂的位置）
        self.master_to_slave_transform = np.eye(4)
        self.master_to_slave_transform[:3, 3] = np.array([0.1, 0.1, 0.0])  # 使用更保守的偏移
        
        # 控制模式
        self.control_mode = 'position'  # 'position', 'velocity', 'force'
        
        # 状态变量
        self.master_joint_positions = None
        self.slave_joint_positions = None
        self.master_ee_pose = None
        self.slave_ee_pose = None
        
        # 初始化安全约束管理器
        self.safety_manager = SafetyConstraintManager()
        
        print("✅ 主从控制器初始化完成")
    
    def set_master_to_slave_transform(self, transform: np.ndarray):
        """设置主从机械臂之间的坐标变换"""
        self.master_to_slave_transform = transform.copy()
        print(f"主从变换矩阵已设置:\n{transform}")
    
    def set_control_mode(self, mode: str):
        """设置控制模式"""
        if mode in ['position', 'velocity', 'force']:
            self.control_mode = mode
            print(f"控制模式已设置为: {mode}")
        else:
            raise ValueError(f"不支持的控制模式: {mode}")
    
    def enable_robots(self):
        """使能机械臂"""
        print("正在使能机械臂...")
        
        # 使能主端
        print("使能MuJoCo D1主端...")
        if self.master_robot.enable():
            print("✅ MuJoCo D1主端使能成功")
            
            # 验证主端就绪状态
            if self.master_robot.is_ready():
                print("✅ MuJoCo D1主端就绪")
            else:
                print("❌ MuJoCo D1主端未就绪")
                return False
        else:
            print("❌ MuJoCo D1主端使能失败")
            return False
        
        # 使能从端
        if self.slave_robot:
            print("使能UR3e从端...")
            if self.slave_robot.enable():
                print("✅ UR3e从端使能成功")
            else:
                print("❌ UR3e从端使能失败")
                return False
        else:
            print("⚠️ UR3e从端未连接，将使用仿真模式")
        
        return True
    
    def start_control(self):
        """启动主从控制"""
        print("=== 启动主从控制 ===")
        
        # 1. 使能机械臂
        if not self.enable_robots():
            print("❌ 机械臂使能失败，无法启动控制")
            return False
        
        # 2. 检查从端连接
        if self.slave_robot:
            print("检查从端连接状态...")
            if not self.check_slave_connection():
                print("❌ 从端连接失败")
                return False
        
        # 3. 循环等候从端就绪
        if self.slave_robot:
            print("循环等候从端就绪...")
            print("提示: 按 Ctrl+C 可以中断等待")
            if not self.wait_for_slave_ready_loop(check_interval=1.0):
                print("❌ 从端未就绪或等待被中断，无法启动控制")
                return False
        
        # 4. 显示从端状态
        if self.slave_robot:
            status = self.get_slave_status()
            print("从端状态信息:")
            for key, value in status.items():
                if value is not None:
                    print(f"  {key}: {value}")
        
        # 5. 启动控制循环
        self.is_running = True
        print("✅ 主从控制已启动")
        return True
    
    def stop_control(self):
        """停止主从控制"""
        self.is_running = False
        
        # 停止机械臂运动
        if self.master_robot:
            self.master_robot.stop_motion(emergency=True)
        
        if self.slave_robot:
            self.slave_robot.stop_motion(emergency=True)
        
        print("✅ 主从控制已停止")
    
    def get_master_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取主端末端执行器位姿"""
        if self.master_robot and self.master_robot.is_ready():
            return self.master_robot.get_end_effector_pose()
        return None, None
    
    def calculate_slave_target_pose(self, master_position: np.ndarray, 
                                   master_orientation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算从端目标位姿
        
        Args:
            master_position: 主端位置
            master_orientation: 主端姿态（四元数）
            
        Returns:
            slave_position: 从端目标位置
            slave_orientation: 从端目标姿态
        """
        # 将主端位姿转换为齐次变换矩阵
        master_transform = np.eye(4)
        master_transform[:3, 3] = master_position
        
        # 四元数转旋转矩阵
        from scipy.spatial.transform import Rotation as R
        rot = R.from_quat([master_orientation[1], master_orientation[2], 
                           master_orientation[3], master_orientation[0]])
        master_transform[:3, :3] = rot.as_matrix()
        
        # 应用主从变换
        slave_transform = self.master_to_slave_transform @ master_transform
        
        # 提取从端位姿
        slave_position = slave_transform[:3, 3]
        slave_rotation = R.from_matrix(slave_transform[:3, :3])
        slave_orientation = slave_rotation.as_quat()
        slave_orientation = np.array([slave_orientation[3], slave_orientation[0], 
                                     slave_orientation[1], slave_orientation[2]])  # wxyz格式
        
        # 应用安全约束
        slave_position, is_safe = self.safety_manager.apply_safety_constraints(slave_position)
        
        # 检查边界警告
        warnings = self.safety_manager.check_boundary_warning(slave_position)
        for warning in warnings:
            print(f"⚠️ {warning}")
        
        return slave_position, slave_orientation
    
    def control_cycle(self):
        """执行一个控制周期"""
        if not self.is_running:
            return
        
        try:
            # 获取主端当前状态
            if self.master_robot and self.master_robot.is_ready():
                self.master_joint_positions = self.master_robot.get_joint_positions()
                self.master_ee_pose = self.master_robot.get_end_effector_pose()
                
                if self.master_ee_pose[0] is not None:
                    master_position, master_orientation = self.master_ee_pose
                    
                    # 计算从端目标位姿
                    slave_target_position, slave_target_orientation = self.calculate_slave_target_pose(
                        master_position, master_orientation)
                    
                    # 控制从端
                    if self.slave_robot and self.slave_robot.is_ready():
                        # 使用位姿控制
                        success = self.slave_robot.move_to_pose(
                            slave_target_position, 
                            slave_target_orientation,
                            blocking=False
                        )
                        
                        if success:
                            self.slave_joint_positions = self.slave_robot.get_joint_positions()
                            self.slave_ee_pose = self.slave_robot.get_end_effector_pose()
                        else:
                            print("⚠️ 从端位姿控制失败")
                    else:
                        print("⚠️ 从端未就绪")
            
            # 渲染主端
            if self.master_robot:
                self.master_robot.render()
                self.master_robot.step_simulation(1)
        
        except Exception as e:
            print(f"❌ 控制周期执行失败: {e}")
    
    def run_control_loop(self, duration: float = 10.0):
        """运行主从控制循环"""
        print(f"开始主从控制循环，持续时间: {duration}秒")
        
        if not self.start_control():
            return
        
        start_time = time.time()
        
        try:
            while self.is_running and (time.time() - start_time) < duration:
                self.control_cycle()
                time.sleep(self.control_dt)
                
                # 打印状态
                if self.master_ee_pose[0] is not None:
                    master_pos = self.master_ee_pose[0]
                    slave_pos = self.slave_ee_pose[0] if self.slave_ee_pose and self.slave_ee_pose[0] is not None else None
                    
                    print(f"时间: {time.time() - start_time:.1f}s, "
                          f"主端位置: {master_pos[:3].round(3)}, "
                          f"从端位置: {slave_pos[:3].round(3) if slave_pos is not None else 'N/A'}")
        
        except KeyboardInterrupt:
            print("\n接收到中断信号，停止控制...")
        
        finally:
            self.stop_control()
    
    def demo_basic_control(self):
        """基本主从控制演示"""
        print("\n=== 基本主从控制演示 ===")
        
        # 检查主端就绪状态
        print("检查主端就绪状态...")
        if not self.master_robot.is_ready():
            print("❌ 主端未就绪，尝试重新使能...")
            if not self.master_robot.enable():
                print("❌ 主端使能失败，无法进行演示")
                return
            else:
                print("✅ 主端重新使能成功")
        
        # 检查从端就绪状态
        if self.slave_robot:
            print("检查从端就绪状态...")
            status = self.get_slave_status()
            print("从端状态:")
            for key, value in status.items():
                if value is not None:
                    print(f"  {key}: {value}")
            
            if not status['ready']:
                print("⚠️ 从端未就绪，将使用仿真模式进行演示")
                # 尝试循环等候从端就绪
                print("尝试循环等候从端就绪...")
                print("提示: 按 Ctrl+C 可以中断等待")
                if self.wait_for_slave_ready_loop(check_interval=1.0):
                    print("✅ 从端已就绪，可以开始控制")
                else:
                    print("⚠️ 从端仍未就绪或等待被中断，继续使用仿真模式")
        else:
            print("⚠️ 从端未初始化，将使用仿真模式进行演示")
        
        # 设置主从变换 - 使用更保守的偏移
        transform = np.eye(4)
        transform[:3, 3] = np.array([0.1, 0.1, 0.0])  # 减小偏移量
        self.set_master_to_slave_transform(transform)
        
        # 获取当前关节位置作为基准
        current_joints = self.master_robot.get_joint_positions()
        print(f"当前关节位置: {current_joints[:4].round(3)}...")
        
        # 定义相对于当前位置的偏移量
        joint_offsets = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 保持当前位置
            np.array([0.1, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]),  # 小幅正向偏移
            np.array([-0.1, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]), # 小幅负向偏移
            np.array([0.05, 0.1, -0.05, 0.0, 0.0, 0.0, 0.0, 0.0]), # 另一个方向的偏移
        ]
        
        for i, offset in enumerate(joint_offsets):
            # 计算目标位置 = 当前位置 + 偏移量
            target_pos = current_joints + offset
            print(f"\n--- 测试 {i+1} ---")
            print(f"关节偏移量: {offset[:4].round(3)}...")
            print(f"目标关节位置: {target_pos[:4].round(3)}...")
            
            # 移动主端
            if self.master_robot.move_to_joint_positions(target_pos, blocking=True):
                print("✅ 主端移动成功")
                
                # 获取主端位姿
                master_pos, master_quat = self.master_robot.get_end_effector_pose()
                print(f"主端末端位置: {master_pos.round(3)}")
                
                # 计算从端目标
                slave_target_pos, slave_target_quat = self.calculate_slave_target_pose(
                    master_pos, master_quat)
                print(f"从端目标位置: {slave_target_pos.round(3)}")
                
                # 移动从端
                if self.slave_robot and self.slave_robot.is_ready():
                    success = self.slave_robot.move_to_pose(
                        slave_target_pos, slave_target_quat, blocking=True)
                    if success:
                        print("✅ 从端移动成功")
                    else:
                        print("❌ 从端移动失败")
                else:
                    print("⚠️ 从端未就绪，跳过从端控制")
                
                time.sleep(2.0)  # 等待2秒
            else:
                print("❌ 主端移动失败")
    
    def demo_realtime_control(self):
        """实时主从控制演示"""
        print("\n=== 实时主从控制演示 ===")
        
        # 检查主端就绪状态
        if not self.master_robot.is_ready():
            print("❌ 主端未就绪，无法进行实时控制")
            return
        
        # 检查从端就绪状态
        if self.slave_robot:
            status = self.get_slave_status()
            if not status['ready']:
                print("⚠️ 从端未就绪，将使用仿真模式进行演示")
                # 尝试循环等候从端就绪
                print("尝试循环等候从端就绪...")
                print("提示: 按 Ctrl+C 可以中断等待")
                if self.wait_for_slave_ready_loop(check_interval=1.0):
                    print("✅ 从端已就绪，可以开始实时控制")
                else:
                    print("⚠️ 从端仍未就绪或等待被中断，继续使用仿真模式")
            else:
                print("✅ 从端已就绪，开始实时控制")
        else:
            print("⚠️ 从端未初始化，将使用仿真模式进行演示")
        
        print("主端将执行正弦波运动，从端跟随...")
        
        # 运行控制循环
        self.run_control_loop(duration=10.0)
    
    def cleanup(self):
        """清理资源"""
        print("正在清理资源...")
        
        if self.master_robot:
            self.master_robot.close()
        
        if self.slave_robot:
            self.slave_robot.shutdown()
        
        print("✅ 资源清理完成")

    def wait_for_slave_ready_loop(self, check_interval: float = 1.0) -> bool:
        """
        循环等候从端就绪
        
        Args:
            check_interval: 检查间隔（秒）
            
        Returns:
            is_ready: 从端是否就绪
        """
        print("开始循环等候从端就绪...")
        print("按 Ctrl+C 可以中断等待")
        
        start_time = time.time()
        check_count = 0
        
        try:
            while True:
                check_count += 1
                elapsed = time.time() - start_time
                
                # 检查从端连接状态
                if self.slave_robot is None:
                    print(f"[{check_count}] ⚠️ 从端未连接，跳过从端就绪检查")
                    return False
                
                # 检查从端就绪状态
                try:
                    if self.slave_robot.is_ready():
                        print(f"\n✅ 从端已就绪！(等待时间: {elapsed:.1f}秒)")
                        return True
                    
                    # 获取从端状态信息
                    slave_state = self.slave_robot.get_state()
                    
                    # 检查是否处于错误状态
                    if slave_state == RobotState.ERROR:
                        print(f"[{check_count}] ❌ 从端处于错误状态: {slave_state}")
                        return False
                    
                    # 尝试获取关节位置来检查连接
                    try:
                        joint_positions = self.slave_robot.get_joint_positions()
                        if joint_positions is not None:
                            print(f"[{check_count}] 等待中... 状态: {slave_state}, 已等待: {elapsed:.1f}s")
                        else:
                            print(f"[{check_count}] ⚠️ 无法获取关节位置，连接可能有问题")
                    except Exception as e:
                        print(f"[{check_count}] ⚠️ 连接检查失败: {e}")
                    
                except Exception as e:
                    print(f"[{check_count}] ⚠️ 检查从端状态时出错: {e}")
                
                # 等待一段时间后再次检查
                time.sleep(check_interval)
                
        except KeyboardInterrupt:
            print(f"\n⚠️ 用户中断等待 (已等待: {elapsed:.1f}秒)")
            return False
    
    def wait_for_slave_ready(self, timeout: float = 30.0, check_interval: float = 0.1) -> bool:
        """
        等待从端就绪（带超时）
        
        Args:
            timeout: 超时时间（秒）
            check_interval: 检查间隔（秒）
            
        Returns:
            is_ready: 从端是否就绪
        """
        print(f"等待从端就绪，超时时间: {timeout}秒...")
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.slave_robot and self.slave_robot.is_ready():
                print("✅ 从端已就绪")
                return True
            
            # 检查从端连接状态
            if self.slave_robot is None:
                print("⚠️ 从端未连接，跳过从端就绪检查")
                return False
            
            # 检查从端状态
            try:
                # 尝试获取从端状态
                slave_state = self.slave_robot.get_state()
                print(f"从端状态: {slave_state}")
                
                if slave_state == RobotState.ERROR:
                    print("❌ 从端处于错误状态")
                    return False
                
            except Exception as e:
                print(f"⚠️ 检查从端状态时出错: {e}")
            
            # 等待一段时间后再次检查
            time.sleep(check_interval)
            
            # 显示等待进度
            elapsed = time.time() - start_time
            remaining = timeout - elapsed
            print(f"\r等待中... 已等待: {elapsed:.1f}s, 剩余: {remaining:.1f}s", end="")
        
        print(f"\n❌ 从端就绪超时 ({timeout}秒)")
        return False
    
    def check_slave_connection(self) -> bool:
        """
        检查从端连接状态
        
        Returns:
            is_connected: 从端是否连接
        """
        if self.slave_robot is None:
            print("❌ 从端未初始化")
            return False
        
        try:
            # 尝试获取从端基本信息
            joint_positions = self.slave_robot.get_joint_positions()
            if joint_positions is not None:
                print("✅ 从端连接正常")
                return True
            else:
                print("❌ 无法获取从端关节位置")
                return False
        except Exception as e:
            print(f"❌ 从端连接检查失败: {e}")
            return False
    
    def get_slave_status(self) -> dict:
        """
        获取从端状态信息
        
        Returns:
            status: 从端状态字典
        """
        status = {
            'connected': False,
            'ready': False,
            'state': None,
            'joint_positions': None,
            'ee_pose': None,
            'error_message': None
        }
        
        if self.slave_robot is None:
            status['error_message'] = "从端未初始化"
            return status
        
        try:
            # 检查连接状态
            status['connected'] = self.check_slave_connection()
            
            if status['connected']:
                # 检查就绪状态
                status['ready'] = self.slave_robot.is_ready()
                
                # 获取状态
                status['state'] = self.slave_robot.get_state()
                
                # 获取关节位置
                status['joint_positions'] = self.slave_robot.get_joint_positions()
                
                # 获取末端位姿
                if status['ready']:
                    status['ee_pose'] = self.slave_robot.get_end_effector_pose()
                
        except Exception as e:
            status['error_message'] = str(e)
        
        return status


def main():
    """主函数"""
    print("MuJoCo D1主端 + UR3e从端主从控制系统")
    print("=" * 60)
    print("功能特性:")
    print("1. MuJoCo D1作为主端，提供可视化仿真环境")
    print("2. 真实UR3e机械臂作为从端，执行实际动作")
    print("3. 实时主从控制，支持位置、速度、力控制")
    print("4. 坐标变换，支持主从机械臂的相对位置调整")
    print("5. 安全约束，确保机械臂在安全范围内运行")
    print("6. 从端就绪检测，等待从端准备就绪")
    print("=" * 60)
    
    # 检查MuJoCo XML文件
    xml_path = "assets/d1_550/mujoco/d1_550.xml"
    if not Path(xml_path).exists():
        print(f"❌ MuJoCo XML文件不存在: {xml_path}")
        return
    
    try:
        # 创建主从控制器
        controller = MuJoCoURMasterSlaveController(
            mujoco_xml_path=xml_path,
            ur_robot_type=RobotType.UR3E
        )
        
        # 循环等候从端就绪
        print("\n=== 循环等候从端就绪 ===")
        if controller.slave_robot:
            print("检查从端连接状态...")
            if controller.check_slave_connection():
                print("从端连接正常，开始循环等候就绪...")
                print("提示: 按 Ctrl+C 可以中断等待")
                
                # 使用循环等候模式
                if controller.wait_for_slave_ready_loop(check_interval=1.0):
                    print("✅ 从端已就绪，可以开始控制")
                else:
                    print("⚠️ 从端未就绪或等待被中断，将使用仿真模式")
            else:
                print("❌ 从端连接失败，将使用仿真模式")
        else:
            print("⚠️ 从端未初始化，将使用仿真模式")
        
        # 显示从端状态
        if controller.slave_robot:
            status = controller.get_slave_status()
            print("\n从端状态信息:")
            for key, value in status.items():
                if value is not None:
                    print(f"  {key}: {value}")
        
        # 基本控制演示
        controller.demo_basic_control()
        
        # 实时控制演示
        controller.demo_realtime_control()
        
        print("\n✅ 主从控制演示完成!")
        
    except Exception as e:
        print(f"❌ 演示过程中出错: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 清理资源
        if 'controller' in locals():
            controller.cleanup()


if __name__ == "__main__":
    main() 