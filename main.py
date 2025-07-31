"""
机械臂主从控制系统主程序
演示关节角度到末端位姿转换，以及基于IK的末端位姿到关节角度逆解
"""

import numpy as np
import yaml
import time
from src.robot_models.d1arm_robot import D1ArmModel
from src.kinematics.forward_kinematics import ForwardKinematics
from src.kinematics.inverse_kinematics import InverseKinematics
from src.control.master_slave_control import MasterSlaveController
from src.control.trajectory_planning import TrajectoryPlanner, PathPlanner
from src.utils.visualization import RobotVisualizer, TrajectoryAnalyzer
from src.utils.math_utils import normalize_quaternion, quaternion_to_euler


def load_config(config_file: str = "config/robot_config.yaml"):
    """加载配置文件"""
    with open(config_file, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    return config


def demo_forward_kinematics():
    """演示正向运动学"""
    print("=== 正向运动学演示 ===")
    
    # 创建机械臂模型
    robot_model = D1ArmModel()
    fk_solver = ForwardKinematics(robot_model.get_dh_params())
    
    # 设置关节角度
    joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print(f"关节角度: {joint_angles}")
    
    # 计算末端位姿
    position, orientation = fk_solver.forward_kinematics(joint_angles)
    print(f"末端位置: {position}")
    print(f"末端姿态 (四元数): {orientation}")
    
    # 转换为欧拉角
    euler_angles = quaternion_to_euler(orientation)
    print(f"末端姿态 (欧拉角): {euler_angles}")
    
    # 可视化
    visualizer = RobotVisualizer(robot_model, fk_solver)
    visualizer.setup_plot()
    visualizer.plot_robot_configuration(joint_angles)
    visualizer.show()


def demo_inverse_kinematics():
    """演示逆向运动学"""
    print("\n=== 逆向运动学演示 ===")
    
    # 创建机械臂模型和求解器
    robot_model = D1ArmModel()
    fk_solver = ForwardKinematics(robot_model.get_dh_params())
    ik_solver = InverseKinematics(fk_solver, robot_model.get_joint_limits())
    
    # 设置目标位姿
    target_position = np.array([0.5, 0.0, 0.5])
    target_orientation = normalize_quaternion(np.array([1.0, 0.0, 0.0, 0.0]))
    
    print(f"目标位置: {target_position}")
    print(f"目标姿态: {target_orientation}")
    
    # 初始猜测
    initial_guess = np.zeros(7)
    
    # 求解逆运动学
    joint_angles, success = ik_solver.ik_optimization_method(
        target_position, target_orientation, initial_guess)
    
    if success:
        print(f"求解成功! 关节角度: {joint_angles}")
        
        # 验证结果
        position, orientation = fk_solver.forward_kinematics(joint_angles)
        print(f"验证位置: {position}")
        print(f"验证姿态: {orientation}")
        
        # 可视化
        visualizer = RobotVisualizer(robot_model, fk_solver)
        visualizer.setup_plot()
        visualizer.plot_robot_configuration(joint_angles)
        visualizer.plot_target_pose(target_position, target_orientation)
        visualizer.show()
    else:
        print("逆运动学求解失败!")


def demo_trajectory_planning():
    """演示轨迹规划"""
    print("\n=== 轨迹规划演示 ===")
    
    # 创建机械臂模型和求解器
    robot_model = D1ArmModel()
    fk_solver = ForwardKinematics(robot_model.get_dh_params())
    ik_solver = InverseKinematics(fk_solver, robot_model.get_joint_limits())
    
    # 创建轨迹规划器
    trajectory_planner = TrajectoryPlanner(fk_solver, ik_solver)
    
    # 设置起始和目标关节角度
    start_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    end_angles = np.array([np.pi/4, np.pi/6, np.pi/3, np.pi/4, np.pi/6, np.pi/3, np.pi/4])
    
    print(f"起始关节角度: {start_angles}")
    print(f"目标关节角度: {end_angles}")
    
    # 生成线性轨迹
    duration = 5.0
    time_array, trajectory = trajectory_planner.linear_trajectory(
        start_angles, end_angles, duration)
    
    print(f"轨迹点数: {len(trajectory)}")
    print(f"轨迹时长: {duration} 秒")
    
    # 分析轨迹
    analyzer = TrajectoryAnalyzer()
    analyzer.print_trajectory_summary(trajectory, time_array)
    
    # 可视化轨迹
    visualizer = RobotVisualizer(robot_model, fk_solver)
    visualizer.setup_plot()
    
    # 绘制起始和结束配置
    visualizer.plot_robot_configuration(start_angles, color='blue', alpha=0.5)
    visualizer.plot_robot_configuration(end_angles, color='red', alpha=0.5)
    
    # 绘制轨迹动画
    anim = visualizer.animate_trajectory(trajectory)
    visualizer.show()


def demo_master_slave_control():
    """演示主从控制"""
    print("\n=== 主从控制演示 ===")
    
    # 创建两个相同的机械臂模型（主从）
    master_model = D1ArmModel()
    slave_model = D1ArmModel()
    
    # 创建主从控制器
    controller = MasterSlaveController(master_model, slave_model)
    
    # 设置主从变换矩阵（这里设置为简单的偏移）
    transform = np.eye(4)
    transform[:3, 3] = np.array([0.2, 0.0, 0.0])  # X方向偏移0.2m
    controller.set_master_to_slave_transform(transform)
    
    print("主从变换矩阵:")
    print(transform)
    
    # 设置主机械臂关节角度
    master_angles = np.array([0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 0.1])
    controller.update_master_joint_angles(master_angles)
    
    print(f"主机械臂关节角度: {master_angles}")
    
    # 执行一个控制周期
    controller.control_cycle()
    
    # 获取从机械臂关节角度
    slave_angles = controller.get_slave_joint_angles()
    print(f"从机械臂关节角度: {slave_angles}")
    
    # 获取控制状态
    status = controller.get_control_status()
    print(f"控制状态: {status}")


def demo_cartesian_trajectory():
    """演示笛卡尔空间轨迹"""
    print("\n=== 笛卡尔空间轨迹演示 ===")
    
    # 创建机械臂模型和求解器
    robot_model = D1ArmModel()
    fk_solver = ForwardKinematics(robot_model.get_dh_params())
    ik_solver = InverseKinematics(fk_solver, robot_model.get_joint_limits())
    
    # 创建轨迹规划器
    trajectory_planner = TrajectoryPlanner(fk_solver, ik_solver)
    
    # 设置起始和目标位姿
    start_position = np.array([0.5, 0.0, 0.5])
    end_position = np.array([0.5, 0.2, 0.7])
    start_orientation = normalize_quaternion(np.array([1.0, 0.0, 0.0, 0.0]))
    end_orientation = normalize_quaternion(np.array([0.707, 0.0, 0.707, 0.0]))
    
    print(f"起始位置: {start_position}")
    print(f"目标位置: {end_position}")
    print(f"起始姿态: {start_orientation}")
    print(f"目标姿态: {end_orientation}")
    
    # 生成笛卡尔轨迹
    duration = 3.0
    time_array, position_trajectory, orientation_trajectory = trajectory_planner.cartesian_trajectory(
        start_position, end_position, start_orientation, end_orientation, duration)
    
    # 转换为关节角度轨迹
    initial_guess = np.zeros(7)
    joint_trajectory = trajectory_planner.trajectory_to_joint_angles(
        position_trajectory, orientation_trajectory, initial_guess)
    
    print(f"笛卡尔轨迹点数: {len(position_trajectory)}")
    print(f"关节轨迹点数: {len(joint_trajectory)}")
    
    # 可视化
    visualizer = RobotVisualizer(robot_model, fk_solver)
    visualizer.setup_plot()
    
    # 绘制起始和结束位姿
    visualizer.plot_target_pose(start_position, start_orientation, size=0.1, color='blue')
    visualizer.plot_target_pose(end_position, end_orientation, size=0.1, color='red')
    
    # 绘制轨迹
    visualizer.plot_trajectory(position_trajectory, color='orange', linewidth=3)
    
    # 动画显示
    anim = visualizer.animate_trajectory(joint_trajectory)
    visualizer.show()


def main():
    """主函数"""
    print("机械臂主从控制系统演示")
    print("=" * 50)
    
    try:
        # 加载配置
        config = load_config()
        print("配置文件加载成功")
        
        # 演示正向运动学
        demo_forward_kinematics()
        
        # 演示逆向运动学
        demo_inverse_kinematics()
        
        # 演示轨迹规划
        demo_trajectory_planning()
        
        # 演示主从控制
        demo_master_slave_control()
        
        # 演示笛卡尔空间轨迹
        demo_cartesian_trajectory()
        
        print("\n所有演示完成!")
        
    except Exception as e:
        print(f"程序执行出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main() 