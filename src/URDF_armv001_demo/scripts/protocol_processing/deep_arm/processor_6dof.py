
import os
import sys

import pybullet_data

# 获取项目根目录
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.append(project_root)

from uart_processing.base import UartProcessorBase
import roboticstoolbox as rtb
import numpy as np
import pybullet as p
import modern_robotics as mr
import time
import os
from spatialmath import SE3
import matplotlib.pyplot as plt
import asyncio

class Processor6DOF(UartProcessorBase):
    """6自由度机械臂处理器
    
    支持多种机器人工具包:
    1. Robotics Toolbox (Peter Corke): 
       - 优点: 完整的机器人学算法库，支持可视化，API友好
       - 缺点: 性能一般，3D显示基于VPython
       
    2. PyBullet:
       - 优点: 高性能物理引擎，真实的动力学仿真，碰撞检测
       - 缺点: API较底层，使用复杂
       
    3. Modern Robotics:
       - 优点: 算法实现清晰，教学友好
       - 缺点: 功能相对简单，无可视化
    """
    
    def __init__(self):
        super().__init__()
        # 初始化不同包的��器人模型
        self.init_rtb_robot()
        self.init_pybullet_robot()
        self.init_mr_robot()
        
        # 当前状态
        self.current_joints = np.zeros(6)
        self.target_joints = np.zeros(6)
        self.current_pose = None
        
    def init_rtb_robot(self):
        """初始化Robotics Toolbox的UR5模型"""
        # 创建UR5机器人模型
        from roboticstoolbox.models.URDF import UR5
        self.rtb_robot = UR5()
        print("\nRobotics Toolbox UR5 模型初始化完成:")
        print(f"- 自由度: {self.rtb_robot.n}")
        print(f"- DH参数:\n{self.rtb_robot}")
        
        # 导出URDF文件
        urdf_dir = os.path.join(os.path.dirname(__file__), 'models', 'ur5')
        os.makedirs(urdf_dir, exist_ok=True)
        urdf_path = os.path.join(urdf_dir, 'ur5.urdf')
        
        if not os.path.exists(urdf_path):
            print(f"\n导出URDF文件到: {urdf_path}")
            # 获取URDF字符串并保存
            urdf_content = self.rtb_robot.urdf_string
            with open(urdf_path, "w") as urdf_file:
                urdf_file.write(urdf_content)
        
        return urdf_path
        
    def init_pybullet_robot(self):
        """初始化PyBullet的UR5模型"""
        # 初始化物理引擎
        self.physics_client = p.connect(p.DIRECT)  # 或使用 GUI 模式: p.GUI
        p.setGravity(0, 0, -9.81)
        
        # 获取URDF路径（从rtb模型导出）
        # urdf_path = self.init_rtb_robot()
        urdf_dir = os.path.join(os.path.dirname(__file__), 'models', 'ur5')
        os.makedirs(urdf_dir, exist_ok=True)
        urdf_path = os.path.join(urdf_dir, 'ur5.urdf')
        print(f"URDF文件路径: {urdf_path}")
        urdf_path = "kuka_iiwa/ur5.urdf"  # model.urdf
        
        # 加载UR5 URDF
        # 设置内置资源路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.pb_robot_id = p.loadURDF(urdf_path)
        
        print("\nPyBullet UR5 模型初始化完成:")
        print(f"- 机器人ID: {self.pb_robot_id}")
        num_joints = p.getNumJoints(self.pb_robot_id)
        print(f"- 关节数量: {num_joints}")
        
    def init_mr_robot(self):
        """初始化Modern Robotics的UR5模型"""
        # UR5的Modern Robotics参数
        self.mr_M = np.array([  # 零位姿态
            [1, 0, 0, 0.817],
            [0, 1, 0, 0.191],
            [0, 0, 1, -0.006],
            [0, 0, 0, 1]
        ])
        
        self.mr_Slist = np.array([  # 螺旋轴
            [0, 0,  1,  0, 0, 0],
            [0, -1, 0,  0.089, 0, 0],
            [0, -1, 0,  0.089, 0, 0.425],
            [0, -1, 0,  0.089, 0, 0.817],
            [0,  0, -1, -0.109, 0.817, 0],
            [0, -1, 0,  0, 0, 0.817]
        ]).T
        
        print("\nModern Robotics UR5 模型初始化完成")
        
    def forward_kinematics(self, joints, method='rtb'):
        """正运动学计算
        Args:
            joints: 关节角度列表 [j1, j2, j3, j4, j5, j6]
            method: 使用的工具包 ('rtb', 'pb', 'mr')
        Returns:
            np.array: 4x4齐次变换矩阵
        """
        if method == 'rtb':
            # Robotics Toolbox方法
            T = self.rtb_robot.fkine(joints)
            return T.A  # 转换为numpy数组
            
        elif method == 'pb':
            # PyBullet方法
            for i in range(6):
                p.resetJointState(self.pb_robot_id, i, joints[i])
            link_state = p.getLinkState(self.pb_robot_id, 5)
            pos = link_state[0]
            orn = link_state[1]
            
            # 创建4x4齐次变换矩阵
            T = np.eye(4)
            # 将四元数转换为旋转矩阵，注意先转换为numpy数组
            rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            T[:3, :3] = rot_matrix
            T[:3, 3] = pos
            return T
            
        elif method == 'mr':
            # Modern Robotics方法
            T = mr.FKinSpace(self.mr_M, self.mr_Slist, joints)
            return T
            
        else:
            raise ValueError(f"Unknown method: {method}")
            
    def _rotation_matrix_to_quaternion(self, R):
        """将3x3旋转矩阵转换为四元数
        Args:
            R: 3x3旋转矩阵
        Returns:
            tuple: 四元数 (x, y, z, w)
        """
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
            
        return (x, y, z, w)

    def inverse_kinematics(self, target_pose, method='rtb'):
        """逆运动学计算
        Args:
            target_pose: 目标位姿 (4x4齐次变换矩阵)
            method: 使用的工具包
        Returns:
            np.array: 关节角度解
        """
        if method == 'rtb':
            # Robotics Toolbox方法
            T = SE3(target_pose)
            sol = self.rtb_robot.ikine_LM(T)  # 使用Levenberg-Marquardt方法
            return sol.q
            
        elif method == 'pb':
            # PyBullet方法
            pos = target_pose[:3, 3]
            # 从旋转矩阵计算四元数
            orn = self._rotation_matrix_to_quaternion(target_pose[:3, :3])
            joint_poses = p.calculateInverseKinematics(
                self.pb_robot_id,
                5,  # 末端执行器链接索引
                pos,
                orn
            )
            return np.array(joint_poses[:6])
            
        elif method == 'mr':
            # Modern Robotics方法
            thetalist0 = np.zeros(6)  # 初始猜测
            eomg = 0.01  # 角度误差阈值
            ev = 0.001   # 位置误差阈值
            joints = mr.IKinSpace(
                self.mr_Slist,
                self.mr_M,
                target_pose,
                thetalist0,
                eomg,
                ev
            )
            return joints
            
        else:
            raise ValueError(f"Unknown method: {method}")
            
    def jacobian(self, joints, method='rtb'):
        """计算雅可比矩阵
        Args:
            joints: 关节角度
            method: 使用的工具包
        Returns:
            np.array: 6xn雅可比矩阵
        """
        if method == 'rtb':
            # Robotics Toolbox方法
            J = self.rtb_robot.jacob0(joints)
            return J
            
        elif method == 'mr':
            # Modern Robotics方法
            J = mr.JacobianSpace(self.mr_Slist, joints)
            return J
            
        else:
            raise ValueError(f"Unknown method: {method}")
            
    def dynamics(self, joints, joint_vel, joint_acc, method='rtb'):
        """动力学计算
        Args:
            joints: 关节角度
            joint_vel: 关节速度
            joint_acc: 关节加速度
            method: 使用的工具包
        Returns:
            np.array: 关节力矩
        """
        if method == 'rtb':
            # Robotics Toolbox方法
            tau = self.rtb_robot.rne(joints, joint_vel, joint_acc)
            return tau
            
        elif method == 'pb':
            # PyBullet方法
            # 设置关节状态
            for i in range(6):
                p.resetJointState(
                    self.pb_robot_id,
                    i,
                    joints[i],
                    joint_vel[i]
                )
            # 计算动力学
            joint_torques = p.calculateInverseDynamics(
                self.pb_robot_id,
                joints.tolist(),
                joint_vel.tolist(),
                joint_acc.tolist()
            )
            return np.array(joint_torques)
            
        else:
            raise ValueError(f"Unknown method: {method}")
            
    async def plot_robot(self, joints=None, method='rtb'):
        """绘制机器人
        Args:
            joints: 关节角度(可选)，一维数组[j1, j2, j3, j4, j5, j6]
            method: 使用的工具包
        """
        if joints is None:
            joints = np.zeros(6)
        
        try:
            # 确保joints是正确的格式
            if isinstance(joints, tuple):
                joints = joints[0]
            joints = np.asarray(joints, dtype=np.float64).flatten()
            if len(joints) != 6:
                raise ValueError("joints must be a 6-element array")
            
            if method == 'rtb':
                # Robotics Toolbox方法
                try:
                    # 创建事件循环
                    loop = asyncio.get_event_loop()
                    
                    # 创建图形窗口
                    fig = await loop.run_in_executor(None, 
                        lambda: self.rtb_robot.plot(joints, block=False))
                    
                    # 设置视角
                    ax = fig.ax
                    ax.view_init(elev=20, azim=45)
                    
                    # 添加标签
                    ax.set_xlabel('X')
                    ax.set_ylabel('Y')
                    ax.set_zlabel('Z')
                    ax.set_title('UR5 Robot Visualization')
                    
                    # 显示并等待
                    plt.show(block=True)
                    
                except Exception as e:
                    print(f"Error plotting with RTB: {e}")
                    
            elif method == 'pb':
                # PyBullet方法
                try:
                    # 切换到GUI模式
                    if self.physics_client != p.GUI:
                        p.disconnect()
                        self.physics_client = p.connect(p.GUI)
                        self.init_pybullet_robot()
                        
                    # 设置关节位置
                    for i in range(6):
                        p.resetJointState(self.pb_robot_id, i, float(joints[i]))
                        
                    # 添加调试可视化
                    p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0])  # X轴
                    p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0])  # Y轴
                    p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1])  # Z轴
                    
                    # 设置相机视角
                    p.resetDebugVisualizerCamera(
                        cameraDistance=2.0,
                        cameraYaw=45,
                        cameraPitch=-20,
                        cameraTargetPosition=[0, 0, 0]
                    )
                    
                    await asyncio.sleep(0.1)  # 使用异步等待
                    
                except Exception as e:
                    print(f"Error plotting with PyBullet: {e}")
            else:
                raise ValueError(f"Unknown visualization method: {method}")
        except Exception as e:
            print(f"Error in plot_robot: {e}")
            raise
        
    def export_urdf(self, filepath):
        """导出URDF模型
        Args:
            filepath: 保存路径
        """
        # Robotics Toolbox提供了URDF导出功能
        self.rtb_robot.write_urdf(filepath)
        print(f"URDF model exported to: {filepath}")
        
    async def simulate(self, trajectory, dt=0.01, method='pb'):
        """轨迹仿真
        Args:
            trajectory: 关节空间轨迹点列表
            dt: 时间步长
            method: 使用的工具包
        """
        if method == 'pb':
            # 切换到GUI模��
            if self.physics_client != p.GUI:
                p.disconnect()
                self.physics_client = p.connect(p.GUI)
                self.init_pybullet_robot()
                
            # 仿真循环
            for joints in trajectory:
                # 设置关节目标位置
                for i in range(6):
                    p.setJointMotorControl2(
                        self.pb_robot_id,
                        i,
                        p.POSITION_CONTROL,
                        joints[i]
                    )
                    
                # 步进仿真
                p.stepSimulation()
                await asyncio.sleep(dt)
                
        elif method == 'rtb':
            # Robotics Toolbox方法
            # 创建动画对象
            loop = asyncio.get_event_loop()
            fig = await loop.run_in_executor(None,
                lambda: self.rtb_robot.plot(trajectory[0], block=False))
            
            # 播放轨迹
            for joints in trajectory:
                await loop.run_in_executor(None,
                    lambda: self.rtb_robot.plot(joints, fig=fig, block=False))
                await asyncio.sleep(dt)
                
        else:
            raise ValueError(f"Unknown method: {method}")
            
    def process(self, data):
        """处理接收到的数据"""
        # 在这里实现串口数据处理逻辑
        pass

    def plan_trajectory(self, start_joints, end_joints, method='rtb', **kwargs):
        """轨迹规划
        Args:
            start_joints: 起始关节角度
            end_joints: 目标关节角度
            method: 使用的工具包和算法
                - 'rtb_jtraj': Robotics Toolbox的关节空间轨迹
                - 'rtb_ctraj': Robotics Toolbox的笛卡尔空间轨迹
                - 'pb_rrt': PyBullet的RRT路径规划
                - 'mr_screw': Modern Robotics的螺旋插值
            kwargs: 其他参数，如:
                - steps: 轨迹点数量
                - max_velocity: 最大速度
                - max_acceleration: 最大加速度
        Returns:
            dict: 包含轨迹信息的字典
        """
        steps = kwargs.get('steps', 50)
        
        if method == 'rtb_jtraj':
            # 关节空间轨迹规划
            traj = rtb.jtraj(start_joints, end_joints, steps)
            return {
                'positions': traj.q,
                'velocities': traj.qd,
                'accelerations': traj.qdd,
                'time': traj.t
            }
            
        elif method == 'rtb_ctraj':
            # 笛卡尔空间轨迹规划
            T1 = self.forward_kinematics(start_joints, 'rtb')
            T2 = self.forward_kinematics(end_joints, 'rtb')
            ctraj = rtb.ctraj(SE3(T1), SE3(T2), steps)
            
            # 对每个轨迹点求逆解
            positions = []
            for T in ctraj:
                q = self.inverse_kinematics(T.A, 'rtb')
                positions.append(q)
                
            return {
                'positions': np.array(positions),
                'cartesian_path': ctraj
            }
            
        elif method == 'pb_rrt':
            # PyBullet RRT路径规划
            # 设置碰撞检测
            p.setCollisionFilterGroupMask(self.pb_robot_id, -1, 1, 0)
            
            def check_collision():
                return len(p.getContactPoints(self.pb_robot_id)) == 0
                
            def distance(q1, q2):
                return np.linalg.norm(np.array(q1) - np.array(q2))
            
            # RRT参数
            max_iter = kwargs.get('max_iter', 1000)
            step_size = kwargs.get('step_size', 0.1)
            
            # 实RRT算法
            tree = {tuple(start_joints): None}
            path = []
            
            for _ in range(max_iter):
                # 随机采样
                random_joints = np.random.uniform(-np.pi, np.pi, 6)
                
                # 找最近节点
                nearest = min(tree.keys(), key=lambda x: distance(x, random_joints))
                
                # 步进
                direction = np.array(random_joints) - np.array(nearest)
                direction = direction / np.linalg.norm(direction)
                new_joints = np.array(nearest) + direction * step_size
                
                # 碰撞检测
                for i in range(6):
                    p.resetJointState(self.pb_robot_id, i, new_joints[i])
                if check_collision():
                    tree[tuple(new_joints)] = nearest
                    
                    # 检查是否到达目标
                    if distance(new_joints, end_joints) < step_size:
                        current = tuple(new_joints)
                        while current is not None:
                            path.append(current)
                            current = tree[current]
                        path.reverse()
                        break
            
            return {'positions': np.array(path)}
            
        elif method == 'mr_screw':
            # Modern Robotics螺旋插值
            T1 = self.forward_kinematics(start_joints, 'mr')
            T2 = self.forward_kinematics(end_joints, 'mr')
            
            # 计算螺旋轴参数
            Stheta = mr.MatrixLog6(np.dot(np.linalg.inv(T1), T2))
            
            positions = []
            for s in np.linspace(0, 1, steps):
                # 计算中间位姿
                T = np.dot(T1, mr.MatrixExp6(Stheta * s))
                # 求逆解
                q = self.inverse_kinematics(T, 'mr')
                positions.append(q)
                
            return {'positions': np.array(positions)}
            
        else:
            raise ValueError(f"Unknown trajectory planning method: {method}")

    def check_collision(self, joints, obstacles=None):
        """碰撞检测
        Args:
            joints: 关节角度
            obstacles: 障碍物列表，每个元素为(position, size)
        Returns:
            bool: 是否发生碰撞
        """
        # 使用PyBullet进行碰撞检测
        if self.physics_client != p.GUI:
            p.disconnect()
            self.physics_client = p.connect(p.DIRECT)
            self.init_pybullet_robot()
            
        # 设置机器人位置
        for i in range(6):
            p.resetJointState(self.pb_robot_id, i, joints[i])
            
        # 添加障碍物
        obstacle_ids = []
        if obstacles:
            for pos, size in obstacles:
                obstacle_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
                p.createMultiBody(baseMass=0, baseCollisionShapeIndex=obstacle_id,
                                basePosition=pos)
                obstacle_ids.append(obstacle_id)
                
        # 检查碰撞
        collision = len(p.getContactPoints(self.pb_robot_id)) > 0
        
        # 清理障碍物
        for obs_id in obstacle_ids:
            p.removeBody(obs_id)
            
        return collision

    def analyze_workspace(self, resolution=20):
        """工作空间分析
        Args:
            resolution: 采样分辨率
        Returns:
            dict: 工作空间分析结果
        """
        # 创建采样网格
        x = np.linspace(-1.0, 1.0, resolution)
        y = np.linspace(-1.0, 1.0, resolution)
        z = np.linspace(0.0, 1.5, resolution)
        
        reachable_points = []
        manipulability = []
        
        for xi in x:
            for yi in y:
                for zi in z:
                    # 创建目标位姿
                    target = np.eye(4)
                    target[:3, 3] = [xi, yi, zi]
                    
                    try:
                        # 尝试求逆解
                        joints = self.inverse_kinematics(target, 'rtb')
                        
                        # 计算可操作度
                        J = self.jacobian(joints, 'rtb')
                        w = np.sqrt(np.linalg.det(J @ J.T))
                        
                        reachable_points.append([xi, yi, zi])
                        manipulability.append(w)
                    except:
                        continue
                        
        return {
            'reachable_points': np.array(reachable_points),
            'manipulability': np.array(manipulability)
        }

    def check_singularity(self, joints, threshold=1e-3):
        """奇异点分析
        Args:
            joints: 关节角度
            threshold: 奇异值阈值
        Returns:
            dict: 奇异性分析结果
        """
        # 计算雅可比矩阵
        J = self.jacobian(joints, 'rtb')
        
        # 奇异值分解
        U, s, Vh = np.linalg.svd(J)
        
        # 判断是否接近奇异构型
        is_singular = any(sv < threshold for sv in s)
        
        # 计算可操作度
        manipulability = np.sqrt(np.prod(s))
        
        # 计算条件数
        condition_number = s[0] / s[-1] if s[-1] > threshold else float('inf')
        
        return {
            'is_singular': is_singular,
            'singular_values': s,
            'manipulability': manipulability,
            'condition_number': condition_number,
            'null_space': Vh[-1] if is_singular else None
        }

    def visualize_workspace(self, workspace_data):
        """可视化工作空间
        Args:
            workspace_data: analyze_workspace的返回结果
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        points = workspace_data['reachable_points']
        w = workspace_data['manipulability']
        
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                           c=w, cmap='jet')
        plt.colorbar(scatter, label='Manipulability')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Robot Workspace Analysis')
        
        plt.show()

    def plot_manipulability_ellipsoid(self, joints):
        """绘制可操���度椭球
        Args:
            joints: 关节角度
        """
        J = self.jacobian(joints, 'rtb')
        JJt = J @ J.T
        
        # 计算特征值和特征向量
        eigenvalues, eigenvectors = np.linalg.eig(JJt[:3, :3])  # 只考虑位置部分
        
        # 创建椭球据
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = np.sqrt(eigenvalues[0]) * np.outer(np.cos(u), np.sin(v))
        y = np.sqrt(eigenvalues[1]) * np.outer(np.sin(u), np.sin(v))
        z = np.sqrt(eigenvalues[2]) * np.outer(np.ones_like(u), np.cos(v))
        
        # 旋转椭球
        for i in range(len(x)):
            for j in range(len(x)):
                point = np.array([x[i,j], y[i,j], z[i,j]])
                rotated_point = eigenvectors @ point
                x[i,j], y[i,j], z[i,j] = rotated_point
        
        # 绘制椭球
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(x, y, z, alpha=0.3)
        
        # 添加坐标轴
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Manipulability Ellipsoid')
        
        plt.show()

# 使用示例
def test_6dof_processor():
    processor = Processor6DOF()
    
    # 测试正运动���
    joints = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    print("\n测试正运动学:")
    for method in ['rtb', 'pb', 'mr']:
        T = processor.forward_kinematics(joints, method)
        print(f"\n{method.upper()} 方法:")
        print(T)
        
    # 测试逆运动学
    target_pose = np.array([
        [0, -1, 0, 0.4],
        [1, 0, 0, 0.2],
        [0, 0, 1, 0.6],
        [0, 0, 0, 1]
    ])
    print("\n测试逆运动学:")
    for method in ['rtb', 'pb', 'mr']:
        joints = processor.inverse_kinematics(target_pose, method)
        print(f"\n{method.upper()} 方法:")
        print(joints)
        
    # 测试雅可比矩阵
    print("\n测试雅可比矩阵:")
    for method in ['rtb', 'mr']:
        J = processor.jacobian(joints, method)
        print(f"\n{method.upper()} 方法:")
        print(J)
        
    # 测试动力学
    joint_vel = np.zeros(6)
    joint_acc = np.zeros(6)
    print("\n测试动力学:")
    for method in ['rtb', 'pb']:
        tau = processor.dynamics(joints, joint_vel, joint_acc, method)
        print(f"\n{method.upper()} 方法:")
        print(tau)
        
    # 测试可视化
    print("\n测试可视化:")
    processor.plot_robot(joints, 'rtb')
    processor.plot_robot(joints, 'pb')
    
    # 测试轨迹仿真
    trajectory = []
    for t in np.linspace(0, 2*np.pi, 100):
        joints = np.array([
            np.sin(t),
            np.cos(t),
            np.sin(2*t),
            np.cos(2*t),
            np.sin(3*t),
            np.cos(3*t)
        ])
        trajectory.append(joints)
        
    print("\n测试轨迹仿真:")
    processor.simulate(trajectory, dt=0.01, method='pb')
    processor.simulate(trajectory, dt=0.01, method='rtb')

if __name__ == "__main__":
    test_6dof_processor() 