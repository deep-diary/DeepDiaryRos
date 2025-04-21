from .base import UartProcessorBase
from .protocol import ScaraProtocol
from .arm import ScaraArm
from .motor import Motor
from .config_manager import ConfigManager
from .parallel_scara import ParallelScara
import matplotlib.pyplot as plt
import time
import math
import threading
import json
import os

class ScaraProcessor(UartProcessorBase):
    """SCARA机械臂处理器类"""
    def __init__(self):
        super().__init__()
        self.config = ConfigManager()
        self.protocol = ScaraProtocol(self.config)
        
        # 创建目标双臂和实际双臂
        self.target_scara = ParallelScara(self.config, "target")
        self.actual_scara = ParallelScara(self.config, "actual")
        
        # 运动控制标志和位置
        self.motion_cancelled = False
        self.target_position = None  # 目标位置
        self._is_moving = False
        
        # 示教相关属性
        self.teaching_mode = False
        self.teach_interval = 1.0
        self.last_teach_time = 0
        self.routine_points = []
        self.routine_angles = []
        self.teach_thread = None
        self.teach_thread_running = False
        
        # 设置数据处理回调
        self.set_callbacks(
            data_callback=self.process,
            visualization_callback=None
        )

    def init_motors(self):
        """初始化所有电机"""
        delay = 0.1
        for motor in self.target_scara.get_motors().values():
            test_sequence = [
                (delay, "Motor Enable", self.protocol.create_motor_enable_frame(motor.id)),
                (delay, "Set Zero Position", self.protocol.create_motor_zero_frame(motor.id)),
                (delay, "Set Position Mode", self.protocol.create_motor_mode_frame(motor.id, self.protocol.index['RUN_MODE'], 1)),
                (delay, "Set Speed Limit", self.protocol.create_motor_write_frame(motor.id, self.protocol.index['LIMIT_SPD'], 2.0)),
                (delay, "Set Initial Position", self.protocol.create_motor_write_frame(motor.id, self.protocol.index['LOC_REF'], 0.0))
            ]
            
            for delay, desc, frame in test_sequence:
                print(f"\nSending {desc} frame: {frame.hex()}")
                self.send_data(frame)
                time.sleep(delay)
        
        # 设置初始位置
        init_pos = self.target_scara.left_arm.get_init_position()
        # 更新目标双臂的位置
        left_valid = self.target_scara.left_arm.inverse_kinematics(*init_pos)
        right_valid = self.target_scara.right_arm.inverse_kinematics(*init_pos)
        if left_valid and right_valid:
            self.target_scara.end_position = init_pos
        # 开始读取数据
        # self.start_reading()

    def reset_motors(self):
        """重置所有电机（非使能）"""
        for motor in self.target_scara.get_motors().values():
            frame = self.protocol.create_motor_reset_frame(motor.id)
            self.send_data(frame)
            time.sleep(0.01)

    def home_position(self):
        """机械臂回零"""
        init_pos = self.target_scara.left_arm.get_init_position()
        self.target_position = init_pos  # 更新目标位置
        return self.set_arm_position(*init_pos,
                                     max_step=self.config.get('control', 'max_step', 50.0),
                                     update_callback=self.visualization_callback)

    def set_motor_position(self, motor_id, position):
        """设置单个电机位置"""
        frame = self.protocol.create_motor_write_frame(
            motor_id,
            self.protocol.index['LOC_REF'],
            position
        )
        return self.send_data(frame)

    def get_target_current_position(self):
        """获取目标双臂当前位置
        Returns:
            tuple: (x, y) 目标双臂的当前位置，如果未设置则返回默认位置(0, 200)
        """
        return self.target_scara.end_position if self.target_scara.end_position else (0, 200)
    

    def cancel_motion(self):
        """取消当前运动"""
        print("Cancelling current motion...")
        self.motion_cancelled = True

    def get_actual_position(self):
        """获取实际并联臂的末端位置"""
        return self.actual_scara.end_position if self.actual_scara else None

    def set_arm_position(self, xt, yt, max_step=50.0, update_callback=None):
        """设置机械臂位置"""
        # 重置取消标志
        self.motion_cancelled = False
        target_reached = False
        
        while not target_reached and not self.motion_cancelled:
            # 检查目标是否改变
            if (xt, yt) != self.target_position:
                print("Target position changed, stopping current movement")
                return False
            # 获取目标双臂当前位置和实际位置
            target_pos = self.target_scara.end_position or (0, 200)
            actual_pos = self.get_actual_position() or target_pos
            
            # 计算目标移动距离
            dx_target = xt - target_pos[0]
            dy_target = yt - target_pos[1]
            dist_target = math.sqrt(dx_target * dx_target + dy_target * dy_target)
            
            # 计算实际位置与最终目标的距离
            dx_actual = xt - actual_pos[0]
            dy_actual = yt - actual_pos[1]
            dist_actual = math.sqrt(dx_actual * dx_actual + dy_actual * dy_actual)
            
            # print(f"\nTarget current position: ({target_pos[0]:.2f}, {target_pos[1]:.2f})")
            # print(f"Actual current position: ({actual_pos[0]:.2f}, {actual_pos[1]:.2f})")
            # print(f"Final target position: ({xt:.2f}, {yt:.2f})")
            # print(f"Distance to target: {dist_actual:.2f}")

            # 检查是否被取消
            if self.motion_cancelled:
                print("Movement cancelled!")
                return False

            # 如果目标距离大于最大步长，计算中间点
            if dist_target > max_step:
                step_ratio = max_step / dist_target
                x_next = target_pos[0] + dx_target * step_ratio
                y_next = target_pos[1] + dy_target * step_ratio
                # print(f"Moving to intermediate point: ({x_next:.2f}, {y_next:.2f})")
            else:
                x_next = xt
                y_next = yt
                # print(f"Moving to final target: ({x_next:.2f}, {y_next:.2f})")

            # 更新目标并联臂的位置
            left_valid = self.target_scara.left_arm.inverse_kinematics(x_next, y_next)
            right_valid = self.target_scara.right_arm.inverse_kinematics(x_next, y_next)

            if not (left_valid and right_valid):
                print("Invalid target position")
                return False

            # 更新目标位置
            self.target_scara.end_position = (x_next, y_next)

            # 设置电机位置
            self.set_motor_position(self.target_scara.left_arm.motors[0].id, 
                                  self.target_scara.left_arm.theta_mt)
            self.set_motor_position(self.target_scara.right_arm.motors[0].id, 
                                  self.target_scara.right_arm.theta_mt)

            # 等待一段时间让电机移动
            time.sleep(0.1)
            
            # 调用更新回调
            if update_callback:
                if update_callback():
                    return False

            # 检查是否到达目标位置（使用实际位置判断）
            if dist_actual < 2.0:  # 可以设置为可配置的阈值
                target_reached = True

        return target_reached

    def process(self, data, update_callback=None):
        """处理接���到的数据"""
        result = self.protocol.parse_frame(data)
        if not result:
            return None
        
        if result['mode'] == 0x02:  # 反馈模式
            motor_id = result['motor_id']
            payload = result['payload']
            
            # 更新电机位置
            for motor in self.actual_scara.get_motors().values():
                if motor.id == motor_id:
                    motor.update_from_feedback(payload)
                    
                    # 更新实际双臂的角度和位置
                    try:
                        if self.actual_scara.update_angles(
                            self.actual_scara.left_arm.motors[0].current_position,
                            self.actual_scara.right_arm.motors[0].current_position
                        ):
                            # 调用更新回调
                            if update_callback:
                                rst = update_callback()
                    except Exception as e:
                        print(f"Error updating actual position: {e}")
                    break
        
        return False

    def routine_teach_start(self):
        """开始示教模式"""
        # 重置示教数据
        self.routine_points = []
        self.routine_angles = []
        self.teaching_mode = True
        self.last_teach_time = time.time()
        
        # 关闭所有电机使能
        for motor in self.target_scara.get_motors().values():
            frame = self.protocol.create_motor_reset_frame(motor.id)
            self.send_data(frame)
        
        # 启动位置查询线程
        self.teach_thread_running = True
        self.teach_thread = threading.Thread(target=self._teaching_thread)
        self.teach_thread.daemon = True
        self.teach_thread.start()
        
        print("\nTeaching mode started")
        print("Move the arm to desired positions")
        print("Press 's' to record a point")
        print("Press 't' to stop teaching")

    def routine_teach_stop(self):
        """停止示教模式"""
        self.teaching_mode = False
        
        # 停止位置查询线程
        if self.teach_thread_running:
            self.teach_thread_running = False
            if self.teach_thread:
                self.teach_thread.join(timeout=1.0)
        
        # 重新使能所有电机
        for motor in self.target_scara.get_motors().values():
            frame = self.protocol.create_motor_enable_frame(motor.id)
            self.send_data(frame)
        
        # 保存示教路径
        self.save_routine_to_config()
        return self.routine_points, self.routine_angles

    def _teaching_thread(self):
        """示教模式下的位置查询线程"""
        while self.teach_thread_running:
            try:
                # 查询左电机位置
                frame = self.protocol.create_motor_write_frame(
                    self.target_scara.left_arm.motors[0].id,
                    self.protocol.index['LOC_REF'],
                    0.0
                )
                print(f"Query left motor position: {frame.hex()}")
                self.send_data(frame)
                
                # 查询右电机位置
                frame = self.protocol.create_motor_write_frame(
                    self.target_scara.right_arm.motors[0].id,
                    self.protocol.index['LOC_REF'],
                    0.0
                )
                print(f"Query right motor position: {frame.hex()}")
                self.send_data(frame)
                
                time.sleep(0.1)
            except Exception as e:
                print(f"Error in teaching thread: {e}")

    def routine_teach_add_point(self):
        """添加当前位置到示教路径"""
        if not self.teaching_mode:
            return False
        
        current_pos = self.actual_scara.end_position
        if current_pos:
            self.routine_points.append(current_pos)
            # 保存电机角度
            angles = (self.actual_scara.left_arm.theta_mt,
                     self.actual_scara.right_arm.theta_mt)
            self.routine_angles.append(angles)
            
            point_index = len(self.routine_points) - 1
            print(f"Added point {point_index + 1}: pos={current_pos}, angles={angles}")
            return True
        return False

    def routine_teach_update(self):
        """更新示教状态（用于定时记录）"""
        if not self.teaching_mode:
            return
        
        current_time = time.time()
        if current_time - self.last_teach_time >= self.teach_interval:
            if self.routine_teach_add_point():
                self.last_teach_time = current_time

    def save_routine_to_config(self, file_path=None):
        """保存示教路径到配置文件
        Args:
            file_path: 配置文件路径，如果为None则使用默认路径
        Returns:
            bool: 是否保存成功
        """
        try:
            # 如果没有指定路径，使用默认路径
            if file_path is None:
                # 使用包目录下的configs目录
                base_dir = os.path.dirname(__file__)  # 获取本文件目录
                output_dir = os.path.join(base_dir, 'configs', 'teach_routines')
                default_path = os.path.join(base_dir, 'configs', 'teach_routine.json')
                os.makedirs(output_dir, exist_ok=True)
                
                # 使用时间戳创建文件名
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                file_path = os.path.join(output_dir, f'teach_routine_{timestamp}.json')

            # 生成点位描述
            descriptions = {}
            for i, point in enumerate(self.routine_points):
                descriptions[f"point_{i}"] = f"Position ({point[0]:.1f}, {point[1]:.1f})"

            # 准备保存的数据
            routine_data = {
                'points': self.routine_points,
                'angles': self.routine_angles,
                'timestamp': time.strftime("%Y%m%d_%H%M%S"),
                'description': descriptions
            }

            # 确保输出目录存在
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            
            # 保存数据
            with open(file_path, 'w') as f:
                json.dump(routine_data, f, indent=2)
            
            # 同时更新默认路径
            with open(default_path, 'w') as f:
                json.dump(routine_data, f, indent=2)
            
            print(f"\nTeaching routine saved to: {file_path}")
            print(f"Default teaching routine file updated: {default_path}")
            print(f"Total points: {len(self.routine_points)}")
            return True
            
        except Exception as e:
            print(f"Failed to save routine: {e}")
            return False

    def load_routine_from_config(self, file_path=None):
        """从配置文件加载示教路径"""
        if file_path is None:
            # 使用相对于当前文件的路径
            import os
            file_path = os.path.join(os.path.dirname(__file__), 'configs', 'teach_routine.json')
        
        try:
            if not os.path.exists(file_path):
                print(f"Creating new routine file at {file_path}")
                # 创建默认的示教路径文件
                default_routine = {
                    "points": [],
                    "angles": [],
                    "timestamp": "",
                    "description": {}
                }
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                with open(file_path, 'w') as f:
                    json.dump(default_routine, f, indent=2)
            
            with open(file_path, 'r') as f:
                routine_data = json.load(f)
            
            self.routine_points = routine_data['points']
            self.routine_angles = routine_data['angles']
            
            print(f"Loaded routine from {file_path}")
            print(f"Total points: {len(self.routine_points)}")
            
            if 'description' in routine_data:
                print("\nPoint descriptions:")
                for point_id, desc in routine_data['description'].items():
                    print(f"{point_id}: {desc}")
            
            return True
        except Exception as e:
            print(f"Failed to load routine: {e}")
            return False

    def execute_routine(self, update_callback=None):
        """执行示教路径"""
        # 先加载路径
        if not self.load_routine_from_config():
            return False
        
        if not self.routine_points:
            print("No routine points to execute")
            return False
        
        print(f"\nExecuting routine with {len(self.routine_points)} points")
        for point in self.routine_points:
            self.target_position = tuple(point)
            if not self.set_arm_position(point[0], point[1], 
                                       max_step=self.config.get('control', 'max_step', 50.0),
                                       update_callback=update_callback):
                print("Failed to execute routine")
                return False
            time.sleep(1)  # 点与点之间的延时
        
        print("Routine execution completed")
        return True

    def is_moving(self):
        """检查是否正在运动"""
        return self._is_moving

    def move_up(self):
        """向上移动一个步长"""
        current_pos = self.target_scara.end_position or (0, 200)
        step_size = self.config.get('control', 'step_size', 5.0)
        new_x, new_y = current_pos[0], current_pos[1] + step_size
        self.target_position = (new_x, new_y)
        return self.set_arm_position(new_x, new_y)

    def move_down(self):
        """向下移动一个步长"""
        current_pos = self.target_scara.end_position or (0, 200)
        step_size = self.config.get('control', 'step_size', 5.0)
        new_x, new_y = current_pos[0], current_pos[1] - step_size
        self.target_position = (new_x, new_y)
        return self.set_arm_position(new_x, new_y)

    def move_left(self):
        """向左移动一个步长"""
        current_pos = self.target_scara.end_position or (0, 200)
        step_size = self.config.get('control', 'step_size', 5.0)
        new_x, new_y = current_pos[0] - step_size, current_pos[1]
        self.target_position = (new_x, new_y)
        return self.set_arm_position(new_x, new_y)

    def move_right(self):
        """向右移动一个步长"""
        current_pos = self.target_scara.end_position or (0, 200)
        step_size = self.config.get('control', 'step_size', 5.0)
        new_x, new_y = current_pos[0] + step_size, current_pos[1]
        self.target_position = (new_x, new_y)
        return self.set_arm_position(new_x, new_y)

    def jog_left_positive(self):
        """左电机JOG正向运动"""
        motor_id = self.target_scara.left_arm.motors[0].id
        speed = int(self.config.get('control', 'jog_speed_positive', '0x2182'), 16)
        frame = self.protocol.create_motor_jog_frame(motor_id, speed)
        return self.send_data(frame)

    def jog_left_negative(self):
        """左电机JOG负向运动"""
        motor_id = self.target_scara.left_arm.motors[0].id
        speed = int(self.config.get('control', 'jog_speed_negative', '0xDD7D'), 16)
        frame = self.protocol.create_motor_jog_frame(motor_id, speed)
        return self.send_data(frame)

    def jog_right_positive(self):
        """右电机JOG正向运动"""
        motor_id = self.target_scara.right_arm.motors[0].id
        speed = int(self.config.get('control', 'jog_speed_positive', '0x2182'), 16)
        frame = self.protocol.create_motor_jog_frame(motor_id, speed)
        return self.send_data(frame)

    def jog_right_negative(self):
        """右电机JOG负向运动"""
        motor_id = self.target_scara.right_arm.motors[0].id
        speed = int(self.config.get('control', 'jog_speed_negative', '0xDD7D'), 16)
        frame = self.protocol.create_motor_jog_frame(motor_id, speed)
        return self.send_data(frame)

    def jog_stop(self, motor_id):
        """停止电机JOG运动"""
        frame = self.protocol.create_motor_jog_frame_stop(motor_id)
        return self.send_data(frame)









