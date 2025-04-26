#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Deep Arm 机械臂示教类
"""

from __future__ import print_function, division, absolute_import

import os
import sys
import time
import logging
import argparse
import threading
import json
from threading import Event
from deep_arm import DeepArm
# Python 2.7 compatibility handling
PY2 = sys.version_info[0] == 2
if PY2:
    print("Running on Python 2.7, applying compatibility fixes...")

# 修复导入路径问题 - 添加 scripts 目录到 Python 路径
current_dir = os.path.dirname(os.path.abspath(__file__))
scripts_dir = os.path.abspath(os.path.join(current_dir, '../../'))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)
    print("\nAdded to Python path: {}".format(scripts_dir))

class Teaching:
    """Deep Arm 机械臂示教类"""

    def __init__(self, deep_arm):
        """
        初始化示教类
        
        Args:
            deep_arm: DeepArm 实例
        """
        self.deep_arm = deep_arm
        self.logger = logging.getLogger('Teaching')
        
        # 示教相关属性
        self.teaching_mode = False
        self.interval_teach = 0.2
        self.interval_add_point = 0.5
        self.last_teach_time = 0
        self.routine_points = []
        self.teach_thread = None
        self.teach_thread_running = False
        self.target_position = None
        self.teaching_status = 'None'  # None, Ready, Teaching, Executing
        self.routine_points_fine_tune = []
        self.is_periodic_add_point = True
        self.fine_tune_freq = 20

        # None: not in teaching mode or stop recording
        # Ready: enable moving the arm without recording
        # Teaching: recording the arm movement
        # Executing: executing the recorded path

    def teach_ready(self):
        """
        准备示教模式
        """
        self.teaching_status = 'Ready'
        # 重置示教数据
        self.routine_points = []
        self.last_teach_time = time.time()

        # 检查并初始化电机
        status = self.deep_arm.arm_get_status()
        for id, init_status in zip(status['ids'], status['initialized']):
            if not init_status:
                self.deep_arm.motor_init(motor_id=id)
                # self.logger.info("Motor %d initialized" % id)
        self.logger.info("All motors initialized --------------------------------")
        
        # 等待3s
        time.sleep(3)
        # 关闭所有电机使能
        self.deep_arm.arm_disable()




    def teach_start(self):
        """开始示教模式"""
        self.teaching_status = 'Teaching'

        if self.teaching_mode:
            self.logger.warning("Teaching mode is already started")
            return False
        
        self.teach_ready()
       
            
        try:
            self.teaching_mode = True
            # 启动位置查询线程
            self.teach_thread_running = True
            self.teach_thread = threading.Thread(target=self._teaching_thread)
            self.teach_thread.daemon = True
            self.teach_thread.start()
            
            self.logger.info("Teaching mode started")
            self.logger.info("Move the arm to desired positions")
            self.logger.info("Press 's' to record a point")
            self.logger.info("Press 't' to stop teaching")
            return True
            
        except Exception as e:
            self.logger.error("Failed to start teaching mode: {}".format(e))
            return False

    def teach_stop(self):
        """停止示教模式"""
        if not self.teaching_mode:
            self.logger.warning("Teaching mode is not started")
            return False
            
        try:
            self.teaching_status = 'None'
            self.teaching_mode = False
            
            # 停止位置查询线程
            if self.teach_thread_running:
                self.teach_thread_running = False
                if self.teach_thread:
                    self.teach_thread.join(timeout=1.0)
            
            # 重新使能所有电机
            self.deep_arm.arm_enable()

            # 回到home位置
            self.deep_arm.arm_back_to_home()
            
            # 保存示教路径
            self.save_routine_to_config()
            self.logger.info("Teaching mode stopped")
            return True
            
        except Exception as e:
            self.logger.error("Failed to stop teaching mode: {}".format(e))
            return False
        

    def _teaching_thread(self):
        """示教模式下的位置查询线程"""
        while self.teach_thread_running:
            try:
                # 查询所有电机位置,查询之前需要发送指令才会返回
                self.deep_arm.arm_set_position(motor_ids=self.deep_arm.motor_ids, positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                time.sleep(self.interval_teach)
                if self.is_periodic_add_point:
                    self.teach_update()
            except Exception as e:
                self.logger.error("Error in teaching thread: {}".format(e))
                time.sleep(0.2)

    def teach_add_point(self):
        """添加当前位置到示教路径"""
        if not self.teaching_mode:
            self.logger.warning("Teaching mode is not started")
            return False
            
        try:
            status = self.deep_arm.arm_get_status()
            if status:
                self.routine_points.append(status['positions'])
                self.logger.info("Point added: {}".format(status['positions']))
                return True
            return False
            
        except Exception as e:
            self.logger.error("Failed to add point: {}".format(e))
            return False

    def teach_update(self):
        """更新示教状态（用于定时记录）"""
        if not self.teaching_mode:
            return
            
        current_time = time.time()
        if current_time - self.last_teach_time >= self.interval_add_point:
            if self.teach_add_point():
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
                if not os.path.exists(output_dir):
                    os.makedirs(output_dir)
                
                # 使用时间戳创建文件名
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                file_path = os.path.join(output_dir, 'teach_routine_' + timestamp + '.json')

            # 准备保存的数据
            routine_data = {
                'points': self.routine_points,
                'timestamp': time.strftime("%Y%m%d_%H%M%S"),
            }

            # 确保输出目录存在
            if not os.path.exists(os.path.dirname(file_path)):
                os.makedirs(os.path.dirname(file_path))
            
            # 保存数据
            with open(file_path, 'w') as f:
                json.dump(routine_data, f, indent=2)
            
            # 同时更新默认路径
            with open(default_path, 'w') as f:
                json.dump(routine_data, f, indent=2)
            
            self.logger.info("Teaching routine saved to: {}".format(file_path))
            self.logger.info("Default teaching routine file updated: {}".format(default_path))
            self.logger.info("Total points: {}".format(len(self.routine_points)))
            return True
            
        except Exception as e:
            self.logger.error("Failed to save routine: {}".format(e))
            return False

    def load_routine_from_config(self, file_path=None, fine_tune_freq=25):
        """从配置文件加载示教路径
        
        Args:
            file_path: 配置文件路径，如果为None则使用默认路径
            
        Returns:
            bool: 是否加载成功
        """
        try:
            if file_path is None:
                # 使用相对于当前文件的路径
                file_path = os.path.join(os.path.dirname(__file__), 'configs', 'teach_routine.json')
            
            if not os.path.exists(file_path):
                self.logger.info("Creating new routine file at {}".format(file_path))
                # 创建默认的示教路径文件
                default_routine = {
                    "points": [],
                    "timestamp": "",
                }
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                with open(file_path, 'w') as f:
                    json.dump(default_routine, f, indent=2)
            
            with open(file_path, 'r') as f:
                routine_data = json.load(f)
            
            self.routine_points = routine_data['points']
            if not self.routine_points:
                self.logger.warning("No routine points to execute")
                return False
            # 对电机轨迹进行五次样条插值
            self.routine_points_fine_tune = self.deep_arm.arm_set_position_quintic(self.routine_points, fine_tune_freq=fine_tune_freq)
            
            self.logger.info("Loaded routine from {}".format(file_path))
            self.logger.info("Total points: {}".format(len(self.routine_points)))
            return True
            
        except Exception as e:
            self.logger.error("Failed to load routine: {}".format(e))
            return False

    def execute_routine(self, excute_times=1, update_callback=None):
        """执行示教路径
        
        Args:
            excute_times: 执行次数
            update_callback: 更新回调函数，用于实时更新状态
            
        Returns:
            bool: 是否执行成功
        """
        try:
            # 先使能电机
            self.deep_arm.arm_enable()



            # 先加载路径
            if not self.load_routine_from_config(fine_tune_freq=self.fine_tune_freq):
                return False
            

            for i in range(excute_times):

                # 从当前值运行到轨迹的第一个点 TODO: 当前值好像不对
                self.logger.info('--------------------------------loop {}--------------------------------'.format(i))
                self.logger.info('----------------will move to first point of routine {} '.format(self.routine_points_fine_tune[0]))


                self.logger.info('----------------current position: {}'.format(self.deep_arm.cur_pos))
                # 循环3次发送查询指令，直到所有电机状态有更新，那就说明已经获取到了最新的电机位置状态
                self.deep_arm.arm_get_position()
                self.logger.info('----------------current position: {}'.format(self.deep_arm.cur_pos))

                self.deep_arm.arm_mov_to_position(position=self.routine_points_fine_tune[0], samples=50)
                time.sleep(1)
                self.logger.info('----------------already move to the first point of routine {}'.format(self.routine_points_fine_tune[0]))
                self.logger.info('----------------current position: {}'.format(self.deep_arm.cur_pos))

                # 开始运行整条轨迹
                self.deep_arm.arm_set_position_trajectory(trajectory=self.routine_points_fine_tune, delay=0.02)
                time.sleep(6)
                self.logger.info('----------------already move to the last point of routine {}'.format(self.routine_points_fine_tune[-1]))
                self.logger.info('----------------current position: {}'.format(self.deep_arm.cur_pos))


            # 回到home位置
            # self.deep_arm.arm_back_to_home()
            
            self.logger.info("Routine execution completed")
            return True
            
        except Exception as e:
            self.logger.error("Failed to execute routine: {}".format(e))
            return False

    def test(self):
        """测试示教功能
        
        Returns:
            bool: 测试是否成功
        """
        # 交互测试函数：
        # i键,init: self.deep_arm.arm_init()
        # e键,enable: self.deep_arm.arm_enable()  
        # d键,disable: self.deep_arm.arm_disable()
        # r键,ready: self.teach_ready()
        # s键,start: self.teach_start()
        # t键,stop: self.teach_stop()
        # e键,execute: self.execute_routine()

        try:
            # 测试开始示教
            self.logger.info("Starting teaching test......")
            if not self.teach_start():
                self.logger.error("Failed to start teaching mode")
                return False
            
            # 等待一段时间
            time.sleep(10)
            
            # 测试停止示教
            self.logger.info("Stopping teaching mode......")
            if not self.teach_stop():
                self.logger.error("Failed to stop teaching mode")
                return False
            
            # 测试执行路径
            self.logger.info("Executing routine......")
            if not self.execute_routine():
                self.logger.error("Failed to execute routine")
                return False
            
            self.logger.info("Teaching test completed successfully")
            return True
            
        except Exception as e:
            self.logger.error("Teaching test failed: {}".format(e))
            return False
        
    def interactive_test(self):
        """交互式测试函数"""
        self.logger.info("Starting interactive test...")
        self.logger.info("Available commands:")
        self.logger.info("  i - Initialize motors")
        self.logger.info("  e - Enable motors")
        self.logger.info("  d - Disable motors")
        self.logger.info("  a - Add the teaching point")
        self.logger.info("  s - Start teaching")
        self.logger.info("  t - Stop teaching")
        self.logger.info("  x - Execute routine")
        self.logger.info("  p - Display curve")
        self.logger.info("  h - Go home")
        self.logger.info("  q - Quit")
        
        try:
            while True:
                cmd = raw_input("\nEnter command (i/e/d/a/s/t/x/p/h/q): ").strip().lower()
                self.logger.info("Received command: {}".format(cmd))
                
                if cmd == 'q':
                    self.logger.info("Exiting interactive test")
                    break
                    
                elif cmd == 'i':
                    self.logger.info("Initializing motors...")
                    self.deep_arm.arm_init()
                    
                elif cmd == 'e':
                    self.logger.info("Enabling motors...")
                    self.deep_arm.arm_enable()
                    
                elif cmd == 'd':
                    self.logger.info("Disabling motors...")
                    self.deep_arm.arm_disable()
                    
                elif cmd == 's':
                    self.logger.info("Starting teaching mode...")
                    self.teach_start()

                elif cmd == 'a':
                    self.logger.info("Adding point...")
                    self.teach_add_point()
                    
                elif cmd == 't':
                    self.logger.info("Stopping teaching mode...")
                    self.teach_stop()
                    
                elif cmd == 'x':
                    self.logger.info("Executing routine...")
                    self.execute_routine(excute_times=1)
                    
                elif cmd == 'p':
                    self.logger.info("Displaying curve...")
                    # 如果路径为空，则先加载路径
                    if not self.routine_points_fine_tune:
                        self.load_routine_from_config()
                    self.deep_arm.plot_trajectory_positions(self.routine_points_fine_tune)
                    
                elif cmd == 'h':
                    self.logger.info("Going home...")
                    self.deep_arm.arm_back_to_home()
                    
                else:
                    self.logger.warning("Unknown command. Please try again.")
                    
        except KeyboardInterrupt:
            self.logger.info("Test interrupted by user")
        except Exception as e:
            self.logger.error("Error during interactive test: {}".format(e))
        finally:
            # 确保电机被禁用
            self.deep_arm.arm_disable()
            self.logger.info("Interactive test completed")


if __name__ == "__main__":
    deep_arm = DeepArm()
    deep_arm.serial_manager.connect(port='COM10')
    teaching = Teaching(deep_arm)
    teaching.interactive_test()
