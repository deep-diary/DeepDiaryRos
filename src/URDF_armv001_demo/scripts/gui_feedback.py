#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
import os
import yaml
import rospkg
import xml.etree.ElementTree as ET
import Tkinter as tk
import ttk
from threading import Thread
import re
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
from control_msgs.msg import FollowJointTrajectoryActionGoal

class JointSlider:
    """关节滑动条控件"""
    def __init__(self, master, name, min_val, max_val, callback):
        self.frame = tk.Frame(master)
        self.frame.pack(fill="x", padx=5, pady=5)
        
        self.name = name
        self.label = tk.Label(self.frame, text=name, width=15, anchor="w")
        self.label.pack(side="left")
        
        self.position_var = tk.DoubleVar()
        self.position_var.trace("w", lambda *args: callback(name, self.position_var.get()))
        
        self.slider = ttk.Scale(self.frame, from_=min_val, to=max_val, 
                              orient="horizontal", length=200,
                              variable=self.position_var)
        self.slider.pack(side="left", fill="x", expand=True, padx=5)
        
        self.value_label = tk.Label(self.frame, text="0.00", width=10)
        self.value_label.pack(side="left")
        
        # 设置初始值
        self.set_value(0.0)
    
    def set_value(self, value):
        """设置滑动条的值并更新显示"""
        self.position_var.set(value)
        self.value_label.config(text="{:.4f}".format(value))
    
    def get_value(self):
        """获取当前滑动条的值"""
        return self.position_var.get()

class GuiFeedbackPublisher:
    """
    带GUI界面的模拟关节状态发布节点
    """
    def __init__(self):
        rospy.init_node('gui_feedback_publisher')
        
        # GUI组件初始化标志
        self.gui_initialized = False
        self.status_label = None
        
        # 获取URDF文件路径
        self.robot_description = self.get_robot_description()
        
        # 获取关节名称和限位
        self.joint_limits = self.parse_urdf_limits()
        
        # 初始化标准关节顺序(1-6)
        self.standard_joint_names = ['first_joint', 'second_joint', 'third_joint', 
                                   'fourth_joint', 'fifth_joint', 'sixth_joint']
        
        # 初始化实际关节顺序（将从轨迹消息中获取并更新）
        self.trajectory_joint_names = None
        
        # 初始化关节状态 - 使用标准顺序进行GUI显示
        self.joint_names = self.standard_joint_names[:]
        self.current_positions = [0.0] * len(self.joint_names)
        
        # 创建发布器
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.serial_pub = rospy.Publisher('/serial_node/data_received', UInt8MultiArray, queue_size=10)
        
        # 订阅发送到串口的命令
        self.serial_sub = rospy.Subscriber('/serial_node/data_to_send', UInt8MultiArray, 
                                         self.command_callback)
        
        # 订阅轨迹目标
        self.trajectory_sub = rospy.Subscriber('/URDF_armv001/arm_joint_controller/follow_joint_trajectory/goal', 
                                             FollowJointTrajectoryActionGoal,
                                             self.trajectory_callback)
        
        # 启动GUI线程
        self.gui_thread = Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()
        
        # 发布初始关节状态
        self.publish_joint_states()
        
        # 定时器 - 确保即使GUI不动，也保持发布
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        
        rospy.loginfo("GUI反馈发布器已初始化")
        rospy.loginfo("关节限位信息: %s", self.joint_limits)
    
    def get_robot_description(self):
        """获取机器人URDF"""
        # 首先尝试从ROS参数服务器获取
        if rospy.has_param('/robot_description'):
            return rospy.get_param('/robot_description')
        
        # 如果参数服务器中没有，则尝试直接读取文件
        rospack = rospkg.RosPack()
        try:
            urdf_path = os.path.join(rospack.get_path('URDF_armv001'), 'urdf', 'URDF_armv001.urdf')
            with open(urdf_path, 'r') as f:
                return f.read()
        except Exception as e:
            rospy.logwarn("无法读取URDF文件: %s", e)
            return None
    
    def parse_urdf_limits(self):
        """从URDF中解析关节限位"""
        limits = {}
        
        if not self.robot_description:
            rospy.logwarn("没有可用的机器人描述，使用默认限位")
            # 使用默认限位
            for joint in ['first_joint', 'second_joint', 'third_joint', 
                         'fourth_joint', 'fifth_joint', 'sixth_joint']:
                limits[joint] = {'lower': -3.14, 'upper': 3.14}
            return limits
        
        try:
            # 解析XML
            root = ET.fromstring(self.robot_description)
            
            # 查找所有关节
            for joint in root.findall(".//joint"):
                name = joint.get('name')
                joint_type = joint.get('type')
                
                # 只处理revolute类型的关节
                if joint_type == 'revolute':
                    limit_elem = joint.find('limit')
                    if limit_elem is not None:
                        lower = float(limit_elem.get('lower', -3.14))
                        upper = float(limit_elem.get('upper', 3.14))
                        limits[name] = {'lower': lower, 'upper': upper}
        
        except Exception as e:
            rospy.logwarn("解析URDF限位时出错: %s", e)
            # 出错时使用默认限位
            for joint in ['first_joint', 'second_joint', 'third_joint', 
                         'fourth_joint', 'fifth_joint', 'sixth_joint']:
                limits[joint] = {'lower': -3.14, 'upper': 3.14}
        
        return limits
    
    def create_gui(self):
        """创建GUI界面"""
        self.root = tk.Tk()
        self.root.title("机械臂关节控制")
        self.root.geometry("600x650")  # 增加高度以容纳文本输入区域
        
        # 顶部框架 - 标题和控制按钮
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill="x", padx=10, pady=5)
        
        title_label = tk.Label(top_frame, text="机械臂关节控制", font=("Arial", 14, "bold"))
        title_label.pack(side="left", pady=10)
        
        # 创建重置按钮
        reset_btn = tk.Button(top_frame, text="重置位置", command=self.reset_positions)
        reset_btn.pack(side="right", padx=5, pady=10)
        
        # 创建滑动条
        sliders_frame = tk.Frame(self.root)
        sliders_frame.pack(fill="x", expand=False, padx=10, pady=5)
        
        # 为每个关节创建滑动条
        self.sliders = {}
        for i, name in enumerate(self.joint_names):
            limits = self.joint_limits.get(name, {'lower': -3.14, 'upper': 3.14})
            slider = JointSlider(sliders_frame, name, 
                               limits['lower'], limits['upper'], 
                               self.on_slider_changed)
            self.sliders[name] = slider
        
        # 创建文本输入区域
        text_frame = tk.LabelFrame(self.root, text="关节位置数组输入", padx=10, pady=10)
        text_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.position_text = tk.Text(text_frame, height=7, width=50)
        self.position_text.pack(side="left", fill="both", expand=True)
        
        text_scrollbar = tk.Scrollbar(text_frame)
        text_scrollbar.pack(side="right", fill="y")
        
        self.position_text.config(yscrollcommand=text_scrollbar.set)
        text_scrollbar.config(command=self.position_text.yview)
        
        # 添加示例说明
        example_label = tk.Label(text_frame, 
                               text="示例: [0, 0.5, 0, 0, 0, 0] - 输入数组后点击应用",
                               fg="gray")
        example_label.pack(side="top", anchor="w", pady=(0, 5))
        
        # 应用按钮
        apply_frame = tk.Frame(self.root)
        apply_frame.pack(fill="x", padx=10, pady=5)
        
        apply_btn = tk.Button(apply_frame, text="应用位置数组", 
                            command=self.apply_position_array, 
                            bg="#4CAF50", fg="white", height=2)
        apply_btn.pack(fill="x", pady=5)
        
        # 监控区域
        monitor_frame = tk.LabelFrame(self.root, text="数据监控", padx=10, pady=10)
        monitor_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.monitor_text = tk.Text(monitor_frame, height=5, width=50)
        self.monitor_text.pack(side="left", fill="both", expand=True)
        self.monitor_text.config(state=tk.DISABLED)  # 只读
        
        monitor_scrollbar = tk.Scrollbar(monitor_frame)
        monitor_scrollbar.pack(side="right", fill="y")
        
        self.monitor_text.config(yscrollcommand=monitor_scrollbar.set)
        monitor_scrollbar.config(command=self.monitor_text.yview)
        
        # 状态框架
        status_frame = tk.Frame(self.root)
        status_frame.pack(fill="x", padx=10, pady=5)
        
        self.status_label = tk.Label(status_frame, text="状态: 已连接", fg="green")
        self.status_label.pack(pady=5)
        
        # 标记GUI已初始化
        self.gui_initialized = True
        
        # 启动GUI主循环
        self.root.mainloop()
    
    def update_monitor(self, text, color="black"):
        """更新监控文本区域"""
        if not hasattr(self, 'monitor_text') or not self.monitor_text:
            return
            
        self.monitor_text.config(state=tk.NORMAL)
        self.monitor_text.insert(tk.END, text + "\n", color)
        self.monitor_text.see(tk.END)  # 滚动到底部
        self.monitor_text.config(state=tk.DISABLED)
    
    def apply_position_array(self):
        """
        解析并应用文本框中的位置数组
        """
        try:
            # 获取文本内容
            text = self.position_text.get("1.0", tk.END).strip()
            
            # 使用正则表达式匹配数组格式
            match = re.search(r'\[(.*?)\]', text)
            if match:
                # 提取数字
                nums_str = match.group(1)
                positions = [float(x.strip()) for x in nums_str.split(',') if x.strip()]
                
                # 检查是否有足够的位置值
                if len(positions) != len(self.joint_names):
                    if self.trajectory_joint_names and len(positions) == len(self.trajectory_joint_names):
                        # 如果与轨迹关节数量匹配，尝试映射
                        positions = self.map_trajectory_to_standard(
                            self.trajectory_joint_names, positions)
                        self.update_monitor("已映射输入位置到标准顺序", "blue")
                    else:
                        # 提示错误
                        self.update_status("位置数量不匹配: 需要%d个值" % len(self.joint_names), "red")
                        self.update_monitor("位置数量错误: %d, 需要: %d" % 
                                         (len(positions), len(self.joint_names)), "red")
                        return
                
                # 应用位置
                self.update_joint_positions(positions)
                self.update_status("已应用位置数组", "green")
                self.update_monitor("已应用位置: %s" % str(positions))
            else:
                self.update_status("无效的位置格式", "red")
                self.update_monitor("无效的位置格式: 需要[n1, n2, ...]格式", "red")
                
        except Exception as e:
            self.update_status("解析位置出错: " + str(e), "red")
            self.update_monitor("解析错误: " + str(e), "red")
    
    def update_joint_positions(self, positions):
        """更新关节位置和滑动条"""
        for i, pos in enumerate(positions):
            if i < len(self.current_positions):
                # 更新内部位置记录
                self.current_positions[i] = pos
                
                # 更新对应的滑动条
                if i < len(self.joint_names) and self.joint_names[i] in self.sliders:
                    self.sliders[self.joint_names[i]].set_value(pos)
        
        # 发布更新的关节状态
        self.publish_joint_states()
    
    def update_status(self, message, color="black"):
        """更新状态标签"""
        if self.gui_initialized and hasattr(self, 'status_label') and self.status_label:
            self.status_label.config(text="状态: " + message, fg=color)
    
    def on_slider_changed(self, name, value):
        """滑动条值变化的回调"""
        try:
            # 更新当前位置
            index = self.joint_names.index(name)
            self.current_positions[index] = value
            
            # 发布更新的关节状态
            self.publish_joint_states()
            
            # 更新状态标签
            if self.gui_initialized:
                self.update_status("已更新 " + name + " = {:.4f}".format(value), "blue")
        except Exception as e:
            rospy.logwarn("滑动条回调错误: %s", e)
            if self.gui_initialized:
                self.update_status("错误: " + str(e), "red")
    
    def reset_positions(self):
        """重置所有关节位置到0"""
        for name, slider in self.sliders.items():
            slider.set_value(0.0)
        
        for i in range(len(self.current_positions)):
            self.current_positions[i] = 0.0
        
        self.publish_joint_states()
        
        if self.gui_initialized:
            self.update_status("已重置所有关节位置", "green")
            self.update_monitor("已重置所有关节位置到0", "green")
    
    def timer_callback(self, event):
        """定时发布关节状态"""
        self.publish_joint_states()
    
    def trajectory_callback(self, msg):
        """处理轨迹目标消息"""
        try:
            # 更新轨迹关节顺序
            if msg.goal.trajectory.joint_names:
                self.trajectory_joint_names = msg.goal.trajectory.joint_names
                rospy.loginfo("更新轨迹关节顺序: %s", self.trajectory_joint_names)
                
                # 添加到监控区域
                if self.gui_initialized:
                    self.root.after(0, lambda: self.update_monitor(
                        "轨迹关节顺序: %s" % self.trajectory_joint_names, "blue"))
            
            # 提取第一个轨迹点的位置
            if len(msg.goal.trajectory.points) > 0:
                traj_positions = msg.goal.trajectory.points[0].positions
                
                # 将轨迹位置映射到标准顺序
                standard_positions = self.map_trajectory_to_standard(
                    self.trajectory_joint_names, list(traj_positions))
                
                rospy.loginfo("收到轨迹目标位置: %s", traj_positions)
                rospy.loginfo("映射到标准顺序: %s", standard_positions)
                
                if self.gui_initialized:
                    # 更新文本区域显示原始目标位置
                    position_str = str(list(traj_positions))
                    self.root.after(0, lambda: self.position_text.delete("1.0", tk.END))
                    self.root.after(0, lambda: self.position_text.insert("1.0", position_str))
                    
                    # 更新监控区域
                    self.root.after(0, lambda: self.update_monitor(
                        "收到轨迹目标: %s" % position_str))
                    self.root.after(0, lambda: self.update_monitor(
                        "标准顺序映射: %s" % str(standard_positions), "green"))
                    
                    # 自动应用位置（使用映射后的标准顺序）
                    self.root.after(0, lambda: self.update_joint_positions(standard_positions))
        except Exception as e:
            rospy.logwarn("处理轨迹目标时出错: %s", e)
            if self.gui_initialized:
                self.update_status("处理轨迹目标时出错: " + str(e), "red")
    
    def map_trajectory_to_standard(self, traj_joint_names, traj_positions):
        """
        将轨迹中的关节位置映射到标准顺序
        """
        if not traj_joint_names or len(traj_joint_names) != len(traj_positions):
            rospy.logwarn("无法映射关节位置: 名称或位置不匹配")
            return traj_positions
            
        # 创建与标准顺序对应的位置数组
        standard_positions = [0.0] * len(self.standard_joint_names)
        
        # 映射每个位置
        for i, joint_name in enumerate(traj_joint_names):
            if joint_name in self.standard_joint_names:
                standard_idx = self.standard_joint_names.index(joint_name)
                standard_positions[standard_idx] = traj_positions[i]
            else:
                rospy.logwarn("未知关节名: %s", joint_name)
                
        return standard_positions
    
    def publish_joint_states(self):
        """发布关节状态"""
        try:
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = self.current_positions
            msg.velocity = [0.0] * len(self.joint_names)
            msg.effort = [0.0] * len(self.joint_names)
            
            self.joint_state_pub.publish(msg)
            
            # 同时发布串口反馈
            self.publish_serial_feedback()
        except Exception as e:
            rospy.logwarn("发布关节状态出错: %s", e)
    
    def publish_serial_feedback(self):
        """发布模拟的串口反馈数据"""
        try:
            # 构建反馈数据包
            data = bytearray()
            data.append(0xFF)  # 起始标记
            data.append(0x02)  # 命令类型 - 状态反馈
            data.append(len(self.joint_names))  # 关节数量
            
            # 添加每个关节的位置、速度和加速度
            for i in range(len(self.joint_names)):
                # 位置 - Python 2.7中转换字节为整数序列
                pos_bytes = struct.pack('<f', self.current_positions[i])
                pos_bytes_array = [ord(b) for b in pos_bytes]
                data.extend(pos_bytes_array)
                
                # 速度 (0.0)
                vel_bytes = struct.pack('<f', 0.0)
                vel_bytes_array = [ord(b) for b in vel_bytes]
                data.extend(vel_bytes_array)
                
                # 加速度 (0.0)
                acc_bytes = struct.pack('<f', 0.0)
                acc_bytes_array = [ord(b) for b in acc_bytes]
                data.extend(acc_bytes_array)
            
            # 计算校验和
            checksum = sum(data) & 0xFF
            data.append(checksum)
            
            # 添加结束标记
            data.append(0xFE)
            
            # 创建并发布消息
            msg = UInt8MultiArray()
            msg.layout.dim.append(MultiArrayDimension(
                label="length",
                size=len(data),
                stride=1
            ))
            msg.data = list(data)
            self.serial_pub.publish(msg)
        except Exception as e:
            rospy.logwarn("创建反馈数据出错: %s", e)
    
    def command_callback(self, msg):
        """
        解析发送给机械臂的命令，并更新GUI滑动条
        """
        try:
            data = bytearray(msg.data)
            
            # 检查数据格式是否合法
            if len(data) < 4 or data[0] != 0xFF or data[-1] != 0xFE:
                return
                
            # 获取关节数量和命令类型
            cmd_type = data[1]
            joint_count = data[2]
            
            # 只处理位置命令
            if cmd_type != 0x01:
                return
            
            # 解析关节位置 - 假设命令中的顺序与trajectory_joint_names一致
            parsed_positions = []
            
            for i in range(joint_count):
                # 计算每个关节的偏移: 头部3字节 + 每个关节8字节(位置4字节+速度4字节)
                base_idx = 3 + i * 8
                
                # 确保索引有效
                if base_idx + 4 <= len(data) - 2:
                    try:
                        # Python 2.7中处理二进制数据
                        pos_bytes_str = ''.join([chr(b) for b in data[base_idx:base_idx+4]])
                        pos = struct.unpack('<f', pos_bytes_str)[0]
                        parsed_positions.append(pos)
                    except Exception as e:
                        rospy.logwarn("解析关节 %d 位置时出错: %s", i, e)
            
            # 如果解析成功且有轨迹关节顺序信息
            if parsed_positions and self.trajectory_joint_names:
                # 1. 检查解析的位置数量与轨迹关节数量是否匹配
                if len(parsed_positions) == len(self.trajectory_joint_names):
                    # 2. 将命令位置映射到标准顺序
                    standard_positions = self.map_trajectory_to_standard(
                        self.trajectory_joint_names, parsed_positions)
                    
                    rospy.loginfo("串口命令位置: %s", parsed_positions)
                    rospy.loginfo("映射到标准顺序: %s", standard_positions)
                    
                    # 3. 更新GUI滑动条和当前位置
                    self.current_positions = standard_positions
                    
                    if self.gui_initialized:
                        for i, joint_name in enumerate(self.standard_joint_names):
                            if joint_name in self.sliders:
                                self.root.after(0, lambda n=joint_name, p=standard_positions[i]: 
                                             self.sliders[n].set_value(p))
                        
                        # 更新监控信息
                        self.root.after(0, lambda: self.update_monitor(
                            "串口命令(原顺序): %s" % str(parsed_positions)))
                        self.root.after(0, lambda: self.update_monitor(
                            "映射后标准顺序: %s" % str(standard_positions), "green"))
                        self.root.after(0, lambda: self.update_status(
                            "已接收并映射外部命令", "green"))
                    
                    # 4. 发布更新的关节状态
                    self.publish_joint_states()
                else:
                    rospy.logwarn("串口命令位置数量 (%d) 与轨迹关节数量 (%d) 不匹配", 
                                 len(parsed_positions), len(self.trajectory_joint_names))
            else:
                # 如果没有轨迹关节顺序信息，直接按标准顺序更新
                if parsed_positions and len(parsed_positions) == len(self.standard_joint_names):
                    self.current_positions = parsed_positions
                    
                    if self.gui_initialized:
                        for i, joint_name in enumerate(self.standard_joint_names):
                            if joint_name in self.sliders:
                                self.root.after(0, lambda n=joint_name, p=parsed_positions[i]: 
                                             self.sliders[n].set_value(p))
                        
                        # 更新监控信息
                        self.root.after(0, lambda: self.update_monitor(
                            "串口命令位置(使用标准顺序): %s" % str(parsed_positions)))
                        self.root.after(0, lambda: self.update_status(
                            "已接收外部命令", "green"))
                    
                    # 发布更新的关节状态
                    self.publish_joint_states()
                
        except Exception as e:
            rospy.logwarn("解析命令数据出错: %s", e)
            import traceback
            rospy.logwarn(traceback.format_exc())

if __name__ == '__main__':
    try:
        publisher = GuiFeedbackPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 