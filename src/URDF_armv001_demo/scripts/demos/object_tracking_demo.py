#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
import threading
import copy
import math
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

class ObjectTrackingDemo:
    """
    对象跟踪演示
    创建一个可交互的标记，用户可以移动它，机械臂会跟踪它
    """
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('object_tracking_demo', anonymous=True)
        
        # 创建发布者
        self.target_pub = rospy.Publisher('/moveit_user_interface/tracking_target', 
                                         PoseStamped, queue_size=10)
        self.control_pub = rospy.Publisher('/moveit_user_interface/tracking_control', 
                                          String, queue_size=10)
        
        # 创建TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 创建交互式标记服务器
        self.server = InteractiveMarkerServer("tracking_object")
        
        # 创建菜单处理器
        self.menu_handler = MenuHandler()
        self.tracking_enabled = False
        
        # 添加菜单项
        self.menu_item_toggle = self.menu_handler.insert("开始/停止跟踪", callback=self.process_menu_feedback)
        
        # 创建交互式标记
        self.make_6dof_marker()
        
        # 应用菜单
        self.server.applyChanges()
        
        # 启动发布线程
        self.stop_thread = False
        self.pub_thread = threading.Thread(target=self.publish_thread)
        self.pub_thread.daemon = True
        self.pub_thread.start()
        
        rospy.loginfo("对象跟踪演示已启动，请在RViz中移动标记进行测试")
        rospy.loginfo("使用右键菜单开始或停止跟踪")
    
    def make_6dof_marker(self):
        """创建6自由度交互式标记"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position.x = 0.3
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.3
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = 0.2  # 增大标记尺寸，便于查看和交互
        int_marker.name = "tracking_target"
        int_marker.description = "用于跟踪的目标对象"
        
        # 创建标记形状
        self.create_marker_visual(int_marker)
        
        # 添加交互控制
        self.add_6dof_controls(int_marker)
        
        # 发布交互式标记
        self.server.insert(int_marker, self.process_marker_feedback)
        self.menu_handler.apply(self.server, int_marker.name)
    
    def create_marker_visual(self, int_marker):
        """创建标记的可视化形状"""
        # 创建一个控制器来保存可视化形状
        control = InteractiveMarkerControl()
        control.always_visible = True
        
        # 创建一个球体作为可视化标记
        visual_marker = Marker()
        visual_marker.type = Marker.SPHERE
        visual_marker.scale.x = 0.1  # 增大球体尺寸
        visual_marker.scale.y = 0.1
        visual_marker.scale.z = 0.1
        visual_marker.color.r = 0.5
        visual_marker.color.g = 0.5
        visual_marker.color.b = 1.0
        visual_marker.color.a = 0.8
        
        # 添加可视化标记到控制器
        control.markers.append(visual_marker)
        
        # 添加控制器到交互式标记
        int_marker.controls.append(control)
    
    def add_6dof_controls(self, int_marker):
        """添加6自由度控制器（平移和旋转）"""
        # 添加X轴控制
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # 添加Y轴控制
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # 添加Z轴控制
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
    
    def process_marker_feedback(self, feedback):
        """处理标记移动反馈"""
        try:
            # 只打印鼠标点击消息，避免控制台被移动消息刷屏
            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                rospy.loginfo("标记移动到位置: %.2f, %.2f, %.2f", 
                             feedback.pose.position.x,
                             feedback.pose.position.y,
                             feedback.pose.position.z)
        except Exception as e:
            rospy.logerr("处理标记反馈出错: %s", e)
    
    def process_menu_feedback(self, feedback):
        """处理菜单交互反馈"""
        try:
            # 切换跟踪状态
            self.tracking_enabled = not self.tracking_enabled
            
            if self.tracking_enabled:
                rospy.loginfo("开始跟踪目标")
                self.control_pub.publish(String("start"))
            else:
                rospy.loginfo("停止跟踪目标")
                self.control_pub.publish(String("stop"))
                
            # 更新菜单文本
            # 这里不应该直接使用feedback.menu_entry_id来修改文本，因为MenuHandler中没有该功能
            # 而是在下次右键点击时根据当前状态显示适当的选项
        except Exception as e:
            rospy.logerr("处理菜单反馈出错: %s", e)
    
    def publish_thread(self):
        """发布目标位置的线程"""
        rate = rospy.Rate(20)  # 20Hz
        
        try:
            while not rospy.is_shutdown() and not self.stop_thread:
                if self.tracking_enabled:
                    try:
                        # 获取标记当前位姿
                        marker = self.server.get("tracking_target")
                        if marker is not None:
                            pose = marker.pose
                            
                            # 创建PoseStamped消息
                            target_pose = PoseStamped()
                            target_pose.header.frame_id = "base_link"
                            target_pose.header.stamp = rospy.Time.now()
                            
                            # 设置位置和姿态
                            target_pose.pose.position.x = pose.position.x
                            target_pose.pose.position.y = pose.position.y
                            target_pose.pose.position.z = pose.position.z
                            target_pose.pose.orientation = pose.orientation
                            
                            # 发布位姿
                            self.target_pub.publish(target_pose)
                    except Exception as e:
                        rospy.logerr("发布目标位置出错: %s", e)
                        
                # 降低频率以减轻系统负担
                rate.sleep()
                
        except Exception as e:
            rospy.logerr("发布线程异常: %s", e)
    
    def run_demo(self):
        """运行演示"""
        try:
            rospy.loginfo("目标跟踪演示已启动")
            rospy.loginfo("请在RViz中使用交互式标记控制机械臂")
            rospy.loginfo("使用右键菜单开始或停止跟踪")
            
            # 循环直到节点关闭
            rospy.spin()
            
        except KeyboardInterrupt:
            rospy.loginfo("用户中断演示")
            
        finally:
            # 清理资源
            self.stop_thread = True
            if self.pub_thread.is_alive():
                self.pub_thread.join(1.0)
            if self.server:
                self.server.clear()
                self.server = None

if __name__ == '__main__':
    try:
        demo = ObjectTrackingDemo()
        demo.run_demo()
    except rospy.ROSInterruptException:
        pass 