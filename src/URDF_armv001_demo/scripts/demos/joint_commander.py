#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import moveit_commander
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import time

class JointCommander:
    """发送关节命令的演示 - 使用预定义位置"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('joint_commander')
        
        # 创建发布器
        self.joint_pub = rospy.Publisher('/moveit_user_interface/joint_target', 
                                        Float64MultiArray, queue_size=10)
        
        # 初始化MoveIt接口获取预定义位置
        self.moveit_init()
        
        # 等待发布器连接
        rospy.loginfo("等待连接到用户接口...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self.joint_pub.get_num_connections() == 0:
            if rospy.Time.now() > timeout:
                rospy.logwarn("超时等待连接，继续执行...")
                break
            rospy.sleep(0.1)
                
        rospy.loginfo("已连接到用户接口或超时继续")
    
    def moveit_init(self):
        """初始化MoveIt获取可用位置"""
        moveit_commander.roscpp_initialize(sys.argv)
        self.group_name = "DeepArm"
        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        
        # 获取已定义的位置
        self.named_targets = self.arm.get_named_targets()
        rospy.loginfo("可用的预定义位置: %s", self.named_targets)
        
        # 预设的关节位置序列
        self.predefined_positions = {}
        
        # 获取每个预定义位置的关节角度
        for target in self.named_targets:
            try:
                self.arm.set_named_target(target)
                planned_path = self.arm.plan()
                
                # 检查是否有规划路径
                if planned_path and len(planned_path.joint_trajectory.points) > 0:
                    # 获取最后一个路径点的位置
                    pos = planned_path.joint_trajectory.points[-1].positions
                    self.predefined_positions[target] = list(pos)
                    rospy.loginfo("已获取位置 '%s': %s", target, list(pos))
            except Exception as e:
                rospy.logwarn("获取位置 '%s' 失败: %s", target, e)
        
        # 添加自定义关节序列
        # 这些关节值来自于正常工作的成功姿态
        self.predefined_positions.update({
            "initial": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint1": [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            "joint2": [0.2, 0.3, 0.0, 0.0, 0.0, 0.0],
            "joint3": [0.2, 0.3, 0.1, 0.0, 0.0, 0.0],
            "joint4": [0.2, 0.3, 0.1, -0.2, 0.0, 0.0],
            "joint5": [0.2, 0.3, 0.1, -0.2, 0.15, 0.0],
            "all_joints": [0.2, 0.3, 0.1, -0.2, 0.15, 0.1],
            "reset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        })
    
    def send_joint_positions(self, positions):
        """发送关节位置命令"""
        rospy.loginfo("发送关节位置: %s", positions)
        
        # 创建正确格式的消息
        msg = Float64MultiArray()
        
        # 设置消息布局
        msg.layout.dim.append(MultiArrayDimension(
            label="joint_positions",
            size=len(positions),
            stride=len(positions)
        ))
        msg.layout.data_offset = 0
        
        # 设置数据
        msg.data = positions
        
        # 发布消息
        self.joint_pub.publish(msg)
    
    def run_demo(self):
        """运行演示 - 使用预定义位置"""
        try:
            # 等待1秒确保节点已就绪
            rospy.sleep(1)
            
            # 发送预定义的关节位置序列
            positions_to_use = []
            
            # 添加home位置（如果可用）
            if "home" in self.predefined_positions:
                positions_to_use.append(("home", self.predefined_positions["home"]))
            
            # 添加其他预定义位置
            for name, pos in self.predefined_positions.items():
                if name != "home":  # 避免重复
                    positions_to_use.append((name, pos))
            
            # 确保有可用位置
            if not positions_to_use:
                rospy.logerr("没有可用的预定义位置")
                return False
            
            # 发送每个位置
            for name, pos in positions_to_use:
                rospy.loginfo("移动到位置: %s", name)
                self.send_joint_positions(pos)
                rospy.sleep(3)  # 等待执行完成
            
            # 最后回到home位置
            if "home" in self.predefined_positions:
                rospy.loginfo("返回home位置")
                self.send_joint_positions(self.predefined_positions["home"])
                rospy.sleep(3)
                
            rospy.loginfo("演示完成")
            return True
            
        except KeyboardInterrupt:
            rospy.loginfo("用户中断演示")
            return False
            
        except Exception as e:
            rospy.logerr("演示出错: %s", e)
            return False
        
        finally:
            # 清理MoveIt资源
            moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        commander = JointCommander()
        commander.run_demo()
    except rospy.ROSInterruptException:
        pass 