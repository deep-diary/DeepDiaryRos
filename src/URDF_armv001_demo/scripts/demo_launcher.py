#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import subprocess
import signal
import argparse

class DemoLauncher:
    """
    演示启动器
    提供命令行接口以便于启动各种演示
    """
    
    def __init__(self):
        # 可用演示列表
        self.demos = {
            "fk": {
                "desc": "正向运动学演示 (设置关节位置)",
                "script": "fk_demo.py"
            },
            "ik": {
                "desc": "逆向运动学演示 (设置末端位姿)",
                "script": "ik_demo.py"
            },
            "cartesian": {
                "desc": "笛卡尔路径演示 (末端执行直线运动)",
                "script": "cartesian_demo.py"
            },
            "object": {
                "desc": "附着物体演示 (模拟物体抓取)",
                "script": "attached_object_demo.py"
            },
            "tracking": {
                "desc": "目标跟踪演示 (使用交互式标记)",
                "script": "object_tracking_demo.py"
            },
            "presentation": {
                "desc": "演示轨迹演示 (循环预设位置序列)",
                "script": "presentation_demo.py"
            },
            "ui": {
                "desc": "用户接口 (启动消息控制接口)",
                "script": "moveit_user_interface.py"
            },
            "cartesian_old": {
                "desc": "原笛卡尔路径演示 (使用旧版本实现)",
                "script": "moveit_cartesian_demo.py"
            },
            "joint_cmd": {
                "desc": "关节位置控制演示 (发送关节角度命令)",
                "script": "joint_commander.py"
            },
            "pose_cmd": {
                "desc": "末端位姿控制演示 (发送位置和姿态命令)",
                "script": "pose_commander.py"
            }
        }
        
        # 解析命令行参数
        parser = argparse.ArgumentParser(description='启动MoveIt演示')
        parser.add_argument('demo', nargs='?', 
                           help='要启动的演示名称，或者使用"list"列出所有可用演示')
        
        args = parser.parse_args(rospy.myargv()[1:])
        
        if not args.demo or args.demo == 'list':
            self.list_demos()
        else:
            self.run_demo(args.demo)
    
    def list_demos(self):
        """列出所有可用的演示"""
        print("\n可用的演示:")
        print("==============")
        
        for key, demo in sorted(self.demos.items()):
            print("  %s: %s" % (key, demo["desc"]))
        
        print("\n使用方式: rosrun URDF_armv001_demo demo_launcher.py <demo_name>")
        print("例如: rosrun URDF_armv001_demo demo_launcher.py fk\n")
    
    def run_demo(self, demo_name):
        """运行指定的演示"""
        if demo_name not in self.demos:
            print("错误: 未知的演示名称: %s" % demo_name)
            self.list_demos()
            return
        
        demo = self.demos[demo_name]
        
        print("\n启动演示: %s (%s)" % (demo_name, demo["desc"]))
        print("==============")
        print("按 Ctrl+C 终止演示\n")
        
        try:
            # 直接使用rosrun命令启动脚本
            process = subprocess.Popen(["rosrun", "URDF_armv001_demo", demo["script"]])
            
            # 等待程序结束或接收到中断信号
            process.wait()
            
        except KeyboardInterrupt:
            # 用户按下Ctrl+C，终止进程
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait()
            print("\n演示已终止")
        
        except Exception as e:
            print("启动演示时出错: %s" % e)

if __name__ == '__main__':
    DemoLauncher() 