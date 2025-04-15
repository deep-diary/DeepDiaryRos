#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
import argparse
from std_msgs.msg import ByteMultiArray, MultiArrayDimension

def generate_test_packet(joint_values, packet_type=1):
    """生成用于测试的数据包"""
    # 协议常量
    START_MARKER = 0xFF
    END_MARKER = 0xFE
    
    # 关节数量
    joint_count = len(joint_values)
    
    # 构建数据包
    data = bytearray()
    data.append(START_MARKER)  # 起始标记
    data.append(packet_type)   # 命令类型
    data.append(joint_count)   # 关节数量
    
    # 添加每个关节的位置、速度和加速度
    for value in joint_values:
        # 位置
        pos_bytes = struct.pack('<f', value)
        data.extend(pos_bytes)
        # 速度 (0.0)
        vel_bytes = struct.pack('<f', 0.0)
        data.extend(vel_bytes)
        # 加速度 (0.0)
        acc_bytes = struct.pack('<f', 0.0)
        data.extend(acc_bytes)
    
    # 计算校验和 (所有前面字节的和的低8位)
    checksum = sum(data) & 0xFF
    data.append(checksum)
    
    # 添加结束标记
    data.append(END_MARKER)
    
    return list(data)

def main():
    parser = argparse.ArgumentParser(description='发送测试串口数据包')
    parser.add_argument('--values', type=float, nargs='+', default=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                        help='关节角度值列表')
    parser.add_argument('--rate', type=float, default=1.0,
                        help='发送频率(Hz)')
    parser.add_argument('--count', type=int, default=10,
                        help='发送次数 (0表示无限发送)')
                        
    args = parser.parse_args()
    
    rospy.init_node('test_serial_feedback')
    pub = rospy.Publisher('/serial_node/data_received', ByteMultiArray, queue_size=10)
    
    rate = rospy.Rate(args.rate)
    counter = 0
    
    while not rospy.is_shutdown():
        # 如果设置了次数限制并达到限制，退出
        if args.count > 0 and counter >= args.count:
            break
            
        # 生成测试数据
        values = args.values
        if counter > 0:
            # 每次稍微增加一点角度
            values = [v + counter * 0.05 for v in values]
            
        data = generate_test_packet(values)
        
        # 创建消息
        msg = ByteMultiArray()
        msg.layout.dim.append(MultiArrayDimension(
            label="length",
            size=len(data),
            stride=1
        ))
        msg.data = data
        
        # 发布消息
        pub.publish(msg)
        rospy.loginfo(f"发送测试数据包 #{counter+1}: 关节角度 = {values}")
        
        counter += 1
        rate.sleep()
        
    rospy.loginfo("测试完成")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 