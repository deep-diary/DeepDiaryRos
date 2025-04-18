#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Deep Arm 机械臂协议演示程序
展示如何使用 deep_arm 协议包控制机械臂
"""

import os
import sys
import time
import logging
import argparse
import threading
from threading import Event

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
# Import robotic arm protocol and motor
from serial_comm import SerialManager
from protocol import DeepArmProtocol, DeepArmCommand, DeepArmResponse
from motor import Motor

class DeepArm:
    """Deep Arm 机械臂演示类"""
    
    def __init__(self, config_file=None):
        """
        初始化演示程序
        
        Args:
            config_file: 配置文件路径，None则使用默认配置文件
        """
        # 初始化日志
        logging.basicConfig(level=logging.INFO, 
                          format='%(asctime)s [%(levelname)s] %(name)s: %(message)s')
        self.logger = logging.getLogger('DeepArmDemo')
        
        # 初始化协议
        self.protocol = DeepArmProtocol(config_file)
        
        # 创建串口管理器
        self.serial_manager = SerialManager(callback=self.data_callback)
        
        # 响应事件
        self.response_received = Event()
        self.last_response = None
        
        self.motor_ids = [1,2,3,4,5,6]
        # 电机对象
        self.arm_create_motors(self.motor_ids)

        
        # 活动电机ID - 所有命令默认发送到这个ID
        self.active_motor_id = 1
        
        self.logger.info("Deep Arm demo program initialized")
    
    def data_callback(self, data, source):
        """
        数据接收回调函数 - 处理收到的数据并更新电机状态
        
        Args:
            data: 接收到的数据
            source: 事件源
        """
        if source == 'data' and isinstance(data, bytearray):
            # Print the received data in hexadecimal format
            # hex_data = ' '.join([format(b, '02X') for b in data])
            # self.logger.info("Received data: %s" % hex_data)
            
            # Attempt to parse the formatted data
            try:
                # Format the frame
                parsed = self.protocol.format_frame(data)
                # print('parsed:', parsed)    
                # Get the motor ID
                motor_id = parsed['motor_id']
                
                # If the motor object does not exist, create it automatically
                if motor_id not in self.motors:
                    self.logger.info("Automatically creating motor %s object" % motor_id)
                    self.motors[motor_id] = Motor(motor_id)
                
                # Parse the payload
                self.motors[motor_id].update_from_feedback(parsed['payload'])
                try:
                    # Get the current motor status
                    motor_state = self.motors[motor_id].get_status()
                    
                    # Format and print the motor_state dictionary
                    if motor_state:
                        motor_state_str = '\n'.join(["%s: %s" % (key, value) for key, value in motor_state.items()])
                        self.logger.info("Motor %s status: %s" % (motor_id, motor_state_str))
                    else:
                        self.logger.warning("Motor %s status is empty" % motor_id)
                except Exception as state_error:
                    self.logger.error("Failed to get motor %s status: %s" % (motor_id, state_error))
                
            except Exception as e:
                self.logger.error("Frame parsing failed: %s" % e)
    
    def get_status(self, motor_ids=None):
        ids = []
        positions = []
        velocities = []
        effort = []
        temperature = []

        if motor_ids is None:
            # 如果未指定电机ID，则获取当前电机ID
            motor_ids = [self.active_motor_id]
        # 打印电机ID
        self.logger.info("get_status: Motor IDs: %s" % motor_ids)

        for motor_id in motor_ids:
            motor_state = self.motors[motor_id].get_status()
            ids.append(motor_id)
            positions.append(motor_state['current_position'])
            velocities.append(motor_state['current_velocity'])
            effort.append(motor_state['current_torque'])
            temperature.append(motor_state['current_temperature'])

        # 写入log
        self.logger.info("All motors status:")
        self.logger.info("ids: %s" % ids)
        self.logger.info("positions: %s" % positions)
        self.logger.info("velocities: %s" % velocities)
        self.logger.info("effort: %s" % effort)
        self.logger.info("temperature: %s" % temperature)

        return {
            'ids': ids,
            'positions': positions,
            'velocities': velocities,
            'effort': effort,
            'temperature': temperature,
        }
    
    def send_frame(self, frame):
        """
        Send communication frame
        
        Args:
            frame: Communication frame data
            
        Returns:
            bool: Whether the frame was successfully sent
        """
        if self.serial_manager is None or not self.serial_manager.is_connected():
            self.logger.error("Not connected to the serial port")
            return False
        
        try:
            # Print the frame content in hexadecimal format
            hex_frame = ' '.join(['%02X' % b for b in frame])
            self.logger.info("Sending data: %s" % hex_frame)
            
            self.serial_manager.write(frame)
            return True
        except Exception as e:
            self.logger.error("Sending failed: %s" % e)
            return False
    
    def send_frames(self, frames, delay=0.02):
        """
        Send multiple communication frames
        
        Args:
            frames: List of communication frames
            delay: Delay between frames (seconds)
            
        Returns:
            bool: Whether all frames were successfully sent
        """
        success = True
        for frame in frames:
            if not self.send_frame(frame):
                success = False
            time.sleep(delay)  # Add delay to avoid sending too fast
        return success

    def send_command(self, command_type, **parameters):
        """
        Asynchronously send a command without waiting for a response
        
        Args:
            command_type: Command type
            **parameters: Command parameters
            
        Returns:
            bool: Whether the command was successfully sent
        """
        try:
            # If motor_id is not specified, use the active motor_id
            if 'motor_id' not in parameters:
                parameters['motor_id'] = self.active_motor_id
            
            # Create the command
            command = self.protocol.create_command(command_type, **parameters)
            print('command:', command)
            
            # Encode the command
            frame = self.protocol.encode_command(command)
            print('frame:', frame, type(frame))
            
            # Print the command content in hexadecimal format
            hex_frame = ' '.join(['%02X' % b for b in frame])
            self.logger.info("Encoded command (%s): %s" % (command_type, hex_frame))
            
            # Send the command
            return self.send_frame(frame)
            
        except Exception as e:
            self.logger.error("Failed to send command: %s" % e)
            return False
      
    def initialize_motors(self, motor_ids=None):
        """
        Initialize motors
        
        Args:
            motor_ids: Motor ID list, None uses the active motor ID
            
        Returns:
            bool: Whether initialization was successful
        """
        if motor_ids is None:
            motor_ids = [self.active_motor_id]
        
        # Create motor objects
        for motor_id in motor_ids:
            if motor_id not in self.motors:
                self.motors[motor_id] = Motor(motor_id)
        
        # Create and send initialization frames

        init_frames = self.protocol.create_motor_init_frame_all(motor_ids)
        
        success = self.send_frames(init_frames)
        
        if success:
            self.logger.info("Motors initialized: %s" % motor_ids)
            # Mark motors as enabled
            for motor_id in motor_ids:
                self.motors[motor_id].is_enabled = True
        else:
            self.logger.error("Motor initialization failed")
        
        return success
    
    def reset_motors(self, motor_ids=None):
        """
        Reset motors
        
        Args:
            motor_ids: Motor ID list, None uses the active motor ID
            
        Returns:
            bool: Whether the reset was successful
        """
        if motor_ids is None:
            motor_ids = [self.active_motor_id]
        
        # Create and send reset frames
        reset_frames = self.protocol.create_motor_reset_frame_all(motor_ids)
        success = self.send_frames(reset_frames)
        
        if success:
            self.logger.info("Motors reset: %s" % motor_ids)
            # Update motor status
            for motor_id in motor_ids:
                if motor_id in self.motors:
                    self.motors[motor_id].is_enabled = False
        else:
            self.logger.error("Motor reset failed")
        
        return success
    
    def motor_enable(self, motor_id=None):
        if motor_id is None:
            motor_id = self.active_motor_id
        self.motors[motor_id].is_enabled = True
        self.protocol.create_motor_enable_frame(motor_id)
        self.send_frame(frame)
        return frame

    def motor_disable(self, motor_id=None):
        if motor_id is None:
            motor_id = self.active_motor_id
        self.motors[motor_id].is_enabled = False
        self.protocol.create_motor_disable_frame(motor_id)
        self.send_frame(frame)
        return frame

    def motor_set_position(self, motor_id=None, position=None):
        if motor_id is None:
            motor_id = self.active_motor_id
        self.motors[motor_id].reference_position = position
        frame = self.protocol.create_motor_set_position_frame(motor_id, position)
        self.send_frame(frame)
        return frame

    def motor_jog(self, motor_id=None, speed=0.5):
        if motor_id is None:
            motor_id = self.active_motor_id
        frame = self.protocol.create_motor_jog_frame(motor_id, speed)
        self.send_frame(frame)
        return frame

    def motor_jog_stop(self, motor_id=None):
        if motor_id is None:
            motor_id = self.active_motor_id
        frame = self.protocol.create_motor_jog_stop_frame(motor_id)
        self.send_frame(frame)
        return frame

    def arm_create_motors(self, motor_ids=None):
        """
        Create arm motors
        """
        if motor_ids is None:
            motor_ids = [1,2,3,4,5,6]

        for motor_id in motor_ids:
            if motor_id not in self.motors:
                self.motors[motor_id] = Motor(motor_id)
    
    def arm_set_active_motor(self, motor_id):
        """
        设置活动电机ID
        
        Args:
            motor_id: 需要设置的电机ID
        """
        self.active_motor_id = motor_id
        self.logger.info("Current active motor ID has been set to: %s" % motor_id)
    
    def arm_enable(self):
        frames = [self.motor_enable(motor_id) for motor_id in self.motor_ids]
        return frames

    def arm_disable(self):  
        frames = [self.motor_disable(motor_id) for motor_id in self.motor_ids]
        return frames

    def arm_set_position(self, position):
        frames = [self.motor_set_position(motor_id, position) for motor_id in self.motor_ids]
        return frames

    def arm_get_status(self):
        ids = []
        positions = []
        velocities = []
        effort = []
        temperature = []
        enabled = []

        if motor_ids is None:
            # 如果未指定电机ID，则获取当前电机ID
            motor_ids = [self.active_motor_id]
        # 打印电机ID
        self.logger.info("get_status: Motor IDs: %s" % motor_ids)

        for motor_id in motor_ids:
            motor_state = self.motors[motor_id].get_status()
            ids.append(motor_id)
            positions.append(motor_state['current_position'])
            velocities.append(motor_state['current_velocity'])
            effort.append(motor_state['current_torque'])
            temperature.append(motor_state['current_temperature'])
            enabled.append(motor_state['is_enabled'])

        # 写入log   
        self.logger.info("All motors status:")
        self.logger.info("ids: %s" % ids)
        self.logger.info("positions: %s" % positions)
        self.logger.info("velocities: %s" % velocities)
        self.logger.info("effort: %s" % effort)
        self.logger.info("temperature: %s" % temperature)
        self.logger.info("enabled: %s" % enabled)

        return {
            'ids': ids,
            'positions': positions,
            'velocities': velocities,
            'effort': effort,
            'temperature': temperature,
            'enabled': enabled,
        }

    def run_interactive_mode(self):
        """Run interactive control mode"""
        self.logger.info("Enter interactive control mode, type 'help' for command help, type 'exit' to quit")
        
        try:
            while True:
                # Display prompt, including the current active motor ID
                cmd = raw_input("Control [Motor{}]> ".format(self.active_motor_id)).strip()
                
                if not cmd:
                    continue
                
                if cmd.lower() == 'exit':
                    self.logger.info("Exit interactive mode")
                    break
                
                if cmd.lower() == 'help':
                    self.logger.info("\nAvailable commands:")
                    self.logger.info("  motor <id>    - Set active motor ID")
                    self.logger.info("  init [id]     - Initialize motor, if no ID specified, use active motor")
                    self.logger.info("  reset [id]    - Reset motor, if no ID specified, use active motor")
                    self.logger.info("  zero          - Set current position as zero")
                    self.logger.info("  angle <value> - Set motor angle")
                    self.logger.info("  status        - Get motor status")
                    self.logger.info("  read <index>  - Read parameter value")
                    self.logger.info("  write <index> <value> - Write parameter value")
                    self.logger.info("  mode <mode>   - Set operation mode (mit:0-Force control, position:1-Position, velocity:2-Velocity, torque:3-Current, zero:4-Zero, jog:7-Jog)")
                    self.logger.info("  velocity <v>  - Set motor velocity")
                    self.logger.info("  torque <t>    - Set motor torque")
                    self.logger.info("  jog <speed>   - Start JOG mode, speed can be positive or negative")
                    self.logger.info("  stop          - Stop JOG mode")
                    self.logger.info("  mit <t> <p> <v> <kp> <kd> - MIT mode control")
                    self.logger.info("  pos <p>or<id1,id2...> <p1,p2...> - Position control (single motor or multiple motors)")
                    self.logger.info("  pos_spd <p> <v>or<id1,id2...> <p1,p2...> <v1,v2...> - Position and speed control (single motor or multiple motors)")
                    self.logger.info("  test          - Run protocol test")
                    self.logger.info("  help          - Display help")
                    self.logger.info("  exit          - Quit program")
                    continue
                
                # Parse command
                parts = cmd.split()
                cmd_type = parts[0].lower()
                
                if cmd_type == 'motor' and len(parts) > 1:
                    try:
                        motor_id = int(parts[1])
                        self.arm_set_active_motor(motor_id)
                    except ValueError:
                        self.logger.error("Motor ID must be a number")
                
                elif cmd_type == 'init':
                    motor_ids = None
                    if len(parts) > 1:
                        try:
                            # If the input is not a number, check if it is a list
                            if ',' in parts[1]:
                                motor_ids = [int(id.strip()) for id in parts[1].split(',')]
                            else:  
                                motor_ids = [int(parts[1])]
                        except ValueError:
                            self.logger.error("Motor ID must be a number or a comma-separated list of numbers")
                            continue
                    
                    self.initialize_motors(motor_ids)
                
                elif cmd_type == 'reset':
                    motor_ids = None
                    if len(parts) > 1:
                        try:
                            # If the input is not a number, check if it is a list
                            if ',' in parts[1]:
                                motor_ids = [int(id.strip()) for id in parts[1].split(',')]
                            else:  
                                motor_ids = [int(parts[1])]
                        except ValueError:
                            self.logger.error("Motor ID must be a number or a comma-separated list of numbers")
                            continue
                    
                    self.reset_motors(motor_ids)
                
                elif cmd_type == 'zero':
                    # Motor reset
                    self.reset_motors(None)
                    self.logger.info("Set current position as zero")
                    self.send_command('zero')
                
                elif cmd_type == 'angle' and len(parts) > 1:
                    try:
                        angle = float(parts[1])
                        # Switch to position mode first
                        self.logger.info("Switch to position mode")
                        print(self.protocol.modes['position'])
                        print(type(self.protocol.modes['position']))
                        self.send_command('mode', 
                                        index='RUN_MODE',
                                        value=self.protocol.modes['position'])
                        time.sleep(0.5)
                        
                        # Set angle
                        self.logger.info("Set motor angle to %s" % angle)
                        self.send_command('write', 
                                        index='LOC_REF',
                                        value=self.protocol.limit_position(self.active_motor_id, angle))
                    except ValueError:
                        self.logger.error("Angle must be a number")
                
                elif cmd_type == 'status':
                    # Check if "all" parameter is provided
                    if len(parts) > 1 and parts[1].lower() == 'all':
                        self.logger.info("Getting status for all motors")
                        motor_ids = self.motors.keys()
                        status = self.get_status(motor_ids=motor_ids)
                    else:
                        # Original code for getting status of active motor
                        self.logger.info("Getting motor %d status" % self.active_motor_id)
                        status = self.get_status()
                
                elif cmd_type == 'read' and len(parts) > 1:
                    index_name = parts[1].upper()
                    if index_name in self.protocol.index:
                        self.logger.info("Read parameter: %s" % index_name)

                        self.send_command('read', index=index_name)  
                    else:
                        self.logger.error("Unknown parameter index: %s" % index_name)
                
                elif cmd_type == 'write' and len(parts) > 2:
                    index_name = parts[1].upper()
                    try:
                        value = float(parts[2])
                        if index_name in self.protocol.index:
                            self.logger.info("Write parameter: %s = %s" % (index_name, value))
                            self.send_command('write', index=index_name, value=value)
                        else:
                            self.logger.error("Unknown parameter index: %s" % index_name)
                    except ValueError:
                        self.logger.error("Parameter value must be a number")
                
                elif cmd_type == 'mode' and len(parts) > 1:
                    mode_name = parts[1].lower()
                    modes = self.protocol.modes
                    
                    if mode_name in modes:
                        mode_value = modes[mode_name]
                        self.logger.info("Set operation mode to %s" % mode_name)
                        self.send_command('mode', 
                                        index='RUN_MODE',
                                        value=mode_value)
                    else:
                        self.logger.error("Unknown mode: %s" % mode_name)
                
                elif cmd_type == 'velocity' and len(parts) > 1:
                    try:
                        velocity = float(parts[1])
                        # Switch to velocity mode first - now it's 2
                        self.logger.info("Switch to velocity mode")
                        self.send_command('mode', 
                                        index='RUN_MODE',
                                        value=self.protocol.modes['velocity'])
                        time.sleep(0.5)
                        
                        # Set velocity
                        self.logger.info("Set motor velocity to %s" % velocity)
                        self.send_command('write', 
                                        index='SPD_REF',
                                        value=velocity)
                    except ValueError:
                        self.logger.error("Velocity must be a number")
                
                elif cmd_type == 'torque' and len(parts) > 1:
                    try:
                        torque = float(parts[1])
                        # Switch to torque mode first - now it's 3
                        self.logger.info("Switch to torque mode")
                        self.send_command('mode', 
                                        index='RUN_MODE',
                                        value=self.protocol.modes['torque'])
                        time.sleep(0.5)
                        
                        # Set torque
                        self.logger.info("Set motor torque to %s" % torque)
                        self.send_command('write', 
                                        index='IMIT_TORQUE',
                                        value=torque)
                    except ValueError:
                        self.logger.error("Torque must be a number")
                
                elif cmd_type == 'jog' and len(parts) > 1:
                    try:
                        speed = float(parts[1])
                        self.logger.info("Start JOG mode, speed: %s" % speed)
                        self.send_command('jog', speed=speed)
                    except ValueError:
                        self.logger.error("Speed must be a number")
                
                elif cmd_type == 'stop':
                    self.logger.info("Stop JOG mode")
                    self.send_command('jog', stop=True)
                
                elif cmd_type == 'mit' and len(parts) > 5:
                    try:
                        t = float(parts[1])  # Torque
                        p = float(parts[2])  # Position
                        v = float(parts[3])  # Speed
                        kp = float(parts[4])  # Position loop gain
                        kd = float(parts[5])  # Speed loop gain
                        
                        self.logger.info("MIT mode control: Torque=%s, Position=%s, Speed=%s, KP=%s, KD=%s" % (t, p, v, kp, kd))
                        self.send_command('mit_mode', torque=t, position=p, speed=v, kp=kp, kd=kd)
                    except ValueError:
                        self.logger.error("Parameters must be numbers")
                
                elif cmd_type == 'pos' and len(parts) > 1:
                    # Check if it's a multi-motor command (the first parameter contains a comma)
                    if ',' in parts[1]:
                        try:
                            # Parse multiple motor IDs and positions
                            motor_ids = [int(id.strip()) for id in parts[1].split(',')]
                            
                            # Check if enough position parameters are provided
                            if len(parts) <= 2:
                                self.logger.error("Missing position parameters")
                                continue
                                
                            positions = [float(p.strip()) for p in parts[2].split(',')]
                            
                            # Check if the lengths match
                            if len(motor_ids) != len(positions):
                                self.logger.error("Motor ID and position list lengths must be the same")
                                continue
                            
                            self.logger.info("Multiple motor position control: %s motors" % len(motor_ids))
                            frames = self.protocol.create_motor_pos_frame_all(motor_ids, positions)
                            for i, frame in enumerate(frames):
                                print('frame:', frame, type(frame))
                                self.logger.info("    Send frame #%s" % (i+1))
                                self.send_frame(frame)
                                time.sleep(0.05)  # Brief delay to avoid sending too fast
                        except ValueError:
                            self.logger.error("Parameter format error, motor ID and position must be numbers, multiple values separated by commas")
                        except Exception as e:
                            self.logger.error("Error executing command: %s" % e)
                    else:
                        # Single motor position control
                        try:
                            position = float(parts[1])
                            
                            self.logger.info("Position control: Position={}".format(position))
                            
                            frame = self.protocol.create_motor_pos_frame(self.active_motor_id, position)
                            print('frame:', frame, type(frame))
                            self.send_frame(frame)
                        except ValueError:
                            self.logger.error("Position parameter must be a number")
                
                elif cmd_type == 'pos_spd' and len(parts) > 1:
                    # Check if it's a multi-motor command (the first parameter contains a comma)
                    if ',' in parts[1]:
                        try:
                            # Parse multiple motor IDs, positions and speeds
                            motor_ids = [int(id.strip()) for id in parts[1].split(',')]
                            
                            # Check if enough parameters are provided
                            if len(parts) <= 3:
                                self.logger.error("Missing position or speed parameters")
                                continue
                                
                            positions = [float(p.strip()) for p in parts[2].split(',')]
                            speeds = [float(v.strip()) for v in parts[3].split(',')]
                            
                            # Check if the lengths match
                            if len(motor_ids) != len(positions) or len(motor_ids) != len(speeds):
                                self.logger.error("Motor ID, position and speed list lengths must be the same")
                                continue
                            
                            self.logger.info("Multiple motor position speed control: {} motors".format(len(motor_ids)))
                            frames = self.protocol.create_motor_frame_all_pos_spd(motor_ids, positions, speeds)
                            self.send_frames(frames)
                        except ValueError:
                            self.logger.error("Parameter format error, motor ID, position and speed must be numbers, multiple values separated by commas")
                        except Exception as e:
                            self.logger.error("Error executing command: {}".format(e))
                    else:
                        # Single motor position speed control
                        try:
                            # Check if enough parameters are provided
                            if len(parts) <= 2:
                                self.logger.error("Missing speed parameter")
                                continue
                                
                            position = float(parts[1])
                            speed = float(parts[2])
                            
                            self.logger.info("Position and speed control: Position=%s, Speed=%s" % (position, speed))
                            frames = self.protocol.create_motor_pos_spd_frame(self.active_motor_id, position, speed)
                            self.send_frames(frames)
                        except ValueError:
                            self.logger.error("Position and speed parameters must be numbers")

                else:
                    self.logger.error("Unknown command: %s" % cmd)
        
        except KeyboardInterrupt:
            self.logger.info("User interrupt, exit interactive mode")
        except Exception as e:
            self.logger.error("Interactive mode error: %s" % e)
    
    def run_demo(self, port=None):
        """
        Run the demo program
        
        Args:
            port: Serial port device name, None will automatically select the first available port
        """
        try:
            # List available serial ports
            self.serial_manager.list_ports()
            
            # If no port is specified and the parameter is list, only list the ports and exit
            if port == 'list':
                return
            
            # Connect to the serial port
            if not self.serial_manager.connect(port):
                return
            
            # Select the demo mode
            print("\nPlease select the demo mode:")
            print("Starting Interactive control mode")
            self.run_interactive_mode()
            
            # Select the active motor ID
            motor_id_str = raw_input("\nEnter active motor ID (default is 1): ").strip()
            if motor_id_str:
                try:
                    motor_id = int(motor_id_str)
                    self.arm_set_active_motor(motor_id)
                except ValueError:
                    self.logger.error("Motor ID must be a number, using default value 1")
        
        finally:
            # Disconnect
            self.serial_manager.disconnect()


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Deep Arm robotic arm control demo')
    parser.add_argument('-p', '--port', help='Serial port device name')
    parser.add_argument('-l', '--list', action='store_true', help='List available serial ports')
    parser.add_argument('-c', '--config', help='Configuration file path')
    parser.add_argument('-m', '--motor', type=int, default=1, help='Active motor ID')
    parser.add_argument('-t', '--test', action='store_true', help='Run protocol test')
    args = parser.parse_args()
    
    if args.list:
        args.port = 'list'
    
    demo = DeepArmDemo(args.config)
    
    # Set the active motor ID
    if args.motor:
        demo.arm_set_active_motor(args.motor)
    
    demo.run_demo(args.port)


if __name__ == "__main__":
    main() 