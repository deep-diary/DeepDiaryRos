#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
DeepArm协议处理器
提供与DeepArm机械臂通信的协议处理功能
"""

from __future__ import division, print_function, absolute_import

import struct
import time
import logging

# 修改相对导入为绝对导入
import sys
import os

# 获取当前文件目录
current_dir = os.path.dirname(os.path.abspath(__file__))
protocol_dir = os.path.abspath(os.path.join(current_dir, '../'))
if protocol_dir not in sys.path:
    sys.path.insert(0, protocol_dir)

# 使用绝对导入
from base import ProtocolProcessor, ProtocolCommand, ProtocolResponse
from protocol_exceptions import PacketError

import yaml
# from .motor import Motor

class DeepArmCommand(ProtocolCommand):
    """Deep Arm Command Class"""
    
    def __init__(self, command_type, parameters=None, command_code=None):
        super(DeepArmCommand, self).__init__(command_type, parameters)
        self.command_code = command_code

class DeepArmResponse(ProtocolResponse):
    """Deep Arm Response Class"""
    
    def __init__(self, response_type, data=None, 
                success=True, error_msg=None, response_code=None):
        super(DeepArmResponse, self).__init__(response_type, data, success, error_msg)
        self.response_code = response_code

class DeepArmProtocol(ProtocolProcessor):
    """SCARA Robot Communication Protocol Class"""
    
    def __init__(self, config_file=None):
        """
        Initialize the protocol object
        
        Args:
            config_file: Configuration file path, None uses the default configuration file
        """
        super(DeepArmProtocol, self).__init__(config_file)
        # Set up logging
        self.logger = logging.getLogger('DeepArmProtocol')
        
        # Load the configuration file
        self.config = self.load_config(config_file)
        
        # Basic protocol configuration
        protocol_config = self.config.get('protocol', {})
        self.AT_HEADER = protocol_config.get('header', 'AT').encode()
        self.END_BYTES = protocol_config.get('end_bytes', '\r\n').encode()
        self.master_id = int(protocol_config.get('master_id', '0x00fd'), 16)

        # Get parameter range constants from the configuration
        motor_range = self.config.get('motor', {}).get('range', {})
        self.T_MIN, self.T_MAX = motor_range.get('torque', [-10.0, 10.0])  # Torque range
        self.P_MIN, self.P_MAX = motor_range.get('position', [-12.5, 12.5])  # Position range
        self.V_MIN, self.V_MAX = motor_range.get('velocity', [-65.0, 65.0])  # Velocity range
        self.KP_MIN, self.KP_MAX = motor_range.get('kp', [0.0, 500.0])  # Position loop gain range
        self.KD_MIN, self.KD_MAX = motor_range.get('kd', [0.0, 5.0])  # Velocity loop gain range

        # Motor parameter index
        self.index = {}
        for key, value in self.config.get('index', {}).items():
            self.index[key] = int(value, 16)
        
        # Operating mode
        self.modes = self.config.get('motor', {}).get('modes', {})
        
        self.logger.info("Deep Arm Protocol initialized")
    
    def load_config(self, config_file=None):
        """
        Load the configuration file
        
        Args:
            config_file: Configuration file path, None uses the default configuration file
            
        Returns:
            dict: Configuration dictionary
        """
        if config_file is None:
            # Use the default configuration file
            module_dir = os.path.dirname(os.path.abspath(__file__))
            config_file = os.path.join(module_dir, 'config.yaml')
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            self.logger.info("Loaded configuration file: %s" % config_file)
            return config
        except Exception as e:
            self.logger.error("Failed to load configuration file: %s" % str(e))
            # Return empty configuration
            return {}
        
    def create_AT_frame(self):
        # Send 'AT+AT' command
        frame = [0x41, 0x54, 0x2B, 0x41, 0x54, 0x0D, 0x0A]
        return frame

    def create_frame(self, mode, motor_id, res, data, payload=None):
        """
        Create a communication frame
        
        Args:
            mode: Command mode
            motor_id: Motor ID
            res: Reserved field
            data: Data field
            payload: Payload data
            
        Returns:
            bytearray: Complete communication frame
        """
        can_id = (res << 29) | (mode << 24) | (data << 8) | motor_id
        if self.config.get('uart', {}).get('use_uart2can', False):
            can_id = (can_id << 3) + 0x04  # If you need to use the usb to can module, you need to convert
        
        frame = bytearray()
        frame.extend(self.AT_HEADER)
        frame.extend(struct.pack('>I', can_id))
        
        if payload:
            frame.append(len(payload))
            frame.extend(payload)
        else:
            frame.append(0)
        
        frame.extend(self.END_BYTES)
        return frame

    def create_motor_enable_frame(self, motor_id):
        """
        Create a motor enable frame (mode 3)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        return self.create_frame(3, motor_id, 0, self.master_id)

    def create_motor_reset_frame(self, motor_id):
        """
        Create a motor reset frame (mode 4)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        return self.create_frame(4, motor_id, 0, self.master_id)

    def create_motor_zero_frame(self, motor_id):
        """
        Create a motor zero frame (mode 6)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        payload = bytearray([0] * 8)
        payload[0] = 1
        payload[1] = 1
        return self.create_frame(6, motor_id, 0, self.master_id, payload)

    def create_motor_mode_frame(self, motor_id, run_mode):
        """
        Create a motor mode setting frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            index: Parameter index
            run_mode: Operating mode
            
        Returns:
            bytearray: Communication frame
        """
        index = self.index['RUN_MODE']
        payload = bytearray(8)
        payload[0:2] = [index & 0xFF, (index >> 8) & 0xFF]
        payload[4] = run_mode
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_jog_frame(self, motor_id, jog_speed):
        """
        Create a motor jog mode setting frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            jog_speed: Jog speed
            
        Returns:
            bytearray: Communication frame
        """
        index = self.index['RUN_MODE']    # write
        run_mode = self.modes.get('jog', 7)  # jog mode
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        payload[4] = run_mode
        payload[5] = 0x01  # 1: enable jog, 0: disable jog
        jog_speed = min(max(jog_speed, -30), 30)
        scaled_speed = int((jog_speed + 30) / 60 * 65535)
        payload[6:8] = struct.pack('>H', scaled_speed)
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_jog_frame_stop(self, motor_id):
        """
        Create a motor jog mode stop frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            bytearray: Communication frame
        """
        index = self.index['RUN_MODE']    # write
        run_mode = self.modes.get('jog', 7)  # jog mode
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        payload[4] = run_mode
        payload[5] = 0x00  # 1: enable jog, 0: disable jog
        payload[6:8] = struct.pack('>H', 0x7fff)
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_write_frame(self, motor_id, index, value):
        """
        Create a motor parameter write frame (mode 0x12)
        
        Args:
            motor_id: Motor ID
            index: Parameter index
            value: Parameter value
            
        Returns:
            bytearray: Communication frame
        """
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        payload[4:8] = struct.pack('<f', float(value))
        return self.create_frame(0x12, motor_id, 0, self.master_id, payload)
    
    def create_motor_read_frame(self, motor_id, index):
        """
        Create a motor parameter read frame (mode 0x11)
        
        Args:
            motor_id: Motor ID
            index: Parameter index
            
        Returns:
            bytearray: Communication frame
        """
        payload = bytearray(8)
        payload[0:2] = struct.pack('<H', index)
        return self.create_frame(0x11, motor_id, 0, self.master_id, payload)
    
    def create_motor_mit_mode_frame(self, motor_id, torque, position, speed, kp, kd):
        """
        Create a motor MIT mode control frame (mode 1)
        
        Args:
            motor_id: Motor ID
            torque: Torque value
            position: Target position
            speed: Target speed
            kp: Position loop gain
            kd: Velocity loop gain
            
        Returns:
            bytearray: Communication frame
        """
        data = self.float_to_uint(torque, self.T_MIN, self.T_MAX, 16)
        payload = bytearray(8)
        pos_uint = self.float_to_uint(position, self.P_MIN, self.P_MAX, 16)
        spd_uint = self.float_to_uint(speed, self.V_MIN, self.V_MAX, 16)
        kp_uint = self.float_to_uint(kp, self.KP_MIN, self.KP_MAX, 16)
        kd_uint = self.float_to_uint(kd, self.KD_MIN, self.KD_MAX, 16)
        
        payload[0:2] = struct.pack('<H', pos_uint)
        payload[2:4] = struct.pack('<H', spd_uint)
        payload[4:6] = struct.pack('<H', kp_uint)
        payload[6:8] = struct.pack('<H', kd_uint)
        
        return self.create_frame(1, motor_id, 0, data, payload)
    
    def create_motor_init_frame(self, motor_id):
        """
        Create a motor initialization frame (including enable, zero and mode setting)
        
        Args:
            motor_id: Motor ID
            
        Returns:
            list: Communication frame list
        """
        return [
            self.create_AT_frame(),
            self.create_motor_reset_frame(motor_id),
            self.create_motor_zero_frame(motor_id),
            self.create_motor_enable_frame(motor_id),
            self.create_motor_mode_frame(motor_id, self.modes.get('position', 1))
        ]
    
    def create_motor_init_frame_all(self, motor_ids):
        """
        Create initialization frames for all motors
        
        Args:
            motor_ids: Motor ID list
            
        Returns:
            list: Communication frame list
        """
        frames = []
        for motor_id in motor_ids:
            frames.extend(self.create_motor_init_frame(motor_id))
        return frames
    
    def create_motor_reset_frame_all(self, motor_ids):
        """
        Create reset frames for all motors
        
        Args:
            motor_ids: Motor ID list
            
        Returns:
            list: Communication frame list
        """
        return [self.create_motor_reset_frame(motor_id) for motor_id in motor_ids]
    
    def create_motor_pos_frame(self, motor_id, position):
        # Calculate the position index and speed index
        loc_index = self.index['LOC_REF']
        # Create and send the position control frame
        return self.create_motor_write_frame(motor_id, loc_index, self.limit_position(motor_id, position))
    
    def create_motor_pos_frame_all(self, motor_ids, positions):
        frames = []
        # Check if the list lengths match
        if len(motor_ids) != len(positions):
            self.logger.error("Motor ID and position list lengths do not match")
            return frames
        
        # Create a position control frame for each motor
        for i, motor_id in enumerate(motor_ids):
            position = positions[i]
            frames.append(self.create_motor_pos_frame(motor_id, self.limit_position(motor_id, position)))

        return frames

    def create_motor_pos_spd_frame(self, motor_id, position, speed):
        """
        Create a motor position and speed control frame
        
        Args:
            motor_id: Motor ID
            position: Target position
            speed: Target speed
            
        Returns:
            bytearray: Communication frame
        """
        # Calculate the position index and speed index
        loc_index = self.index['LOC_REF']
        spd_index = self.index['LIMIT_SPD']
        
        # Create and send the position control frame
        position_frame = self.create_motor_write_frame(motor_id, loc_index, self.limit_position(motor_id, position))
        
        # Create and send the speed control frame
        speed_frame = self.create_motor_write_frame(motor_id, spd_index, speed)
        
        # Return two frames, the caller needs to send them in order
        return [position_frame, speed_frame]

    def create_motor_frame_all_pos_spd(self, motor_ids, positions, speeds):
        """
        Create multiple motor position and speed control frames
        
        Args:
            motor_ids: Motor ID list
            positions: Target position list
            speeds: Target speed list
            
        Returns:
            list: Communication frame list
        """
        frames = []
        
        # Check if the list lengths match
        if len(motor_ids) != len(positions) or len(motor_ids) != len(speeds):
            self.logger.error("Motor ID, position and speed list lengths do not match")
            return frames
        
        # Create a control frame for each motor
        for i, motor_id in enumerate(motor_ids):
            position = positions[i]
            speed = speeds[i]
            # Get the position and speed control frame
            pos_spd_frames = self.create_motor_pos_spd_frame(motor_id, position, speed)
            frames.extend(pos_spd_frames)
        
        return frames

    def limit_position(self, motor_id, position):
        # Get the corresponding joint name according to the motor_id
        joint_name = 'joint' + str(motor_id)
        # Get the joint range
        joint_range = self.config.get('motor', {}).get('range', {}).get(joint_name, [-0.5, 0.5])
        # Limit the position within the joint range
        position = min(max(position, joint_range[0]), joint_range[1])
        return position
    
    def create_motor_sinwave_test(self, motor_id, amplitude, frequency, start_stop):
        # 41 54 90 07 e8 34 08 03 70 00 00 00 00 80 3f 0d 0a # Set amplitude
        # 41 54 90 07 e8 34 08 02 70 00 00 00 00 80 3f 0d 0a # Set frequency
        # 41 54 90 07 e8 34 08 01 70 00 00 01 00 00 00 0d 0a # Start sine test
        # 41 54 90 07 e8 34 08 01 70 00 00 00 00 00 00 0d 0a # Stop sine test
        # 41 54 20 07 e8 34 08 00 00 00 00 00 00 00 00 0d 0a # Reset motor

        # 41 54 98 07 e8 34 08 1b 82 30 02 a0 42 32 0e 0d 0a # Load parameter table
        pass

    def create_motor_scope_disp(self, motor_id, frequency, channel, start_stop):
    # 41 54 50 07 e8 0c 08 14 00 11 00 00 10 0e 00 0d 0a    Set sampling frequency
    # 41 54 50 0f e8 0c 08 16 30 16 30 16 30 16 30 0d 0a    Set channel
    # 41 54 50 17 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a    Start sampling
    # 41 54 50 1f e8 0c 08 00 00 00 00 00 00 00 00 0d 0a    Stop sampling
        pass

    def float_to_uint(self, x, x_min, x_max, bits):
        """
        Convert a float to an unsigned integer
        
        Args:
            x: Input float
            x_min: Minimum value
            x_max: Maximum value
            bits: Number of bits
            
        Returns:
            int: Converted unsigned integer
        """
        span = x_max - x_min
        offset = x_min
        x = min(max(x, x_min), x_max)
        return int((x - offset) * ((1 << bits) - 1) / span)

    def uint_to_float(self, uint, x_min, x_max, bits):
        """
        Convert an unsigned integer to a float
        
        Args:
            uint: Unsigned integer
            x_min: Minimum value
            x_max: Maximum value
            bits: Number of bits
            
        Returns:
            float: Converted float
        """
        span = x_max - x_min
        offset = x_min
        return uint * span / ((1 << bits) - 1) + offset

    def parse_frame(self, data):
        """
        Parse the communication frame
        
        Args:
            data: Received data
            
        Returns:
            dict: Parsed data dictionary, None if parsing fails
        """
        if not data or len(data) < 7:
            return None
        
        if data[0:2] != self.AT_HEADER or data[-2:] != self.END_BYTES:
            # self.logger.error("Invalid frame: %s" % data)
            return None
        
        can_id = struct.unpack('>I', data[2:6])[0]
        # If using usb to can module, conversion is needed
        if self.config.get('uart', {}).get('use_uart2can', False):
            can_id = can_id >> 3   # Right shift by 3 bits
        
        data_length = data[6]
        # print('data_length, data_type:', data_length, type(data_length))
        payload = data[7:7+data_length] if data_length > 0 else b''
        
        return {
            'res': (can_id >> 29) & 0xFF,
            'mode': (can_id >> 24) & 0xFF,
            'data': can_id & 0xFF,
            'motor_id': (can_id >> 8) & 0xFF,
            'payload': payload
        }
    
    def format_frame(self, frame):
        """
        Format the frame into a readable string
        
        Args:
            frame: Communication frame
            
        Returns:
            str: Formatted string
        """
        if not frame:
            return "Invalid frame"
        
        try:
            # Parse the frame
            parsed = self.parse_frame(frame)
            if not parsed:
                return "Frame format error"
            
            # Format the result
            # mode_str = "0x%02X" % parsed['mode']
            # motor_str = str(parsed['motor_id'])
            # data_str = "0x%02X" % parsed['data']
            # payload_str = ' '.join(["%02X" % b for b in parsed['payload']])

            # formatted = "Mode:%s Motor:%s Data:%s Payload:[%s]" % (mode_str, motor_str, data_str, payload_str)
            # self.logger.info("Parsed frame: %s" % formatted)

            return parsed
        except Exception as e:
            return "Parse error: %s" % str(e)

    def create_command(self, command_type, **parameters):
        """
        Create a DeepArmCommand command object
        
        Args:
            command_type: Command type, such as 'enable', 'reset', 'zero', 'mit_mode', etc.
            **parameters: Command parameters, different for different command types
            
        Returns:
            DeepArmCommand: Created command object
        """
        command_codes = {
            'enable': 3,
            'reset': 4,
            'zero': 6,
            'read': 0x11,
            'write': 0x12,
            'mode': 0x12,
            'mit_mode': 1,
            'jog': 7
        }
        
        # Get the command code, default to None
        command_code = command_codes.get(command_type)
        
        return DeepArmCommand(command_type, parameters, command_code)

    def encode_command(self, command):
        """
        将命令对象编码为字节序列
        
        Args:
            command: DeepArmCommand命令对象
            
        Returns:
            bytes: 编码后的字节序列
        """
        command_type = command.command_type
        params = command.parameters or {}
        
        if command_type == 'enable':
            motor_id = params.get('motor_id', 1)
            return self.create_motor_enable_frame(motor_id)
        
        elif command_type == 'reset':
            motor_id = params.get('motor_id', 1)
            return self.create_motor_reset_frame(motor_id)
        
        elif command_type == 'zero':
            motor_id = params.get('motor_id', 1)
            return self.create_motor_zero_frame(motor_id)
        elif command_type == 'mode':
            motor_id = params.get('motor_id', 1)
            value = params.get('value')
            return self.create_motor_mode_frame(motor_id, value)
        
        elif command_type == 'mit_mode':
            motor_id = params.get('motor_id', 1)
            torque = params.get('torque', 0.0)
            position = params.get('position', 0.0)
            speed = params.get('speed', 0.0)
            kp = params.get('kp', 0.0)
            kd = params.get('kd', 0.0)
            return self.create_motor_mit_mode_frame(motor_id, torque, position, speed, kp, kd)
        
        elif command_type == 'write':
            motor_id = params.get('motor_id', 1)
            index = params.get('index')
            value = params.get('value')
            
            if index is None or value is None:
                raise ValueError("写入命令必须提供index和value参数")
            
            # 如果index是字符串，则从self.index中获取
            if isinstance(index, str):
                index = self.index.get(index)
                if index is None:
                    raise ValueError("Unknown index name: %s" % params.get('index'))
            return self.create_motor_write_frame(motor_id, index, value)
        
        elif command_type == 'read':
            motor_id = params.get('motor_id', 1)
            index = params.get('index')
            
            if index is None:
                raise ValueError("读取命令必须提供index参数")
            
            # 如果index是字符串，则从self.index中获取
            if isinstance(index, str):
                index = self.index.get(index)
                if index is None:
                    raise ValueError("Unknown index name: %s" % params.get('index'))
            
            return self.create_motor_read_frame(motor_id, index)
        
        elif command_type == 'jog':
            motor_id = params.get('motor_id', 1)
            jog_speed = params.get('speed', 0)
            
            if 'stop' in params and params['stop']:
                return self.create_motor_jog_frame_stop(motor_id)
            else:
                return self.create_motor_jog_frame(motor_id, jog_speed)
        
        else:
            raise ValueError("Unsupported command type: %s" % command_type)

    def decode_response(self, data):
        """
        Decode the received byte sequence into a response object
        
        Args:
            data: The received byte sequence
            
        Returns:
            DeepArmResponse: The decoded response object
        """
        try:
            # Parse the frame data
            parsed = self.parse_frame(data)
            if not parsed:
                return DeepArmResponse('error', None, False, "Invalid data frame")
            
            response_mode = parsed['mode']
            motor_id = parsed['motor_id']
            payload = parsed['payload']
            
            response_data = {
                'motor_id': motor_id,
                'mode': response_mode,
                'data': parsed['data'],
                'raw_payload': payload
            }
            
            # Handle data based on different response modes
            if response_mode == 0x11:  # Read parameter response
                if len(payload) >= 8:
                    # Extract index and value
                    index = struct.unpack('<H', payload[0:2])[0]
                    value = struct.unpack('<f', payload[4:8])[0]
                    response_data['index'] = index
                    response_data['value'] = value
                    
                    # Attempt to find the name corresponding to the index
                    for name, idx in self.index.items():
                        if idx == index:
                            response_data['index_name'] = name
                            break
                    
                    return DeepArmResponse('read_response', response_data, True, None, response_mode)
                else:
                    return DeepArmResponse('error', response_data, False, "Read response data incomplete", response_mode)
            
            elif response_mode == 0x19:  # Status response
                if len(payload) >= 8:
                    # Parse status information
                    status_data = {
                        'position': struct.unpack('<f', payload[0:4])[0],
                        'velocity': struct.unpack('<f', payload[4:8])[0]
                    }
                    response_data.update(status_data)
                    return DeepArmResponse('status', response_data, True, None, response_mode)
                else:
                    return DeepArmResponse('error', response_data, False, "Status response data incomplete", response_mode)
            
            # For other response modes, only return basic information
            return DeepArmResponse('response', response_data, True, None, response_mode)
            
        except Exception as e:
            return DeepArmResponse('error', None, False, "Failed to parse response: {}".format(str(e)))

    def validate_response(self, response, command=None):
        """
        Validate if the response is valid
        
        Args:
            response: Response object
            command: Command object, optional, used to validate if the response matches the command
            
        Returns:
            bool: If the response is valid
        """
        # First, check if the response is successful
        if not response.success:
            self.logger.error("Response error: %s" % response.error_msg)
            return False
        
        # If no command is provided, only check if the response is successful
        if command is None:
            return True
        
        # Validate the response based on the command type
        command_type = command.command_type
        response_type = response.response_type
        
        if command_type == 'read' and response_type == 'read_response':
            # Validate the response for read commands
            cmd_motor_id = command.parameters.get('motor_id', 1)
            resp_motor_id = response.data.get('motor_id')
            
            if cmd_motor_id != resp_motor_id:
                self.logger.warning("Response motor ID (%s) does not match command motor ID (%s)" % (resp_motor_id, cmd_motor_id))
                return False
            
            # Validate if the index matches
            cmd_index = command.parameters.get('index')
            resp_index = response.data.get('index')
            
            # If the command index is a string, get it from self.index
            if isinstance(cmd_index, str):
                cmd_index = self.index.get(cmd_index)
            
            if cmd_index != resp_index:
                self.logger.warning("Response index (%s) does not match command index (%s)" % (resp_index, cmd_index))
                return False
            
            return True
        
        # For other command types, consider the response valid if it is successful
        return True