#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Deep Arm motor class implementation
"""

from __future__ import division, print_function, absolute_import

import struct
import logging
import time
from collections import deque

class Motor:
    """
    Motor class, used to track and manage motor status
    """
    
    def __init__(self, motor_id, position_limits=None):
        """
        Initialize motor object
        
        Args:
            motor_id: Motor ID
            position_limits: Position limits in format [min, max]
        """
        self.id = motor_id
        self.logger = logging.getLogger("Motor_{}".format(motor_id))
        
        # Current status
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_torque = 0.0
        self.current_temperature = 0.0
        
        # Data conversion parameters
        self.POSITION_RANGE = (-4 * 3.14159, 4 * 3.14159)  # -4π ~ 4π
        self.VELOCITY_RANGE = (-30, 30)                     # -30rad/s ~ 30rad/s
        self.TORQUE_RANGE = (-12, 12)                      # -12Nm ~ 12Nm
        
        # Position limits
        self.position_limits = position_limits or [-1.0, 1.0]
        
        # Motion control
        self.reference_position = 0.0
        self.is_enabled = False
        self.is_initialized = False
        self.is_updated = False
        self.step_finished = False
        
        # History records
        self.position_history = []
        self.routine_pts = []
        
        self.logger.info("Motor initialization completed, ID: %s, Position limits: %s" % (motor_id, self.position_limits))

    def update_from_feedback(self, data_bytes):
        """
        Update motor position
        
        Args:
            data_bytes: 8-byte feedback data
            
        Raises:
            ValueError: Incorrect data length
        """
        if len(data_bytes) != 8:
            raise ValueError("Feedback data must be 8 bytes")
            
        # Parse position data (Byte 0-1)
        position_raw = struct.unpack('>H', data_bytes[0:2])[0]
        position_raw = position_raw - 32767
        self.current_position = self._scale_value(position_raw, -32768, 32767,
                                                self.POSITION_RANGE[0],
                                                self.POSITION_RANGE[1])
        
        # Parse velocity data (Byte 2-3)
        velocity_raw = struct.unpack('>H', data_bytes[2:4])[0]
        velocity_raw = velocity_raw - 32767
        self.current_velocity = self._scale_value(velocity_raw, -32768, 32767,
                                               self.VELOCITY_RANGE[0],
                                               self.VELOCITY_RANGE[1])
        
        # Parse torque data (Byte 4-5)
        torque_raw = struct.unpack('>H', data_bytes[4:6])[0]
        torque_raw = torque_raw - 32767
        self.current_torque = self._scale_value(torque_raw, -32768, 32767,
                                             self.TORQUE_RANGE[0],
                                             self.TORQUE_RANGE[1])
        
        # Parse temperature data (Byte 6-7)
        self.current_temperature = struct.unpack('>H', data_bytes[6:8])[0]
        self.current_temperature = self.current_temperature / 10

        # update is_updated status
        self.is_updated = True
        
        # Update history records
        self.add_to_history(self.current_position)
        
        self.logger.debug("Position: %.2f, Velocity: %.2f, Torque: %.2f, Temperature: %.2f" % 
                         (self.current_position, self.current_velocity, self.current_torque, self.current_temperature))

    def _scale_value(self, value, in_min, in_max, out_min, out_max):
        """
        Update motor temperature
        
        Args:
            value: Input value
            in_min: Input minimum value
            in_max: Input maximum value
            out_min: Output minimum value
            out_max: Output maximum value
            
        Returns:
            float: Mapped value
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def set_reference_position(self, position):
        """
        Set reference position for the motor
        
        Args:
            position: Target position
            
        Returns:
            float: The actual position set after limits check
        """
        # Apply position limits using the same logic as before
        self.reference_position = max(min(position,
                                      self.position_limits[1]),
                                      self.position_limits[0])
        self.logger.debug("Reference position set: {}".format(self.reference_position))
        return self.reference_position
    
    def add_to_history(self, position):
        """
        Add position to history
        
        Args:
            position: Position to add
        """
        self.position_history.append(position)
        # Limit history record length
        if len(self.position_history) > 1000:
            self.position_history = self.position_history[-1000:]

    def add_to_routine(self, position):
        """
        Add position to teaching routine
        
        Args:
            position: Path point position
        """
        self.routine_pts.append(position)
        self.logger.info("Added teaching point: %s" % position)

    def clear_history(self):
        """Clear history records"""
        self.position_history = []
        self.logger.info("Cleared history records")

    def clear_routine(self):
        """Clear teaching routine"""
        self.routine_pts = []
        self.logger.info("Cleared teaching routine")

    def get_status(self):
        """
        Get motor state as dictionary
        
        Returns:
            dict: Motor state information
        """
        return {
            'id': self.id,
            'current_position': self.current_position,
            'current_velocity': self.current_velocity,
            'current_torque': self.current_torque,
            'current_temperature': self.current_temperature,
            'is_enabled': self.is_enabled,
            'is_initialized': self.is_initialized,
            'history_length': len(self.position_history),
            'routine_length': len(self.routine_pts),
            'is_updated': self.is_updated
        }
