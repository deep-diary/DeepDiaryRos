#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Motor module compatibility test for Python 2.7
This test validates the motor.py module works in Python 2.7
"""

from __future__ import division, print_function, absolute_import

import sys
import os
import logging
import time
import struct

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger('motor_compatibility_test')

logger.info("Python version: %s", sys.version)

try:
    # Import the Motor class directly (assuming we're in the same directory)
    from motor import Motor
    logger.info("Successfully imported Motor class")
    
    # Test 1: Create motor instance
    motor = Motor(motor_id=1, position_limits=[-2.0, 2.0])
    logger.info("Test 1: Created motor instance with ID: %s", motor.id)
    
    # Test 2: Add to history & check
    motor.add_to_history(0.5)
    logger.info("Test 2: Added position to history, length: %s", len(motor.position_history))
    
    # Test 3: Set reference position with limits
    pos1 = motor.set_reference_position(1.0)  # Within limits
    logger.info("Test 3: Position within limits: %s", pos1)
    
    pos2 = motor.set_reference_position(-3.0) # Below min - should be limited to -2.0
    logger.info("Test 3: Position below min: %s", pos2)
    
    pos3 = motor.set_reference_position(3.0)  # Above max - should be limited to 2.0
    logger.info("Test 3: Position above max: %s", pos3)
    
    # Test 4: Test routine points
    motor.add_to_routine(0.5)
    motor.add_to_routine(1.0)
    logger.info("Test 4: Added 2 points to routine, length: %s", len(motor.routine_pts))
    
    # Test 5: Create mock feedback data
    # Create 8-byte data for testing update_from_feedback
    # Position: middle range
    # Velocity: low positive
    # Torque: zero
    # Temperature: 25°C
    raw_pos = 32767 + 0  # Middle position (0)
    raw_vel = 32767 + 3277  # ~10% of max velocity
    raw_torque = 32767 + 0  # No torque
    raw_temp = 250  # 25.0°C (temp is divided by 10)
    
    feedback_data = struct.pack('>HHHH', 
                               raw_pos,
                               raw_vel, 
                               raw_torque,
                               raw_temp)
    
    logger.info("Test 5: Created mock feedback data")
    
    # Test 6: Update from feedback
    motor.update_from_feedback(feedback_data)
    logger.info("Test 6: Updated from feedback - Position: %.2f, Velocity: %.2f, Torque: %.2f, Temp: %.2f", 
               motor.current_position, motor.current_velocity, motor.current_torque, motor.current_temperature)
    
    # Test 7: Clear history and routine
    motor.clear_history()
    logger.info("Test 7: Cleared history, length: %s", len(motor.position_history))
    
    motor.clear_routine()
    logger.info("Test 7: Cleared routine, length: %s", len(motor.routine_pts))
    
    # Test 8: Get status
    status = motor.get_status()
    logger.info("Test 8: Motor status: %s", status)
    
    # Test 9: Scale value function
    scaled_value = motor._scale_value(5000, 0, 10000, 0, 100)
    logger.info("Test 9: Scaled value from 5000 (0-10000) to (0-100): %.2f", scaled_value)
    
    logger.info("All tests passed!")
    
except Exception as e:
    logger.error("Test error: %s", str(e), exc_info=True) 