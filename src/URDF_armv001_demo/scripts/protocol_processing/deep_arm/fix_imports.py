#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Fix import paths for Python 2.7 compatibility
"""

import os
import sys

def add_project_paths():
    """Add project paths to Python module search path"""
    # Get the directory of this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Go up to 'scripts' directory
    scripts_dir = os.path.abspath(os.path.join(current_dir, '.../../'))
    
    # Add scripts directory to path if not already there
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
        print("Added to Python path: {}".format(scripts_dir))
    
    # Also add the parent of scripts directory (should be 'src')
    src_dir = os.path.abspath(os.path.join(scripts_dir, '../'))
    if src_dir not in sys.path:
        sys.path.insert(0, src_dir)
        print("Added to Python path: {}".format(src_dir))
    
    # Check if we can now import serial_comm
    try:
        import serial_comm
        print("Successfully imported serial_comm from {}".format(serial_comm.__file__))
        return True
    except ImportError as e:
        print("Still cannot import serial_comm: {}".format(e))
        return False

# Only run if executed directly
if __name__ == "__main__":
    add_project_paths() 