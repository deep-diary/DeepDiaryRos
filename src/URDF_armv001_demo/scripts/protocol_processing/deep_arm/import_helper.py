#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Import helper module to help locate and import the serial_comm package
when running in Python 2.7
"""

import os
import sys
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger('import_helper')

def find_serial_comm_module():
    """
    Find the serial_comm module and make it importable
    
    Returns:
        bool: True if successful, False otherwise
    """
    # Start with the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    logger.info("Current directory: %s", current_dir)
    
    # Check the scripts directory path
    scripts_dir = os.path.abspath(os.path.join(current_dir, '../../../'))
    logger.info("Scripts directory: %s", scripts_dir)
    
    serial_comm_path = os.path.join(scripts_dir, 'serial_comm')
    
    if os.path.exists(serial_comm_path) and os.path.isdir(serial_comm_path):
        logger.info("Found serial_comm directory at: %s", serial_comm_path)
        
        # Add the scripts directory to sys.path if not already there
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
            logger.info("Added %s to Python path", scripts_dir)
        
        # Check if we can import now
        try:
            import serial_comm
            logger.info("Successfully imported serial_comm from %s", serial_comm.__file__)
            return True
        except ImportError as e:
            logger.error("Still cannot import serial_comm: %s", e)
            
    else:
        logger.error("serial_comm directory not found at: %s", serial_comm_path)
        logger.info("Searching for serial_comm in other locations...")
        
        # Look for serial_comm in all directories in sys.path
        for path in sys.path:
            potential_path = os.path.join(path, 'serial_comm')
            if os.path.exists(potential_path) and os.path.isdir(potential_path):
                logger.info("Found serial_comm at: %s", potential_path)
                return True
        
        # Search the entire project directory tree
        project_dir = os.path.abspath(os.path.join(scripts_dir, '../'))
        logger.info("Searching in project directory: %s", project_dir)
        
        for root, dirs, files in os.walk(project_dir):
            if 'serial_comm' in dirs:
                serial_comm_path = os.path.join(root, 'serial_comm')
                logger.info("Found serial_comm directory at: %s", serial_comm_path)
                
                # Add the parent directory to sys.path
                parent_dir = os.path.dirname(serial_comm_path)
                if parent_dir not in sys.path:
                    sys.path.insert(0, parent_dir)
                    logger.info("Added %s to Python path", parent_dir)
                
                # Try importing again
                try:
                    import serial_comm
                    logger.info("Successfully imported serial_comm from %s", serial_comm.__file__)
                    return True
                except ImportError as e:
                    logger.error("Still cannot import serial_comm: %s", e)
    
    return False

if __name__ == "__main__":
    # Test the function
    success = find_serial_comm_module()
    if success:
        logger.info("Module search and import test successful")
        
        # Show what we can import from serial_comm
        import serial_comm
        logger.info("serial_comm contains: %s", dir(serial_comm))
    else:
        logger.error("Module search and import test failed") 