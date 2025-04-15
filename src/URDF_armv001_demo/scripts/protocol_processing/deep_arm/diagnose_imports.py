#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
详细诊断 Python 导入问题
"""

import os
import sys
import inspect

print("=" * 60)
print("Python Version: {}".format(sys.version))
print("Current Working Directory: {}".format(os.getcwd()))
print("Script Location: {}".format(os.path.abspath(__file__)))
print("=" * 60)

# Add possible paths
current_dir = os.path.dirname(os.path.abspath(__file__))
scripts_dir = os.path.abspath(os.path.join(current_dir, '../../'))
print("\nscripts_dir Path: {}".format(scripts_dir))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)
    print("\nPath added: {}".format(scripts_dir))

project_dir = os.path.abspath(os.path.join(scripts_dir, '../'))
if project_dir not in sys.path:
    sys.path.insert(0, project_dir)
    print("Path added: {}".format(project_dir))

parent_dir = os.path.abspath(os.path.join(project_dir, '../'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    print("Path added: {}".format(parent_dir))

print("\nPython Paths:")
for i, p in enumerate(sys.path):
    print("{}: {}".format(i, p))

# Check if the serial_comm directory exists
serial_comm_paths = [
    os.path.join(scripts_dir, 'serial_comm'),
    os.path.join(project_dir, 'serial_comm'),
    os.path.join(parent_dir, 'serial_comm'),
]

print("\nChecking serial_comm directory:")
for path in serial_comm_paths:
    exists = os.path.exists(path)
    is_dir = os.path.isdir(path) if exists else False
    print("Path: {}\n  Exists: {}, Is Directory: {}".format(path, exists, is_dir))
    
    if exists and is_dir:
        print("  Contents:")
        for item in os.listdir(path):
            full_path = os.path.join(path, item)
            item_type = "Directory" if os.path.isdir(full_path) else "File"
            print("    - {} ({})".format(item, item_type))
        
        # Check __init__.py
        init_path = os.path.join(path, '__init__.py')
        if os.path.exists(init_path):
            print("  __init__.py exists")
            # Attempt to display content
            try:
                with open(init_path, 'r') as f:
                    content = f.read()
                print("  __init__.py content ({} bytes):".format(len(content)))
                if len(content) > 500:
                    print("    {}...".format(content[:500]))
                else:
                    print("    {}".format(content))
            except Exception as e:
                print("  Unable to read __init__.py: {}".format(e))
        else:
            print("  __init__.py does not exist")

# Attempt different import methods
print("\nAttempting different import methods:")

try:
    import serial_comm
    print("Success: import serial_comm")
    print("  Import path: {}".format(serial_comm.__file__))
    print("  dir(serial_comm): {}".format(dir(serial_comm)))
except ImportError as e:
    print("Failure: import serial_comm - {}".format(e))

try:
    from scripts import serial_comm
    print("Success: from scripts import serial_comm")
    print("  Import path: {}".format(serial_comm.__file__))
    print("  dir(serial_comm): {}".format(dir(serial_comm)))
except ImportError as e:
    print("Failure: from scripts import serial_comm - {}".format(e))

try:
    sys.path.append(os.path.join(scripts_dir, 'serial_comm'))
    from . import serial_comm
    print("Success: from . import serial_comm")
    print("  Import path: {}".format(serial_comm.__file__))
    print("  dir(serial_comm): {}".format(dir(serial_comm)))
except ImportError as e:
    print("Failure: from . import serial_comm - {}".format(e))

# If the previous diagnosis shows that serial_comm indeed exists, attempt to directly import its modules
comm_dir = os.path.join(scripts_dir, 'serial_comm')
if os.path.isdir(comm_dir):
    print("\nAttempting to directly import serial_comm's submodules:")
    for item in os.listdir(comm_dir):
        if item.endswith('.py') and item != '__init__.py':
            module_name = item[:-3]  # Remove .py extension
            try:
                # Dynamically import using __import__
                module = __import__('serial_comm.' + module_name)
                print("Success: import serial_comm.{}".format(module_name))
                print("  Module: {}".format(module))
            except ImportError as e:
                print("Failure: import serial_comm.{} - {}".format(module_name, e))

print("\n=" * 60)
print("Diagnosis complete")
print("=" * 60) 