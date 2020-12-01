#!/usr/bin/env python
"load config as global variables to use across modules"
import os
import yaml

def init(config_file_name):
    global config_path
    global config
    config_path = os.path.expanduser("~/catkin_ws/src/shallow_depth_insertion/config/") + config_file_name #TODO:change the workspace name
    with open(config_path, 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)