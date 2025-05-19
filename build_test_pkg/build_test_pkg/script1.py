import rclpy
# import the Node module from ROS2 python library
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def open_yaml_file(yaml_path):
    with open(yaml_path, "r") as stream:
        contents = None
        try:
            contents = yaml.safe_load(stream)            
        except yaml.YAMLError as exc:
            print(exc)

    print(contents)

    return contents

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    package_description = "build_test_pkg"
    yaml_file_path = os.path.join(get_package_share_directory(package_description), "params_files", "params.yaml")

    print("Path to files=="+str(yaml_file_path))

    
    yaml_data = open_yaml_file(yaml_file_path)

    print("YAML DATA XXXXX="+str(yaml_data))

    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function