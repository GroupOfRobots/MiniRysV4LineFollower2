from launch import LaunchDescription
from ament_index_python.packages import get_package_prefix
import launch_ros.actions
import os
package = "robot_line_follower"
 
 
def generate_launch_description():
    path = get_package_prefix(package)
    path = path[1:]
    path_list = path.split('/')
    del path_list[-2:]
    full_path = ''
    for path in path_list:
        full_path = full_path + '/' + path

    return LaunchDescription([
    launch_ros.actions.Node(
            package=package, node_executable='data_pub_example', output='screen',
            parameters=[full_path + "/src/" + package + "/yaml/data_node_params.yaml"]),

    ])
