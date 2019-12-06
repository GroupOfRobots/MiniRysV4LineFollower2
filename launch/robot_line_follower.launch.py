from launch import LaunchDescription
from ament_index_python.packages import get_package_prefix
import launch_ros.actions
import os
package = "robot_line_follower"
 
 
def generate_launch_description():
    path = get_package_prefix(package)
    path = path[1:]
    path_list = path.split('/')
    print(path_list)
    del path_list[-2:]
    full_path = ''
    for path in path_list:
        full_path = full_path + '/' + path

    print(full_path)
    return LaunchDescription([
    launch_ros.actions.Node(
            # the name of the executable is set in CMakeLists.txt, towards the end of
            # the file, in add_executable(...) and the directives following it
            package= package, node_executable='tmp_node.py', output='screen', 
            parameters=[full_path + "/src/" + package + "/yaml/tmp_node_params.yaml"]),

	launch_ros.actions.Node(
            # the name of the executable is set in CMakeLists.txt, towards the end of
            # the file, in add_executable(...) and the directives following it
            package='robot_line_follower', node_executable='main', output='screen'),

    ])
