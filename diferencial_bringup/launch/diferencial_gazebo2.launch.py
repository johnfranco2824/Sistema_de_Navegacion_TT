from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('diferencial_description'), 
                             'urdf', 'diferencial.urdf.xacro')

    return LaunchDescription([
        Node(
            package='diferencial_description',
            executable='controller2.py',
            name='diff_robot_controller',
            output='screen'
        )
    ])