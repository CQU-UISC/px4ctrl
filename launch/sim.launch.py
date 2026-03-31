from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4ctrl',
            executable='px4ctrl_node',
            name='px4ctrl',
            output='screen',
            parameters=[{
                'px4ctrl_base_dir': PathJoinSubstitution([
                    FindPackageShare('px4ctrl')
                ]),
                'px4ctrl_cfg_name': 'gz500.json',
                'px4ctrl_transport_cfg_name': 'transport.json',
            }],
            remappings=[
                ('/px4ctrl/ext_odom', '/ekf_estimate'),
            ],
            emulate_tty=True
        )
    ])
