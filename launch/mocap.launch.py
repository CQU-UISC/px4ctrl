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
                'px4ctrl_cfg_name': 'xi35.yaml',
                'px4ctrl_zmq_cfg_name': 'zmq.yaml',
            }],
            remappings=[
                ('odom', '/ekf_estimate'),
                ('cmd', '/drone/poscmd'),
                ('allow_cmd', '/drone/allow_cmd'),
            ],
            emulate_tty=True
        )
    ])