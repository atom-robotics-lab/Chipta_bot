from launch import LaunchDescription
from launch_ros.actions import Node 


def generate_launch_description():

    twist2pwm = Node(
        package='chipta_bot',
        executable='twist_2_pwm',
        name='twist2pwm',
        output = 'screen',
        # namespace='',
    )

    diff_tf = Node(
        package='chipta_bot',
        executable='diff_tf',
        name='diff_tf',
        output = 'screen',
        # namespace='',
    )

    fkenc = Node(
        package='chipta_bot',
        executable='fkenc',
        name='fkenc',
        output = 'screen',
        # namespace='',
    )
        
    return LaunchDescription([
        twist2pwm,
        diff_tf,
        # fkenc,
    ])