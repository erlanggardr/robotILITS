import launch
import launch_ros.actions


def generate_launch_description():

    controller_node = launch_ros.actions.Node(
        package="controller", executable="control_node", name="controller_node"
    )

    asio_node = launch_ros.actions.Node(
        package="stm32-comm", executable="stm_comm", name="asio_node"
    )

    joy_node = launch_ros.actions.Node(
        package="joy", executable="joy_node", name="joy_node"
    )

    return launch.LaunchDescription([joy_node, controller_node, asio_node])
