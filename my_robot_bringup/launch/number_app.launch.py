from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher = Node(
        package="my_py_pkg",
        executable="number_publisher",
        namespace="test",
        name="my_number_publisher",
        remappings=[("number","my_number")]
    )

    number_counter = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        namespace="test",
        name="my_number_counter",
        remappings=[("number_count","my_number_count"),("number","my_number")]
    )

    ld.add_action(number_publisher)
    ld.add_action(number_counter)

    return ld