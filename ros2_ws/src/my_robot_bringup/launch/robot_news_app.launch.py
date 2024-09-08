from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_news_1 = Node(
        package = "my_py_pkg",
        # Use executable name from the pkg setup.py file
        executable = "robot_news_station",
        name="robot_news_1"
    )
    robot_news_2 = Node(
        package = "my_cpp_pkg",
        # Use executable name from the pkg setup.py file
        executable = "robot_news_station",
        name="robot_news_2",
        parameters=[
            {"robot_name":"robot_2"}
        ]
    )
    robot_news_3 = Node(
        package = "my_cpp_pkg",
        # Use executable name from the pkg setup.py file
        executable = "robot_news_station",
        name="robot_news_3",
        parameters=[
            {"robot_name":"robot_3"}
        ]
    )
    robot_news_4 = Node(
        package = "my_cpp_pkg",
        # Use executable name from the pkg setup.py file
        executable = "robot_news_station",
        name="robot_news_4",
        parameters=[
            {"robot_name":"robot_4"}
        ]
    )
    robot_news_5 = Node(
        package = "my_cpp_pkg",
        # Use executable name from the pkg setup.py file
        executable = "robot_news_station",
        name="robot_news_5",
        parameters=[
            {"robot_name":"robot_5"}
        ]
    )
    smartphone = Node(
        package = "my_py_pkg",
        executable = "smartphone",
        #name="smartphone_listener"
    )

    ld.add_action(robot_news_1)
    ld.add_action(robot_news_2)
    ld.add_action(robot_news_3)
    ld.add_action(robot_news_4)
    ld.add_action(robot_news_5)
    ld.add_action(smartphone)

    return ld