import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description(): 
	# 声明包名、文件名，方便以后换文件
    package_name = 'azure_kinect_ros_driver'
    urdf_name = 'azure_kinect.urdf.xacro'
    rviz_name = 'display.rviz'

    ld = LaunchDescription()
	# 获取功能包路径（注意，这个路径是在工作空间的install文件夹里
    pkg_description = get_package_share_directory(package_name)
	# 声明文件路径，os.path.join将口号内的str用\连接，组成路径
    robot_description = os.path.join(pkg_description,
                                      'urdf',
                                      urdf_name,)
    doc = xacro.parse(open(robot_description))
    xacro.process_doc(doc)
    
    rviz_config_path = os.path.join(pkg_description,
                                    'rviz',
                                    rviz_name)
    # 发布关节状态，无关节调节可视化窗口
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        )
    # 发布关节状态，有关节调节可视化窗口
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        )
	# 发布机器人状态
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': doc.toxml()
            }]
        )
    # 启动rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path]
    )
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node) # 这行和下一行有其中一个就行，不要同时都用
    # ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
