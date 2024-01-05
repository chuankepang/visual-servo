# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="z1_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="z1.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value="True",
            description='If true, use simulated clock'),
    )
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "display.rviz"]
    )
    
    z1_description_pkg = get_package_share_directory('z1_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(os.path.join(z1_description_pkg), 'world', 'test.sdf')
    print(world_path)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [ros_gz_sim_pkg, "launch", 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': f'-r {world_path} -v 4'}.items(),
        # launch_arguments={'ign_args': f'-r empty.sdf -v 4'}.items(),
    )
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    # path = os.path.join(
    #     get_package_share_directory('z1_description'))
    # xacro_file = os.path.join(path,
    #                           'urdf',
    #                           'z1_dual.urdf')
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # robot_description_content = doc.toxml()

    robot_description = {"robot_description": robot_description_content}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
        
    # Ignition Gazebo - Spawn Entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            # '-topic', 'robot_description',
                   '-string', robot_description_content,
                   '-name', 'z1',
                   '-x', '0',
                   '-z', '0',
                   '-Y', '0',
                   '-allow_renaming', 'true'],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controllers'],
        output='screen'
    )

    # Bridge
    camera_id = ['']
    camera_link = ['link06']
    arg_bridge = []
    remap_bridge = []
    model_name = 'z1'
    for id in range(len(camera_id)):
        arg_bridge.append(f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo') 
        arg_bridge.append(f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/image@sensor_msgs/msg/Image@ignition.msgs.Image') 
        arg_bridge.append(f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image') 
        arg_bridge.append(f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked') 

        remap_bridge.append((f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/camera_info', f'/camera/camera_info')) 
        remap_bridge.append((f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/image', f'/camera/image')) 
        remap_bridge.append((f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/depth_image', f'/camera/depth_image')) 
        remap_bridge.append((f'/world/empty/model/{model_name}/link/{camera_link[id]}/sensor/oak_d_lite/points', f'/camera/points')) 
        
    # EntityWrench
    # arg_bridge.append('/world/empty/wrench@ros_gz_interfaces/msg/EntityWrench@ignition.msgs.EntityWrench')
    # remap_bridge.append(('/world/empty/wrench', '/entity_wrench'))
    
    # arg_bridge.append('/world/empty/wrench/persistent@ros_gz_interfaces/msg/EntityWrench@ignition.msgs.EntityWrench')
    # remap_bridge.append(('/world/empty/wrench/persistent', '/entity_wrench_persistent'))
    
    # arg_bridge.append('/world/empty/wrench/clear@ros_gz_interfaces/msg/Entity@ignition.msgs.Entity')
    # remap_bridge.append(('/world/empty/wrench/clear', '/entity_wrench_clear'))
        
    bridge_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=arg_bridge,
        remappings=remap_bridge,
        output='screen'
    )

    nodes = [
        gazebo_launch,
        gz_spawn_entity,
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_effort_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[bridge_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[rviz_node],
            )
        ),
        # rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)

    # ros2 topic pub /effort_controllers/commands std_msgs/msg/Float64MultiArray "data:
    #         - 10.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0
    #         - 0.0"

    # ExecuteProcess(
    #     cmd=[
    #         'ign', 'gazebo', '-r',
    #         world_path
    #     ]
    # ),

    def ros_gz_bridge(topic, ros_type, gz_type, direction):
        param = f'{topic}@{ros_type}{direction}{gz_type}'
        cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', param]
        return ExecuteProcess(cmd=cmd, output='screen')

    def ros_gz_image_bridge(topic):
        ros_type = "sensor_msgs/msg/Image"
        gz_type = "gz.msgs.Image"
        direction = "["
        return ros_gz_bridge(topic, ros_type, gz_type, direction)

    def ros_gz_gimbal_joint_cmd_bridge(topic):
        ros_type = "std_msgs/msg/Float64"
        gz_type = "gz.msgs.Double"
        direction = "]"
        return ros_gz_bridge(topic, ros_type, gz_type, direction)

    def ros_gz_gimbal_joint_state_bridge(topic):
        ros_type = "sensor_msgs/msg/JointState"
        gz_type = "gz.msgs.Model"
        direction = "["
        return ros_gz_bridge(topic, ros_type, gz_type, direction)

    def ros_gz_camera_info_bridge(topic):
        ros_type = "sensor_msgs/msg/CameraInfo"
        gz_type = "gz.msgs.CameraInfo"
        direction = "["
        return ros_gz_bridge(topic, ros_type, gz_type, direction)

    def ros_gz_pose_bridge(topic):
        ros_type = "geometry_msgs/msg/PoseStamped"
        gz_type = "gz.msgs.Pose"
        direction = "["
        return ros_gz_bridge(topic, ros_type, gz_type, direction)
