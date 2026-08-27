#!/usr/bin/env python3
"""
Lanza move_group y RViz para el PiPER X en la celda.

Requiere que Gazebo ya este corriendo con los controladores activos:
    ros2 launch piper_celda_gazebo celda_piper.launch.py

y despues, en otra terminal:
    ros2 launch piper_celda_moveit moveit.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_moveit = get_package_share_directory("piper_celda_moveit")
    # Ruta absoluta: en el layout de instalacion cada paquete vive en su
    # propio install/<pkg>/share/<pkg>, asi que una ruta relativa desde
    # piper_celda_moveit no llega a piper_celda_description.
    pkg_desc = get_package_share_directory("piper_celda_description")
    xacro = os.path.join(pkg_desc, "urdf", "piper_x.urdf.xacro")

    # La misma pose que usa Gazebo. Si esto no coincidiera, MoveIt planificaria
    # con el brazo en una orientacion distinta a la simulada y todos los planes
    # serian invalidos sin que nada diera error.
    with open(os.path.join(pkg_desc, "config", "pose_brazo.yaml")) as f:
        pose = yaml.safe_load(f)

    cfg = (
        MoveItConfigsBuilder("piper_x", package_name="piper_celda_moveit")
        .robot_description(
            file_path=xacro,
            mappings={
                "usar_gazebo": "true",
                "fijar_a_mundo": "true",
                "base_x": str(pose["x"]),
                "base_y": str(pose["y"]),
                "base_z": str(pose["z"]),
                "base_yaw": str(pose["yaw"]),
            },
        )
        .robot_description_semantic(file_path="config/piper_x.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            cfg.to_dict(),
            {"use_sim_time": True},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(pkg_moveit, "config", "moveit.rviz")],
        parameters=[
            cfg.robot_description,
            cfg.robot_description_semantic,
            cfg.robot_description_kinematics,
            cfg.planning_pipelines,
            cfg.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # La escena se publica una sola vez, cuando move_group ya expone el
    # servicio apply_planning_scene. Antes de eso el script fallaria.
    escena = TimerAction(
        period=LaunchConfiguration("retardo"),
        actions=[
            Node(
                package="piper_celda_moveit",
                executable="publicar_escena_celda.py",
                output="screen",
                condition=IfCondition(LaunchConfiguration("escena")),
                parameters=[{
                    "use_sim_time": True,
                    # sigue en vivo lo que muevas en la GUI de Gazebo
                    "seguir": LaunchConfiguration("seguir"),
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("escena", default_value="true"),
        DeclareLaunchArgument("retardo", default_value="10.0"),
        DeclareLaunchArgument("seguir", default_value="true"),
        move_group,
        rviz,
        escena,
    ])
