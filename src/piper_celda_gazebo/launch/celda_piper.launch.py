#!/usr/bin/env python3
"""
Lanza la celda de digitalizacion en Gazebo Harmonic con el brazo PiPER X
y sus controladores ros2_control.

Ejemplos:
    ros2 launch piper_celda_gazebo celda_piper.launch.py
    ros2 launch piper_celda_gazebo celda_piper.launch.py x:=0.0 y:=0.10 z:=0.752
    ros2 launch piper_celda_gazebo celda_piper.launch.py gui:=false
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('piper_celda_gazebo')
    pkg_desc = get_package_share_directory('piper_celda_description')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    # Pose de la base: un solo archivo, leido tambien por moveit.launch.py.
    with open(os.path.join(pkg_desc, 'config', 'pose_brazo.yaml')) as f:
        pose = yaml.safe_load(f)

    # ---------------- argumentos ----------------
    args = [
        DeclareLaunchArgument('mundo', default_value='celda_piper.sdf',
                              description='Archivo .sdf dentro de worlds/'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Abrir la GUI de Gazebo'),
        DeclareLaunchArgument('pausado', default_value='false',
                              description='Arrancar la simulacion en pausa'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Abrir RViz'),
        # Pose confirmada de la base del brazo sobre la placa estabilizadora.
        DeclareLaunchArgument('x',   default_value=str(pose['x'])),
        DeclareLaunchArgument('y',   default_value=str(pose['y'])),
        DeclareLaunchArgument('z',   default_value=str(pose['z'])),
        DeclareLaunchArgument('yaw', default_value=str(pose['yaw']),
                              description='Rotacion de la base en radianes. '
                                          'Por defecto, config/pose_brazo.yaml.'),
    ]

    mundo = LaunchConfiguration('mundo')
    gui = LaunchConfiguration('gui')
    pausado = LaunchConfiguration('pausado')

    # ---------------- recursos de Gazebo ----------------
    # Para que resuelva los model:// de mesa, microscopio_msr, escurridor_60, laminilla
    ruta_modelos = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_gazebo, 'models'))
    # Para que resuelva las mallas package:// del brazo
    ruta_paquetes = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_desc))

    ruta_mundo = PathJoinSubstitution([pkg_gazebo, 'worlds', mundo])

    # Nombre del <world> dentro del SDF: hace falta para el topico de poses.
    import xml.etree.ElementTree as _ET
    try:
        _w = _ET.parse(os.path.join(pkg_gazebo, 'worlds',
                                    'celda_piper.sdf')).getroot().find('world')
        nombre_mundo = _w.get('name', 'celda')
    except Exception:
        nombre_mundo = 'celda' 

    # gui:=false -> solo servidor.  pausado:=true -> no arrancar la fisica.
    flag_gui = PythonExpression(["'' if '", gui, "' == 'true' else ' -s'"])
    flag_run = PythonExpression(["' -r' if '", pausado, "' == 'false' else ''"])

    # ---------------- Gazebo ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                ruta_mundo,
                ' -v 3',
                flag_gui,
                flag_run,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------------- descripcion del robot ----------------
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_desc, 'urdf', 'piper_x.urdf.xacro']),
            ' usar_gazebo:=true',
            ' fijar_a_mundo:=true',
            ' base_x:=', LaunchConfiguration('x'),
            ' base_y:=', LaunchConfiguration('y'),
            ' base_z:=', LaunchConfiguration('z'),
            ' base_yaw:=', LaunchConfiguration('yaw'),
            ' controllers_file:=',
            PathJoinSubstitution([pkg_desc, 'config', 'piper_controllers.yaml']),
        ]),
        value_type=str,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ---------------- spawn del brazo ----------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'piper_x',
            # Sin offset: la pose de la base va dentro del URDF, en la junta
            # world -> base_link, para que Gazebo y MoveIt coincidan.
        ],
    )

    # ---------------- puente de reloj ----------------
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Poses de todos los modelos. Permite que la escena de MoveIt
            # siga lo que muevas en la GUI de Gazebo.
            f'/world/{nombre_mundo}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            (f'/world/{nombre_mundo}/pose/info', '/poses_gazebo'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ---------------- controladores ----------------
    # Se cargan en cadena para no pelear con el controller_manager,
    # que solo existe una vez que el plugin arranca dentro de Gazebo.
    def spawner(nombre):
        return Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[nombre, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60'],
            parameters=[{'use_sim_time': True}],
        )

    jsb = spawner('joint_state_broadcaster')
    brazo = spawner('brazo_controller')
    pinza = spawner('pinza_controller')

    tras_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb]))
    tras_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[brazo]))
    tras_brazo = RegisterEventHandler(
        OnProcessExit(target_action=brazo, on_exit=[pinza]))

    # ---------------- RViz ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [
        ruta_modelos,
        ruta_paquetes,
        gazebo,
        rsp,
        puente,
        spawn,
        tras_spawn,
        tras_jsb,
        tras_brazo,
        rviz,
    ])
