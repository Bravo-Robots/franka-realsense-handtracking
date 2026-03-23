from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")
    use_realsense = LaunchConfiguration("use_realsense")

    # 1) MoveIt oficial FR3
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("franka_fr3_moveit_config"),
                "launch",
                "moveit.launch.py",
            ])
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "use_rviz": use_rviz,
        }.items(),
    )

    # 2) Cámara + handcv (tu launch)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("handcv"),
                "launch",
                "camera.launch.py",
            ])
        ),
        launch_arguments={
            "use_realsense": use_realsense,
        }.items(),
    )

    # 3) Conectar TF: base -> camera_link (extrínseca)
    #    NOTA: ahora está a identidad; cuando calibres, cambias xyz/rpy.
    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base", "camera_link"],
    )

    # 4) Bridge (versión IK->JointTrajectory, 2D)
    bridge_node = Node(
        package="cv_franka_bridge",
        executable="cv_franka_bridge",
        name="cv_franka_bridge",
        output="screen",
        parameters=[
            {"base_frame": "base"},                # IMPORTANTE: en tu TF existe 'base'
            {"ee_frame": "fr3_hand_tcp"},          # para leer pose del EE por TF
            {"hand_frame_default": "camera_link"}, # waypoint interpretado en cámara

            {"waypoint_units": "mm"},              # tu waypoint está en mm (356, 413,...)
            {"teleop_mode": "2d"},                 # 2D (x,y) y z fija
            {"teleop_gain_xy": 1.0},             # escala mm->m (si ya haces /1000, esto puede ser 1.0)
            {"teleop_axis_map": "x,y"},            # mano x->EE x, mano y->EE y (simple)

            {"compute_ik_service": "/compute_ik"},
            {"planning_group": "fr3_arm"},
            {"ik_link_name": "fr3_link8"},
            {"ik_timeout_sec": 0.05},
            {"avoid_collisions": False},

            {"arm_traj_topic": "/fr3_arm_controller/joint_trajectory"},
            {"publish_period": 0.02},
            {"trajectory_time_from_start": 0.10},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="dont-care"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_realsense", default_value="true"),

        moveit_launch,
        camera_launch,
        base_to_camera_tf,
        bridge_node,
    ])
