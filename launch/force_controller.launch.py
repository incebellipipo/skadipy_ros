import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import uuid

ARGUMENTS = [
    launch.actions.DeclareLaunchArgument(
        "vessel_name", default_value="cybership", description="Vessel name"
    ),
    launch.actions.DeclareLaunchArgument(
        "param_file",
        default_value="/dev/null",
        description="Path to the parameter file",
    ),
]


def generate_launch_description():

    param_file = launch.substitutions.PathJoinSubstitution(
        [
            launch_ros.substitutions.FindPackageShare(package="skadipy_ros"),
            "config",
            "force_controller.yaml",
        ]
    )

    node_force_controller = launch_ros.actions.Node(
        namespace=launch.substitutions.LaunchConfiguration("vessel_name"),
        package="skadipy_ros",
        executable="force_controller.py",
        name=f"force_controller{str(uuid.uuid4().hex[:12])}",
        parameters=[param_file],
        output="screen",
        respawn=True,
        respawn_delay=5,
    )

    ld = launch.LaunchDescription()

    for arg in ARGUMENTS:
        ld.add_action(arg)

    ld.add_action(node_force_controller)

    return ld
