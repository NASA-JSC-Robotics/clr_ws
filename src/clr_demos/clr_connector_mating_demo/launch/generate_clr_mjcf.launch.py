import os
import tempfile

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import (
    FindPackageShare,
)


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_pregenerated_assets_dir",
            default_value="false",
            description="Use pre-generated assets dir. This is useful if you are just modifying an existing structure",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "save_only",
            default_value="true",
            description="Whether to save the mjcf (true) or to publish it to a topic (false).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "include_scene_objects",
            default_value="true",
            description="Whether to include scene objects.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "model_env",
            default_value="true",
            description="Whether to model the iMETRO environment in the MJCF.",
        )
    )

    include_scene_objects = LaunchConfiguration("include_scene_objects")
    model_env = LaunchConfiguration("model_env")

    clr_mujoco_package_name = "clr_connector_mating_demo"
    clr_mujoco_description_file = "clr_connector_demo_mujoco.xacro.urdf"

    generate_arguments = [
        "--save_only",
    ]

    mujoco_launch_arguments = [
        "--publish_topic",
        "/mujoco_robot_description",
    ]

    assets_dir = [
        "--asset_dir",
        PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "description", "assets"]),
    ]

    # Main robot description for CLR
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "urdf", clr_mujoco_description_file]),
            # Grasp frames should not be converted to MJCF objects
            " add_grasp_push_frames:=false",
            " model_env:=",
            model_env,
            " include_scene_objects:=",
            include_scene_objects,
        ]
    )

    # Using an inline opaque function to write the URDF for mujoco to a tempfile...
    # This prevents it from being dumped into the console on conversion errors.
    def launch_mjcf_node(context):
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
        tmp.write(robot_description_content.perform(context))
        tmp.close()

        # Ensure the file gets deleted
        def cleanup(event, context):
            if os.path.exists(tmp.name):
                os.remove(tmp.name)

        args = [
            "--urdf",
            tmp.name,
            "--convert_stl_to_obj",
        ]

        if LaunchConfiguration("save_only").perform(context) == "true":
            args += generate_arguments
        else:
            args += mujoco_launch_arguments
        if LaunchConfiguration("use_pregenerated_assets_dir").perform(context) == "true":
            args += assets_dir

        return [
            Node(
                package="mujoco_ros2_control",
                executable="make_mjcf_from_robot_description.py",
                output="both",
                emulate_tty=True,
                arguments=args,
            ),
            RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        ]

    generate_mjcf = OpaqueFunction(function=launch_mjcf_node)

    return LaunchDescription(declared_arguments + [generate_mjcf])
