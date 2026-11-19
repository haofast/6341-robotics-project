"""Launch a standalone SO-101 MoveIt planner with no hardware controller."""

from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = Path(__file__).resolve().parent
    urdf = config_dir.parent / "models/so101/so101_wrist_flex_103.urdf"
    robot = ET.parse(urdf).getroot()
    for mesh in robot.iter("mesh"):
        mesh.set("filename", (urdf.parent / mesh.attrib["filename"]).as_uri())

    semantic = ET.Element("robot", name=robot.attrib["name"])
    arm = ET.SubElement(semantic, "group", name="manipulator")
    ET.SubElement(arm, "chain", base_link="base_link", tip_link="gripper_frame_link")
    # Adjacent links share their joint geometry. Leave all other pairs enabled.
    for joint in robot.findall("joint"):
        ET.SubElement(semantic, "disable_collisions",
                      link1=joint.find("parent").attrib["link"],
                      link2=joint.find("child").attrib["link"],
                      reason="Adjacent")

    # Conservative limits for timing the planning preview, in radians/s and radians/s².
    joint_limits = {}
    for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
        prefix = f"robot_description_planning.joint_limits.{name}"
        joint_limits.update({
            f"{prefix}.has_velocity_limits": True,
            f"{prefix}.max_velocity": 0.5,
            f"{prefix}.has_acceleration_limits": True,
            f"{prefix}.max_acceleration": 1.0,
        })
    descriptions = {
        "robot_description": ParameterValue(ET.tostring(robot, encoding="unicode"), value_type=str),
        "robot_description_semantic": ParameterValue(
            ET.tostring(semantic, encoding="unicode"), value_type=str),
    }
    kinematics = {
        "robot_description_kinematics.manipulator.kinematics_solver":
            "kdl_kinematics_plugin/KDLKinematicsPlugin",
        "robot_description_kinematics.manipulator.kinematics_solver_timeout": 0.1,
    }
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(package="rviz2", executable="rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(config_dir / "so101_preview.rviz")],
             parameters=[descriptions, kinematics]),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[{
                **joint_limits,
                **descriptions,
                **kinematics,
                "allow_trajectory_execution": False,
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "ompl.planning_plugins": ["ompl_interface/OMPLPlanner"],
                "ompl.request_adapters": [
                    "default_planning_request_adapters/ResolveConstraintFrames",
                    "default_planning_request_adapters/ValidateWorkspaceBounds",
                    "default_planning_request_adapters/CheckStartStateBounds",
                    "default_planning_request_adapters/CheckStartStateCollision",
                ],
                "ompl.response_adapters": [
                    "default_planning_response_adapters/AddTimeOptimalParameterization",
                    "default_planning_response_adapters/ValidateSolution",
                ],
                "ompl.planner_configs.RRTConnect.type": "geometric::RRTConnect",
                "ompl.manipulator.planner_configs": ["RRTConnect"],
                "ompl.manipulator.longest_valid_segment_fraction": 0.005,
            }],
        ),
    ])
