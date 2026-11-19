#!/usr/bin/env python3
"""Prepare or plan a homework-style SO-101 pickup for a taught board cell."""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
SETUP = ROOT / "setup-data/3x6"
URDF = ROOT / "models/so101/so101_wrist_flex_103.urdf"


def prepare(args, key, output):
    import numpy as np
    from lerobot.model.kinematics import RobotKinematics
    from pick_block import plan_free_wrist_lift, plan_pick
    from trace_perimeter import JOINTS, joint_limits, joint_vector, plan_line, read_pose

    poses = json.loads(args.pickup_poses.read_text())
    if key not in poses:
        raise ValueError(f"Cell ({key}) needs a taught grasp. Run pick_block.py with --teach-grasp.")
    limits = joint_limits(URDF)
    model = RobotKinematics(str(URDF), target_frame_name="gripper_frame_link", joint_names=list(JOINTS))
    selected = poses[key].copy()
    if args.lift_mm is not None:
        selected["lift_mm"] = args.lift_mm
    standoff, _, _, descent, lift = plan_pick(selected, model, limits)
    _, grasp_point, grasp = descent[-1]
    retreat = plan_free_wrist_lift(grasp, lift[-1][1][2] - grasp_point[2], model, limits)
    if args.current_start:
        start = joint_vector(read_pose(args.port, args.robot_id))
        print("Using measured arm joints as the MoveIt start state.")
    else:
        start = joint_vector(json.loads(args.corner_poses.read_text())["origin"])
    joint_targets = {"origin": start, "standoff": standoff, "grasp": grasp, "lift": retreat[-1][2]}
    destination = None
    if args.drop is not None:
        destination = f"{args.drop[0]},{args.drop[1]}"
        if destination not in poses:
            raise ValueError(f"Destination cell ({destination}) needs a taught grasp.")
        placed = poses[destination].copy()
        if args.lift_mm is not None:
            placed["lift_mm"] = args.lift_mm
        height_mm = args.drop_layer * 25.4 + args.drop_raise_mm
        if height_mm or args.drop_row_offset or args.drop_column_offset:
            clearance = float(placed.get("lift_mm", 40)) - height_mm
            if clearance < 5:
                raise ValueError("Increase --lift-mm to provide at least 5 mm above the stack placement.")
            placed["lift_mm"] = clearance
            base_joints = joint_vector(placed["grasp"])
            base_pose = model.forward_kinematics(base_joints)
            correction = np.zeros(3)
            board = json.loads(args.workspace.read_text())["board"]
            for axis, offset, count in (
                    (0, args.drop_row_offset, board["rows"]),
                    (1, args.drop_column_offset, board["columns"])):
                if not offset:
                    continue
                before = list(args.drop)
                after = list(args.drop)
                before[axis] = max(0, min(before[axis], count - 2))
                after[axis] = before[axis] + 1
                neighbor_keys = [f"{cell[0]},{cell[1]}" for cell in (before, after)]
                if any(cell not in poses for cell in neighbor_keys):
                    raise ValueError(f"Offset requires taught neighboring cells {neighbor_keys}.")
                points = [model.forward_kinematics(joint_vector(poses[cell]["grasp"]))[:3, 3]
                          for cell in neighbor_keys]
                correction[:2] += offset * (points[1] - points[0])[:2]
            correction[2] = height_mm / 1000
            stack_top = base_pose[:3, 3] + correction
            route = plan_line(0, "stack placement", base_pose[:3, 3], stack_top,
                              base_joints, model, limits, fixed_rotation=base_pose[:3, :3])
            stacked_joints = route[-1][2]
            stacked_pose = model.forward_kinematics(stacked_joints)
            angle = np.arccos(np.clip((np.trace(base_pose[:3, :3].T @ stacked_pose[:3, :3]) - 1) / 2, -1, 1))
            if angle > np.deg2rad(3):
                raise ValueError("Stack placement changes the taught wrist orientation by more than 3 degrees.")
            placed["grasp"] = dict(placed["grasp"])
            placed["grasp"].update({f"{name}.pos": float(value)
                                   for name, value in zip(JOINTS, stacked_joints)})
            print(f"Destination ({destination}) layer {args.drop_layer}: "
                  f"placement raised {height_mm:g} mm above its taught board pose; "
                  f"XY correction {correction[0]*1000:+.1f}, {correction[1]*1000:+.1f} mm.")
        above, _, _, down, _ = plan_pick(placed, model, limits)
        joint_targets.update(drop_standoff=above, place=joint_vector(placed["grasp"]))
    for name, joints in joint_targets.items():
        if np.any(joints < limits[0]) or np.any(joints > limits[1]):
            raise ValueError(f"{name} target is outside the URDF joint limits.")
    output.write_text(json.dumps({
        "cell": key,
        "destination": destination,
        "drop_layer": args.drop_layer,
        "joint_names": JOINTS,
        "targets_rad": {name: np.deg2rad(q).tolist() for name, q in joint_targets.items()},
        "targets_xyz_m": {name: model.forward_kinematics(q)[:3, 3].tolist()
                          for name, q in joint_targets.items()},
    }, indent=2, allow_nan=False) + "\n")
    print(f"Prepared ({key}) targets for MoveIt at {output}.")


def plan(args, key, source):
    import rclpy
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
    from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK
    from moveit_msgs.msg import DisplayTrajectory
    from rclpy.qos import QoSProfile, DurabilityPolicy

    targets = json.loads(source.read_text())
    if targets["cell"] != key:
        raise ValueError(f"Target file contains ({targets['cell']}), expected ({key}).")
    jaw = None
    if args.export_grasp:
        pose = json.loads(args.pickup_poses.read_text())[key]
        opened = float(pose["gripper_open"] if "gripper_open" in pose else pose["hover"]["gripper.pos"])
        closed = float(pose["grasp"]["gripper.pos"])
        if not 0 <= closed < opened <= 100:
            raise ValueError("Invalid taught gripper openings. Expected 0 <= grasp < open <= 100.")
        # LeRobot uses a calibrated 0–100 opening scale for the gripper.
        jaw = {"open": opened, "close": max(0.0, closed - 2.0)}
    names = targets["joint_names"]
    q = targets["targets_rad"]
    root = ET.parse(URDF).getroot()
    expected = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}
    if len(names) != len(expected) or set(names) != expected:
        raise ValueError("Target file must contain the five SO-101 arm joints.")
    bounds = {j.attrib["name"]: j.find("limit").attrib for j in root.findall("joint")
              if j.attrib["name"] in expected}
    for label in q:
        values = q[label]
        if len(values) != len(names) or any(
                not math.isfinite(v) or not float(bounds[n]["lower"]) <= v <= float(bounds[n]["upper"])
                for n, v in zip(names, values)):
            raise ValueError(f"Invalid joint target for {label}.")

    def state(positions, jaw_deg):
        msg = RobotState()
        msg.joint_state.name = names + ["gripper"]
        msg.joint_state.position = list(positions) + [math.radians(jaw_deg)]
        return msg

    def endpoint(trajectory, label):
        path = trajectory.joint_trajectory
        if not path.points or len(path.joint_names) != len(names) or set(path.joint_names) != expected:
            raise RuntimeError(f"{label} returned an empty or mismatched arm trajectory.")
        if len(path.points[-1].positions) != len(names) or not all(
                math.isfinite(value) for value in path.points[-1].positions):
            raise RuntimeError(f"{label} returned an invalid endpoint.")
        joints = dict(zip(path.joint_names, path.points[-1].positions))
        print(f"{label}: {len(path.points)} planned points.", flush=True)
        return [joints[name] for name in names]

    rclpy.init()
    node = None
    try:
        node = rclpy.create_node("so101_board_pick_preview")

        def client(service_type, name):
            connection = node.create_client(service_type, name)
            if not connection.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"{name} unavailable. Start the SO-101 MoveIt planning launch first.")
            return connection

        def call(connection, request):
            future = connection.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
            if not future.done():
                connection.remove_pending_request(future)
                raise RuntimeError(f"{connection.srv_name} timed out.")
            return future.result()

        def check(code, label):
            if code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"MoveIt {label} failed (error {code.val}).")

        motion = client(GetMotionPlan, "/plan_kinematic_path")
        fk = client(GetPositionFK, "/compute_fk")
        cartesian = client(GetCartesianPath, "/compute_cartesian_path")

        def tool_pose(positions, jaw_deg):
            request = GetPositionFK.Request()
            request.header.frame_id = "base_link"
            request.fk_link_names = ["gripper_frame_link"]
            request.robot_state = state(positions, jaw_deg)
            reply = call(fk, request)
            check(reply.error_code, "forward kinematics")
            if len(reply.pose_stamped) != 1:
                raise RuntimeError("MoveIt did not return the tool pose.")
            return reply.pose_stamped[0].pose

        trajectories = []

        def move_to_joints(label, start, target, jaw_deg):
            request = GetMotionPlan.Request()
            spec = request.motion_plan_request
            spec.group_name = args.group
            spec.start_state = state(start, jaw_deg)
            spec.allowed_planning_time = 5.0
            spec.num_planning_attempts = 3
            spec.max_velocity_scaling_factor = 0.1
            spec.max_acceleration_scaling_factor = 0.1
            goal = Constraints()
            for name, position in zip(names, target):
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = position
                joint.tolerance_above = joint.tolerance_below = math.radians(0.05 if label in ("Standoff", "Transfer", "Place", "Retreat") else 0.5)
                joint.weight = 1.0
                goal.joint_constraints.append(joint)
            spec.goal_constraints.append(goal)
            reply = call(motion, request).motion_plan_response
            check(reply.error_code, label)
            trajectories.append(reply.trajectory)
            return endpoint(reply.trajectory, label)

        grasp_pose = tool_pose(q["grasp"], args.mock_open_deg)
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.group_name = args.group
        request.link_name = "gripper_frame_link"
        request.waypoints = [grasp_pose]
        request.max_step = 0.002
        request.avoid_collisions = True
        if hasattr(request, "revolute_jump_threshold"):
            request.revolute_jump_threshold = math.radians(5)
        else:
            request.jump_threshold = 5.0
        for setting in ("max_velocity_scaling_factor", "max_acceleration_scaling_factor"):
            if hasattr(request, setting):
                setattr(request, setting, 0.1)
        for attempt in range(1, 4):
            standoff = move_to_joints("Standoff", q["origin"], q["standoff"], args.mock_open_deg)
            request.start_state = state(standoff, args.mock_open_deg)
            reply = call(cartesian, request)
            check(reply.error_code, "descent")
            if 0.999 <= reply.fraction <= 1.0:
                break
            trajectories.pop()
            error = max(abs(actual - target) for actual, target in zip(standoff, q["standoff"]))
            print(f"Attempt {attempt}/3: descent reached {reply.fraction:.1%}; "
                  f"standoff joint error {math.degrees(error):.3f}°.", flush=True)
        if not 0.999 <= reply.fraction <= 1.0:
            raise RuntimeError(f"MoveIt completed {reply.fraction:.1%} of the descent. Lift planning skipped.")
        grasp = endpoint(reply.solution, "Descent")
        trajectories.append(reply.solution)
        lift_jaw = args.mock_open_deg if args.export_empty else args.mock_closed_deg
        raised = move_to_joints("Lift", grasp, q["lift"], lift_jaw)
        reached_grasp = tool_pose(grasp, lift_jaw)
        raised_pose = tool_pose(raised, lift_jaw)
        rise = raised_pose.position.z - reached_grasp.position.z
        requested_rise = targets["targets_xyz_m"]["lift"][2] - targets["targets_xyz_m"]["grasp"][2]
        if not math.isfinite(rise) or rise <= 0 or abs(rise - requested_rise) > 0.006:
            raise RuntimeError(f"Planned rise {rise * 1000:.1f} mm differs from the "
                               f"requested {requested_rise * 1000:.1f} mm.")
        labels = ["Standoff", "Descent", "Lift"]
        if targets.get("destination") is not None:
            transferred = move_to_joints("Transfer", raised, q["drop_standoff"], lift_jaw)
            placed = move_to_joints("Place", transferred, q["place"], lift_jaw)
            joint_error = max(abs(actual - target) for actual, target in zip(placed, q["place"]))
            if joint_error > math.radians(0.2):
                raise RuntimeError("Placement plan does not reach the taught destination joints.")
            above_pose = tool_pose(transferred, lift_jaw)
            placed_pose = tool_pose(placed, lift_jaw)
            lower_mm = (above_pose.position.z - placed_pose.position.z) * 1000
            if not math.isfinite(lower_mm) or lower_mm < 5:
                raise RuntimeError("Destination approach does not lower at least 5 mm to the taught grasp.")
            print(f"Placement lowers {lower_mm:.1f} mm to the taught board-level pose "
                  f"at ({targets['destination']}), layer {targets.get('drop_layer', 0)}.", flush=True)
            move_to_joints("Retreat", placed, q["drop_standoff"], args.mock_open_deg)
            labels.extend(["Transfer", "Place", "Retreat"])
        print(f"Planned tool rise: {rise * 1000:.1f} mm.")
        print(f"MoveIt planning preview passed for ({key}). No hardware commands sent.")
        export_path = args.export_empty or args.export_grasp
        if export_path:
            stages = []
            for label, trajectory in zip(labels, trajectories):
                path = trajectory.joint_trajectory
                stages.append({
                    "stage": label,
                    "joint_names": list(path.joint_names),
                    "points": [{"seconds": point.time_from_start.sec + point.time_from_start.nanosec / 1e9,
                                "rad": list(point.positions)} for point in path.points],
                })
            data = {"cell": key, "drop_layer": targets.get("drop_layer", 0), "destination": targets.get("destination"), "empty_gripper": jaw is None, "stages": stages}
            if jaw is not None:
                data["gripper"] = jaw
            with export_path.open("x") as output:
                json.dump(data, output, allow_nan=False)
            print(f"Saved {'empty-gripper' if jaw is None else 'grasp'} trajectory to {export_path}.")
        if args.display:
            publisher = node.create_publisher(
                DisplayTrajectory, "/display_planned_path",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            message = DisplayTrajectory()
            message.model_id = root.attrib["name"]
            message.trajectory_start = state(q["origin"], args.mock_open_deg)
            message.trajectory = trajectories
            publisher.publish(message)
            print("Published preview to /display_planned_path. Open RViz and press Ctrl+C when done.",
                  flush=True)
            # Volatile subscribers joining late need a fresh copy.
            timer = node.create_timer(2.0, lambda: publisher.publish(message))
            try:
                rclpy.spin(node)
            finally:
                node.destroy_timer(timer)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("row", type=int)
    parser.add_argument("column", type=int)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--prepare", action="store_true", help="calculate targets using the local LeRobot model")
    modes.add_argument("--plan", action="store_true", help="request three MoveIt plans in a ROS 2 environment")
    parser.add_argument("--workspace", type=Path, default=SETUP / "workspace.json")
    parser.add_argument("--pickup-poses", type=Path, default=SETUP / "pickup_poses.json")
    parser.add_argument("--corner-poses", type=Path, default=SETUP / "perimeter_poses.json")
    parser.add_argument("--drop", nargs=2, type=int, metavar=("ROW", "COLUMN"),
                        help="place at a taught destination cell")
    parser.add_argument("--drop-row-offset", type=float, default=0, help="placement correction along taught rows, in cells")
    parser.add_argument("--drop-column-offset", type=float, default=0, help="placement correction along taught columns, in cells")
    parser.add_argument("--drop-raise-mm", type=float, default=0, help="extra release height above the selected layer")
    parser.add_argument("--drop-layer", type=int, default=0,
                        help="number of existing 1-inch cubes below the placed cube (default 0)")
    parser.add_argument("--lift-mm", type=float, help="clearance above source and destination grasps")
    parser.add_argument("--targets", type=Path, help="path to the intermediate target JSON")
    parser.add_argument("--current-start", action="store_true",
                        help="with --prepare, read the arm's current pose over USB")
    parser.add_argument("--port", default="/dev/tty.usbmodem5B3E0906941")
    parser.add_argument("--robot-id", default="my_awesome_follower_arm")
    parser.add_argument("--group", default="manipulator")
    parser.add_argument("--display", action="store_true", help="publish the validated plan for RViz")
    exports = parser.add_mutually_exclusive_group()
    exports.add_argument("--export-grasp", type=Path, help="export a plan with taught gripper commands")
    exports.add_argument("--export-empty", type=Path, help="export a plan with the jaw open throughout")
    parser.add_argument("--mock-open-deg", type=float, default=45.0, help="simulated jaw joint angle")
    parser.add_argument("--mock-closed-deg", type=float, default=15.0, help="simulated jaw joint angle")
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (
            args.drop_row_offset, args.drop_column_offset, args.drop_raise_mm)):
        parser.error("Placement offsets must be finite.")
    if args.prepare and args.drop is None and any((
            args.drop_row_offset, args.drop_column_offset, args.drop_raise_mm)):
        parser.error("Placement offsets require --drop.")
    if args.drop_layer < 0 or (args.drop_layer and args.drop is None and args.prepare):
        parser.error("--drop-layer must be nonnegative and requires --drop during preparation")
    if args.current_start and not args.prepare:
        parser.error("--current-start requires --prepare")
    if (args.display or args.export_empty or args.export_grasp) and not args.plan:
        parser.error("--display and trajectory export require --plan")
    try:
        board = json.loads(args.workspace.read_text())["board"]
        if not (0 <= args.row < board["rows"] and 0 <= args.column < board["columns"]):
            raise ValueError(f"Cell ({args.row},{args.column}) is outside the board.")
        if args.drop is not None and not (
                0 <= args.drop[0] < board["rows"] and 0 <= args.drop[1] < board["columns"]):
            raise ValueError("Destination cell is outside the board.")
        if args.lift_mm is not None and (not math.isfinite(args.lift_mm) or args.lift_mm < 5):
            raise ValueError("--lift-mm must be finite and at least 5.")
        key = f"{args.row},{args.column}"
        targets = args.targets or Path(f"/tmp/so101-pick-{args.row}-{args.column}.json")
        if args.prepare:
            prepare(args, key, targets)
        else:
            if not -10 <= args.mock_closed_deg < args.mock_open_deg <= 100:
                raise ValueError("Mock jaw angles must be ordered within the model range of -10° to 100°.")
            plan(args, key, targets)
    except KeyboardInterrupt:
        return
    except ImportError as error:
        environment = "the LeRobot environment" if args.prepare else "a sourced ROS 2 environment with MoveIt 2"
        parser.exit(1, f"Run this step in {environment}: {error}\n")
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.exit(1, f"MoveIt pickup preview failed: {error}\n")


if __name__ == "__main__":
    main()
