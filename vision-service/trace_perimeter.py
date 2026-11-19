#!/usr/bin/env python3
"""Teach or trace a three-row workspace with an unloaded SO-101."""

import argparse
from itertools import groupby
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
import xml.etree.ElementTree as ET

import numpy as np


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
CORNERS = ("origin", "positive_x", "opposite", "positive_y")
STEP_M = 0.002
IK_WAYPOINT_SPACING_M = 0.001
MAX_JOINT_STEP_DEG = 2.0
TRACK_DEG = 1.5
RELAXED_CELL_TRACK_DEG = 3.0
RELAXED_CELL_POSITION_ERROR_M = 0.010
ORIGIN_TOLERANCE_DEG = 3.0
POSITION_ERROR_M = 0.004
COMMAND_PERIOD_S = 0.01
COARSE_COMMAND_PERIOD_S = 0.04
MIN_MOVE_DURATION_S = 0.90
TRACK_PERIOD_S = 0.05
PATH_STOP_ERROR_M = 0.010
ENDPOINT_SETTLE_S = 2.0
MAX_COMMAND_LEAD_DEG = 4.0
JOINT_SPEED_DEG_S = 30.0
ORIGIN_SPEED_DEG_S = 10.0
EDGE_SPEED_M_S = 0.035


def joint_vector(observation):
    values = np.array([float(observation[f"{joint}.pos"]) for joint in JOINTS])
    if not np.isfinite(values).all():
        raise ValueError("Joint positions must be finite values in degrees.")
    return values


def joint_limits(urdf):
    joints = {element.attrib["name"]: element for element in ET.parse(urdf).getroot().findall("joint")}
    limits = []
    for name in JOINTS:
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"The URDF has no limits for {name}.")
        limits.append([float(limit.attrib["lower"]), float(limit.attrib["upper"])])
    return np.rad2deg(limits).T


def plan_perimeter(board, taught, model, limits, edge_tolerance_mm=15.0, *,
                   taught_perimeter=False, raised_start=None, raise_mm=0.0, visit_cells=False,
                   first_cell_only=False, first_row_only=False, metric_grid=False):
    seed = taught[0] if raised_start is None else raised_start
    lower, upper = limits
    if np.any(taught < lower) or np.any(taught > upper):
        raise ValueError("A taught pose is outside the URDF joint limits.")

    targets = np.array([model.forward_kinematics(joints).copy() for joints in taught])
    points = targets[:, :3, 3].copy()
    if not np.isfinite(points).all():
        raise ValueError("The robot model returned a non-finite position for a taught corner.")
    height_spread = np.ptp(points[:, 2])
    print(f"Taught hover heights span {height_spread * 1000:.1f} mm. "
          f"Planning at the origin hover height plus {raise_mm:g} mm.")
    points[:, 2] = points[0, 2] + raise_mm / 1000
    closed = np.vstack((points, points[0]))
    edges = np.diff(closed, axis=0)
    lengths = np.linalg.norm(edges, axis=1)
    square_mm = float(board["square_size_mm"])
    width = float(board.get("width_mm", board["columns"] * square_mm)) / 1000
    height = float(board.get("height_mm", board["rows"] * square_mm)) / 1000
    if not all(math.isfinite(value) and value > 0 for value in (width, height)):
        raise ValueError("Workspace width and height must be positive and finite.")
    if not math.isfinite(edge_tolerance_mm) or edge_tolerance_mm < 0:
        raise ValueError("Edge tolerance must be finite and non-negative.")
    expected = (width, height, width, height)
    next_edges = np.roll(edges, -1, axis=0)
    turns = edges[:, 0] * next_edges[:, 1] - edges[:, 1] * next_edges[:, 0]
    if np.any(lengths < STEP_M) or not (np.all(turns > 0) or np.all(turns < 0)):
        raise ValueError("Taught points must form a convex perimeter in corner order.")
    if not (taught_perimeter or metric_grid) and not np.allclose(lengths, expected, atol=edge_tolerance_mm / 1000, rtol=0):
        measured = ", ".join(
            f"{corner} to {CORNERS[(index + 1) % len(CORNERS)]}: {length * 1000:.1f} mm "
            f"({(length - nominal) * 1000:+.1f} mm)"
            for index, (corner, length, nominal) in enumerate(zip(CORNERS, lengths, expected))
        )
        raise ValueError(
            f"Modeled horizontal edges do not match the {width * 1000:.1f} × {height * 1000:.1f} mm "
            f"workspace ({edge_tolerance_mm:g} mm tolerance per edge). {measured}."
        )
    right_angles = np.sum(edges * next_edges, axis=1)
    if not (taught_perimeter or metric_grid) and np.any(abs(right_angles) > 0.15 * lengths * np.roll(lengths, -1)):
        raise ValueError("Taught corners do not form a rectangle in the expected order.")
    if taught_perimeter or metric_grid:
        measured = ", ".join(f"{length * 1000:.1f}" for length in lengths)
        print(f"Taught horizontal edges (model mm): {measured}.")

    if visit_cells:
        origin, positive_x, opposite, positive_y = points
        rows, columns = board["rows"], board["columns"]
        if metric_grid:
            # Keep the taught origin and column direction. Use the row-side
            # corner to choose which side of that axis contains the board.
            x_axis = positive_x - origin
            x_axis /= np.linalg.norm(x_axis)
            y_axis = np.array([-x_axis[1], x_axis[0], 0.0])
            if np.dot(y_axis, positive_y - origin) < 0:
                y_axis = -y_axis
            pitch = square_mm / 1000
            positive_x = origin + columns * pitch * x_axis
            positive_y = origin + rows * pitch * y_axis
            opposite = positive_x + rows * pitch * y_axis
            offsets = np.linalg.norm(np.array([origin, positive_x, opposite, positive_y]) - points, axis=1)
            print(f"Using {square_mm:g} mm square cells anchored to the taught origin and column direction. "
                  f"Largest corner correction: {offsets.max() * 1000:.1f} mm.")
        centers = [origin]
        for row in range(rows):
            v = (row + 0.5) / rows
            left = (1 - v) * origin + v * positive_y
            right = (1 - v) * positive_x + v * opposite
            for column in range(columns):
                u = (column + 0.5) / columns
                centers.append((1 - u) * left + u * right)
        if first_cell_only:
            centers = centers[:2]
        elif first_row_only:
            centers = centers[:columns + 1]
        closed = np.array(centers)
        print(f"Planning {len(centers) - 1} cell centers from (0, 0) "
              f"through {divmod(len(centers) - 2, columns)}.")
        for index, center in enumerate(centers[1:]):
            xyz = ", ".join(f"{value * 1000:.1f}" for value in center)
            print(f"  Cell {divmod(index, columns)} target (model mm): {xyz}")

    route = []
    for edge, (start, end) in enumerate(zip(closed, closed[1:]), start=1):
        label = f"Edge {edge}"
        if visit_cells:
            row, column = divmod(edge - 1, board["columns"])
            label = f"Cell ({row}, {column})"
        segment = plan_line(edge, label, start, end, seed, model, limits)
        route.extend(segment)
        seed = segment[-1][2]
    return route


def plan_line(edge, label, start, end, seed, model, limits, *, final_joints=None,
              orientation_target=None, fixed_rotation=None, orientation_weight=None):
    lower, upper = limits
    delta = end - start
    count = max(1, math.ceil(np.linalg.norm(delta) / IK_WAYPOINT_SPACING_M))
    segment = []
    initial_joints = seed
    if orientation_target is None:
        orientation_target = final_joints
    if orientation_weight is None:
        orientation_weight = 0.1 if fixed_rotation is not None else 0.01
    for step in range(1, count + 1):
        fraction = step / count
        point = start + delta * fraction
        if step == count and final_joints is not None:
            # Finish at the original joint target so the preplanned row can
            # start from exactly the same configuration.
            joints = final_joints
        else:
            orientation_joints = seed
            if orientation_target is not None:
                orientation_joints = initial_joints + fraction * (orientation_target - initial_joints)
            pose = model.forward_kinematics(orientation_joints).copy()
            pose[:3, 3] = point
            if fixed_rotation is not None:
                pose[:3, :3] = fixed_rotation
            joints = model.inverse_kinematics(seed, pose, orientation_weight=orientation_weight, max_iters=16)
        if not np.isfinite(joints).all() or np.any(joints < lower) or np.any(joints > upper):
            raise ValueError(f"{label}, waypoint {step}: IK solution is outside joint limits.")
        if np.max(abs(joints - seed)) > MAX_JOINT_STEP_DEG:
            raise ValueError(f"{label}, waypoint {step}: joint step is too large.")
        actual = model.forward_kinematics(joints)[:3, 3]
        if not np.isfinite(actual).all() or np.linalg.norm(actual - point) > POSITION_ERROR_M:
            raise ValueError(f"{label}, waypoint {step}: gripper target is unreachable within 4 mm.")
        segment.append((edge, point, joints))
        seed = joints
    return segment


def plan_taught_row(columns, start, row_joints, model, limits):
    lower, upper = limits
    if np.any(start < lower) or np.any(start > upper):
        raise ValueError("The taught origin is outside the URDF joint limits.")
    if np.any(row_joints < lower) or np.any(row_joints > upper):
        raise ValueError("A taught row endpoint is outside the URDF joint limits.")
    first, last = [model.forward_kinematics(joints)[:3, 3].copy() for joints in row_joints]
    span = np.linalg.norm((last - first)[:2])
    if not math.isfinite(span) or span < STEP_M:
        raise ValueError("The taught first-row endpoints must be distinct.")
    print(f"Taught first-row span: {span * 1000:.1f} mm in the model. "
          f"Endpoint height difference: {(last[2] - first[2]) * 1000:+.1f} mm.")
    centers = np.linspace(first, last, columns)
    point = model.forward_kinematics(start)[:3, 3].copy()
    route = []
    orientation_targets = np.linspace(row_joints[0], row_joints[1], columns)
    for column, (target, orientation_target) in enumerate(zip(centers, orientation_targets)):
        label = f"cell (0, {column})"
        print(f"  {label} target (model mm): " + ", ".join(f"{v * 1000:.1f}" for v in target))
        final_joints = orientation_target if column in (0, columns - 1) else None
        segment = plan_line(column + 1, label, point, target, start, model, limits,
                            final_joints=final_joints, orientation_target=orientation_target)
        route.extend(segment)
        point, start = segment[-1][1:]
    return route


def raised_origin(origin, model, limits, raise_mm):
    if not raise_mm:
        return origin
    lower, upper = limits
    pose = model.forward_kinematics(origin).copy()
    pose[2, 3] += raise_mm / 1000
    joints = model.inverse_kinematics(origin, pose, orientation_weight=0.01, max_iters=16)
    if not np.isfinite(joints).all() or np.any(joints < lower) or np.any(joints > upper):
        raise ValueError("The raised origin is outside the modeled joint limits.")
    actual = model.forward_kinematics(joints)[:3, 3]
    if not np.isfinite(actual).all() or np.linalg.norm(actual - pose[:3, 3]) > 0.0005:
        raise ValueError("The solver could not place the gripper within 0.5 mm of the raised origin.")
    return joints


def follower(port, robot_id):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    return SO101Follower(SO101FollowerConfig(
        port=port, id=robot_id, use_degrees=True, max_relative_target=None,
        disable_torque_on_disconnect=False,
    ))


def read_pose(port, robot_id, *, include_gripper=False):
    robot = follower(port, robot_id)
    # Open just the serial bus to read the current pose without configuring motors.
    try:
        robot.bus.connect()
        if not robot.is_calibrated:
            raise ValueError(f"The follower calibration does not match {robot_id}.")
        positions = robot.bus.sync_read("Present_Position")
        pose = {f"{name}.pos": positions[name] for name in JOINTS}
        result = dict(zip(pose, joint_vector(pose).tolist()))
        if include_gripper:
            gripper = float(positions["gripper"])
            if not math.isfinite(gripper) or not 0 <= gripper <= 100:
                raise ValueError("Gripper reading is outside its calibrated 0–100 range.")
            result["gripper.pos"] = gripper
        return result
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(False)


def release_torque(robot):
    robot.bus.disable_torque(num_retry=2)
    print("Motor torque disabled. Support the arm while repositioning it.")


def hold_after_failure(robot):
    try:
        current = joint_vector(robot.get_observation())
        robot.send_action({f"{name}.pos": float(value) for name, value in zip(JOINTS, current)})
        print("Sent a hold command at the measured pose after the failed move.", file=sys.stderr)
    except Exception as error:
        print(f"Could not send a hold command: {error}", file=sys.stderr)


def release_after_failure(robot):
    hold_after_failure(robot)
    if not sys.stdin.isatty():
        print("Torque may still be enabled. Support the arm and use --release-torque when ready.", file=sys.stderr)
        return
    try:
        answer = input("Support the arm before releasing torque. Type release to disable torque, "
                       "or press Enter to close the serial port: ")
        if answer.strip().lower() == "release":
            release_torque(robot)
        else:
            print("Torque was left enabled. Use --release-torque when ready.", file=sys.stderr)
    except (Exception, KeyboardInterrupt) as error:
        print(f"Torque release was not confirmed. Support the arm before switching off motor power. {error}",
              file=sys.stderr)


def save_poses(path, poses):
    with TemporaryDirectory(dir=path.parent) as directory:
        staged = Path(directory) / path.name
        staged.write_text(json.dumps(poses, indent=2) + "\n")
        staged.replace(path)


def record_corner(path, corner, port, robot_id):
    poses = json.loads(path.read_text()) if path.exists() else {}
    poses[corner] = read_pose(port, robot_id)
    save_poses(path, poses)
    print(f"Recorded {corner} in {path}")


def teach_first_row(path, columns, port, robot_id):
    print("Use the same point on the empty gripper and the same hover height at both cell centers.")
    poses = {}
    for name, column in (("first", 0), ("last", columns - 1)):
        input(f"Center the follower over cell (0, {column}), close teleop, and press Enter to read: ")
        poses[name] = read_pose(port, robot_id)
        print(f"Captured cell (0, {column}).")
    save_poses(path, {"columns": columns, "poses": poses})
    print(f"Saved first-row endpoints to {path}.")


def draw_teaching_view(cv2, frame, corners, reference_size, index):
    height, width = frame.shape[:2]
    if not math.isclose(width / height, reference_size[0] / reference_size[1], rel_tol=0.005):
        raise ValueError("Camera aspect ratio differs from the saved setup. Run the overhead setup again.")
    scale = min(1.0, 1100 / width, 750 / height)
    size = (round(width * scale), round(height * scale))
    view = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
    pixels = np.rint(corners * (np.array(size) / reference_size)).astype(np.int32)
    cv2.polylines(view, [pixels], True, (0, 255, 255), 2, cv2.LINE_AA)
    point = tuple(pixels[index])
    cv2.circle(view, point, 14, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.circle(view, point, 14, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.drawMarker(view, point, (0, 255, 0), cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
    # Put the instructions below the image so they cannot cover a corner.
    view = cv2.copyMakeBorder(view, 0, 58, 0, 0, cv2.BORDER_CONSTANT)
    cv2.putText(view, f"{index + 1}/4  {CORNERS[index]}", (10, size[1] + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(view, "Close teleop, then Enter to capture | Q to cancel", (10, size[1] + 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    return view


def teach_corners(path, port, robot_id, config):
    import cv2

    board = config["board"]
    corners = np.array(board["top_down_corners_px"], dtype=float)
    reference_size = np.array(board["top_down_frame_size_px"], dtype=float)
    if (corners.shape != (4, 2) or not np.isfinite(corners).all()
            or reference_size.shape != (2,) or not np.isfinite(reference_size).all()
            or np.any(reference_size <= 0) or np.any(corners < 0) or np.any(corners >= reference_size)):
        raise ValueError("Invalid overhead corner coordinates or frame size in workspace.json.")

    camera_index = config["cameras"]["top_down"]
    camera = cv2.VideoCapture(camera_index)
    window = "Teach perimeter corners"
    poses = {}
    try:
        if not camera.isOpened():
            raise RuntimeError(f"Could not open overhead camera {camera_index}.")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, reference_size[0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, reference_size[1])
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        try:
            print(f"Check that the outline matches the selected {board['rows']}x{board['columns']} workspace. "
                  "If it has shifted, redo setup.")
            print("Use teleoperation in a separate terminal. Keep the gripper closed and empty at a fixed hover height.")
            for index, corner in enumerate(CORNERS):
                while True:
                    for _ in range(5):
                        ok, frame = camera.read()
                        if ok and frame is not None and frame.size:
                            break
                    else:
                        raise RuntimeError(f"No frames from overhead camera {camera_index}.")
                    cv2.imshow(window, draw_teaching_view(cv2, frame, corners, reference_size, index))
                    key = cv2.waitKey(20) & 0xFF
                    if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) == 0:
                        raise KeyboardInterrupt
                    if key in (10, 13):
                        try:
                            poses[corner] = read_pose(port, robot_id)
                        except OSError as error:
                            print(f"Could not read {corner}. Close teleoperation and retry Enter. {error}", file=sys.stderr)
                            continue
                        print(f"Captured {corner}. The serial port is free for teleoperation.")
                        break
        finally:
            cv2.destroyWindow(window)
    except cv2.error as error:
        raise RuntimeError(f"Overhead camera preview failed: {error}") from error
    finally:
        camera.release()
    save_poses(path, poses)
    print(f"Saved all four fresh corner poses to {path}")
    return poses


def check_start(actual, start, *, report_all=False, tolerance=ORIGIN_TOLERANCE_DEG, label="taught origin"):
    errors = actual - start
    mismatches = abs(errors) > tolerance
    for name, current, target, error, mismatch in zip(JOINTS, actual, start, errors, mismatches):
        if report_all or mismatch:
            print(f"{name}: current {current:.2f}°, target {target:.2f}°, difference {error:+.2f}°")
    if np.any(mismatches):
        raise ValueError(f"Arm is outside the {tolerance:g}° tolerance for the {label}.")


def plan_origin_move(current, origin, model, limits, *, label="origin"):
    lower, upper = limits
    if np.any(current < lower) or np.any(current > upper):
        raise ValueError("Current arm pose is outside the URDF joint limits.")
    delta = origin - current
    count = math.ceil(float(np.max(abs(delta))) / MAX_JOINT_STEP_DEG)
    floor = min(model.forward_kinematics(current)[2, 3], model.forward_kinematics(origin)[2, 3])
    approach = []
    for joints in np.linspace(current, origin, count + 1)[1:]:
        point = model.forward_kinematics(joints)[:3, 3].copy()
        if not np.isfinite(point).all() or point[2] < floor - POSITION_ERROR_M:
            raise ValueError(f"The automatic move to {label} would dip below both endpoint heights in the model. "
                             "Start from a clear hover pose before retrying.")
        approach.append((0, point, joints))
    return approach


def motion_group(edge, continuous_columns):
    """Keep approaches and row changes separate from straight row sweeps."""
    if continuous_columns and edge > 0:
        row, column = divmod(edge - 1, continuous_columns)
        if column:
            return row * continuous_columns + 2
    return edge


def stream_samples(route, start, model, *, continuous_columns=None, command_period=COMMAND_PERIOD_S,
                   joint_speed_deg_s=JOINT_SPEED_DEG_S):
    previous = start
    previous_point = model.forward_kinematics(start)[:3, 3].copy()
    for _, group in groupby(route, key=lambda waypoint: motion_group(waypoint[0], continuous_columns)):
        segment = list(group)
        edge = segment[-1][0]
        angles = np.vstack([previous, *(joints for _, _, joints in segment)])
        points = np.vstack([previous_point, *(point for _, point, _ in segment)])
        intervals = np.max(abs(np.diff(angles, axis=0)), axis=1) / joint_speed_deg_s
        if edge:
            intervals = np.maximum(intervals, np.linalg.norm(np.diff(points, axis=0), axis=1) / EDGE_SPEED_M_S)
        knot_times = np.r_[0.0, np.cumsum(intervals)]
        # Quintic time scaling starts and ends at zero velocity/acceleration.
        # Its peak derivative is 1.875; stretch duration to retain speed limits.
        duration = max(MIN_MOVE_DURATION_S, 1.875 * knot_times[-1])
        count = math.ceil(duration / command_period)
        phase = np.linspace(0, 1, count + 1)[1:]
        times = knot_times[-1] * (10 * phase**3 - 15 * phase**4 + 6 * phase**5)
        joint_commands = np.column_stack([np.interp(times, knot_times, values) for values in angles.T])
        positions = np.column_stack([np.interp(times, knot_times, values) for values in points.T])
        # Check intermediate joint blends too when commands are spaced farther apart.
        checks_per_step = math.ceil(command_period / COMMAND_PERIOD_S)
        last_position, last_joints = previous_point, previous
        for position, joints in zip(positions, joint_commands):
            if edge:
                for check in range(1, checks_per_step + 1):
                    fraction = check / checks_per_step
                    check_joints = last_joints + fraction * (joints - last_joints)
                    check_point = last_position + fraction * (position - last_position)
                    modeled = model.forward_kinematics(check_joints)[:3, 3]
                    if not np.isfinite(modeled).all() or np.linalg.norm(modeled - check_point) > POSITION_ERROR_M:
                        raise ValueError(f"Interpolated commands leave the modeled line on edge {edge}.")
            yield edge, position, joints
            last_position, last_joints = position, joints
        previous, previous_point = angles[-1], points[-1]


def distance_to_segment(point, start, end):
    direction = end - start
    length_squared = np.dot(direction, direction)
    if length_squared == 0:
        return np.linalg.norm(point - start)
    fraction = np.clip(np.dot(point - start, direction) / length_squared, 0, 1)
    return np.linalg.norm(point - (start + fraction * direction))


def trace(samples, start, model, port, robot_id, *, auto_origin=False, origin_only=False, limits=None,
          cell_columns=None, relaxed_cell_tracking=False, continuous_cells=False,
          command_period=COMMAND_PERIOD_S, joint_speed_deg_s=JOINT_SPEED_DEG_S):
    track_every_commands = max(1, math.floor(TRACK_PERIOD_S / command_period))
    robot = follower(port, robot_id)
    track_deg = RELAXED_CELL_TRACK_DEG if relaxed_cell_tracking else TRACK_DEG
    position_tolerance = RELAXED_CELL_POSITION_ERROR_M if relaxed_cell_tracking else POSITION_ERROR_M
    if relaxed_cell_tracking:
        print(f"Cell tracking tolerances: {track_deg:g}° joint error and "
              f"{position_tolerance * 1000:g} mm modeled position error.", flush=True)
    if auto_origin:
        print("Automatic move to origin enabled. Keep the unloaded arm's full sweep clear.", flush=True)
    else:
        print("Start the unloaded arm at the taught origin hover pose. Keep the board and arm path clear.", flush=True)
    motion_started = False
    connected = False
    try:
        robot.connect(calibrate=False)
        connected = True
        if not robot.is_calibrated:
            raise ValueError(f"The follower calibration does not match {robot_id}.")
        current = joint_vector(robot.get_observation())
        approach = []
        if auto_origin and np.max(abs(current - start)) > ORIGIN_TOLERANCE_DEG:
            if origin_only or cell_columns:
                print("Joint angles before the move to origin:")
                for name, actual, target in zip(JOINTS, current, start):
                    print(f"{name}: current {actual:.2f}°, target {target:.2f}°")
                input("Clear the unloaded arm's full sweep. Press Enter to move to the saved origin: ")
                confirmed = joint_vector(robot.get_observation())
                if np.max(abs(confirmed - current)) > TRACK_DEG:
                    raise ValueError("Arm moved while waiting for confirmation. Restart to plan from its new pose.")
                current = confirmed
            approach = plan_origin_move(current, start, model, limits)
        else:
            check_start(current, start)
        origin_speed = min(joint_speed_deg_s, ORIGIN_SPEED_DEG_S)
        samples = list(stream_samples(approach, current, model, command_period=command_period,
                                      joint_speed_deg_s=origin_speed)) + samples
        segment_start = model.forward_kinematics(start)[:3, 3].copy()
        previous_target = start
        for edge, group in groupby(samples, key=lambda sample: sample[0]):
            segment = list(group)
            segment_end = segment[-1][1]
            label = f"edge {edge}" if edge else "the move to origin"
            if auto_origin and edge == 1 and cell_columns is None:
                check_start(joint_vector(robot.get_observation()), start)
                input("At the saved origin. Press Enter to begin the perimeter trace: ")
                current = joint_vector(robot.get_observation())
                check_start(current, start)
            if edge and cell_columns:
                row, column = divmod(edge - 1, cell_columns)
                label = f"cell ({row}, {column})"
                if edge == 1:
                    input("Positioning step: press Enter to move from the board corner to cell (0, 0): ")
                elif continuous_cells and column > 0:
                    label = f"row {row} through cell ({row}, {column})"
                    input(f"Clear the row. Press Enter to sweep {label}: ")
                elif continuous_cells:
                    input(f"Clear the row transition. Press Enter to move to {label}: ")
                else:
                    input(f"At the previous cell. Press Enter to move to {label}: ")
                current = joint_vector(robot.get_observation())
                if edge == 1:
                    check_start(current, previous_target)
                    measured_start = model.forward_kinematics(current)[:3, 3].copy()
                    start_error = np.linalg.norm(measured_start - segment_start)
                    if start_error > POSITION_ERROR_M:
                        lower, upper = limits
                        if np.any(current < lower) or np.any(current > upper):
                            raise ValueError("Measured origin is outside the URDF joint limits.")
                        print(f"Replanning approach to {label} from the measured origin "
                              f"({start_error * 1000:.1f} mm from the taught origin).", flush=True)
                        route = plan_line(edge, label, measured_start, segment_end, current, model, limits,
                                          final_joints=segment[-1][2])
                        segment = list(stream_samples(route, current, model, command_period=command_period,
                                                      joint_speed_deg_s=joint_speed_deg_s))
                        segment_start = measured_start
                else:
                    check_start(current, previous_target, tolerance=track_deg, label="previous cell")
            else:
                print(f"Tracing edge {edge} of 4" if edge else
                      f"Moving to the saved origin (joint cap {origin_speed:g}°/s)", flush=True)
            peak_deviation = 0.0
            for index, (_, point, joints) in enumerate(segment, start=1):
                tick = time.monotonic()
                # Bound each command against the last feedback sample without
                # making send_action perform another serial read at every tick.
                lead = abs(joints - current)
                if np.max(lead) > MAX_COMMAND_LEAD_DEG:
                    joint = JOINTS[int(np.argmax(lead))]
                    raise RuntimeError(f"Command {index} on segment {edge} would put {joint} "
                                       f"{np.max(lead):.2f}° ahead of its last reading.")
                action = {f"{name}.pos": float(value) for name, value in zip(JOINTS, joints)}
                motion_started = True
                sent = joint_vector(robot.send_action(action))
                if np.max(abs(sent - joints)) > 0.1:
                    raise RuntimeError("LeRobot clamped a target. Stopping the trace.")
                # Read feedback without waiting for every intermediate target.
                # A stalled joint still trips the command-lead guard.
                if index % track_every_commands == 0 or index == len(segment):
                    current = joint_vector(robot.get_observation())
                    actual_point = model.forward_kinematics(current)[:3, 3]
                    if np.max(abs(current - joints)) > MAX_COMMAND_LEAD_DEG:
                        raise RuntimeError(f"Tracking exceeded {MAX_COMMAND_LEAD_DEG:g}° on {label}.")
                    if edge:
                        deviation = distance_to_segment(actual_point, segment_start, segment_end)
                        peak_deviation = max(peak_deviation, deviation)
                        if deviation > PATH_STOP_ERROR_M:
                            raise RuntimeError(f"Path deviation on {label}: {deviation * 1000:.1f} mm "
                                               f"(stop limit {PATH_STOP_ERROR_M * 1000:g} mm).")
                # Account for feedback time; never burst commands to catch up.
                time.sleep(max(0.0, command_period - (time.monotonic() - tick)))

            # Only the destination gets a settling period. Motors retain the
            # final goal while feedback is read; no intermediate stop/wait loop.
            print(f"Settling at {label} for {ENDPOINT_SETTLE_S:g} seconds.", flush=True)
            deadline = time.monotonic() + ENDPOINT_SETTLE_S
            tolerance = track_deg if edge else ORIGIN_TOLERANCE_DEG
            while True:
                current = joint_vector(robot.get_observation())
                errors = abs(current - joints)
                actual_point = model.forward_kinematics(current)[:3, 3]
                position_error = np.linalg.norm(actual_point - point)
                reached = np.max(errors) <= tolerance and (not edge or position_error <= position_tolerance)
                # Continue monitoring throughout the full settling interval.
                # Do not turn a transient pass of the loose tolerance into success.
                if np.max(errors) > MAX_COMMAND_LEAD_DEG:
                    raise RuntimeError(f"Tracking exceeded {MAX_COMMAND_LEAD_DEG:g}° while settling on {label}.")
                if edge:
                    deviation = distance_to_segment(actual_point, segment_start, segment_end)
                    peak_deviation = max(peak_deviation, deviation)
                    if deviation > PATH_STOP_ERROR_M:
                        raise RuntimeError(f"Path deviation while settling on {label}: "
                                           f"{deviation * 1000:.1f} mm.")
                if time.monotonic() >= deadline:
                    print(f"Endpoint report for {label}:")
                    for name, measured, target in zip(JOINTS, current, joints):
                        print(f"  {name}: measured {measured:.2f}°, target {target:.2f}°, "
                              f"error {measured - target:+.2f}°")
                    print("  Target XYZ (model mm): " + ", ".join(f"{v * 1000:.1f}" for v in point))
                    print("  Measured XYZ (model mm): " + ", ".join(f"{v * 1000:.1f}" for v in actual_point))
                    print(f"  Endpoint error: {position_error * 1000:.1f} mm; "
                          f"peak sampled line deviation: {peak_deviation * 1000:.1f} mm."
                          if edge else f"  Modeled endpoint error: {position_error * 1000:.1f} mm.")
                    if not reached:
                        raise RuntimeError(f"Endpoint settling timed out on {label}. "
                                           f"Largest joint error {np.max(errors):.2f}°; "
                                           f"modeled position error {position_error * 1000:.1f} mm.")
                    break
                time.sleep(0.05)
            if edge:
                segment_start = segment_end
            previous_target = segment[-1][2]
        if origin_only:
            check_start(joint_vector(robot.get_observation()), start, report_all=True)
            print("Reached the saved origin. Perimeter motion was not requested.")
    except (Exception, KeyboardInterrupt) as error:
        if motion_started:
            print(f"Motion stopped: {error}", file=sys.stderr)
            release_after_failure(robot)
        elif not connected:
            print("Motor connection failed before any movement command. "
                  "Check motor power and close any other app using the serial port.", file=sys.stderr)
        raise
    finally:
        if robot.is_connected:
            robot.disconnect()
            print("Serial connection closed.")



def verify_mapping(board, taught, model, port, robot_id):
    """Read two independently positioned board references; never command motion."""
    readings = []
    print("Mapping check: no movement commands or saved-pose changes.")
    print("Use teleoperation to position the same gripper tip at the original hover height.")
    print("The references are OUTER board corners, not cell centers.")
    for index, corner in enumerate(CORNERS[:2]):
        pixel = board["top_down_corners_px"][index]
        print(f"Reference {corner}: saved overhead pixel ({pixel[0]:.1f}, {pixel[1]:.1f}).")
        input(f"Position over {corner}, close teleoperation, then press Enter to read: ")
        measured = joint_vector(read_pose(port, robot_id))
        expected_xyz = model.forward_kinematics(taught[index])[:3, 3].copy()
        measured_xyz = model.forward_kinematics(measured)[:3, 3].copy()
        for name, actual, expected in zip(JOINTS, measured, taught[index]):
            print(f"  {name}: measured {actual:.2f}°, saved {expected:.2f}°, "
                  f"difference {actual - expected:+.2f}°")
        print("  Saved XYZ (model mm): " + ", ".join(f"{v * 1000:.1f}" for v in expected_xyz))
        print("  Live XYZ (model mm): " + ", ".join(f"{v * 1000:.1f}" for v in measured_xyz))
        readings.append(measured_xyz)
    delta = readings[1] - readings[0]
    width = board["columns"] * board["square_size_mm"]
    print(f"Known board width: {width:.1f} mm.")
    print(f"Live modeled horizontal width: {np.linalg.norm(delta[:2]) * 1000:.1f} mm; "
          f"height difference: {delta[2] * 1000:+.1f} mm.")
    print("If the tip was aligned with both physical corners at the same height, a width "
          "mismatch indicates the model/calibration/tool reference needs investigation.")
    print("This check does not fit a transform or verify positions with the camera.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("workspace", type=Path, help="workspace.json from the overhead setup")
    parser.add_argument("--poses", required=True, type=Path,
                        help="JSON object of four joint readouts keyed by " + ", ".join(CORNERS))
    parser.add_argument("--urdf", type=Path, help="SO-101 follower URDF matching your calibration")
    parser.add_argument("--port", default="/dev/tty.usbmodem5B3E0906941")
    parser.add_argument("--robot-id", default="my_awesome_follower_arm")
    parser.add_argument("--edge-tolerance-mm", type=float, default=15.0,
                        help="allowed difference between each modeled and measured edge (default: 15)")
    parser.add_argument("--taught-perimeter", action="store_true",
                        help="trace the taught quadrilateral using modeled positions at the origin hover height")
    parser.add_argument("--raise-mm", type=float, default=0.0,
                        help="raise the planned origin and all perimeter edges by this many millimeters")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--verify-mapping", action="store_true",
                      help="read independently positioned origin and positive_x corners; no motion or pose edits")
    mode.add_argument("--teach", action="store_true", help="prompt for a fresh set of all four corner poses")
    mode.add_argument("--teach-first-row", action="store_true",
                      help="read the physical first and last cell poses into --row-poses")
    mode.add_argument("--record-corner", choices=CORNERS, help="read and save the current joint positions")
    mode.add_argument("--check-origin", action="store_true", help="compare live joint angles with the taught origin")
    mode.add_argument("--origin-only", action="store_true", help="move to and verify the saved origin without tracing")
    mode.add_argument("--visit-cells", action="store_true", help="visit each grid cell center left to right, row by row")
    extent = parser.add_mutually_exclusive_group()
    extent.add_argument("--first-cell-only", action="store_true",
                        help="with --visit-cells, position at cell (0, 0) only")
    extent.add_argument("--first-row-only", action="store_true",
                        help="with --visit-cells, position at (0, 0), then visit (0, 1) onward")
    parser.add_argument("--continuous-cells", action="store_true",
                        help="with --visit-cells, sweep each row without pausing at intermediate centers")
    parser.add_argument("--metric-grid", action="store_true",
                        help="with --visit-cells, use square_size_mm and perpendicular board axes")
    parser.add_argument("--row-poses", type=Path,
                        help="capture or load physical first-row endpoints for --first-row-only")
    parser.add_argument("--relaxed-cell-tracking", action="store_true",
                        help="with --visit-cells, allow 3° joint error and 10 mm modeled endpoint error after settling")
    mode.add_argument("--release-torque", action="store_true", help="disable motor torque after supporting the arm")
    parser.add_argument("--execute", action="store_true", help="send the checked plan to the arm")
    parser.add_argument("--auto-origin", action="store_true",
                        help="move from the current joint pose to the taught origin before tracing")
    parser.add_argument("--coarse-steps", action="store_true",
                        help="stream at 25 Hz with larger joint increments and the same planned speed limits")
    parser.add_argument("--joint-speed-deg-s", type=float, default=JOINT_SPEED_DEG_S,
                        help=f"planned joint speed cap (default: %(default)g deg/s); "
                             f"the move to origin is also capped at {ORIGIN_SPEED_DEG_S:g} deg/s")
    args = parser.parse_args()
    if not 0 < args.joint_speed_deg_s <= JOINT_SPEED_DEG_S:
        parser.error(f"--joint-speed-deg-s must be greater than 0 and at most {JOINT_SPEED_DEG_S:g}.")
    command_period = COARSE_COMMAND_PERIOD_S if args.coarse_steps else COMMAND_PERIOD_S
    if args.teach_first_row and args.row_poses is None:
        parser.error("--teach-first-row requires --row-poses.")
    if args.row_poses is not None:
        if args.row_poses.resolve() in {args.poses.resolve(), args.workspace.resolve()}:
            parser.error("--row-poses requires a separate file from the corner poses and workspace.")
        if not (args.teach_first_row or args.first_row_only):
            parser.error("--row-poses requires --teach-first-row or --first-row-only.")
        if args.metric_grid or args.taught_perimeter or args.raise_mm:
            parser.error("--row-poses uses the taught cell centers and their recorded heights.")
    if (args.first_cell_only or args.first_row_only) and not args.visit_cells:
        parser.error("--first-cell-only and --first-row-only require --visit-cells.")
    if (args.continuous_cells or args.metric_grid) and not args.visit_cells:
        parser.error("--continuous-cells and --metric-grid require --visit-cells.")
    if args.relaxed_cell_tracking and not args.visit_cells:
        parser.error("--relaxed-cell-tracking requires --visit-cells.")
    if not math.isfinite(args.raise_mm) or args.raise_mm < 0:
        parser.error("--raise-mm must be finite and non-negative.")
    if args.raise_mm and (args.check_origin or args.record_corner or args.release_torque or args.verify_mapping or args.urdf is None):
        parser.error("--raise-mm requires --urdf and applies to planning or executing a move.")
    if args.auto_origin and not args.execute:
        parser.error("--auto-origin requires --execute.")
    if args.origin_only and not args.execute:
        parser.error("--origin-only requires --execute.")
    if (args.record_corner or args.check_origin or args.release_torque or args.verify_mapping or
            args.teach_first_row) and args.execute:
        parser.error("Read-only checks and recording/release modes cannot be used with --execute.")
    if args.urdf is None and (args.execute or not (
            args.teach or args.teach_first_row or args.record_corner or args.check_origin or args.release_torque)):
        parser.error("--urdf is required for planning and execution.")
    try:
        if args.release_torque:
            input("Support the arm so it cannot drop. Press Enter to disable motor torque: ")
            robot = follower(args.port, args.robot_id)
            try:
                robot.bus.connect()
                release_torque(robot)
            finally:
                if robot.bus.is_connected:
                    robot.bus.disconnect(False)
            return 0
        config = json.loads(args.workspace.read_text())
        board = config["board"]
        if board["rows"] != 3 or board["columns"] not in (6, 8):
            raise ValueError("Select a 3x6 or 3x8 workspace with vision-service/setup.py, then teach its corners.")
        if board["corner_order"] != list(CORNERS):
            raise ValueError("Workspace corner order differs from the taught pose order.")
        if args.teach_first_row:
            teach_first_row(args.row_poses, board["columns"], args.port, args.robot_id)
            return 0
        if args.record_corner is not None:
            record_corner(args.poses, args.record_corner, args.port, args.robot_id)
            return 0
        if args.teach:
            if args.urdf is not None and not args.urdf.is_file():
                raise ValueError(f"URDF file not found: {args.urdf}. Pass the actual file path with --urdf.")
            poses = teach_corners(args.poses, args.port, args.robot_id, config)
            if args.urdf is None:
                return 0
        else:
            poses = json.loads(args.poses.read_text()) if args.poses.exists() else {}
        if not isinstance(poses, dict):
            raise ValueError(f"Expected a JSON object in {args.poses}.")
        if args.check_origin:
            if "origin" not in poses:
                raise ValueError(f"No origin pose recorded in {args.poses}.")
            start = joint_vector(poses["origin"])
            check_start(joint_vector(read_pose(args.port, args.robot_id)), start, report_all=True)
            print("All five joints are within the taught-origin tolerance.")
            return 0
        corners = ("origin",) if args.origin_only or args.row_poses is not None else CORNERS
        missing = [corner for corner in corners if corner not in poses]
        if missing:
            raise ValueError(f"Missing corner poses in {args.poses}: {', '.join(missing)}. "
                             "Jog the arm to each corner and save it with --record-corner NAME.")
        taught = np.array([joint_vector(poses[corner]) for corner in corners])
        if not args.urdf.is_file():
            raise ValueError(f"URDF file not found: {args.urdf}. Pass the actual file path with --urdf.")
        limits = joint_limits(args.urdf)
        from lerobot.model.kinematics import RobotKinematics

        model = RobotKinematics(str(args.urdf), joint_names=list(JOINTS))
        if args.verify_mapping:
            verify_mapping(board, taught, model, args.port, args.robot_id)
            return 0
        start = raised_origin(taught[0], model, limits, args.raise_mm)
        if args.origin_only:
            trace([], start, model, args.port, args.robot_id, auto_origin=True, origin_only=True,
                  limits=limits, command_period=command_period, joint_speed_deg_s=args.joint_speed_deg_s)
            return 0
        if args.row_poses is not None:
            row_data = json.loads(args.row_poses.read_text())
            if row_data["columns"] != board["columns"]:
                raise ValueError("The taught row was recorded for a different number of columns.")
            row_joints = np.array([joint_vector(row_data["poses"][name]) for name in ("first", "last")])
            route = plan_taught_row(board["columns"], start, row_joints, model, limits)
        else:
            route = plan_perimeter(board, taught, model, limits, args.edge_tolerance_mm,
                                   taught_perimeter=args.taught_perimeter, raised_start=start, raise_mm=args.raise_mm,
                                   visit_cells=args.visit_cells, first_cell_only=args.first_cell_only,
                                   first_row_only=args.first_row_only, metric_grid=args.metric_grid)
        samples = list(stream_samples(route, start, model,
                                      continuous_columns=board["columns"] if args.continuous_cells else None,
                                      command_period=command_period, joint_speed_deg_s=args.joint_speed_deg_s))
        excursions = np.ptp(np.vstack([start, *(joints for _, _, joints in samples)]), axis=0)
        print(f"Planned {len(route)} Cartesian waypoints and {len(samples)} streamed commands "
              f"at {1 / command_period:g} Hz with a {args.joint_speed_deg_s:g}°/s joint cap.")
        print("Planned joint ranges: " + ", ".join(
            f"{name} {value:.1f}°" for name, value in zip(JOINTS, excursions)
        ))
        if args.execute:
            if args.teach:
                if not args.auto_origin:
                    print("Jog back to origin at the taught height.")
                input("Close teleoperation and clear the path, then press Enter to start the trace: ")
            trace(samples, start, model, args.port, args.robot_id, auto_origin=args.auto_origin, limits=limits,
                  cell_columns=board["columns"] if args.visit_cells else None,
                  relaxed_cell_tracking=args.relaxed_cell_tracking, continuous_cells=args.continuous_cells,
                  command_period=command_period, joint_speed_deg_s=args.joint_speed_deg_s)
        else:
            print("Dry run complete. Run with --execute after reviewing the taught poses and clearing the path.")
    except (KeyboardInterrupt, EOFError):
        print("Trace interrupted.", file=sys.stderr)
        return 1
    except (ImportError, OSError, ValueError, KeyError, RuntimeError, ET.ParseError) as error:
        print(f"Perimeter test failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
