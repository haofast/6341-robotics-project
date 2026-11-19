#!/usr/bin/env python3
"""Teach and replay a single block pickup on the SO-101."""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

from trace_perimeter import (
    COARSE_COMMAND_PERIOD_S,
    IK_WAYPOINT_SPACING_M,
    JOINTS,
    MAX_COMMAND_LEAD_DEG,
    MAX_JOINT_STEP_DEG,
    POSITION_ERROR_M,
    check_start,
    distance_to_segment,
    follower,
    joint_limits,
    joint_vector,
    plan_line,
    plan_origin_move,
    read_pose,
    release_after_failure,
    save_poses,
    stream_samples,
)

GRIPPER = "gripper.pos"
PICK_SPEED_DEG_S = 10.0
LIFT_SPEED_DEG_S = 5.0
PICK_POSITION_ERROR_M = 0.006
PICK_ORIENTATION_ERROR_DEG = 5.0
LIFT_PAUSE_DEVIATION_M = 0.0045
RELAXED_LIFT_PAUSE_M = 0.007
RELAXED_LIFT_LIMIT_M = 0.008
LIFT_RECOVERY_TIMEOUT_S = 0.75
LIFT_STEP_DWELL_S = 0.15
LIFT_STEP_JOINT_TOLERANCE_DEG = 2.5
LIFT_STEP_MAX_INCREMENT_DEG = LIFT_SPEED_DEG_S * LIFT_STEP_DWELL_S
DOWN_AXIS = np.array([0.0, 0.0, -1.0])


def plan_free_wrist_lift(grasp, rise, model, limits):
    """Plan a vertical lift with position-only inverse kinematics."""
    bottom = model.forward_kinematics(grasp)[:3, 3].copy()
    top = bottom + np.array([0, 0, rise])
    route = plan_line(2, "free-wrist lift", bottom, top, grasp, model, limits,
                      orientation_weight=0.0)
    samples = list(stream_samples(route, grasp, model, command_period=COARSE_COMMAND_PERIOD_S,
                                  joint_speed_deg_s=LIFT_SPEED_DEG_S))
    print(f"Free-wrist lift plan: {rise * 1000:.0f} mm straight up, {len(samples)} commands.",
          flush=True)
    return samples


def check_tool_axis(rotation, target_axis, label, error_type=ValueError):
    cosine = np.dot(rotation[:, 2], target_axis)
    error = np.degrees(np.arccos(np.clip(cosine, -1, 1)))
    if not np.isfinite(error) or error > PICK_ORIENTATION_ERROR_DEG:
        raise error_type(f"{label}: tool tilt error {error:.1f}° (limit {PICK_ORIENTATION_ERROR_DEG:g}°).")


def plan_downward_lift(grasp, rise, model, limits):
    """Lift toward the base while leveling the tool, with free yaw."""
    lower, upper = limits
    start = model.forward_kinematics(grasp).copy()
    bottom = start[:3, 3]
    if not np.isfinite(start).all() or start[2, 2] < -1.001 or start[2, 2] > -0.96:
        raise ValueError("Teach a grasp with the tool already pointing nearly straight down.")
    radius = np.linalg.norm(bottom[:2])
    if radius < 0.025:
        raise ValueError("Grasp is too close to the base for the inward lift.")
    inward = -bottom[:2] / radius
    retreat = min(0.012, rise * 0.3)
    count = int(np.ceil((rise + retreat) / IK_WAYPOINT_SPACING_M))
    seed = grasp
    route = []
    for index in range(1, count + 1):
        height = rise * index / count
        # Rise above the grasp first, then tuck inward to reduce arm extension.
        shift = retreat * max(0, (height - 0.010) / max(rise - 0.010, 0.001))
        point = bottom + np.r_[inward * shift, height]
        axis = start[:3, 2] + min(height / 0.012, 1) * (DOWN_AXIS - start[:3, 2])
        axis /= np.linalg.norm(axis)
        pose = model.forward_kinematics(seed).copy()
        # Use the current tool heading to seed yaw rather than locking it to the grasp.
        heading = pose[:3, 0] - np.dot(pose[:3, 0], axis) * axis
        heading /= np.linalg.norm(heading)
        pose[:3, :3] = np.column_stack((heading, np.cross(axis, heading), axis))
        pose[:3, 3] = point
        joints = model.inverse_kinematics(seed, pose, orientation_weight=0.1, max_iters=16)
        actual = model.forward_kinematics(joints)
        if (not np.isfinite(joints).all() or np.any(joints < lower) or np.any(joints > upper)
                or np.max(abs(joints - seed)) > MAX_JOINT_STEP_DEG
                or not np.isfinite(actual).all()
                or np.linalg.norm(actual[:3, 3] - point) > POSITION_ERROR_M):
            raise ValueError(f"Downward lift waypoint {index} is outside the modeled motion limits.")
        check_tool_axis(actual[:3, :3], axis, f"planned downward lift waypoint {index}")
        route.append((2, point, joints))
        seed = joints
    samples = list(stream_samples(route, grasp, model, command_period=COARSE_COMMAND_PERIOD_S,
                                  joint_speed_deg_s=LIFT_SPEED_DEG_S))
    check_tool_axis(model.forward_kinematics(samples[-1][2])[:3, :3], DOWN_AXIS, "planned lift endpoint")
    print(f"Downward lift plan: {rise * 1000:.0f} mm up, "
          f"{shift * 1000:.1f} mm toward the base, {len(samples)} commands.", flush=True)
    return samples


def lift_step_targets(samples, start_joints, model, reference):
    """Group adjacent lift samples within the joint increment for one dwell."""
    targets = []
    held = start_joints
    candidate = None
    for sample in samples:
        if np.max(abs(sample[2] - held)) > LIFT_STEP_MAX_INCREMENT_DEG:
            if candidate is None:
                raise ValueError("A planned lift increment is too large for step mode.")
            targets.append(candidate)
            held = candidate[2]
            if np.max(abs(sample[2] - held)) > LIFT_STEP_MAX_INCREMENT_DEG:
                raise ValueError("A planned lift increment is too large for step mode.")
        candidate = sample
    if candidate is not None:
        targets.append(candidate)
    held = start_joints
    start_point = model.forward_kinematics(held)[:3, 3].copy()
    for _, point, joints in targets:
        for fraction in (0.25, 0.5, 0.75, 1.0):
            pose = model.forward_kinematics(held + fraction * (joints - held))
            check_orientation(pose[:3, :3], reference, "planned step lift")
            expected = start_point + fraction * (point - start_point)
            error = np.linalg.norm(pose[:3, 3] - expected)
            if not np.isfinite(error) or error > 0.004:
                raise ValueError("A grouped lift step leaves the modeled path by more than 4 mm.")
        held, start_point = joints, point
    return targets


def check_orientation(rotation, reference, label, error_type=ValueError):
    cosine = (np.trace(reference.T @ rotation) - 1) / 2
    error = np.degrees(np.arccos(np.clip(cosine, -1, 1)))
    if not np.isfinite(error) or error > PICK_ORIENTATION_ERROR_DEG:
        raise error_type(f"Gripper orientation differs from the taught grasp by {error:.1f}° "
                         f"at {label} (limit {PICK_ORIENTATION_ERROR_DEG:g}°).")


def plan_pick(poses, model, limits):
    if "lift_mm" in poses:
        return plan_pick_from_grasp(poses, model, limits)

    hover, grasp = (joint_vector(poses[name]) for name in ("hover", "grasp"))
    lower, upper = limits
    if any(np.any(joints < lower) or np.any(joints > upper) for joints in (hover, grasp)):
        raise ValueError("A pickup pose is outside the URDF joint limits.")
    hover_pose, grasp_pose = [model.forward_kinematics(q).copy() for q in (hover, grasp)]
    top, bottom = hover_pose[:3, 3], grasp_pose[:3, 3]
    reference = grasp_pose[:3, :3]
    drop = top - bottom
    if not np.isfinite(drop).all():
        raise ValueError("The robot model returned a non-finite pickup position.")
    drop_mm = drop[2] * 1000
    sideways_mm = np.linalg.norm(drop[:2]) * 1000
    print(f"Modeled grasp relative to hover: {drop_mm:+.1f} mm down, {sideways_mm:.1f} mm sideways.", flush=True)
    problems = []
    if drop[2] < 0.005:
        problems.append("use a hover at least 5 mm above the grasp")
    if sideways_mm > 10:
        problems.append("align the open hover and grasp over the same spot (maximum 10 mm sideways)")
    if problems:
        raise ValueError("Adjust the taught poses: " + " and ".join(problems) + ".")
    opened, closed = (float(poses[name][GRIPPER]) for name in ("hover", "grasp"))
    if not 0 <= closed < opened <= 100:
        raise ValueError("Teach a wider gripper opening at hover than at grasp (range 0–100).")

    check_orientation(hover_pose[:3, :3], reference, "hover")
    down = plan_line(1, "descent", top, bottom, hover, model, limits,
                     final_joints=grasp, fixed_rotation=reference)
    return sample_pick(down, hover, grasp, top, reference, opened, closed, drop_mm, model)


def plan_pick_from_grasp(poses, model, limits):
    grasp = joint_vector(poses["grasp"])
    lower, upper = limits
    if np.any(grasp < lower) or np.any(grasp > upper):
        raise ValueError("The taught grasp is outside the URDF joint limits.")
    opened = float(poses["gripper_open"])
    closed = float(poses["grasp"][GRIPPER])
    if not 0 <= closed < opened <= 100:
        raise ValueError("Set --gripper-open above the taught grasp opening (range 0–100).")
    lift_mm = float(poses["lift_mm"])
    if not np.isfinite(lift_mm) or lift_mm < 5:
        raise ValueError("The requested lift must be finite and at least 5 mm.")
    pose = model.forward_kinematics(grasp).copy()
    bottom, reference = pose[:3, 3], pose[:3, :3]
    top = bottom + np.array([0, 0, lift_mm / 1000])
    # Solve outward from the physically taught pose. Retracing that solution
    # guarantees the descent ends at the recorded grasp configuration.
    try:
        outward = plan_line(2, "vertical lift", bottom, top, grasp, model, limits,
                            fixed_rotation=reference)
        hover = outward[-1][2]
        down = [(1, point, joints)
                for _, point, joints in reversed([(2, bottom, grasp), *outward[:-1]])]
        return sample_pick(down, hover, grasp, top, reference, opened, closed, lift_mm, model)
    except ValueError as error:
        raise ValueError(f"Could not plan the requested {lift_mm:g} mm vertical lift: {error}") from error


def sample_pick(down, hover, grasp, top, reference, opened, closed, drop_mm, model):
    up = [(2, point, joints) for _, point, joints in reversed([(1, top, hover), *down[:-1]])]
    descent = list(stream_samples(down, hover, model, command_period=COARSE_COMMAND_PERIOD_S,
                                  joint_speed_deg_s=PICK_SPEED_DEG_S))
    lift = list(stream_samples(up, grasp, model, command_period=COARSE_COMMAND_PERIOD_S,
                               joint_speed_deg_s=LIFT_SPEED_DEG_S))
    for label, start, samples in (("descent", hover, descent), ("lift", grasp, lift)):
        previous = start
        for _, _, joints in samples:
            for fraction in (0.25, 0.5, 0.75, 1.0):
                pose = model.forward_kinematics(previous + fraction * (joints - previous))
                check_orientation(pose[:3, :3], reference, f"planned {label}")
            previous = joints
    print(f"Planned {drop_mm:.1f} mm descent and lift with {len(descent) + len(lift)} commands.")
    print(f"Joint speed caps: {PICK_SPEED_DEG_S:g}°/s descent, {LIFT_SPEED_DEG_S:g}°/s lift.")
    print("Start at hover joints (degrees): " +
          ", ".join(f"{name}={value:.2f}" for name, value in zip(JOINTS, hover)))
    return hover, opened, closed, descent, lift


def teach_pick(cell, model, limits, port, robot_id):
    def capture(instruction):
        input(f"Cell ({cell}): {instruction}. Close teleop, then press Enter to record: ")
        return read_pose(port, robot_id, include_gripper=True)

    hover_instruction = "Center the open gripper high above the block with the same tilt as the grasp"
    hover = capture(hover_instruction)
    while True:
        grasp = capture("Lower and lightly close the gripper around the block with the cube flat on the board")
        captured = {"hover": hover, "grasp": grasp}
        try:
            plan_pick(captured, model, limits)
        except ValueError as error:
            print(f"Pose check failed: {error}", file=sys.stderr)
            while True:
                choice = input("Enter to retry the grasp, H to recapture hover, or Q to cancel: ").strip().lower()
                if choice in ("", "h", "q"):
                    break
            if choice == "q":
                raise KeyboardInterrupt
            if choice == "h":
                hover = capture(hover_instruction)
        else:
            return captured


def stream_pick(robot, samples, model, line_start, label, reference=None, *, loaded_lift=False,
                relaxed_lift_tracking=False, step_lift=False, downward_lift=False,
                verify_rise=False):
    relaxed = loaded_lift and relaxed_lift_tracking
    path_limit = RELAXED_LIFT_LIMIT_M if relaxed else PICK_POSITION_ERROR_M
    pause_at = RELAXED_LIFT_PAUSE_M if relaxed else LIFT_PAUSE_DEVIATION_M
    line_end = samples[-1][1]
    previous_point = line_start
    previous_joints = None
    sent_at = None
    peak_deviation = 0.0
    initial_height = None
    if downward_lift:
        # The inward lift bends, so measure against its two legs.
        bend = line_start + np.array([0, 0, min(0.010, line_end[2] - line_start[2])])
    for index, (_, point, joints) in enumerate(samples, start=1):
        tick = time.monotonic()
        context = f"{label}, command {index}/{len(samples)}"
        segment_start, segment_end = ((line_start, line_end) if loaded_lift or reference is not None
                                      else (previous_point, point))
        if downward_lift:
            axis_joints = joints if previous_joints is None else previous_joints
            target_axis = model.forward_kinematics(axis_joints)[:3, 2].copy()
        recovery_deadline = None
        while True:
            measured = joint_vector(robot.get_observation())
            errors = abs(joints - measured)
            worst = int(np.argmax(errors))
            if errors[worst] > MAX_COMMAND_LEAD_DEG:
                raise RuntimeError(
                    f"{context}: {JOINTS[worst]} measured {measured[worst]:.2f}°, "
                    f"next target {joints[worst]:.2f}° ({errors[worst]:.2f}° apart, "
                    f"limit {MAX_COMMAND_LEAD_DEG:g}°)."
                )
            pose = model.forward_kinematics(measured)
            if downward_lift:
                check_tool_axis(pose[:3, :3], target_axis, context, RuntimeError)
            elif reference is not None:
                check_orientation(pose[:3, :3], reference, context, RuntimeError)
            position = pose[:3, 3]
            if not np.isfinite(position).all():
                raise RuntimeError(f"{context}: non-finite modeled gripper position.")
            if verify_rise and initial_height is None:
                initial_height = position[2]
            if downward_lift:
                deviation = min(distance_to_segment(position, line_start, bend),
                                distance_to_segment(position, bend, line_end))
            else:
                deviation = distance_to_segment(position, segment_start, segment_end)
            peak_deviation = max(peak_deviation, deviation)
            if not np.isfinite(deviation) or deviation > path_limit:
                raise RuntimeError(f"{context}: modeled path deviation {deviation * 1000:.1f} mm "
                                   f"(limit {path_limit * 1000:g} mm).")
            if step_lift and previous_joints is not None:
                now = time.monotonic()
                held_error = measured - previous_joints
                worst_held = int(np.argmax(abs(held_error)))
                elapsed = now - sent_at
                if (elapsed >= LIFT_STEP_DWELL_S
                        and abs(held_error[worst_held]) <= LIFT_STEP_JOINT_TOLERANCE_DEG):
                    break
                if elapsed >= LIFT_RECOVERY_TIMEOUT_S:
                    raise RuntimeError(
                        f"{context}: previous step did not settle after {elapsed:.2f} s. "
                        f"{JOINTS[worst_held]} error {held_error[worst_held]:+.2f}° "
                        f"(limit {LIFT_STEP_JOINT_TOLERANCE_DEG:g}°), "
                        f"modeled path deviation {deviation * 1000:.1f} mm."
                    )
                time.sleep(COARSE_COMMAND_PERIOD_S)
                continue
            if step_lift or not loaded_lift or deviation < pause_at:
                break
            if recovery_deadline is None:
                recovery_deadline = time.monotonic() + LIFT_RECOVERY_TIMEOUT_S
                print(f"{context}: waiting for path deviation to settle from {deviation * 1000:.1f} mm.",
                      flush=True)
            if time.monotonic() >= recovery_deadline:
                held_error = measured - (joints if previous_joints is None else previous_joints)
                joint_report = ", ".join(f"{name} {error:+.2f}°"
                                         for name, error in zip(JOINTS, held_error))
                raise RuntimeError(f"{context}: path deviation stayed at {deviation * 1000:.1f} mm "
                                   f"after holding the last target for {LIFT_RECOVERY_TIMEOUT_S:g} s. "
                                   f"Measured minus {'initial planned' if previous_joints is None else 'held'} "
                                   f"joint target: {joint_report}.")
            time.sleep(COARSE_COMMAND_PERIOD_S)
        robot.send_action({f"{name}.pos": float(q) for name, q in zip(JOINTS, joints)})
        previous_point = point
        previous_joints = joints
        sent_at = time.monotonic()
        time.sleep(max(0.0, COARSE_COMMAND_PERIOD_S - (time.monotonic() - tick)))
    time.sleep(1)
    measured = joint_vector(robot.get_observation())
    pose = model.forward_kinematics(measured)
    if downward_lift:
        check_tool_axis(pose[:3, :3], DOWN_AXIS, f"{label} endpoint", RuntimeError)
    elif reference is not None:
        check_orientation(pose[:3, :3], reference, f"{label} endpoint", RuntimeError)
    position_error = np.linalg.norm(pose[:3, 3] - line_end)
    errors = abs(joints - measured)
    worst = int(np.argmax(errors))
    if not np.isfinite(position_error) or errors[worst] > 3 or position_error > PICK_POSITION_ERROR_M:
        raise RuntimeError(f"{label} endpoint did not settle: {JOINTS[worst]} error {errors[worst]:.2f}° "
                           f"(limit 3°), modeled position error {position_error * 1000:.1f} mm "
                           f"(limit {PICK_POSITION_ERROR_M * 1000:g} mm).")
    if verify_rise:
        measured_rise = pose[2, 3] - initial_height
        minimum_rise = max(0.001, line_end[2] - line_start[2] - PICK_POSITION_ERROR_M)
        if measured_rise < minimum_rise:
            raise RuntimeError(f"{label}: modeled rise was {measured_rise * 1000:.1f} mm "
                               f"(minimum {minimum_rise * 1000:.1f} mm).")
        print(f"{label.capitalize()} gained {measured_rise * 1000:.1f} mm in modeled height.")
    return peak_deviation, position_error


def execute_pick(plan, model, limits, port, robot_id, *, relaxed_lift_tracking=False,
                 empty_lift_test=False, step_lift=False, downward_lift=False, free_wrist_lift=False):
    hover, opened, closed, descent, lift = plan
    top = model.forward_kinematics(hover)[:3, 3].copy()
    _, bottom, grasp = descent[-1]
    reference = model.forward_kinematics(grasp)[:3, :3].copy()
    lift_targets = lift
    if free_wrist_lift:
        lift_targets = plan_free_wrist_lift(grasp, lift[-1][1][2] - bottom[2], model, limits)
    elif downward_lift:
        lift_targets = plan_downward_lift(grasp, lift[-1][1][2] - bottom[2], model, limits)
    elif step_lift:
        lift_targets = lift_step_targets(lift, grasp, model, reference)
    robot = follower(port, robot_id)
    commanded = False
    try:
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise ValueError(f"The follower calibration does not match {robot_id}.")
        measured = joint_vector(robot.get_observation())
        lower, upper = limits
        if np.any(measured < lower) or np.any(measured > upper):
            raise ValueError("Current arm pose is outside the URDF joint limits.")
        current_point = model.forward_kinematics(measured)[:3, 3].copy()
        if not np.isfinite(current_point).all():
            raise ValueError("The model returned a non-finite current gripper position.")
        at_grasp = (np.max(abs(measured - grasp)) <= 3
                    and np.linalg.norm(current_point - bottom) <= PICK_POSITION_ERROR_M)
        at_hover = (np.max(abs(measured - hover)) <= 3
                    and np.linalg.norm(current_point - top) <= PICK_POSITION_ERROR_M)
        approach = []
        if at_grasp:
            check_orientation(model.forward_kinematics(measured)[:3, :3], reference, "recorded grasp")
        elif not at_hover:
            route = plan_origin_move(measured, hover, model, limits, label="pickup hover")
            approach = list(stream_samples(route, measured, model, command_period=COARSE_COMMAND_PERIOD_S,
                                           joint_speed_deg_s=PICK_SPEED_DEG_S))
        else:
            check_orientation(model.forward_kinematics(measured)[:3, :3], reference, "pickup hover")
        if empty_lift_test:
            input("Remove the cube, clear the arm's path, and press Enter to run the empty lift test: ")
        else:
            input("Clear the arm's path and press Enter to start the automatic pickup: ")
        confirmed = joint_vector(robot.get_observation())
        if np.max(abs(confirmed - measured)) > 1:
            raise ValueError("The arm moved while waiting. Rerun to plan from its new pose.")
        commanded = True
        robot.send_action({GRIPPER: opened})
        time.sleep(1)
        if at_grasp:
            stream_pick(robot, lift, model, bottom, "move to hover", reference)
        elif approach:
            stream_pick(robot, approach, model, current_point, "move to hover")
        measured = joint_vector(robot.get_observation())
        check_start(measured, hover, tolerance=3, label="pickup hover")
        actual = model.forward_kinematics(measured)
        check_orientation(actual[:3, :3], reference, "pickup hover")
        if not np.linalg.norm(actual[:3, 3] - top) <= PICK_POSITION_ERROR_M:
            raise ValueError("The arm did not reach the pickup hover within 6 mm in the model.")
        print("At pickup hover. Descending to the taught grasp.", flush=True)
        stream_pick(robot, descent, model, top, "descent", reference)
        robot.send_action({GRIPPER: closed})
        time.sleep(1)
        if empty_lift_test:
            input("Confirm the jaws are empty and the lift is clear, then press Enter to lift: ")
        else:
            input("Visually confirm the block is held and the lift is clear, then press Enter to lift: ")
        if step_lift:
            limit = RELAXED_LIFT_LIMIT_M if relaxed_lift_tracking else PICK_POSITION_ERROR_M
            print(f"Step lift uses {len(lift_targets)} targets, waits at least {LIFT_STEP_DWELL_S:g} s between them, "
                  f"requires joints within {LIFT_STEP_JOINT_TOLERANCE_DEG:g}° of the held target, "
                  f"and stops above {limit * 1000:g} mm of path deviation.", flush=True)
        elif relaxed_lift_tracking:
            print(f"Lift path check pauses at {RELAXED_LIFT_PAUSE_M * 1000:g} mm "
                  f"and stops above {RELAXED_LIFT_LIMIT_M * 1000:g} mm.", flush=True)
        label = "empty lift" if empty_lift_test else "lift"
        lift_reference = None if free_wrist_lift else reference
        peak, endpoint = stream_pick(robot, lift_targets, model, bottom, label, lift_reference, loaded_lift=True,
                                     relaxed_lift_tracking=relaxed_lift_tracking, step_lift=step_lift,
                                     downward_lift=downward_lift, verify_rise=free_wrist_lift)
        print(f"{label.capitalize()} measured peak modeled path deviation {peak * 1000:.1f} mm, "
              f"endpoint error {endpoint * 1000:.1f} mm.")
        print("Lift complete. The arm is holding at the lift endpoint with torque enabled.")
    except (Exception, KeyboardInterrupt) as error:
        if commanded:
            reason = "interrupted" if isinstance(error, KeyboardInterrupt) else str(error)
            print(f"Pickup stopped: {reason}", file=sys.stderr, flush=True)
            release_after_failure(robot)
        raise
    finally:
        if robot.is_connected:
            robot.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("row", type=int, help="zero-based row")
    parser.add_argument("column", type=int, help="zero-based column")
    parser.add_argument("--workspace", type=Path, default=Path("vision-service/setup-data/3x6/workspace.json"))
    parser.add_argument("--pickup-poses", type=Path, default=Path("vision-service/setup-data/3x6/pickup_poses.json"))
    parser.add_argument("--urdf", type=Path, default=Path("vision-service/models/so101/so101_wrist_flex_103.urdf"))
    parser.add_argument("--port", default="/dev/tty.usbmodem5B3E0906941")
    parser.add_argument("--robot-id", default="my_awesome_follower_arm")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--teach", action="store_true", help="capture hover and grasp from the follower")
    mode.add_argument("--teach-grasp", action="store_true", help="capture one grasp and calculate a vertical hover")
    mode.add_argument("--execute", action="store_true", help="move to hover and run the pickup on the arm")
    parser.add_argument("--lift-mm", type=float, help="vertical clearance above a grasp-only pose (default: 40)")
    parser.add_argument("--gripper-open", type=float, help="open gripper target for a grasp-only pose (default: 75)")
    parser.add_argument("--relaxed-lift-tracking", action="store_true",
                        help="allow up to 8 mm modeled path deviation during the loaded lift")
    parser.add_argument("--empty-lift-test", action="store_true",
                        help="replay the pickup with the cube removed for comparison")
    lift_mode = parser.add_mutually_exclusive_group()
    lift_mode.add_argument("--step-lift", action="store_true",
                        help="hold each lift target and check joint feedback before advancing")
    lift_mode.add_argument("--downward-lift", action="store_true",
                        help="level the gripper as it rises, then tuck slightly toward the base")
    lift_mode.add_argument("--free-wrist-lift", action="store_true",
                           help="lift straight up while allowing the wrist to rotate")
    args = parser.parse_args()
    if args.relaxed_lift_tracking and not args.execute:
        parser.error("--relaxed-lift-tracking requires --execute.")
    if args.empty_lift_test and not args.execute:
        parser.error("--empty-lift-test requires --execute.")
    if args.step_lift and not args.execute:
        parser.error("--step-lift requires --execute.")
    if (args.downward_lift or args.free_wrist_lift) and (args.teach or args.teach_grasp):
        parser.error("Alternate lift modes require a saved grasp and apply to dry runs or execution.")
    cell = f"{args.row},{args.column}"
    try:
        board = json.loads(args.workspace.read_text())["board"]
        if not (0 <= args.row < board["rows"] and 0 <= args.column < board["columns"]):
            raise ValueError(f"Cell ({cell}) is outside the {board['rows']}×{board['columns']} workspace.")
        poses = json.loads(args.pickup_poses.read_text()) if args.pickup_poses.exists() else {}
        if not (args.teach or args.teach_grasp) and cell not in poses:
            raise ValueError(f"Cell ({cell}) has no pickup poses. Run with --teach-grasp first.")
        from lerobot.model.kinematics import RobotKinematics

        limits = joint_limits(args.urdf)
        model = RobotKinematics(str(args.urdf), joint_names=list(JOINTS))
        if args.teach_grasp:
            input(f"Cell ({cell}): lightly grip the block while it sits flat. Close teleop and press Enter to record: ")
            captured = {
                "grasp": read_pose(args.port, args.robot_id, include_gripper=True),
                "lift_mm": 40.0 if args.lift_mm is None else args.lift_mm,
                "gripper_open": 75.0 if args.gripper_open is None else args.gripper_open,
            }
            # Keep the physical reference so a different clearance can be tried
            # without another hardware capture.
            poses[cell] = captured
            save_poses(args.pickup_poses, poses)
            print(f"Saved the grasp reference for ({cell}) to {args.pickup_poses}.")
            plan_pick(captured, model, limits)
        elif args.teach:
            poses[cell] = teach_pick(cell, model, limits, args.port, args.robot_id)
            save_poses(args.pickup_poses, poses)
            print(f"Saved pickup poses for ({cell}) to {args.pickup_poses}.")
        else:
            selected = poses[cell].copy()
            for key, value in (("lift_mm", args.lift_mm), ("gripper_open", args.gripper_open)):
                if value is not None:
                    if "lift_mm" not in selected:
                        raise ValueError("Clearance and gripper overrides require a pose captured with --teach-grasp.")
                    selected[key] = value
            plan = plan_pick(selected, model, limits)
            if args.execute:
                execute_pick(plan, model, limits, args.port, args.robot_id,
                             relaxed_lift_tracking=args.relaxed_lift_tracking,
                             empty_lift_test=args.empty_lift_test, step_lift=args.step_lift,
                             downward_lift=args.downward_lift, free_wrist_lift=args.free_wrist_lift)
            else:
                if args.downward_lift:
                    _, bottom, grasp = plan[3][-1]
                    plan_downward_lift(grasp, plan[4][-1][1][2] - bottom[2], model, limits)
                elif args.free_wrist_lift:
                    _, bottom, grasp = plan[3][-1]
                    plan_free_wrist_lift(grasp, plan[4][-1][1][2] - bottom[2], model, limits)
                print(f"Dry run passed for ({cell}). Add --execute to move to hover and run the pickup.")
    except (KeyboardInterrupt, EOFError):
        print("Pickup interrupted.", file=sys.stderr)
        return 1
    except (ImportError, OSError, KeyError, RuntimeError, ValueError) as error:
        print(f"Pickup failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
