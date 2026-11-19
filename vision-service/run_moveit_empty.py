"""Validate and optionally replay a MoveIt plan in empty-gripper or taught-grasp mode."""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

from moveit_pick import URDF
from trace_perimeter import JOINTS, follower, hold_after_failure, joint_limits, joint_vector, release_torque

PERIOD = 0.04
SPEED = 5.0
START_TOLERANCE = 2.0
TRACK_TOLERANCE = 4.0
ENDPOINT_TOLERANCE = 3.0


def load_plan(data, speed=SPEED):
    stages = data["stages"]
    expected = ["Standoff", "Descent", "Lift"]
    if data.get("destination") is not None:
        expected += ["Transfer", "Place", "Retreat"]
    if [s["stage"] for s in stages] != expected:
        raise ValueError("Expected Standoff, Descent, and Lift stages.")
    lower, upper = joint_limits(URDF)
    previous = None
    for stage in stages:
        names = stage["joint_names"]
        if len(names) != len(JOINTS) or set(names) != set(JOINTS):
            raise ValueError("Plan joint names do not match the SO-101 arm.")
        points = stage["points"]
        times = np.array([p["seconds"] for p in points], dtype=float)
        radians = np.array([p["rad"] for p in points], dtype=float)
        if (len(times) < 2 or not np.isfinite(times).all() or abs(times[0]) > 1e-6
                or np.any(np.diff(times) <= 0) or radians.shape != (len(times), len(JOINTS))
                or not np.isfinite(radians).all()):
            raise ValueError(f"Invalid timing or joint positions in {stage['stage']}.")
        angles = np.rad2deg(radians[:, [names.index(name) for name in JOINTS]])
        if np.any(angles < lower) or np.any(angles > upper):
            raise ValueError(f"Joint limits exceeded in {stage['stage']}.")
        if previous is not None and np.max(abs(angles[0] - previous)) > 0.1:
            raise ValueError(f"Discontinuous start of {stage['stage']}.")
        previous = angles[-1]
        peak = np.max(abs(np.diff(angles, axis=0)) / np.diff(times)[:, None])
        cap = SPEED if stage["stage"] in ("Lift", "Transfer", "Place", "Retreat") else speed
        times *= max(SPEED / cap, peak / cap)
        yield stage["stage"], times, angles


def execute(stages, port, robot_id, jaw=None):
    robot = follower(port, robot_id)
    moving = False
    jaw_target = jaw["open"] if jaw is not None else None
    holding_cube = False
    try:
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            raise RuntimeError("Robot calibration does not match the configured ID.")
        for label, times, angles in stages:
            print(f"Starting {label}.", flush=True)
            if jaw is None:
                input(f"Clear the workspace and confirm the jaws are empty and open. Enter to start {label}: ")
            elif label == "Standoff":
                input("Clear the arm's path and press Enter to open the gripper and approach: ")
            elif label == "Descent":
                input("Verify the jaws are open and the cube is aligned. Press Enter to descend: ")
            elif label == "Transfer":
                input("Clear the path to the destination. Press Enter to transfer the cube: ")
            elif label == "Place":
                input("Confirm the destination support surface is aligned and clear for this cube. Press Enter to lower the cube: ")
            elif label == "Retreat":
                input("Confirm the cube is resting on the board. Press Enter to release it and retreat: ")
            else:
                input("Confirm the cube is held and the lift is clear. Press Enter to lift: ")
            measured = joint_vector(robot.get_observation())
            errors = measured - angles[0]
            tolerance = START_TOLERANCE if label == "Standoff" else ENDPOINT_TOLERANCE
            if np.max(abs(errors)) > tolerance:
                details = ", ".join(f"{name}: {error:+.1f}°" for name, error in zip(JOINTS, errors))
                raise RuntimeError(f"{label} start differs from the plan by over {tolerance:g}°: {details}")
            if jaw is not None and label in ("Standoff", "Retreat"):
                moving = True
                jaw_target = jaw["open"]
                holding_cube = False
                print("Opening gripper at " + label + ".", flush=True)
                robot.send_action({"gripper.pos": jaw_target})
                time.sleep(0.75)
            count = int(np.ceil(times[-1] / PERIOD))
            for index in range(count + 1):
                tick = time.monotonic()
                at = min(index * PERIOD, times[-1])
                target = np.array([np.interp(at, times, joint) for joint in angles.T])
                error = target - joint_vector(robot.get_observation())
                worst = int(np.argmax(abs(error)))
                if abs(error[worst]) > TRACK_TOLERANCE:
                    raise RuntimeError(f"{label} step {index}: {JOINTS[worst]} error {error[worst]:+.2f}°.")
                moving = True
                action = {f"{name}.pos": float(value) for name, value in zip(JOINTS, target)}
                if jaw_target is not None:
                    action["gripper.pos"] = jaw_target
                robot.send_action(action)
                time.sleep(max(0.0, PERIOD - (time.monotonic() - tick)))
            deadline = time.monotonic() + 2.0
            while True:
                errors = joint_vector(robot.get_observation()) - angles[-1]
                if np.max(abs(errors)) <= ENDPOINT_TOLERANCE:
                    break
                if time.monotonic() >= deadline:
                    details = ", ".join(f"{name}: {error:+.2f}°"
                                        for name, error in zip(JOINTS, errors))
                    raise RuntimeError(f"{label} did not settle within {ENDPOINT_TOLERANCE:g}° "
                                       f"after 2 seconds. Joint errors: {details}")
                robot.send_action(action)
                time.sleep(PERIOD)
            print(f"{label} complete. Largest endpoint error {np.max(abs(errors)):.2f}°.", flush=True)
            if jaw is not None and label == "Descent":
                holding_cube = True
                jaw_target = jaw["close"]
                robot.send_action({"gripper.pos": jaw_target})
                time.sleep(0.75)
        print("Replay complete.")
    except (Exception, KeyboardInterrupt) as error:
        if robot.is_connected:
            detail = str(error) or type(error).__name__
            status = "Motion stopped" if moving else "Replay stopped before sending any movement commands"
            print(f"{status}: {detail}", file=sys.stderr, flush=True)
        if moving and robot.is_connected:
            hold_after_failure(robot)
        raise
    finally:
        if robot.is_connected:
            try:
                if holding_cube:
                    print("Cube may still be held above the board. Releasing torque also removes "
                          "the gripper holding force. Support the cube and arm first.", flush=True)
                if sys.stdin.isatty() and not input(
                    "Support the arm and any held cube. Press Enter to release motor torque, "
                    "or type keep to leave it enabled: "
                ).strip():
                    release_torque(robot)
                else:
                    print("Motor torque may still be enabled.", file=sys.stderr)
            except (Exception, KeyboardInterrupt) as error:
                print(f"Torque release could not be confirmed: {error}", file=sys.stderr)
            finally:
                robot.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("--port", default="/dev/tty.usbmodem5B3E0906941")
    parser.add_argument("--robot-id", default="my_awesome_follower_arm")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--grasp", action="store_true", help="use taught jaw commands from a grasp export")
    parser.add_argument("--speed", type=float, default=SPEED,
                        help="Standoff and Descent joint speed cap in degrees/sec (up to 30)")
    args = parser.parse_args()
    try:
        if not 0 < args.speed <= 30:
            raise ValueError("--speed must be greater than 0 and at most 30°/s.")
        data = json.loads(args.plan.read_text())
        if data["empty_gripper"] is not (not args.grasp):
            raise ValueError("Use --grasp with --export-grasp plans and omit it with --export-empty plans.")
        jaw = None
        if args.grasp:
            jaw = {key: float(data["gripper"][key]) for key in ("open", "close")}
            if not 0 <= jaw["close"] < jaw["open"] <= 100:
                raise ValueError("Invalid gripper openings in the plan.")
            print(f"Taught gripper positions: open {jaw['open']:g}, grasp {jaw['close']:g}.")
        if data.get("destination") is None:
            print(f"Pickup only at ({data['cell']}). No destination was requested.")
        else:
            print(f"Pick ({data['cell']}) -> place ({data['destination']}), "
                  f"layer {data.get('drop_layer', 0)}. "
                  "Gripper stays closed through Lift, Transfer, and Place.")
        stages = list(load_plan(data, args.speed))
        for label, times, angles in stages:
            cap = SPEED if label in ("Lift", "Transfer", "Place", "Retreat") else args.speed
            print(f"{label}: {len(times)} waypoints, {times[-1]:.1f}s at up to {cap:g}°/s.")
        print("Required start (degrees): " +
              ", ".join(f"{name}={value:.2f}" for name, value in zip(JOINTS, stages[0][2][0])))
        if args.execute:
            execute(stages, args.port, args.robot_id, jaw)
        else:
            print("Dry run passed. No hardware commands sent.")
    except (ImportError, OSError, KeyError, TypeError, ValueError, RuntimeError) as error:
        parser.exit(1, f"MoveIt replay failed: {error}\n")
    except (KeyboardInterrupt, EOFError):
        parser.exit(1, "MoveIt replay interrupted.\n")


if __name__ == "__main__":
    main()
