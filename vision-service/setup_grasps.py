#!/usr/bin/env python3
"""Select a 3×6 board and teach a grasp at each cell using the overhead camera."""

import argparse
from contextlib import suppress
import faulthandler
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from moveit_pick import SETUP, URDF
from pick_block import plan_free_wrist_lift, plan_pick
from setup import project, save_setup, scan_and_select
from trace_perimeter import JOINTS, joint_limits, read_pose, save_poses


def draw_cell(frame, corners, reference_size, row, column):
    height, width = frame.shape[:2]
    if not np.isclose(width / height, reference_size[0] / reference_size[1], rtol=0.005):
        raise ValueError("Camera aspect ratio changed. Select the board again in a new output directory.")
    scale = min(1, 960 / width, 680 / height)
    size = (round(width * scale), round(height * scale))
    view = cv2.resize(frame, size)
    destination = corners * (np.array(size) / reference_size)
    transform = cv2.getPerspectiveTransform(
        np.float32([(0, 0), (6, 0), (6, 3), (0, 3)]), destination.astype(np.float32)
    )
    for lines in (
        ([(x, 0), (x, 3)] for x in range(7)),
        ([(0, y), (6, y)] for y in range(4)),
    ):
        for line in lines:
            a, b = project(transform, line).round().astype(int)
            cv2.line(view, tuple(a), tuple(b), (0, 255, 255), 1)
    cell = [(column, row), (column + 1, row), (column + 1, row + 1), (column, row + 1)]
    cv2.polylines(view, [project(transform, cell).round().astype(np.int32)], True, (0, 255, 0), 3)
    center = tuple(project(transform, [(column + 0.5, row + 0.5)])[0].round().astype(int))
    cv2.drawMarker(view, center, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
    view = cv2.copyMakeBorder(view, 0, 36, 0, 0, cv2.BORDER_CONSTANT)
    cv2.putText(view, f"Cell ({row},{column}) | Enter capture | Q pause", (8, size[1] + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return view


def teach(output, config, args):
    from lerobot.model.kinematics import RobotKinematics

    board = config["board"]
    if (board["rows"], board["columns"]) != (3, 6):
        raise ValueError("This setup requires a 3×6 workspace.")
    corners = np.asarray(board["top_down_corners_px"], dtype=float)
    reference_size = np.asarray(board["top_down_frame_size_px"], dtype=float)
    path = output / "pickup_poses.json"
    poses = json.loads(path.read_text()) if path.exists() else {}
    pending = [(r, c) for r in range(3) for c in range(6) if f"{r},{c}" not in poses]
    if not pending:
        return

    model = RobotKinematics(str(URDF), target_frame_name="gripper_frame_link", joint_names=list(JOINTS))
    limits = joint_limits(URDF)
    index = config["cameras"]["top_down"]
    backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
    camera = cv2.VideoCapture(index, backend)
    window = "Teach board grasps"
    try:
        if not camera.isOpened():
            raise RuntimeError(f"Could not open overhead camera {index}.")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, reference_size[0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, reference_size[1])
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        print("Verify the grid overlay is aligned. Keep the camera and board fixed.")
        for row, column in pending:
            print(f"Cell ({row},{column}): center a cube on the cross and lightly grip it while it sits flat. "
                  "Close teleop, focus the camera window, and press Enter.", flush=True)
            missed_frames = 0
            while True:
                ok, frame = camera.read()
                if not ok or frame is None or not frame.size:
                    missed_frames += 1
                    if missed_frames >= 5:
                        raise RuntimeError(f"No frames from overhead camera {index}.")
                    continue
                missed_frames = 0
                cv2.imshow(window, draw_cell(frame, corners, reference_size, row, column))
                key = cv2.waitKey(20) & 0xFF
                if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    raise KeyboardInterrupt
                if key not in (10, 13):
                    continue
                try:
                    pose = {
                        "grasp": read_pose(args.port, args.robot_id, include_gripper=True),
                        "lift_mm": 40.0,
                        "gripper_open": 75.0,
                    }
                    _, _, _, descent, lift = plan_pick(pose, model, limits)
                    _, bottom, grasp = descent[-1]
                    plan_free_wrist_lift(grasp, lift[-1][1][2] - bottom[2], model, limits)
                except (OSError, RuntimeError, ValueError) as error:
                    print(f"Capture rejected. Check the pose or close teleop and retry Enter. {error}",
                          file=sys.stderr, flush=True)
                    continue
                poses[f"{row},{column}"] = pose
                save_poses(path, poses)
                print(f"Saved cell ({row},{column}). You can restart teleop for the next cell.", flush=True)
                break
    finally:
        camera.release()
        with suppress(cv2.error):
            cv2.destroyWindow(window)


def main():
    faulthandler.enable()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=SETUP, help="setup directory to create or resume")
    parser.add_argument("--top-camera", type=int, default=0, help="camera used for a new workspace")
    parser.add_argument("--port", default="/dev/tty.usbmodem5B3E0906941")
    parser.add_argument("--robot-id", default="my_awesome_follower_arm")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    workspace = output / "workspace.json"
    try:
        if not workspace.exists():
            if output.exists():
                raise ValueError(f"{output} exists without workspace.json. Choose a new output directory.")
            frame, corners = scan_and_select(args.top_camera, 6)
            save_setup(output, args.top_camera, frame, corners, 6)
        teach(output, json.loads(workspace.read_text()), args)
    except KeyboardInterrupt:
        print(f"Setup paused. Rerun with the same --output to resume from {output}.")
        return 130
    except (ImportError, OSError, RuntimeError, ValueError, KeyError, cv2.error) as error:
        parser.exit(1, f"Grasp setup failed: {error}\n")
    print(f"All 18 cells are taught in {output / 'pickup_poses.json'}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
