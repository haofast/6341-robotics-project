#!/usr/bin/env python3
"""Calibrate an overhead/side camera pair and measure a tabletop in millimeters.

SETUP NOTES -- READ BEFORE STARTING
==================================
This is the calibration and manual measurement stage of a pick-and-place project.
It saves a shared table coordinate system; it does not detect arbitrary objects,
infer hidden objects in stacks, determine robot reach, or command a robot.

1. Fix both cameras in place. Keep focus, zoom, and resolution fixed after lens
   calibration. Changing those requires new lens calibration. Moving a camera
   requires new workspace calibration. Give the app Camera access on macOS.
2. Print a checkerboard, mount it flat on a rigid backing, and measure a square
   with a ruler. The defaults are 9 x 6 INNER CORNERS (10 x 7 printed squares).
   Use --columns / --rows if your board differs. Printer scaling matters.
3. For each camera, capture 25 different board poses: move it across the image,
   tilt it in different directions, and vary distance. Keep the entire pattern
   visible. Press SPACE to capture; Q or ESC cancels without saving.
4. Mark four corners of a measured rectangular working area ON the table:
       far left (0, depth) -------- far right (width, depth)
                 |                           |
       near left (0, 0) ----------- near right (width, 0)
   X runs right, Y runs away, and Z runs up. These are physical labels, regardless
   of how a camera rotates the image. Click the SAME physical corners in BOTH
   images: near left, near right, far right, far left. The side camera must look
   slightly down so all four marks are visible and not nearly collinear. Move
   objects out of the way. R clears clicks; ENTER accepts; Q or ESC cancels.
5. In 'view', click tabletop marks to check X/Y. The default assumes Z=0, so a
   click on a tall object's top is NOT its correct table position. If its height
   is known, use --plane-height-mm to intersect that height plane instead.
6. In 'measure', keep objects stationary, press SPACE to freeze both views, then
   click the SAME visible physical feature (e.g. one block corner) in each view.
   Press ENTER for X/Y/Z, with Z equal to height above the table. Bounding-box
   centers or different corners are not matching features. Cameras are not
   hardware-synchronized. SPACE takes a new pair; R clears clicks.
7. Check independent ruler-measured marks and blocks across the workspace.
   Reprojection error is a geometry check, NOT a guarantee of millimeter accuracy.
   Calibrate table-to-robot coordinates and the gripper separately before use.

RUN FROM THE PROJECT ROOT (replace square/table dimensions with YOUR measurements)
    python3 -m pip install -r vision-service/requirements.txt
    python3 vision-service/calibrate_workspace.py intrinsics --camera 0 --square-mm 25
    python3 vision-service/calibrate_workspace.py intrinsics --camera 1 --square-mm 25
    python3 vision-service/calibrate_workspace.py workspace --overhead-camera 0 --side-camera 1 --table-width-mm 600 --table-depth-mm 400
    python3 vision-service/calibrate_workspace.py view
    python3 vision-service/calibrate_workspace.py measure

Camera 0 is assumed overhead and 1 side only in the example; set the actual IDs.
Files are saved to computervision/calibration/ by default. Each command supports
--calibration-dir. 'view' and 'measure' load camera IDs/resolutions from workspace.json.
Useful references:
    https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
    https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack, contextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
from typing import Any, Iterator

try:
    import cv2
    import numpy as np
except ImportError as exc:
    raise SystemExit(
        "Install dependencies: python3 -m pip install -r vision-service/requirements.txt"
    ) from exc


DEFAULT_DIR = Path(__file__).resolve().parents[1] / "computervision" / "calibration"
CORNER_NAMES = ("near left / origin", "near right / +X", "far right / +X,+Y", "far left / +Y")
QUIT_KEYS = (27, ord("q"), ord("Q"))


class CalibrationError(ValueError):
    """An invalid calibration or unreliable measurement."""


def positive_float(value: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("Must be a finite number greater than zero.")
    return result


def positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("Must be greater than zero.")
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    commands = parser.add_subparsers(dest="command", required=True)
    for name, help_text in (
        ("intrinsics", "Calibrate one camera's lens with a checkerboard."),
        ("workspace", "Register both cameras to a measured rectangle on the table."),
        ("view", "Click points in the overhead view for table X/Y coordinates."),
        ("measure", "Click a matching feature in both views for X/Y/Z coordinates."),
    ):
        command = commands.add_parser(name, help=help_text)
        command.add_argument("--calibration-dir", type=Path, default=DEFAULT_DIR)
        if name in ("intrinsics", "workspace", "measure"):
            command.add_argument("--max-error-px", type=positive_float, default=2.0,
                                 help="Maximum allowed reprojection error in pixels (default: 2).")
        if name == "intrinsics":
            command.add_argument("--camera", type=int, required=True)
            command.add_argument("--square-mm", type=positive_float, required=True)
            command.add_argument("--columns", type=positive_int, default=9, help="Inner corner columns.")
            command.add_argument("--rows", type=positive_int, default=6, help="Inner corner rows.")
            command.add_argument("--samples", type=positive_int, default=25)
            command.add_argument("--width", type=positive_int, default=1280)
            command.add_argument("--height", type=positive_int, default=720)
        elif name == "workspace":
            command.add_argument("--overhead-camera", type=int, default=0)
            command.add_argument("--side-camera", type=int, default=1)
            command.add_argument("--table-width-mm", type=positive_float, required=True)
            command.add_argument("--table-depth-mm", type=positive_float, required=True)
        elif name == "view":
            command.add_argument("--plane-height-mm", type=float, default=0.0,
                                 help="Known height of clicked surface above the table (default: 0).")
        else:
            command.add_argument("--min-angle-deg", type=positive_float, default=2.0,
                                 help="Reject triangulation with nearly parallel viewing rays.")
    return parser.parse_args(argv)


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def save_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise CalibrationError(f"Missing {path}. Run the preceding calibration step first.")
    data = json.loads(path.read_text())
    if data.get("version") != 1 or data.get("units") != "mm":
        raise CalibrationError(f"Unsupported calibration format in {path}.")
    return data


def camera_matrices(camera: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(camera["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(camera["distortion_coefficients"], dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise CalibrationError("Invalid camera matrix or distortion coefficients.")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or abs(np.linalg.det(matrix)) < 1e-12:
        raise CalibrationError("Invalid focal lengths or singular camera matrix.")
    return matrix, distortion


def camera_pose(camera: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    rotation, _ = cv2.Rodrigues(np.asarray(camera["rvec"], dtype=np.float64).reshape(3, 1))
    translation = np.asarray(camera["tvec_mm"], dtype=np.float64).reshape(3, 1)
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise CalibrationError("Invalid camera pose.")
    return rotation, translation


@contextmanager
def open_camera(index: int, image_size: tuple[int, int], exact: bool = True) -> Iterator[Any]:
    if index < 0:
        raise CalibrationError("Camera IDs must be nonnegative.")
    cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION) if sys.platform == "darwin" else cv2.VideoCapture(index)
    try:
        if not cap.isOpened():
            raise CalibrationError(f"Cannot open camera {index}. Check its ID and Camera permission.")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size[1])
        for _ in range(8):
            frame = read_frame(cap)
            time.sleep(0.03)
        actual = (frame.shape[1], frame.shape[0])
        if exact and actual != image_size:
            raise CalibrationError(f"Camera {index} returned {actual}; calibration needs {image_size}.")
        yield cap
    finally:
        cap.release()


def read_frame(cap: Any) -> np.ndarray:
    ok, frame = cap.read()
    if not ok or frame is None or not frame.size:
        raise CalibrationError("Camera did not return an image.")
    return frame


def read_pair(caps: list[Any], cameras: list[dict[str, Any]]) -> list[np.ndarray]:
    # Grab from both before decoding to reduce delay; this is not hardware sync.
    if not all([cap.grab() for cap in caps]):
        raise CalibrationError("Could not capture from both cameras.")
    frames = []
    for cap, camera in zip(caps, cameras):
        ok, frame = cap.retrieve()
        if not ok or frame is None or not frame.size:
            raise CalibrationError("Camera did not return an image.")
        check_image_size(frame, camera)
        frames.append(frame)
    return frames


def check_image_size(frame: np.ndarray, camera: dict[str, Any]) -> None:
    if [frame.shape[1], frame.shape[0]] != camera["image_size"]:
        raise CalibrationError("Capture resolution changed. Reopen at the calibrated resolution.")


def label(frame: np.ndarray, lines: list[str]) -> None:
    for row, line in enumerate(lines):
        point = (12, 28 + row * 26)
        cv2.putText(frame, line, point, cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, line, point, cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)


def window_open(name: str) -> bool:
    return cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1


def calibrate_intrinsics(args: argparse.Namespace) -> None:
    if min(args.columns, args.rows) < 3 or args.samples < 10:
        raise CalibrationError("Use at least 3 inner corners on each axis and at least 10 samples.")
    pattern = (args.columns, args.rows)
    board_points = np.zeros((args.columns * args.rows, 3), dtype=np.float32)
    board_points[:, :2] = np.mgrid[:args.columns, :args.rows].T.reshape(-1, 2) * args.square_mm
    samples: list[np.ndarray] = []
    name = f"Lens calibration / camera {args.camera}"
    print("Move and tilt the board between samples. SPACE captures; Q cancels.")
    with open_camera(args.camera, (args.width, args.height), exact=False) as cap:
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        image_size = None
        notice = "Show the whole checkerboard."
        while len(samples) < args.samples:
            frame = read_frame(cap)
            current_size = (frame.shape[1], frame.shape[0])
            if image_size is not None and current_size != image_size:
                raise CalibrationError("Capture resolution changed during lens calibration.")
            image_size = current_size
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            )
            if found:
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            display = frame.copy()
            if found:
                cv2.drawChessboardCorners(display, pattern, corners, found)
            label(display, [f"Samples: {len(samples)}/{args.samples} | SPACE capture | Q cancel",
                            "Board found" if found else "Board not found", notice])
            cv2.imshow(name, display)
            key = cv2.waitKey(20) & 0xFF
            if key in QUIT_KEYS or not window_open(name):
                raise CalibrationError("Calibration canceled; no calibration was saved.")
            if key == ord(" ") and found:
                diagonal = np.hypot(*image_size)
                if any(np.sqrt(np.mean((corners - old) ** 2)) < diagonal * 0.015 for old in samples):
                    notice = "Too similar to a saved pose. Move, tilt, or change distance."
                    continue
                samples.append(corners.copy())
                notice = "Captured. Move and tilt the board before the next sample."
                print(f"Captured {len(samples)}/{args.samples}")
        cv2.destroyWindow(name)
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        [board_points.copy() for _ in samples], samples, image_size, None, None
    )
    if not np.isfinite(rms) or rms > args.max_error_px:
        raise CalibrationError(f"Lens reprojection RMS is {rms:.2f}px; limit is {args.max_error_px:.2f}px. "
                               "Repeat with sharper images and more varied poses.")
    per_view = []
    for observed, rvec, tvec in zip(samples, rvecs, tvecs):
        projected, _ = cv2.projectPoints(board_points, rvec, tvec, matrix, distortion)
        per_view.append(float(np.sqrt(np.mean(np.sum((observed - projected) ** 2, axis=2)))))
    data = {"version": 1, "units": "mm", "created_at": timestamp(), "camera_id": args.camera,
            "image_size": list(image_size), "camera_matrix": matrix.tolist(),
            "distortion_coefficients": distortion.tolist(), "rms_error_px": float(rms),
            "per_view_rms_px": per_view, "board_inner_corners": list(pattern),
            "square_mm": args.square_mm, "sample_count": len(samples)}
    path = args.calibration_dir / f"camera-{args.camera}.json"
    save_json(path, data)
    print(f"Saved {path}\nLens reprojection RMS: {rms:.3f}px. Now calibrate the workspace.")


def table_corners(width: float, depth: float) -> np.ndarray:
    if not np.isfinite([width, depth]).all() or width <= 0 or depth <= 0:
        raise CalibrationError("Table dimensions must be positive and finite.")
    return np.array([[0, 0, 0], [width, 0, 0], [width, depth, 0], [0, depth, 0]], dtype=np.float64)


def validate_corner_clicks(pixels: np.ndarray, image_size: list[int]) -> None:
    if pixels.shape != (4, 2) or not np.isfinite(pixels).all():
        raise CalibrationError("Select exactly four finite corner points.")
    contour = pixels.astype(np.float32).reshape(-1, 1, 2)
    if not cv2.isContourConvex(contour):
        raise CalibrationError("Corners cross or repeat. Select the four perimeter corners in the stated order.")
    area = abs(cv2.contourArea(contour))
    distances = np.linalg.norm(pixels - np.roll(pixels, 1, axis=0), axis=1)
    if area < image_size[0] * image_size[1] * 0.005 or min(distances) < 15:
        raise CalibrationError("Table marks are too small or nearly edge-on. Tilt the side camera down "
                               "or choose a larger visible rectangle, then repeat workspace calibration.")


def solve_table_pose(camera: dict[str, Any], pixels: np.ndarray,
                     width: float, depth: float, max_error_px: float) -> dict[str, Any]:
    validate_corner_clicks(pixels, camera["image_size"])
    world = table_corners(width, depth)
    matrix, distortion = camera_matrices(camera)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(world, pixels, matrix, distortion, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if ok:
        for rvec, tvec in zip(rvecs, tvecs):
            rotation, _ = cv2.Rodrigues(rvec)
            camera_center = (-rotation.T @ tvec).reshape(3)
            points_in_camera = (rotation @ world.T + tvec).T
            if camera_center[2] <= 0 or np.any(points_in_camera[:, 2] <= 0):
                continue
            projected, _ = cv2.projectPoints(world, rvec, tvec, matrix, distortion)
            errors = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)
            candidates.append((float(np.sqrt(np.mean(errors ** 2))), float(max(errors)), rvec, tvec))
    if not candidates:
        raise CalibrationError("No pose with the camera above the table and marks in front of it. "
                               "Check the physical corner order and lens calibration.")
    rms, worst, rvec, tvec = min(candidates, key=lambda candidate: candidate[0])
    if worst > max_error_px:
        raise CalibrationError(f"Worst table-corner reprojection error {worst:.2f}px exceeds "
                               f"{max_error_px:.2f}px. Check clicks and table dimensions.")
    rotation, _ = cv2.Rodrigues(rvec)
    homography = matrix @ np.column_stack((rotation[:, :2], tvec.reshape(3)))
    if abs(np.linalg.det(homography)) < 1e-12:
        raise CalibrationError("Table projection is singular; the camera is edge-on to the table.")
    return {**camera, "rvec": rvec.reshape(3).tolist(), "tvec_mm": tvec.reshape(3).tolist(),
            "table_corner_pixels": pixels.tolist(), "table_reprojection_rms_px": rms,
            "table_reprojection_max_px": worst,
            "undistorted_pixel_to_table_homography": np.linalg.inv(homography).tolist()}


def select_corners(frame: np.ndarray, role: str) -> np.ndarray:
    name = f"Table corners / {role}"
    points: list[tuple[int, int]] = []

    def clicked(event: int, x: int, y: int, flags: int, parameter: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((x, y))

    cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(name, clicked)
    try:
        while True:
            display = frame.copy()
            for index, point in enumerate(points):
                cv2.circle(display, point, 6, (0, 255, 255), -1)
                cv2.putText(display, str(index + 1), (point[0] + 8, point[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            next_point = CORNER_NAMES[len(points)] if len(points) < 4 else "All selected. ENTER accepts."
            label(display, [f"{role}: click SAME physical corners in order", f"Next: {next_point}",
                            "1 origin, 2 +X, 3 +X,+Y, 4 +Y | R reset | ENTER accept | Q cancel"])
            cv2.imshow(name, display)
            key = cv2.waitKey(20) & 0xFF
            if key in QUIT_KEYS or not window_open(name):
                raise CalibrationError("Workspace calibration canceled; no workspace was saved.")
            if key in (ord("r"), ord("R")):
                points.clear()
            if key in (10, 13) and len(points) == 4:
                return np.asarray(points, dtype=np.float64)
    finally:
        cv2.destroyWindow(name)


def calibrate_workspace(args: argparse.Namespace) -> None:
    ids = [args.overhead_camera, args.side_camera]
    if ids[0] == ids[1]:
        raise CalibrationError("Overhead and side camera IDs must differ.")
    cameras = [load_json(args.calibration_dir / f"camera-{index}.json") for index in ids]
    for index, camera in zip(ids, cameras):
        if camera["camera_id"] != index:
            raise CalibrationError("Lens calibration file has the wrong camera ID.")
    with ExitStack() as stack:
        caps = [stack.enter_context(open_camera(camera["camera_id"], tuple(camera["image_size"])))
                for camera in cameras]
        frames = read_pair(caps, cameras)
        poses = []
        for role, frame, camera in zip(("overhead", "side"), frames, cameras):
            pixels = select_corners(frame, role)
            pose = solve_table_pose(camera, pixels, args.table_width_mm, args.table_depth_mm,
                                    args.max_error_px)
            pose["role"] = role
            poses.append(pose)
            print(f"{role}: corner reprojection RMS {pose['table_reprojection_rms_px']:.3f}px")
    data = {"version": 1, "units": "mm", "created_at": timestamp(),
            "frame": "origin=near-left; X=right; Y=away; Z=up; tabletop Z=0",
            "table_width_mm": args.table_width_mm, "table_depth_mm": args.table_depth_mm,
            "cameras": {camera["role"]: camera for camera in poses},
            "note": "Measured table boundary only; robot reach and clearance are not calibrated."}
    path = args.calibration_dir / "workspace.json"
    save_json(path, data)
    print(f"Saved {path}\nVerify independent table marks with 'view' and known blocks with 'measure'.")


def pixel_ray(camera: dict[str, Any], pixel: tuple[float, float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    point = np.asarray(pixel, dtype=np.float64)
    if point.shape != (2,) or not np.isfinite(point).all():
        raise CalibrationError("Pixel coordinates must be two finite numbers.")
    matrix, distortion = camera_matrices(camera)
    normalized = cv2.undistortPoints(point.reshape(1, 1, 2), matrix, distortion).reshape(2)
    rotation, translation = camera_pose(camera)
    origin = (-rotation.T @ translation).reshape(3)
    direction = rotation.T @ np.array([normalized[0], normalized[1], 1.0])
    return origin, direction / np.linalg.norm(direction)


def pixel_to_plane(camera: dict[str, Any], pixel: tuple[float, float] | np.ndarray,
                   height_mm: float = 0.0) -> np.ndarray:
    if not np.isfinite(height_mm):
        raise CalibrationError("Plane height must be finite.")
    origin, direction = pixel_ray(camera, pixel)
    if abs(direction[2]) < 1e-8:
        raise CalibrationError("Viewing ray is parallel to the selected height plane.")
    distance = (height_mm - origin[2]) / direction[2]
    if distance <= 0:
        raise CalibrationError("The selected height plane is behind the camera.")
    return origin + distance * direction


def inside_table(point: np.ndarray, workspace: dict[str, Any]) -> bool:
    return bool(0 <= point[0] <= workspace["table_width_mm"] and
                0 <= point[1] <= workspace["table_depth_mm"])


def triangulate_feature(cameras: list[dict[str, Any]], pixels: list[tuple[float, float]],
                        max_error_px: float = 2.0, min_angle_deg: float = 2.0) -> dict[str, Any]:
    if len(cameras) != 2 or len(pixels) != 2:
        raise CalibrationError("Triangulation needs two cameras and one matching point per camera.")
    rays = [pixel_ray(camera, pixel) for camera, pixel in zip(cameras, pixels)]
    cosine = np.clip(abs(np.dot(rays[0][1], rays[1][1])), 0.0, 1.0)
    angle = float(np.degrees(np.arccos(cosine)))
    if angle < min_angle_deg:
        raise CalibrationError(f"Viewing rays differ by only {angle:.2f} degrees; depth is unreliable.")
    projections, normalized = [], []
    for camera, pixel in zip(cameras, pixels):
        matrix, distortion = camera_matrices(camera)
        rotation, translation = camera_pose(camera)
        projections.append(np.column_stack((rotation, translation)))
        normalized.append(cv2.undistortPoints(np.asarray(pixel, dtype=np.float64).reshape(1, 1, 2),
                                              matrix, distortion).reshape(2, 1))
    homogeneous = cv2.triangulatePoints(projections[0], projections[1], normalized[0], normalized[1]).reshape(4)
    if abs(homogeneous[3]) < 1e-10:
        raise CalibrationError("Point is too far away to triangulate reliably.")
    point = homogeneous[:3] / homogeneous[3]
    if not np.isfinite(point).all():
        raise CalibrationError("Triangulation produced an invalid point.")
    errors = []
    for camera, pixel in zip(cameras, pixels):
        matrix, distortion = camera_matrices(camera)
        rotation, translation = camera_pose(camera)
        if (rotation @ point + translation.reshape(3))[2] <= 0:
            raise CalibrationError("Matching clicks place the point behind a camera. Check the feature match.")
        projected, _ = cv2.projectPoints(point.reshape(1, 3), np.asarray(camera["rvec"], dtype=np.float64),
                                         translation, matrix, distortion)
        errors.append(float(np.linalg.norm(projected.reshape(2) - pixel)))
    if max(errors) > max_error_px:
        raise CalibrationError(f"Reprojection errors {errors[0]:.2f}/{errors[1]:.2f}px exceed the limit. "
                               "Click the same physical point more carefully in both views.")
    if point[2] < -5.0:
        raise CalibrationError("Point is below the tabletop. Check the corner order and feature match.")
    return {"position_mm": dict(zip(("x", "y", "z"), map(float, point))),
            "reprojection_error_px": errors, "ray_angle_deg": angle}


def project_world(camera: dict[str, Any], points: np.ndarray) -> np.ndarray:
    matrix, distortion = camera_matrices(camera)
    projected, _ = cv2.projectPoints(np.asarray(points, dtype=np.float64),
                                     np.asarray(camera["rvec"], dtype=np.float64),
                                     np.asarray(camera["tvec_mm"], dtype=np.float64), matrix, distortion)
    return projected.reshape(-1, 2)


def draw_table(frame: np.ndarray, camera: dict[str, Any], workspace: dict[str, Any]) -> None:
    width, depth = workspace["table_width_mm"], workspace["table_depth_mm"]
    # Sample edges so distortion is reflected in the displayed grid as well.
    for fraction in np.linspace(0, 1, 5):
        for start, end in (([fraction * width, 0, 0], [fraction * width, depth, 0]),
                           ([0, fraction * depth, 0], [width, fraction * depth, 0])):
            pixels = project_world(camera, np.linspace(start, end, 30))
            if np.isfinite(pixels).all() and np.max(np.abs(pixels)) < 1e7:
                cv2.polylines(frame, [np.rint(pixels).astype(np.int32)], False, (0, 190, 0), 1, cv2.LINE_AA)


def view_table(args: argparse.Namespace, workspace: dict[str, Any]) -> None:
    if not np.isfinite(args.plane_height_mm) or args.plane_height_mm < 0:
        raise CalibrationError("Plane height must be finite and nonnegative.")
    camera = workspace["cameras"]["overhead"]
    name = "Overhead / click to measure"
    selected: list[tuple[int, int]] = []

    def clicked(event: int, x: int, y: int, flags: int, parameter: Any) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            selected[:] = [(x, y)]
            try:
                point = pixel_to_plane(camera, (x, y), args.plane_height_mm)
                print(json.dumps({"position_mm": dict(zip(("x", "y", "z"), map(float, point))),
                                  "inside_table_xy": inside_table(point, workspace),
                                  "mode": "intersection_with_known_height_plane"}, allow_nan=False))
            except CalibrationError as exc:
                print(f"Measurement rejected: {exc}", file=sys.stderr)

    with open_camera(camera["camera_id"], tuple(camera["image_size"])) as cap:
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(name, clicked)
        while True:
            frame = read_frame(cap)
            check_image_size(frame, camera)
            draw_table(frame, camera, workspace)
            lines = [f"Click points at Z={args.plane_height_mm:g}mm | Grid is on tabletop | Q quit"]
            if selected:
                try:
                    point = pixel_to_plane(camera, selected[0], args.plane_height_mm)
                    status = "inside table" if inside_table(point, workspace) else "OUTSIDE table"
                    lines.append(f"X={point[0]:.1f}  Y={point[1]:.1f}  Z={point[2]:.1f} mm | {status}")
                    cv2.circle(frame, selected[0], 6, (0, 255, 255), -1)
                except CalibrationError as exc:
                    lines.append(str(exc))
            label(frame, lines)
            cv2.imshow(name, frame)
            if cv2.waitKey(20) & 0xFF in QUIT_KEYS or not window_open(name):
                return


def measure_points(args: argparse.Namespace, workspace: dict[str, Any]) -> None:
    if args.min_angle_deg >= 90:
        raise CalibrationError("Minimum ray angle must be less than 90 degrees.")
    roles = ("overhead", "side")
    cameras = [workspace["cameras"][role] for role in roles]
    names = [f"3D measurement / {role}" for role in roles]
    selected: list[tuple[int, int] | None] = [None, None]
    frozen: list[np.ndarray] | None = None
    pair_time = ""
    result_text = "SPACE freezes a pair. Keep the object stationary."

    def callback(index: int) -> Any:
        def clicked(event: int, x: int, y: int, flags: int, parameter: Any) -> None:
            if event == cv2.EVENT_LBUTTONDOWN and frozen is not None:
                selected[index] = (x, y)
        return clicked

    print("SPACE freezes both images. Click the SAME physical point in each. ENTER measures.")
    with ExitStack() as stack:
        caps = [stack.enter_context(open_camera(camera["camera_id"], tuple(camera["image_size"])))
                for camera in cameras]
        for index, name in enumerate(names):
            cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(name, callback(index))
        while True:
            frames = read_pair(caps, cameras) if frozen is None else frozen
            for index, (name, frame, camera) in enumerate(zip(names, frames, cameras)):
                display = frame.copy()
                draw_table(display, camera, workspace)
                if selected[index] is not None:
                    cv2.circle(display, selected[index], 6, (0, 255, 255), -1)
                label(display, ["FROZEN: click matching feature" if frozen is not None else "LIVE preview",
                                "SPACE new frozen pair | R clear | ENTER measure | Q quit", result_text])
                cv2.imshow(name, display)
            key = cv2.waitKey(20) & 0xFF
            if key in QUIT_KEYS or not all(window_open(name) for name in names):
                return
            if key == ord(" "):
                frozen = read_pair(caps, cameras)
                pair_time = timestamp()
                selected[:] = [None, None]
                result_text = "Click the same visible block corner in each image."
            if key in (ord("r"), ord("R")):
                selected[:] = [None, None]
                result_text = "Clicks cleared. Select matching points."
            if key in (10, 13) and all(point is not None for point in selected):
                try:
                    result = triangulate_feature(cameras, selected, args.max_error_px, args.min_angle_deg)
                    position = result["position_mm"]
                    result_text = f"X={position['x']:.1f} Y={position['y']:.1f} Z={position['z']:.1f} mm"
                    result.update({"measured_at": timestamp(), "pair_captured_at": pair_time,
                                   "workspace_created_at": workspace["created_at"],
                                   "inside_table_xy": inside_table(np.array(list(position.values())), workspace),
                                   "camera_pixels": {role: point for role, point in zip(roles, selected)},
                                   "mode": "manual_matching_feature", "units": "mm"})
                    args.calibration_dir.mkdir(parents=True, exist_ok=True)
                    with (args.calibration_dir / "measurements.jsonl").open("a") as output:
                        output.write(json.dumps(result, allow_nan=False) + "\n")
                    print(json.dumps(result, allow_nan=False))
                except CalibrationError as exc:
                    result_text = f"Rejected: {exc}"
                    print(result_text, file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.command == "intrinsics":
            calibrate_intrinsics(args)
        elif args.command == "workspace":
            calibrate_workspace(args)
        else:
            workspace = load_json(args.calibration_dir / "workspace.json")
            if args.command == "view":
                view_table(args, workspace)
            else:
                measure_points(args, workspace)
        return 0
    except (CalibrationError, OSError, KeyError, cv2.error) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Canceled.", file=sys.stderr)
        return 130
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
