#!/usr/bin/env python3
"""Estimate a newly placed object's location using cameras 0 and 1.

This script learns an empty workspace, then looks for the largest visual change
after an object is placed in the area.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import cv2
except ImportError as exc:
    raise SystemExit(
        "OpenCV is required. Install it with: python3 -m pip install opencv-python"
    ) from exc


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "computervision"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Locate a random object using camera 0 and camera 1."
    )
    parser.add_argument("--camera-x", type=int, default=0, help="Camera for x axis.")
    parser.add_argument("--camera-y", type=int, default=1, help="Camera for y axis.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Directory for annotated captures. Default: {DEFAULT_OUTPUT_DIR}",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--background-samples",
        type=int,
        default=20,
        help="Frames to average when learning the empty workspace.",
    )
    parser.add_argument(
        "--capture-samples",
        type=int,
        default=8,
        help="Frames to read before measuring the object.",
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=55,
        help="Difference threshold from the empty workspace.",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=800.0,
        help="Smallest changed area to count as an object.",
    )
    parser.add_argument(
        "--max-area-ratio",
        type=float,
        default=0.35,
        help="Ignore detections larger than this fraction of the camera frame.",
    )
    parser.add_argument(
        "--blur-size",
        type=int,
        default=11,
        help="Odd blur kernel size. Smaller values produce tighter object edges.",
    )
    parser.add_argument(
        "--open-iterations",
        type=int,
        default=2,
        help="Noise cleanup passes before contour detection.",
    )
    parser.add_argument(
        "--close-iterations",
        type=int,
        default=1,
        help="Small hole-filling passes before contour detection.",
    )
    parser.add_argument(
        "--dilate-iterations",
        type=int,
        default=0,
        help="Mask expansion passes. Increase only if the object is fragmented.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print only JSON results.",
    )
    return parser.parse_args()


def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture:
    if sys.platform == "darwin":
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    else:
        cap = cv2.VideoCapture(index)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def read_frame(cap: cv2.VideoCapture, samples: int) -> Any | None:
    frame = None
    ok = False
    for _ in range(max(1, samples)):
        ok, frame = cap.read()
        time.sleep(0.04)
    if not ok or frame is None or not frame.size:
        return None
    return frame


def odd_kernel_size(value: int) -> int:
    value = max(3, value)
    return value if value % 2 else value + 1


def grayscale_blur(frame: Any, blur_size: int) -> Any:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kernel_size = odd_kernel_size(blur_size)
    return cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)


def learn_background(cap: cv2.VideoCapture, samples: int, blur_size: int) -> Any | None:
    frames = []
    for _ in range(max(1, samples)):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size:
            frames.append(grayscale_blur(frame, blur_size).astype("float"))
        time.sleep(0.04)

    if not frames:
        return None

    background = frames[0]
    for frame in frames[1:]:
        cv2.accumulateWeighted(frame, background, 1.0 / len(frames))
    return cv2.convertScaleAbs(background)


def detect_largest_change(
    frame: Any,
    background: Any,
    threshold: int,
    min_area: float,
    max_area_ratio: float,
    blur_size: int,
    open_iterations: int,
    close_iterations: int,
    dilate_iterations: int,
) -> dict[str, Any] | None:
    blurred = grayscale_blur(frame, blur_size)
    diff = cv2.absdiff(background, blurred)
    _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    if open_iterations > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=open_iterations)
    if close_iterations > 0:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, None, iterations=close_iterations)
    if dilate_iterations > 0:
        mask = cv2.dilate(mask, None, iterations=dilate_iterations)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_h, frame_w = frame.shape[:2]
    max_area = frame_h * frame_w * max_area_ratio
    contours = [
        contour
        for contour in contours
        if min_area <= cv2.contourArea(contour) <= max_area
    ]
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    area = float(cv2.contourArea(contour))
    center_x = x + w / 2
    center_y = y + h / 2

    return {
        "bbox_px": {"x": x, "y": y, "width": w, "height": h},
        "center_px": {"x": round(center_x, 2), "y": round(center_y, 2)},
        "center_normalized": {
            "x": round(center_x / frame_w, 4),
            "y": round(center_y / frame_h, 4),
        },
        "area_px": round(area, 2),
        "area_ratio": round(area / (frame_h * frame_w), 4),
    }


def annotate_frame(frame: Any, detection: dict[str, Any] | None, label: str) -> Any:
    annotated = frame.copy()
    cv2.putText(
        annotated,
        label,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        3,
        cv2.LINE_AA,
    )

    if detection is None:
        cv2.putText(
            annotated,
            "No object detected",
            (20, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    box = detection["bbox_px"]
    center = detection["center_px"]
    cv2.rectangle(
        annotated,
        (box["x"], box["y"]),
        (box["x"] + box["width"], box["y"] + box["height"]),
        (0, 255, 0),
        3,
    )
    cv2.circle(annotated, (int(center["x"]), int(center["y"])), 7, (0, 255, 255), -1)
    cv2.putText(
        annotated,
        f"norm=({detection['center_normalized']['x']}, {detection['center_normalized']['y']})",
        (20, 85),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return annotated


def measure_camera(
    cap: cv2.VideoCapture,
    background: Any,
    camera_index: int,
    axis_name: str,
    args: argparse.Namespace,
    timestamp: str,
) -> dict[str, Any]:
    frame = read_frame(cap, args.capture_samples)
    if frame is None:
        return {
            "camera": camera_index,
            "axis": axis_name,
            "status": "no_frame",
            "detection": None,
        }

    detection = detect_largest_change(
        frame=frame,
        background=background,
        threshold=args.threshold,
        min_area=args.min_area,
        max_area_ratio=args.max_area_ratio,
        blur_size=args.blur_size,
        open_iterations=args.open_iterations,
        close_iterations=args.close_iterations,
        dilate_iterations=args.dilate_iterations,
    )
    annotated = annotate_frame(frame, detection, f"camera {camera_index} / {axis_name}")
    image_path = args.output_dir / f"object-{axis_name}-camera-{camera_index}-{timestamp}.jpg"
    cv2.imwrite(str(image_path), annotated)

    return {
        "camera": camera_index,
        "axis": axis_name,
        "status": "detected" if detection else "not_detected",
        "detection": detection,
        "annotated_image": str(image_path),
    }


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    cap_x = open_camera(args.camera_x, args.width, args.height)
    cap_y = open_camera(args.camera_y, args.width, args.height)

    if not cap_x.isOpened() or not cap_y.isOpened():
        cap_x.release()
        cap_y.release()
        print(
            "Could not open both cameras. Check camera indexes and Camera privacy "
            "permissions.",
            file=sys.stderr,
        )
        return 1

    if not args.json:
        print("Leave the workspace empty, then press Enter to learn the background.")
    input()

    background_x = learn_background(cap_x, args.background_samples, args.blur_size)
    background_y = learn_background(cap_y, args.background_samples, args.blur_size)
    if background_x is None or background_y is None:
        cap_x.release()
        cap_y.release()
        print("Could not learn the empty workspace from both cameras.", file=sys.stderr)
        return 1

    if not args.json:
        print("Place one object in the workspace, then press Enter to locate it.")
    input()

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    result_x = measure_camera(cap_x, background_x, args.camera_x, "x", args, timestamp)
    result_y = measure_camera(cap_y, background_y, args.camera_y, "y", args, timestamp)

    cap_x.release()
    cap_y.release()

    fused = None
    detection_x = result_x["detection"]
    detection_y = result_y["detection"]
    if detection_x and detection_y:
        fused = {
            "x_normalized": detection_x["center_normalized"]["x"],
            "y_normalized": detection_y["center_normalized"]["x"],
            "note": (
                "Prototype estimate: camera x supplies x, camera y supplies y. "
                "Add calibration to convert this to inches or centimeters."
            ),
        }

    output = {
        "coordinate_system": "normalized prototype coordinates",
        "cameras": [result_x, result_y],
        "object": fused,
    }

    print(json.dumps(output, indent=None if args.json else 2))
    return 0 if fused else 2


if __name__ == "__main__":
    raise SystemExit(main())
