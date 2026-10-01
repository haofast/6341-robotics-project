#!/usr/bin/env python3
"""Capture one image from every camera OpenCV can access."""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

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
        description="Take one picture using each available camera."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Directory for captured images. Default: {DEFAULT_OUTPUT_DIR}",
    )
    parser.add_argument(
        "--max-index",
        type=int,
        default=10,
        help="Highest camera index to scan. Default: 10",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=12,
        help="Frames to read before saving, allowing exposure/focus to settle.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=1280,
        help="Requested capture width. Cameras may choose a nearby value.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=720,
        help="Requested capture height. Cameras may choose a nearby value.",
    )
    return parser.parse_args()


def open_camera(index: int) -> cv2.VideoCapture:
    if sys.platform == "darwin":
        return cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    return cv2.VideoCapture(index)


def capture_camera(
    index: int,
    output_dir: Path,
    width: int,
    height: int,
    warmup_frames: int,
) -> Path | None:
    cap = open_camera(index)
    if not cap.isOpened():
        cap.release()
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    frame = None
    ok = False
    for _ in range(max(1, warmup_frames)):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size:
            time.sleep(0.05)

    cap.release()

    if not ok or frame is None or not frame.size:
        return None

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    image_path = output_dir / f"camera-{index}-{timestamp}.jpg"
    if not cv2.imwrite(str(image_path), frame):
        raise RuntimeError(f"OpenCV could not write {image_path}")
    return image_path


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    saved_images: list[Path] = []
    attempted = range(max(0, args.max_index) + 1)

    for index in attempted:
        image_path = capture_camera(
            index=index,
            output_dir=args.output_dir,
            width=args.width,
            height=args.height,
            warmup_frames=args.warmup_frames,
        )
        if image_path is None:
            print(f"camera {index}: unavailable")
            continue

        saved_images.append(image_path)
        print(f"camera {index}: saved {image_path}")

    if saved_images:
        print(f"\nSaved {len(saved_images)} image(s).")
        return 0

    print(
        "\nNo cameras were captured. If you are on macOS, check that the "
        "terminal or app running this script has Camera permission."
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
