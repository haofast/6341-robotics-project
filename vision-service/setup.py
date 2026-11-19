#!/usr/bin/env python3
"""Detect and select a three-row chessboard workspace."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress
from datetime import datetime
import faulthandler
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time

import cv2
import numpy as np


BOARD_SIZE = 8
STRIP_COLUMNS = 6
STRIP_ROWS = 3
SQUARE_SIZE_MM = 38.1
WINDOW = "Select workspace strip"


def project(transform, points):
    return cv2.perspectiveTransform(np.float32([points]), transform)[0]


def find_board(frame, strip_columns):
    height, width = frame.shape[:2]
    scale = min(1, 960 / width, 680 / height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    image = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    enhanced = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
    # Try exact sizes from the full board down to the smallest supported crop.
    # The SBWithMeta detector with CALIB_CB_LARGER can crash on live frames.
    patterns = [
        (columns, rows)
        for columns in range(BOARD_SIZE - 1, strip_columns - 2, -1)
        for rows in range(columns, STRIP_ROWS - 1, -1)
    ]
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    scale_to_frame = np.float32([width / size[0], height / size[1]])
    for candidate in (gray, enhanced):
        for pattern in patterns:
            found, corners = cv2.findChessboardCorners(
                candidate, pattern, flags
            )
            if not found:
                continue
            corners = cv2.cornerSubPix(candidate, corners, (5, 5), (-1, -1), criteria)
            grid = corners.reshape(pattern[1], pattern[0], 2) * scale_to_frame
            result = select_detected_grid(frame, grid, strip_columns)
            if result is not None:
                return result
    return None


def select_detected_grid(frame, grid, strip_columns):
    height, width = frame.shape[:2]
    if grid.shape[0] == grid.shape[1]:
        # A square detection gives two possible strip directions.
        x_edge = grid[0, -1] - grid[0, 0]
        y_edge = grid[-1, 0] - grid[0, 0]
        if abs(x_edge[0]) / np.linalg.norm(x_edge) < abs(y_edge[0]) / np.linalg.norm(y_edge):
            grid = np.rot90(grid)
    if grid[0, 0].sum() > grid[-1, -1].sum():
        grid = grid[::-1, ::-1]

    rows, columns = (count + 1 for count in grid.shape[:2])
    # The first interior intersection is one square in from each outer edge.
    board_points = np.float32([(x, y) for y in range(1, rows) for x in range(1, columns)])
    image_points = grid.reshape(-1, 2)
    transform, _ = cv2.findHomography(board_points, image_points)
    if transform is None:
        return None

    visible_starts = []
    for row in range(rows - STRIP_ROWS + 1):
        for column in range(columns - strip_columns + 1):
            start = (column, row)
            outline = strip_corners(transform, start, strip_columns)
            if (np.isfinite(outline).all() and cv2.isContourConvex(outline)
                    and np.all(outline >= 0) and np.all(outline < (width, height))
                    and has_checker_pattern(frame, transform, start, strip_columns)):
                visible_starts.append(start)
    if not visible_starts:
        return None
    center = ((columns - strip_columns) / 2, (rows - STRIP_ROWS) / 2)
    return transform, visible_starts, min(visible_starts, key=lambda start: math.dist(start, center))


def has_checker_pattern(frame, transform, start, strip_columns):
    x, y = start
    cells = [(column + 0.5, row + 0.5) for row in range(STRIP_ROWS)
             for column in range(strip_columns)]
    pixels = project(transform, [(x + cx, y + cy) for cx, cy in cells]).astype(int)
    if np.any(pixels < 0) or np.any(pixels >= frame.shape[1::-1]):
        return False
    levels = frame[pixels[:, 1], pixels[:, 0]].mean(axis=1)
    parity = np.array([(int(cx) + int(cy)) % 2 == 0 for cx, cy in cells])
    even, odd = np.median(levels[parity]), np.median(levels[~parity])
    if abs(even - odd) < 8:
        return False
    light = parity if even > odd else ~parity
    return np.count_nonzero((levels > (even + odd) / 2) == light) >= len(cells) - 2


def reverse_origin(transform, start, strip_columns):
    x, y = start
    return transform @ np.array([
        [-1, 0, 2 * x + strip_columns], [0, -1, 2 * y + STRIP_ROWS], [0, 0, 1]
    ])


def strip_corners(transform, start, strip_columns):
    x, y = start
    right, bottom = x + strip_columns, y + STRIP_ROWS
    return project(transform, [(x, y), (right, y), (right, bottom), (x, bottom)])


def show_preview(frame, transform, start, status, strip_columns):
    height, width = frame.shape[:2]
    scale = min(1, 960 / width, 680 / height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    image = cv2.resize(frame, size)
    if transform is not None:
        display = np.diag([size[0] / width, size[1] / height, 1]) @ transform
        outline = strip_corners(display, start, strip_columns).round().astype(np.int32)
        left, top = start
        right, bottom = left + strip_columns, top + STRIP_ROWS
        highlight = image.copy()
        cv2.fillConvexPoly(highlight, outline, (0, 180, 0))
        cv2.addWeighted(highlight, 0.30, image, 0.70, 0, image)
        for line in ([(x, top), (x, bottom)] for x in range(left, right + 1)):
            a, b = project(display, line).round().astype(int)
            cv2.line(image, tuple(a), tuple(b), (0, 255, 255), 1)
        for line in ([(left, y), (right, y)] for y in range(top, bottom + 1)):
            a, b = project(display, line).round().astype(int)
            cv2.line(image, tuple(a), tuple(b), (0, 255, 255), 1)
        cv2.polylines(image, [outline], True, (0, 255, 0), 2)
        origin = project(display, [(left + 0.25, top + 0.5)]).round().astype(int)[0]
        cv2.putText(image, "(0,0)", tuple(origin), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2, cv2.LINE_AA)
        message = f"3x{strip_columns} selected | WASD shift if visible | R flip origin | Enter save"
    else:
        message = status

    cv2.rectangle(image, (0, 0), (size[0], 32), (0, 0, 0), -1)
    cv2.putText(image, message, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.53,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow(WINDOW, image)


def scan_and_select(index, strip_columns, reference=None, image=None):
    print(f"Show at least four complete rows of {strip_columns} squares for detection.", flush=True)
    print(f"WASD moves the 3x{strip_columns} selection. R flips the origin. Enter saves, Space rescans, Q quits.",
          flush=True)
    if image is not None:
        frame = cv2.imread(str(image))
        if frame is None:
            raise ValueError(f"Could not read overhead image: {image}")
        return preview_selection(lambda: (True, frame), index, strip_columns, reference, still=True)

    if sys.platform == "darwin":
        print(f"Opening camera {index} with AVFoundation.", flush=True)
        camera = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    else:
        print(f"Opening camera {index}.", flush=True)
        camera = cv2.VideoCapture(index)
    try:
        if not camera.isOpened():
            raise RuntimeError(f"Could not open camera {index}. Check the connection and camera permissions.")
        print(f"Camera {index} opened. Waiting for a frame.", flush=True)
        return preview_selection(camera.read, index, strip_columns, reference)
    finally:
        camera.release()


def preview_selection(read_frame, index, strip_columns, reference, *, still=False):
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    try:
        with ThreadPoolExecutor(max_workers=1) as scanner:
            return select_strip(read_frame, scanner, index, strip_columns, reference, still=still)
    finally:
        with suppress(cv2.error):
            cv2.destroyWindow(WINDOW)


def select_strip(read_frame, scanner, index, strip_columns, reference=None, *, still=False):
    frame_deadline = time.monotonic() + 3.0
    transform = None
    scan = None
    scanning = f"Looking for the chessboard for a 3x{strip_columns} selection..."
    status = scanning
    start = (0, 0)
    starts = []
    shifts = {'w': (0, -1), 'a': (-1, 0), 's': (0, 1), 'd': (1, 0)}
    flipped = False
    first_frame = True
    while True:
        if transform is None:
            ok, frame = read_frame()
            if not ok or frame is None or not frame.size:
                if time.monotonic() >= frame_deadline:
                    raise RuntimeError(f"No frame from camera {index}. Close other camera previews and retry.")
                cv2.waitKey(25)
                time.sleep(0.025)
                continue
            frame_deadline = time.monotonic() + 3.0
            if first_frame:
                print("First overhead frame received. Detecting the board.", flush=True)
                first_frame = False
            if scan is None:
                scan_frame = frame.copy()
                if reference is None:
                    scan = scanner.submit(find_board, scan_frame, strip_columns)
                else:
                    scan = scanner.submit(select_from_reference, scan_frame, reference, strip_columns)
            elif scan.done():
                result = scan.result()
                scan = None
                if result is not None:
                    transform, starts, start = result
                    flipped = False
                    frame = scan_frame
                else:
                    if still:
                        raise RuntimeError("No board detected in the overhead image. Use a clear image of the full strip.")
                    status = f"Grid not found. Show four rows of {strip_columns} and reduce glare. Retrying..."

        display = reverse_origin(transform, start, strip_columns) if flipped else transform
        show_preview(frame, display, start, status, strip_columns)
        key = chr(cv2.waitKey(20) & 0xFF).lower()
        if key in ("q", "\x1b") or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) == 0:
            raise KeyboardInterrupt
        if transform is None:
            continue
        if key in ("\n", "\r"):
            return frame, strip_corners(display, start, strip_columns).tolist()
        if key == " ":
            transform = None
            flipped = False
            frame_deadline = time.monotonic() + 3.0
            status = scanning
        elif key == "r":
            flipped = not flipped
        elif key in shifts:
            dx, dy = shifts[key]
            next_start = (start[0] + dx, start[1] + dy)
            if next_start in starts:
                start = next_start


def select_from_reference(frame, board, strip_columns):
    columns, rows = board["columns"], board["rows"]
    width, height = frame.shape[1], frame.shape[0]
    old_width, old_height = board["top_down_frame_size_px"]
    if not math.isclose(width / height, old_width / old_height, rel_tol=0.005):
        raise ValueError("Camera aspect ratio changed. Run setup without --from-workspace.")
    if not (strip_columns <= columns <= BOARD_SIZE and STRIP_ROWS <= rows <= BOARD_SIZE):
        raise ValueError(f"The saved workspace must contain a 3x{strip_columns} strip.")
    old_corners = np.array(board["top_down_corners_px"]) * [width / old_width, height / old_height]
    transform = cv2.getPerspectiveTransform(
        np.float32([(0, 0), (columns, 0), (columns, rows), (0, rows)]), np.float32(old_corners)
    )
    starts = [(x, y) for y in range(rows - STRIP_ROWS + 1) for x in range(columns - strip_columns + 1)]
    return transform, starts, ((columns - strip_columns) // 2, (rows - STRIP_ROWS) // 2)


def save_setup(output, index, frame, corners, strip_columns):
    config = {
        "cameras": {"top_down": index},
        "reference_images": {"top_down": "top-down.jpg"},
        "board": {
            "columns": strip_columns,
            "rows": STRIP_ROWS,
            "square_size_mm": SQUARE_SIZE_MM,
            "width_mm": round(strip_columns * SQUARE_SIZE_MM, 3),
            "height_mm": round(STRIP_ROWS * SQUARE_SIZE_MM, 3),
            "corner_order": ["origin", "positive_x", "opposite", "positive_y"],
            "top_down_corners_px": corners,
            "top_down_frame_size_px": [frame.shape[1], frame.shape[0]],
        },
        "cube_size_mm": 25.4,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=output.parent) as temp:
        staging = Path(temp) / "setup"
        staging.mkdir()
        if not cv2.imwrite(str(staging / "top-down.jpg"), frame):
            raise RuntimeError("Could not save top-down.jpg.")
        (staging / "workspace.json").write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
        if output.exists():
            raise RuntimeError(f"Output already exists: {output}. Choose a new --output directory.")
        staging.rename(output)


def main():
    faulthandler.enable()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--top-camera", type=int, default=0)
    parser.add_argument("--image", type=Path, help="use a saved overhead image from the current board and camera position")
    parser.add_argument("--columns", type=int, choices=(6, 8), default=STRIP_COLUMNS)
    parser.add_argument("--from-workspace", type=Path,
                        help="select inside a saved outline if the board and camera stayed fixed")
    default_output = Path(__file__).resolve().parent / "setup-data" / datetime.now().strftime("%Y%m%d-%H%M%S")
    parser.add_argument("--output", type=Path, default=default_output)
    args = parser.parse_args()
    if args.top_camera < 0:
        parser.error("Use a non-negative camera index.")
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error(f"Output already exists: {output}. Use the saved workspace or choose a new --output directory.")

    try:
        reference = None
        if args.from_workspace is not None:
            reference = json.loads(args.from_workspace.read_text())["board"]
            print("Using saved board coordinates. Verify the highlighted grid still lines up with the live board.")
        frame, corners = scan_and_select(args.top_camera, args.columns, reference, args.image)
        save_setup(output, args.top_camera, frame, corners, args.columns)
    except KeyboardInterrupt:
        print("\nSetup cancelled.")
        return 1
    except (OSError, RuntimeError, ValueError, KeyError, cv2.error) as error:
        print(f"Setup failed: {error}", file=sys.stderr)
        return 1

    print(f"Saved setup to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
