"""Geometry checks using synthetic cameras; no hardware or windows are opened.

Run: python3 -m unittest discover -s vision-service -p 'test_*.py' -v
"""

import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

import calibrate_workspace as calibration


def synthetic_camera(center, target, index):
    """Create a known world-to-camera pose independently of the pose solver."""
    center = np.asarray(center, dtype=np.float64)
    forward = np.asarray(target, dtype=np.float64) - center
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 1.0, 0.0]) if abs(forward[2]) > 0.9 else np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.stack((right, down, forward))
    rvec, _ = cv2.Rodrigues(rotation)
    return {
        "version": 1,
        "units": "mm",
        "camera_id": index,
        "image_size": [1280, 720],
        "camera_matrix": [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        "distortion_coefficients": [0.08, -0.04, 0.001, -0.001, 0.01],
        "rvec": rvec.reshape(3).tolist(),
        "tvec_mm": (-rotation @ center).tolist(),
    }


def observed_pixels(camera, points):
    projected, _ = cv2.projectPoints(
        np.asarray(points, dtype=np.float64).reshape(-1, 3),
        np.asarray(camera["rvec"], dtype=np.float64),
        np.asarray(camera["tvec_mm"], dtype=np.float64),
        np.asarray(camera["camera_matrix"], dtype=np.float64),
        np.asarray(camera["distortion_coefficients"], dtype=np.float64),
    )
    return projected.reshape(-1, 2)


class WorkspaceGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cameras = [synthetic_camera([310, 180, 950], [300, 200, 0], 0),
                        synthetic_camera([300, -800, 450], [300, 200, 40], 1)]
        self.corners = np.array([[0, 0, 0], [600, 0, 0], [600, 400, 0], [0, 400, 0]], dtype=np.float64)

    def test_register_both_cameras_to_measured_table(self):
        for camera in self.cameras:
            with self.subTest(camera=camera["camera_id"]):
                clicks = observed_pixels(camera, self.corners)
                pose = calibration.solve_table_pose(camera, clicks, 600, 400, 2.0)
                rotation, translation = calibration.camera_pose(pose)
                expected_rotation, expected_translation = calibration.camera_pose(camera)
                np.testing.assert_allclose(rotation, expected_rotation, atol=1e-5)
                np.testing.assert_allclose(translation, expected_translation, atol=0.01)
                self.assertLess(pose["table_reprojection_max_px"], 0.01)

    def test_plane_mapping_across_table_with_distorted_pixels(self):
        camera = self.cameras[0]
        for x in (25, 200, 575):
            for y in (25, 150, 375):
                world = np.array([x, y, 0.0])
                pixel = observed_pixels(camera, [world])[0]
                recovered = calibration.pixel_to_plane(camera, pixel)
                np.testing.assert_allclose(recovered, world, atol=0.005)

    def test_known_height_corrects_parallax(self):
        world = np.array([100.0, 300.0, 120.0])
        pixel = observed_pixels(self.cameras[0], [world])[0]
        recovered = calibration.pixel_to_plane(self.cameras[0], pixel, height_mm=120)
        np.testing.assert_allclose(recovered, world, atol=0.005)
        wrong_plane = calibration.pixel_to_plane(self.cameras[0], pixel)
        self.assertGreater(np.linalg.norm(wrong_plane[:2] - world[:2]), 10)

    def test_saved_homography_uses_undistorted_pixels(self):
        camera = self.cameras[0]
        pose = calibration.solve_table_pose(camera, observed_pixels(camera, self.corners), 600, 400, 2.0)
        world = np.array([75.0, 340.0, 0.0])
        pixel = observed_pixels(camera, [world])[0]
        matrix, distortion = calibration.camera_matrices(camera)
        undistorted = cv2.undistortPoints(pixel.reshape(1, 1, 2), matrix, distortion, P=matrix).reshape(2)
        xyw = np.asarray(pose["undistorted_pixel_to_table_homography"]) @ np.r_[undistorted, 1.0]
        np.testing.assert_allclose(xyw[:2] / xyw[2], world[:2], atol=0.01)

    def test_triangulation_recovers_varied_block_points(self):
        generator = np.random.default_rng(6341)
        for world in generator.uniform([20, 20, 0], [580, 380, 200], size=(30, 3)):
            clicks = [observed_pixels(camera, [world])[0] for camera in self.cameras]
            result = calibration.triangulate_feature(self.cameras, clicks)
            recovered = np.array([result["position_mm"][axis] for axis in ("x", "y", "z")])
            np.testing.assert_allclose(recovered, world, atol=0.01)
            self.assertLess(max(result["reprojection_error_px"]), 0.01)

    def test_mismatched_features_are_rejected(self):
        world = [150.0, 230.0, 75.0]
        clicks = [observed_pixels(camera, [world])[0] for camera in self.cameras]
        clicks[1] = clicks[1] + np.array([80.0, 30.0])
        with self.assertRaisesRegex(calibration.CalibrationError, "Reprojection"):
            calibration.triangulate_feature(self.cameras, clicks)

    def test_identical_viewing_rays_are_rejected(self):
        camera = self.cameras[0]
        click = observed_pixels(camera, [[150.0, 230.0, 75.0]])[0]
        with self.assertRaisesRegex(calibration.CalibrationError, "depth is unreliable"):
            calibration.triangulate_feature([camera, camera], [click, click])

    def test_below_table_point_is_rejected(self):
        clicks = [observed_pixels(camera, [[150.0, 230.0, -30.0]])[0] for camera in self.cameras]
        with self.assertRaisesRegex(calibration.CalibrationError, "below the tabletop"):
            calibration.triangulate_feature(self.cameras, clicks)

    def test_crossed_or_edge_on_table_corners_are_rejected(self):
        with self.assertRaisesRegex(calibration.CalibrationError, "cross or repeat"):
            calibration.validate_corner_clicks(np.array([[100, 100], [600, 400], [600, 100], [100, 400]]), [1280, 720])
        with self.assertRaisesRegex(calibration.CalibrationError, "edge-on"):
            calibration.validate_corner_clicks(np.array([[100, 100], [600, 100], [600, 102], [100, 102]]), [1280, 720])

    def test_plane_behind_camera_is_rejected(self):
        with self.assertRaisesRegex(calibration.CalibrationError, "behind the camera"):
            calibration.pixel_to_plane(self.cameras[0], (640, 360), height_mm=2000)

    def test_changed_capture_resolution_is_rejected(self):
        with self.assertRaisesRegex(calibration.CalibrationError, "resolution changed"):
            calibration.check_image_size(np.zeros((480, 640, 3), dtype=np.uint8), self.cameras[0])

    def test_calibration_round_trip_and_units_validation(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "camera-0.json"
            calibration.save_json(path, self.cameras[0])
            self.assertEqual(calibration.load_json(path), self.cameras[0])
            data = json.loads(path.read_text())
            data["units"] = "inches"
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(calibration.CalibrationError, "Unsupported calibration format"):
                calibration.load_json(path)


if __name__ == "__main__":
    unittest.main()
