import math
import unittest
from types import SimpleNamespace

from mocap_autonomy_ladder import (
    Bounds,
    distance_2d,
    generate_circle_points,
    generate_figure8_points,
    planned_flight_points,
    validate_planned_flight,
    validate_path_inside_bounds,
)


class MocapAutonomyLadderTest(unittest.TestCase):
    def test_figure8_points_return_to_start_and_stay_near_center(self):
        center = (1.0, -1.0)
        z = 0.35
        radius_x = 0.06
        radius_y = 0.05

        points = generate_figure8_points(
            center,
            radius_x,
            radius_y,
            period_s=24.0,
            command_period_s=0.75,
            z=z,
        )

        self.assertGreaterEqual(len(points), 9)
        self.assertAlmostEqual(points[0][0], center[0])
        self.assertAlmostEqual(points[0][1], center[1])
        self.assertAlmostEqual(points[-1][0], center[0], places=6)
        self.assertAlmostEqual(points[-1][1], center[1], places=6)
        for x, y, point_z in points:
            self.assertLessEqual(abs(x - center[0]), radius_x + 1e-9)
            self.assertLessEqual(abs(y - center[1]), radius_y + 1e-9)
            self.assertEqual(point_z, z)

    def test_circle_points_have_requested_radius(self):
        center = (0.0, 0.0)
        radius = 0.05
        z = 0.35

        points = generate_circle_points(
            center,
            radius,
            period_s=24.0,
            command_period_s=0.75,
            z=z,
        )

        self.assertGreaterEqual(len(points), 5)
        for point in points:
            self.assertTrue(math.isclose(distance_2d(point, center), radius, abs_tol=1e-9))
            self.assertEqual(point[2], z)

    def test_bounds_respect_configured_margin(self):
        bounds = Bounds(
            x_min=0.0,
            x_max=1.0,
            y_min=0.0,
            y_max=1.0,
            z_min=0.0,
            z_max=1.0,
            margin=0.10,
        )

        safe, _ = bounds.check((0.50, 0.50, 0.20))
        self.assertTrue(safe)

        safe, reason = bounds.check((0.05, 0.50, 0.20))
        self.assertFalse(safe)
        self.assertIn("x_min+margin", reason)

    def test_validate_path_rejects_radius_overrun(self):
        bounds = Bounds(
            x_min=-1.0,
            x_max=1.0,
            y_min=-1.0,
            y_max=1.0,
            z_min=0.0,
            z_max=1.0,
            margin=0.0,
        )
        args = SimpleNamespace(
            max_height_above_start=0.60,
            max_radius_from_start=0.45,
        )

        with self.assertRaisesRegex(RuntimeError, "max radius"):
            validate_path_inside_bounds(
                points=[(0.0, 0.0, 0.30), (0.50, 0.0, 0.30)],
                bounds=bounds,
                start_position=(0.0, 0.0, 0.0),
                args=args,
            )

    def test_validate_path_rejects_height_overrun(self):
        bounds = Bounds(
            x_min=-1.0,
            x_max=1.0,
            y_min=-1.0,
            y_max=1.0,
            z_min=0.0,
            z_max=1.0,
            margin=0.0,
        )
        args = SimpleNamespace(
            max_height_above_start=0.35,
            max_radius_from_start=0.45,
        )

        with self.assertRaisesRegex(RuntimeError, "max height"):
            validate_path_inside_bounds(
                points=[(0.0, 0.0, 0.30), (0.0, 0.0, 0.40)],
                bounds=bounds,
                start_position=(0.0, 0.0, 0.0),
                args=args,
            )

    def test_planned_hover_is_validated_before_flight(self):
        bounds = Bounds(
            x_min=-1.0,
            x_max=1.0,
            y_min=-1.0,
            y_max=1.0,
            z_min=0.0,
            z_max=1.0,
            margin=0.0,
        )
        args = SimpleNamespace(
            height=0.40,
            max_height_above_start=0.35,
            max_radius_from_start=0.45,
            step_distance=0.10,
            circle_radius=0.05,
            figure8_radius_x=0.06,
            figure8_radius_y=0.05,
            path_period=24.0,
            path_command_period=0.75,
        )

        points = planned_flight_points('hover', (0.0, 0.0, 0.0), args)
        self.assertEqual(points, [(0.0, 0.0, 0.40)])
        with self.assertRaisesRegex(RuntimeError, "max height"):
            validate_planned_flight('hover', (0.0, 0.0, 0.0), bounds, args)


if __name__ == '__main__':
    unittest.main()
