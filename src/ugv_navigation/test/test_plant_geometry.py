#!/usr/bin/env python3
"""TEST_ONLY ray geometry: finite obstacles must not become infinite walls."""
import math
import unittest

from test_odom_navigation import TestOnlyDifferentialPlant
from probe_dwa_safety import footprint_intersects_rectangle, RECTANGLE


class FiniteRayGeometryTest(unittest.TestCase):
    def test_nearest_finite_vertical_segment_is_a_real_hit(self):
        self.assertAlmostEqual(TestOnlyDifferentialPlant._ray(
            0., 0., 0., None, [((.75, -.15), (.75, .15))]), .75)

    def test_ray_above_segment_endpoint_reaches_room_wall(self):
        self.assertAlmostEqual(TestOnlyDifferentialPlant._ray(
            0., .2, 0., None, [((.75, -.15), (.75, .15))]), 4.)

    def test_segment_endpoint_is_inclusive(self):
        self.assertAlmostEqual(TestOnlyDifferentialPlant._ray(
            0., .15, 0., None, [((.75, -.15), (.75, .15))]), .75)

    def test_oblique_ray_hits_horizontal_rectangle_edge(self):
        self.assertAlmostEqual(TestOnlyDifferentialPlant._ray(
            0., 0., math.pi / 4., None, [((.5, .75), (1., .75))]),
            .75 * math.sqrt(2.))

    def test_existing_full_width_wall_remains_nearest(self):
        self.assertAlmostEqual(TestOnlyDifferentialPlant._ray(
            0., 0., 0., .6, [((.75, -.15), (.75, .15))]), .6)

    def test_collinear_segment_reports_its_nearest_endpoint(self):
        self.assertAlmostEqual(TestOnlyDifferentialPlant._ray(
            0., 0., 0., None, [((.75, 0.), (1., 0.))]), .75)

    def test_ground_truth_separates_clear_footprint_from_rectangle(self):
        self.assertFalse(footprint_intersects_rectangle((0., 0., 0.), RECTANGLE))
        self.assertFalse(footprint_intersects_rectangle((.8, .5, 0.), RECTANGLE))

    def test_ground_truth_detects_front_overhang_and_rotated_rear(self):
        self.assertTrue(footprint_intersects_rectangle((.6, 0., 0.), RECTANGLE))
        self.assertTrue(footprint_intersects_rectangle((.8, .3, math.pi / 2.), RECTANGLE))

    def test_ground_truth_counts_boundary_contact_as_intersection(self):
        self.assertTrue(footprint_intersects_rectangle((.48, 0., 0.), RECTANGLE))


if __name__ == '__main__':
    unittest.main()
