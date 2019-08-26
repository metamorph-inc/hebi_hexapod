#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: test_calc_moments_of_inertia.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 11/20/2017
# Edit Date: 11/20/2017
#
# Description:
# Unit tests for calc_moments_of_inertia.py
# $ python -m unittest -v test_calc_moments_of_inertia
# OR
# $ python test_calc_moments_of_inertia.py
'''

import unittest
import numpy as np
import calc_moments_of_inertia as test_module

# TODO: Refactor/organize unittests
class ParseArgsTestSuite(unittest.TestCase):
    """Basic test cases for parse_args(args)."""

    # TODO: Add actual assert checks instead of just text outputs
    def test_parse_args_no_args(self):
        # See https://stackoverflow.com/a/5943381/8670609
        with self.assertRaises(SystemExit):
            parser = test_module.parse_args([])

    def test_parse_args_help(self):
        with self.assertRaises(SystemExit):
            parser = test_module.parse_args(['-h'])

    def test_parse_args_cuboid_help(self):
        with self.assertRaises(SystemExit):
            parser = test_module.parse_args(['cuboid', '-h'])

    def test_parse_args_cuboid(self):
        parser = test_module.parse_args(['cuboid', 'solid', '1', '1', '1', '1'])
        print(parser)

    def test_parse_args_cuboid_negative_val(self):
        with self.assertRaises(SystemExit):
            parser = test_module.parse_args(['cuboid', 'solid', '1', '-1', '1', '1'])

    def test_parse_args_cuboid_zero_val(self):
        with self.assertRaises(SystemExit):
            parser = test_module.parse_args(['cuboid', 'solid', '1', '1', '1', '0'])

    def test_parse_args_cuboid_no_type_flag(self):
        with self.assertRaises(SystemExit):
            parser = test_module.parse_args(['cuboid', '1', '1', '1', '1'])

    def test_parse_args_cylinder(self):
        parser = test_module.parse_args(['cylinder', 'solid', '1', '1', '1'])
        print(parser)

    def test_parse_args_sphere(self):
        parser = test_module.parse_args(['sphere', 'solid', '1', '1'])
        print(parser)

    # TODO: Add more tests here


class CalcHollowSphereTestSuite(unittest.TestCase):
    """Basic test cases for calc_hollow_sphere(inner_radius, outer_radius, mass)."""
    def test_calc_hollow_sphere(self):
        expected_inertial_matrix = np.array([[1.3217949, 0.0000000, 0.0000000],
                                             [0.0000000, 1.3217949, 0.0000000],
                                             [0.0000000, 0.0000000, 1.3217949]])
        test_inertial_matrix = test_module.calc_hollow_sphere(0.5, 1.25, 2.0)
        tol = 1e-5
        self.assertTrue(np.allclose(expected_inertial_matrix, test_inertial_matrix, tol))

    # TODO: Add more tests here

# TODO: Add more tests suites here


if __name__ == "__main__":
    unittest.main()
