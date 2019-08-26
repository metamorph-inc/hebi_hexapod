#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
# Name: calc_moments_of_inertia.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 11/20/2017
# Edit Date: 11/20/2017
#
# Description:
# Simple script to calculate moments of inerta for some common shapes
# Reference: https://en.wikipedia.org/wiki/List_of_moments_of_inertia

   +z                 height
    |                   |
    o--> +y    <=>      o--> width
   /                   /
+x                  depth
'''

import sys
import argparse
import math as m
import numpy as np

# TODO: Replace with object-oriented approach ( e.g. shape = SolidCylinder(height=...) --> shape.get_inertial_matrix() )
def main():
    shape_implemented = True
    shape_type_implemented = True
    parser = parse_args(sys.argv[1:])
    if parser.shape == 'cuboid':
        if parser.type == 'solid':
            inertial_matrix = calc_solid_cuboid(x=parser.x, y=parser.y, z=parser.z, mass=parser.mass)
        else:
            shape_type_implemented = False
    elif parser.shape == 'cylinder':
        if parser.type == 'solid':
            inertial_matrix = calc_solid_cylinder(height=parser.height, radius=parser.radius, mass=parser.mass)
        elif parser.type == 'hollow':
            inertial_matrix = calc_hollow_cylinder(height=parser.height, inner_radius=parser.inner_radius, outer_radius=parser.outer_radius, mass=parser.mass)
        else:
            shape_type_implemented = False
    elif parser.shape == 'sphere':
        if parser.type == 'solid':
            inertial_matrix = calc_solid_sphere(radius=parser.radius, mass=parser.mass)
        elif parser.type == 'hollow':
            inertial_matrix = calc_hollow_sphere(inner_radius=parser.inner_radius, outer_radius=parser.outer_radius, mass=parser.mass)
        else:
            shape_type_implemented = False
    else:
        shape_implemented = False

    if not shape_implemented:
        print("UNIMPLEMENTED SHAPE: {}".format(parser.shape.upper()))
    elif not shape_type_implemented:
        print("UNIMPLEMENTED {} TYPE: {}".format(parser.shape.upper(), parser.type.upper()))
    else:
        print(np.around(inertial_matrix, 8))
        print("ixx="+str(round(inertial_matrix[0][0], 8))
            + " iyy="+str(round(inertial_matrix[1][1], 8))
            + " izz="+str(round(inertial_matrix[2][2], 8)))


def parse_args(args):
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='sub-command help', dest='shape')
    # cuboid
    cuboid_parser = subparsers.add_parser('cuboid', help='cuboid help')
    cuboid_subparsers = cuboid_parser.add_subparsers(dest='type')
    cuboid_solid_parser = cuboid_subparsers.add_parser('solid', help='solid help')
    cuboid_solid_parser.add_argument('x', type=check_positive)
    cuboid_solid_parser.add_argument('y', type=check_positive)
    cuboid_solid_parser.add_argument('z', type=check_positive)
    cuboid_solid_parser.add_argument('mass', type=check_positive)

    # cylinder
    cylinder_parser = subparsers.add_parser('cylinder', help='cylinder help')
    cylinder_subparsers = cylinder_parser.add_subparsers(dest='type')
    cylinder_solid_parser = cylinder_subparsers.add_parser('solid', help='solid help')
    cylinder_solid_parser.add_argument('height', type=check_positive)
    cylinder_solid_parser.add_argument('radius', type=check_positive)
    cylinder_solid_parser.add_argument('mass', type=check_positive)
    cylinder_hollow_parser = cylinder_subparsers.add_parser('hollow', help='hollow help')
    cylinder_hollow_parser.add_argument('height', type=check_positive)
    cylinder_hollow_parser.add_argument('inner_radius', type=check_positive)
    cylinder_hollow_parser.add_argument('outer_radius', type=check_positive)
    cylinder_hollow_parser.add_argument('mass', type=check_positive)
    # sphere
    sphere_parser = subparsers.add_parser('sphere', help='sphere help')
    sphere_subparsers = sphere_parser.add_subparsers(dest='type')
    sphere_solid_parser = sphere_subparsers.add_parser('solid', help='solid help')
    sphere_solid_parser.add_argument('radius', type=check_positive)
    sphere_solid_parser.add_argument('mass', type=check_positive)
    sphere_hollow_parser = sphere_subparsers.add_parser('hollow', help='solid help')
    sphere_hollow_parser.add_argument('inner_radius', type=check_positive)
    sphere_hollow_parser.add_argument('outer_radius', type=check_positive)
    sphere_hollow_parser.add_argument('mass', type=check_positive)
    return parser.parse_args(args)


def check_positive(value):
    float_value = float(value)
    if float_value <= 0.0:
        raise argparse.ArgumentTypeError("Received input value {}. Input value must be a positive number".format(float_value))
    return float_value


def calc_solid_cuboid(x, y, z, mass):
    inertial_matrix = np.zeros((3, 3))
    ixx = (1.0/12.0)*mass*(z**2 + y**2)
    iyy = (1.0/12.0)*mass*(x**2 + z**2)
    izz = (1.0/12.0)*mass*(y**2 + x**2)
    inertial_matrix[0][0] = ixx
    inertial_matrix[1][1] = iyy
    inertial_matrix[2][2] = izz
    return inertial_matrix


def calc_solid_cylinder(height, radius, mass):
    inertial_matrix = np.zeros((3, 3))
    ixx = (1.0/12.0)*mass*(3*radius**2 + height**2)
    iyy = (1.0/12.0)*mass*(3*radius**2 + height**2)
    izz = (1.0/2.0)*mass*(radius**2)
    inertial_matrix[0][0] = ixx
    inertial_matrix[1][1] = iyy
    inertial_matrix[2][2] = izz
    return inertial_matrix


def calc_hollow_cylinder(height, inner_radius, outer_radius, mass):
    inertial_matrix = np.zeros((3, 3))
    ixx = (1.0/12.0)*mass*(3*(inner_radius**2 + outer_radius**2) + height**2)
    iyy = (1.0/12.0)*mass*(3*(inner_radius**2 + outer_radius**2) + height**2)
    izz = (1.0/2.0)*mass*(inner_radius**2 + outer_radius**2)
    inertial_matrix[0][0] = ixx
    inertial_matrix[1][1] = iyy
    inertial_matrix[2][2] = izz
    return inertial_matrix


def calc_solid_sphere(radius, mass):
    inertial_matrix = np.zeros((3, 3))
    ixx = (2.0/5.0)*mass*radius**2
    iyy = (2.0/5.0)*mass*radius**2
    izz = (2.0/5.0)*mass*radius**2
    inertial_matrix[0][0] = ixx
    inertial_matrix[1][1] = iyy
    inertial_matrix[2][2] = izz
    return inertial_matrix


def calc_hollow_sphere(inner_radius, outer_radius, mass):
    inertial_matrix = np.zeros((3, 3))
    ixx = (2.0/5.0)*mass*((outer_radius**5 - inner_radius**5)/(outer_radius**3 - inner_radius**3))
    iyy = (2.0/5.0)*mass*((outer_radius**5 - inner_radius**5)/(outer_radius**3 - inner_radius**3))
    izz = (2.0/5.0)*mass*((outer_radius**5 - inner_radius**5)/(outer_radius**3 - inner_radius**3))
    inertial_matrix[0][0] = ixx
    inertial_matrix[1][1] = iyy
    inertial_matrix[2][2] = izz
    return inertial_matrix


if __name__ == '__main__':
    main()
