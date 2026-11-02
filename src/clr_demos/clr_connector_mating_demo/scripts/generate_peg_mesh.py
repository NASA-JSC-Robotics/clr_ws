#!/usr/bin/env python3
#
# Copyright (c) 2026, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
Generates the chamfered peg mesh used by urdf/peg_macro.xacro.

The peg is a cylinder along +z whose base sits at z = 0, with a 45 degree
chamfer at the tip so it self-centers in the receptacle bore. Dimensions are
in meters. Re-run this (and update the macro's properties) to change the peg:

    python3 scripts/generate_peg_mesh.py --radius 0.022 --length 0.157 --chamfer 0.005
"""

import argparse
import os

import numpy as np
import trimesh


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radius", type=float, default=0.022, help="Peg radius in meters.")
    parser.add_argument("--length", type=float, default=0.157, help="Total peg length in meters.")
    parser.add_argument("--chamfer", type=float, default=0.005, help="Chamfer size (axial and radial) in meters.")
    parser.add_argument("--segments", type=int, default=64, help="Number of segments around the axis.")
    parser.add_argument(
        "--output",
        default=os.path.join(os.path.dirname(__file__), "..", "meshes", "visual", "chamfered_peg.stl"),
        help="Output STL path.",
    )
    args = parser.parse_args()

    # Profile in the (radius, z) plane, revolved about z.
    profile = np.array(
        [
            [0.0, 0.0],
            [args.radius, 0.0],
            [args.radius, args.length - args.chamfer],
            [args.radius - args.chamfer, args.length],
            [0.0, args.length],
        ]
    )
    mesh = trimesh.creation.revolve(profile, sections=args.segments)
    if not mesh.is_watertight:
        raise RuntimeError("Generated peg mesh is not watertight.")
    mesh.export(os.path.abspath(args.output))
    print(
        f"Wrote {os.path.abspath(args.output)}: {len(mesh.faces)} faces, "
        f"bounds {np.round(mesh.bounds, 4).tolist()}, watertight={mesh.is_watertight}, convex={mesh.is_convex}"
    )


if __name__ == "__main__":
    main()
