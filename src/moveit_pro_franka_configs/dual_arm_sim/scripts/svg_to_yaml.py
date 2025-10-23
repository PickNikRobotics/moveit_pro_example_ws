#!/usr/bin/env python3
#
# Copyright 2025 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
SVG to Waypoints Converter

This module converts SVG paths into 3D waypoints formatted as YAML PoseStamped messages.
It extracts paths from SVG files, samples points along the paths based on specified density,
and generates waypoints with appropriate z-clearance for transitions between paths.
"""

import argparse
import copy
import numpy as np
import pygame
from svgpathtools import svg2paths


def write_waypoints_to_file(waypoints, filename, scale=1.0):
    with open(filename, "w") as file:
        file.write(waypoints_to_string(waypoints, scale))


def waypoints_to_string(waypoints, scale=1.0):
    yaml = ""
    for i in range(len(waypoints) - 1):
        w1 = waypoints[i]
        w2 = waypoints[i + 1]

        # Add the first waypoint to the string
        yaml += waypoint_to_string(w1, scale=scale)

        # Calculate the distance between the two waypoints
        distance = np.linalg.norm(np.array(w1) - np.array(w2))

        # If the distance is more than 2 cm, add elevated copies of both waypoints
        if distance > 0.02:  # 2 cm in meters
            elevated_w1 = (w1[0], w1[1], w1[2] + 0.01)  # Elevate w1 by 1 cm
            elevated_w2 = (w2[0], w2[1], w2[2] + 0.01)  # Elevate w2 by 1 cm
            yaml += waypoint_to_string(elevated_w1, scale=scale)
            yaml += waypoint_to_string(elevated_w2, scale=scale)

    # Add the last waypoint to the string
    yaml += waypoint_to_string(waypoints[-1], scale=scale)

    return yaml


def waypoint_to_string(w, scale=1.0):
    pose_stamped_string = f"""
---
header:
  frame_id: local
pose:
  position:
    x: {scale * w[0]}
    y: {scale * w[1]}
    z: {scale * w[2]}
  orientation:
    x: 0
    y: 0
    z: 1
    w: 0"""
    return pose_stamped_string


def draw_waypoints(waypoints, scale=1):
    pygame.init()
    screen = pygame.display.set_mode([500, 500])
    screen.fill((255, 255, 255))

    for waypoint in waypoints:
        pygame.draw.circle(
            screen, (0, 0, 255), (waypoint[0] * scale, waypoint[1] * scale), 1
        )
    pygame.display.flip()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return


def log(info):
    """Print if the VERBOSE flag was set"""
    if VERBOSE:
        print(info)


def extract_waypoints_from_paths(paths, density):
    all_waypoints = []
    # Clearance on transition strokes, i.e. from the end of one path to the beginning of another.
    z_transition_clearance = 10.0  # mm
    # Iterate on each continuous path (letter/connected shape)
    for path in paths:
        points = path_to_points(path, density)
        log(f"Path has {len(points)} waypoints")
        waypoints = [
            list(a)
            for a in zip(
                points.real, points.imag, np.zeros(len(points.real)), strict=True
            )
        ]
        log(f"Subpath has {len(waypoints)} waypoints")
        # Add a pre-waypoint and post-waypoint with Z clearance for transitions between subpaths.
        waypoints.insert(0, copy.deepcopy(waypoints[0]))
        waypoints[0][2] = -z_transition_clearance
        waypoints.append(copy.deepcopy(waypoints[-1]))
        waypoints[-1][2] = -z_transition_clearance

        all_waypoints = all_waypoints + waypoints
    return all_waypoints


def path_to_points(path, sample_density=10):
    points = []
    for s in np.linspace(0, 1, sample_density):
        points += [path.point(s)]
    # points_length = 0
    # for segment in path:
    #     if type(segment).__name__ == "Line":
    #         # Just add the start and end points of straight line segments
    #         points += [segment.start, segment.end]
    #     else:
    #         # If it's a curve, then sample several waypoints along it depending on sampling desity (but always sample at least 2)
    #         for s in np.linspace(
    #             0, 1, max(2, int(np.ceil((sample_density * 0.1) * segment.length())))
    #         ):
    #             points += [path.point(s)]
    #     log(f"Added {len(points) - points_length} points for '{type(segment).__name__}' of length {(segment.length())}")
    #     points_length = len(points)
    return np.array(points)


def prune_waypoints(waypoints):
    i = 0
    while waypoints[i] != waypoints[-1] or i == 0:
        distance = waypoint_distance(i, waypoints[i], waypoints[i + 1])
        if distance < 1:
            del waypoints[i + 1]
            log(f"pruned waypoint {i + 1}")
            continue
        i += 1


def check_waypoints(waypoints):
    i = 0
    while waypoints[i] != waypoints[-1] or i == 0:
        waypoint_distance(i, waypoints[i], waypoints[i + 1])
        i += 1


def waypoint_distance(i, w1, w2):
    distance = np.linalg.norm(np.array(w1) - np.array(w2))
    log(f"distance between waypoint {i} and waypoint {i+1}: {distance}")
    if distance < 1:
        log("NEED TO PRUNE")
    return distance


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description="Translate SVG paths to waypoints")
    parser.add_argument("input_file", help="Path to the input SVG file")
    parser.add_argument("output_file", help="Path to the output YAML file")
    parser.add_argument(
        "-d",
        "--density",
        type=int,
        default=5,
        action="store",
        help="Waypoint sampling density",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose mode"
    )
    args = parser.parse_args()
    global VERBOSE
    VERBOSE = args.verbose

    svg_attributes = svg2paths(args.input_file)
    paths = svg_attributes[0]
    paths.reverse()
    log(f"Found {len(paths)} paths")

    log("Extracting waypoints")
    all_waypoints = extract_waypoints_from_paths(paths, args.density)

    log("Pruning waypoints")
    prune_waypoints(all_waypoints)

    # log("Checking waypoints")
    # check_waypoints(all_waypoints)

    if VERBOSE:
        print("Drawing waypoints")
        draw_waypoints(all_waypoints)

    # Write waypoints with PoseStamped YAML format
    log("Saving waypoints")
    # SVG units are close to mm (actually pixels), divide by 1000.0 to bring it near meters
    write_waypoints_to_file(all_waypoints, args.output_file, scale=(1 / 1000))
