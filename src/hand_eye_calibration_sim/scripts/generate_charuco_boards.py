#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
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

"""Generate the ChArUco board textures used by the hand_eye_calibration_sim scene.

Regenerates description/assets/charuco_world_board.png and
description/assets/charuco_flange_board.png. The board parameters here are the
single source of truth for the physical boards in the scene; the MJCF plate
geoms must stay dimensioned to squares_x * square_length_m by
squares_y * square_length_m.

The boards are drawn with marginSize=0 so the chessboard exactly fills the
image, which makes the texture span the plate geom's top face with no border
offset: ChArUco corner (i, j) sits exactly at (i * square_length_m,
j * square_length_m) from the plate corner.

Uses the pre-4.7 OpenCV aruco API (the dev container ships OpenCV 4.6.0). Run
inside the MoveIt Pro dev container:

    docker run --rm --entrypoint bash -v "$(pwd)":/ws moveit-pro-dev:main-jazzy \
        -c 'python3 /ws/src/hand_eye_calibration_sim/scripts/generate_charuco_boards.py \
            --output-dir /ws/src/hand_eye_calibration_sim/description/assets'
"""

import argparse
from pathlib import Path

import cv2

# Pixels per chessboard square in the generated textures. High enough that the
# markers stay crisp in 1280x720 renders at the working distances in the scene.
PIXELS_PER_SQUARE = 200

# Board definitions. The world board parameters are the defaults of the
# DetectCharucoBoard Behavior ports.
BOARDS = [
    {
        "filename": "charuco_world_board.png",
        "dictionary": cv2.aruco.DICT_4X4_50,
        "squares_x": 7,
        "squares_y": 5,
        "square_length_m": 0.035,
        "marker_length_m": 0.026,
    },
    {
        "filename": "charuco_flange_board.png",
        "dictionary": cv2.aruco.DICT_5X5_50,
        "squares_x": 5,
        "squares_y": 4,
        "square_length_m": 0.030,
        "marker_length_m": 0.022,
    },
]


def generate_board(board_spec: dict, output_dir: Path) -> Path:
    """Draw one ChArUco board PNG and return the written path."""
    dictionary = cv2.aruco.getPredefinedDictionary(board_spec["dictionary"])
    board = cv2.aruco.CharucoBoard_create(
        board_spec["squares_x"],
        board_spec["squares_y"],
        board_spec["square_length_m"],
        board_spec["marker_length_m"],
        dictionary,
    )
    size_px = (
        board_spec["squares_x"] * PIXELS_PER_SQUARE,
        board_spec["squares_y"] * PIXELS_PER_SQUARE,
    )
    image = board.draw(size_px, marginSize=0, borderBits=1)
    output_path = output_dir / board_spec["filename"]
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Failed to write {output_path}")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "description" / "assets",
        help="Directory the PNGs are written to.",
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for board_spec in BOARDS:
        path = generate_board(board_spec, args.output_dir)
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
