#!/usr/bin/env python3
"""Generate deterministic perspective-warped ChArUco views for workflow tests.

The output validates orchestration only. It is not camera-calibration evidence.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate synthetic ChArUco views")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--train", type=int, default=12)
    parser.add_argument("--validation", type=int, default=4)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    return parser


def destination_corners(index: int, width: int, height: int) -> np.ndarray:
    columns = (-0.18, 0.0, 0.18)
    rows = (-0.10, 0.08)
    x_shift = columns[index % len(columns)] * width
    y_shift = rows[(index // len(columns)) % len(rows)] * height
    scale = (0.50, 0.58, 0.66, 0.72)[index % 4]
    board_h = height * scale
    board_w = board_h * 0.70
    cx = width * 0.50 + x_shift
    cy = height * 0.51 + y_shift
    skew = ((index % 5) - 2) * width * 0.008
    tilt = ((index % 7) - 3) * height * 0.006
    return np.float32(
        [
            [cx - board_w / 2 + skew, cy - board_h / 2 + tilt],
            [cx + board_w / 2, cy - board_h / 2 - tilt],
            [cx + board_w / 2 - skew, cy + board_h / 2 + tilt],
            [cx - board_w / 2, cy + board_h / 2 - tilt],
        ]
    )


def render_view(board_image: np.ndarray, index: int, width: int, height: int) -> np.ndarray:
    source = np.float32(
        [[0, 0], [board_image.shape[1] - 1, 0],
         [board_image.shape[1] - 1, board_image.shape[0] - 1],
         [0, board_image.shape[0] - 1]]
    )
    transform = cv2.getPerspectiveTransform(source, destination_corners(index, width, height))
    warped = cv2.warpPerspective(board_image, transform, (width, height), borderValue=238)
    mask_source = np.full(board_image.shape, 255, dtype=np.uint8)
    mask = cv2.warpPerspective(mask_source, transform, (width, height), borderValue=0)
    canvas = np.full((height, width), 238, dtype=np.uint8)
    canvas[mask > 0] = warped[mask > 0]
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def main() -> None:
    args = build_parser().parse_args()
    if args.train < 5 or args.validation < 1:
        raise ValueError("train must be >= 5 and validation must be >= 1")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    board = cv2.aruco.CharucoBoard((7, 10), 0.0565, 0.0422, dictionary)
    board.setLegacyPattern(False)
    board_image = board.generateImage((700, 1000), marginSize=24, borderBits=1)

    for group, count, offset in (
        ("intrinsic", args.train, 0),
        ("validation", args.validation, 101),
    ):
        directory = args.output / group
        directory.mkdir(parents=True, exist_ok=True)
        for number in range(count):
            image = render_view(board_image, number + offset, args.width, args.height)
            path = directory / f"synthetic-{number + 1:03d}.jpg"
            if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 96]):
                raise RuntimeError(f"cannot write {path}")
    print(
        f"generated train={args.train} validation={args.validation} "
        f"size={args.width}x{args.height} output={args.output} synthetic_only=1"
    )


if __name__ == "__main__":
    main()
