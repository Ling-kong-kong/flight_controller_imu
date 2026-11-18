#!/usr/bin/env python3
"""Package only a 128-KB-offset image for an existing board-ID-53 PX4/AP bootloader.

Standalone builds intentionally do not invoke this script. This does not install a
bootloader and does not validate the bootloader on a connected physical board.
"""
import argparse
import base64
import json
from pathlib import Path
import zlib


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    image = args.binary.read_bytes()
    # AP/PX4 uploaders require word-aligned image length.
    image += b"\xff" * (-len(image) % 4)
    if len(image) > (2048 - 128) * 1024:
        parser.error("Image exceeds the 128-KB-offset Flash region")
    package = {
        "magic": "APJFWv1",
        "board_id": 53,
        "board_revision": 0,
        "description": "STM32H743 BMI088 raw USB IMU; app offset 128 KB",
        "summary": "Raw accelerometer 1600 Hz + gyroscope 2000 Hz",
        "version": "1.0.0",
        "git_identity": "local-build",
        "image": base64.b64encode(zlib.compress(image, 9)).decode("ascii"),
        "image_size": len(image),
    }
    args.output.write_text(json.dumps(package, indent=2) + "\n")


if __name__ == "__main__":
    main()
