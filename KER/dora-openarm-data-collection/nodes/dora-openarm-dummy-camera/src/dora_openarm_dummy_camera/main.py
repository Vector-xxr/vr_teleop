# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""dora-rs node that mimics camera for testing."""

import argparse
import cv2
import dora
import os
import numpy as np
import pyarrow as pa


def _generate_random_image(h, w):
    return np.random.randint(0, 256, size=(h, w, 3), dtype=np.uint8)


def main():
    """Mimics camera."""
    parser = argparse.ArgumentParser(description="Record data as OpenArm dataset")
    parser.add_argument(
        "--encoding",
        default=os.getenv("ENCODING", "jpeg"),
        help="The encoding.",
        type=str,
    )
    parser.add_argument(
        "--image-width",
        default=int(os.getenv("IMAGE_WIDTH", 960)),
        help="The width of the image output. Default is 960.",
        type=int,
    )
    parser.add_argument(
        "--image-height",
        default=int(os.getenv("IMAGE_HEIGHT", 600)),
        help="The height of the camera. Default is 600.",
        type=int,
    )
    parser.add_argument(
        "--jpeg-quality",
        default=os.getenv("JPEG_QUALITY", 95),
        help="The JPEG quality. (0-100) Default is 95.",
        type=int,
    )
    args = parser.parse_args()

    node = dora.Node()
    for event in node:
        if event["type"] != "INPUT":
            continue

        # Main process
        metadata = event["metadata"]
        metadata["encoding"] = args.encoding
        metadata["width"] = args.image_width
        metadata["height"] = args.image_height

        image = _generate_random_image(args.image_height, args.image_width)

        if args.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif args.encoding == "yuv420":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420)
        elif args.encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
            encode_params = []
            if args.encoding in ["jpeg", "jpg", "jpe"]:
                encode_params += [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
            success, image = cv2.imencode("." + args.encoding, image, encode_params)
            if not success:
                print("Failed to encode image")
                continue

        node.send_output("image", pa.array(image.ravel()), metadata)


if __name__ == "__main__":
    main()
