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

"""dora-rs node to split one image to multiple images.

Accepts a raw BGRimage and splits it into multiple JPEG sub-image streams.

Inputs:
  - image: Raw image as a flat UInt8 Arrow array (H x W x channels).
           Metadata must carry "encoding", "width", "height".

Env vars / CLI args:
  SPLIT_MODE     / --split-mode    : "bbox" | "horizontal" | "vertical"  (default: "vertical")
  NUM_SPLITS     / --num-splits    : equal splits for horizontal/vertical mode (default: 2)
  JPEG_QUALITY   / --jpeg-quality  : JPEG quality for output encoding (default: 90)
  IMAGE_WIDTH    / --image-width   : fallback image width  (default: 640)
  IMAGE_HEIGHT   / --image-height  : fallback image height (default: 480)
  IMAGE_ENCODING / --image-encoding: fallback encoding
  BBOXES         / --bboxes        : flat list x1 y1 x2 y2 ... for bbox mode

Outputs (dynamic, named by index):
  image_0, image_1, ..., image_N-1
  Each output is JPEG-encoded and carries the same metadata as the input
  plus updated "encoding", "width", and "height" fields.
"""

import argparse
import os
import cv2
import numpy as np
import pyarrow as pa
from dora import Node


def _split_horizontal(frame: np.ndarray, n: int) -> list[np.ndarray]:
    """Split into n equal horizontal bands (stacked rows)."""
    h = frame.shape[0]
    step = h // n
    return [frame[i * step : (i + 1) * step, :, :] for i in range(n)]


def _split_vertical(frame: np.ndarray, n: int) -> list[np.ndarray]:
    """Split into n equal vertical bands (stacked columns)."""
    w = frame.shape[1]
    step = w // n
    return [frame[:, i * step : (i + 1) * step, :] for i in range(n)]


def _split_by_bboxes(frame: np.ndarray, bboxes: np.ndarray) -> list[np.ndarray]:
    """Crop frame to each bounding box [x1, y1, x2, y2]."""
    crops = []
    h, w = frame.shape[:2]
    for box in bboxes:
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 > x1 and y2 > y1:
            crops.append(frame[y1:y2, x1:x2, :])
        else:
            # Degenerate box — emit a 1×1 placeholder so indices stay aligned
            crops.append(np.zeros((1, 1, frame.shape[2]), dtype=frame.dtype))
    return crops


def _encode_jpeg(crop: np.ndarray, quality: int) -> np.ndarray:
    """JPEG-encode a crop; returns a 1-D uint8 array of compressed bytes."""
    success, buf = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not success:
        raise RuntimeError("cv2.imencode failed")
    return buf.ravel()


def _build_metadata(base_metadata: dict, crop: np.ndarray) -> dict:
    """Return metadata dict with updated width, height, and encoding for this crop."""
    meta = dict(base_metadata)
    meta["encoding"] = "jpeg"
    meta["width"] = crop.shape[1]
    meta["height"] = crop.shape[0]
    return meta


def main():
    """Split incoming raw images into JPEG sub-streams."""
    parser = argparse.ArgumentParser(description="Split one image into multiple images")
    parser.add_argument(
        "--split-mode",
        choices=["horizontal", "vertical", "bbox"],
        default=os.getenv("SPLIT_MODE", "vertical"),
        help="Split strategy: horizontal (rows), vertical (columns), or bbox",
        type=str,
    )
    parser.add_argument(
        "--num-splits",
        default=int(os.getenv("NUM_SPLITS", 2)),
        help="Number of equal splits for horizontal/vertical mode (default: 2)",
        type=int,
    )
    parser.add_argument(
        "--jpeg-quality",
        default=int(os.getenv("JPEG_QUALITY", 90)),
        help="JPEG quality for output encoding, 0-100 (default: 90)",
        type=int,
    )
    parser.add_argument(
        "--image-width",
        default=int(os.getenv("IMAGE_WIDTH", 640)),
        help="Fallback image width when metadata is absent (default: 640)",
        type=int,
    )
    parser.add_argument(
        "--image-height",
        default=int(os.getenv("IMAGE_HEIGHT", 480)),
        help="Fallback image height when metadata is absent (default: 480)",
        type=int,
    )
    parser.add_argument(
        "--image-encoding",
        default=os.getenv("IMAGE_ENCODING"),
        help="Fallback image encoding when metadata is absent",
        type=str,
    )
    parser.add_argument(
        "--bboxes",
        default=None,
        help="Bounding boxes for bbox split mode, as flat list: x1 y1 x2 y2 ...",
        nargs="+",
        type=int,
    )
    args = parser.parse_args()

    bboxes: np.ndarray | None = None
    if args.bboxes is not None:
        bboxes = np.array(args.bboxes, dtype=np.int32).reshape(-1, 4)

    node = Node()

    for event in node:
        if event["type"] == "STOP":
            break

        if event["type"] != "INPUT" or event["id"] != "image":
            continue

        metadata: dict = event["metadata"]
        width = metadata.get("width", args.image_width)
        height = metadata.get("height", args.image_height)
        encoding = metadata.get("encoding", args.image_encoding)

        if encoding == "bgr8":
            num_channels = 3
            frame = np.array(event["value"], dtype=np.uint8).reshape(
                (height, width, num_channels)
            )
        elif encoding == "rgb8":
            num_channels = 3
            frame = np.array(event["value"], dtype=np.uint8).reshape(
                (height, width, num_channels)
            )
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
            frame = cv2.imdecode(
                np.array(event["value"], dtype=np.uint8), cv2.IMREAD_COLOR
            )
        else:
            print(f"Unsupported image encoding: {encoding}")
            continue

        if args.split_mode == "bbox" and bboxes is not None:
            crops = _split_by_bboxes(frame, bboxes)
        elif args.split_mode == "vertical":
            crops = _split_vertical(frame, args.num_splits)
        else:
            crops = _split_horizontal(frame, args.num_splits)

        for idx, crop in enumerate(crops):
            node.send_output(
                f"image_{idx}",
                pa.array(_encode_jpeg(crop, args.jpeg_quality)),
                _build_metadata(metadata, crop),
            )


if __name__ == "__main__":
    main()
