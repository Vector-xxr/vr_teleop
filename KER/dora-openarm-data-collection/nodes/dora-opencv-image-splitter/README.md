# dora-opencv-image-splitter

A [dora-rs](https://dora-rs.ai/) node to split one image into multiple images.

## Usage

### Inputs

| ID      | Type                  | Metadata                          | Description                                            |
|---------|-----------------------|-----------------------------------|--------------------------------------------------------|
| `image` | `UInt8` Arrow array   | `encoding`, `width`, `height`     | Raw image as a flat array of shape H × W × channels   |

Supported encodings: `bgr8`, `rgb8`, `jpeg`, `jpg`, `jpe`, `bmp`, `webp`, `png`.

### Outputs

The outputs are dynamically named by index: `image_0`, `image_1`, ..., `image_N-1`.

| ID          | Type                | Metadata                          | Description                                             |
|-------------|---------------------|-----------------------------------|---------------------------------------------------------|
| `image_N`   | `UInt8` Arrow array | `encoding`, `width`, `height`     | JPEG-encoded sub-image cropped from the input image     |

Each output carries the same metadata as the input, with `encoding` set to `"jpeg"` and `width`/`height` updated to the sub-image dimensions.

### Configuration

The node can be configured via environment variables or command-line arguments:

| Environment variable | CLI argument        | Default      | Description                                                          |
|----------------------|---------------------|--------------|----------------------------------------------------------------------|
| `SPLIT_MODE`         | `--split-mode`      | `"vertical"` | Split strategy: `"horizontal"` (rows), `"vertical"` (columns), or `"bbox"` |
| `NUM_SPLITS`         | `--num-splits`      | `2`          | Number of equal splits for `horizontal`/`vertical` mode             |
| `JPEG_QUALITY`       | `--jpeg-quality`    | `90`         | JPEG quality for output encoding, `0`–`100`                          |
| `IMAGE_WIDTH`        | `--image-width`     | `640`        | Fallback image width when metadata is absent                         |
| `IMAGE_HEIGHT`       | `--image-height`    | `480`        | Fallback image height when metadata is absent                        |
| `IMAGE_ENCODING`     | `--image-encoding`  | *(none)*     | Fallback image encoding when metadata is absent                      |
| `BBOXES`             | `--bboxes`          | *(none)*     | Bounding boxes for `bbox` mode as a flat list: `x1 y1 x2 y2 ...`    |

### Example dataflow

See [`example/dataflow.yaml`](example/dataflow.yaml) for an example that splits a camera image into two vertical halves and displays each half separately.

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
