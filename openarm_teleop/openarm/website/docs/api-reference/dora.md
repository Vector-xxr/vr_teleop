---
title: Dora
description: ...
sidebar_position: 5
---

## Overview

These are the Dora nodes that make up the OpenArm ecosystem. Each node is an
independent Python process; they communicate by passing Arrow arrays over [Dora](https://dora-rs.ai/)
topics. A dataflow YAML wires them together.

Nodes are installed per-dataflow inside a `build:` step in
the YAML. Mock nodes (prefix `dummy`) let you run a dataflow without physical
hardware.

Below is a list of nodes we currently use.

---

### Robot Control

| Node | Description |
|------|-------------|
| [dora-openarm](https://github.com/enactic/dora-openarm) | Controls the OpenArm |
| [dora-openarm-ker](https://github.com/enactic/dora-openarm-ker) | Reads the KER leader device |
| [dora-openarm-cell-lifter](https://github.com/enactic/dora-openarm-cell-lifter) | Drives the cell lifter from joystick or command |
| [dora-openarm-kinematics](https://github.com/enactic/dora-openarm-kinematics) | Computes kinematics based on OpenArm 2.0 |


### Data Collection

| Node | Description |
|------|-------------|
| [dora-openarm-data-collection](https://github.com/enactic/dora-openarm-data-collection) | Configures dataflows to collect teleoperation data with OpenArm |
| [dora-openarm-data-collection-ui](https://github.com/enactic/dora-openarm-data-collection-ui) | Episode UI (web); operator starts/stops recording |
| [dora-openarm-dataset-recorder](https://github.com/enactic/dora-openarm-dataset-recorder) | Writes one episode file per recording |
| [dora-opencv-image-splitter](https://github.com/enactic/dora-opencv-image-splitter) | Splits an image into sub-images (vertical/horizontal/bbox) |

### Bridging Nodes

| Node | Description |
|------|-------------|
| [dora-openarm-mujoco](https://github.com/enactic/dora-openarm-mujoco) | Simulates the OpenArm bimanual in MuJoCo |

### Inference / Policy

| Node | Description |
|------|-------------|
| [dora-openarm-observer](https://github.com/enactic/dora-openarm-observer) | Buffers the latest observations and bundles them on each tick |
| [dora-openarm-inference-controller](https://github.com/enactic/dora-openarm-inference-controller) | Waits for arms to be ready, starts episodes, detects success/timeout, retries |
| [dora-openarm-actions-executor](https://github.com/enactic/dora-openarm-actions-executor) | Unpacks action chunk; optionally upsamples and low-pass filters |
| [dora-openarm-local-policy-server](https://github.com/enactic/dora-openarm-local-policy-server) | Bridges dora to an external model process over a UNIX socket (`$SOCKET`) |
| [dora-openarm-docker-policy-server](https://github.com/enactic/dora-openarm-docker-policy-server) | Launches and bridges to a policy server Docker container (`$IMAGE`) |

### Utilities

| Node | Description |
|------|-------------|
| [dora-openarm-quitter](https://github.com/enactic/dora-openarm-quitter) | Passes through any data topic; stops the dataflow when `command` is `quit` |

### Testing / Mock Nodes

Drop-in replacements that emit plausible data without physical hardware. Useful
for CI and dataflow development.

| Node | Description |
|------|-------------|
| [dora-openarm-dummy](https://github.com/enactic/dora-openarm-dummy) | Mimics OpenArm |
| [dora-openarm-dummy-ker](https://github.com/enactic/dora-openarm-dummy-ker) | Mimics OpenArm KER |
| [dora-openarm-dummy-cell-lifter](https://github.com/enactic/dora-openarm-dummy-cell-lifter) | Mimics the Cell Lifter |
| [dora-openarm-dummy-camera](https://github.com/enactic/dora-openarm-dummy-camera) | Mimics a camera |
| [dora-openarm-dummy-policy-server](https://github.com/enactic/dora-openarm-dummy-policy-server) | Mimics a policy server |