# dora-openarm-ker

A [dora-rs](https://dora-rs.ai/) node for leader OpenArm KER (Kinematic Equivalent Replica).

---

## Quick Start

### 1. Install system dependencies

```bash
sudo apt install libusb-1.0-0-dev
```

### 2. Set up udev rules (run once)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-m5stack.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. Connect M5Stack and verify

Plug the M5Stack CoreS3 into your PC via USB and verify the device is recognized:

```bash
lsusb | grep 303a
# 303a:4002 should appear
```

### 4. Run

```bash
openarm-can-cli can_configure
```

```bash
uv run dora build config/dataflow-data-collection.yaml --uv
uv run dora run config/dataflow-data-collection.yaml --uv
```

---

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
