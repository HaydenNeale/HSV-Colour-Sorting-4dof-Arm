# HSV-Colour-Sorting-4dof-Arm

Vision-guided 4DOF robotic arm that detects and sorts coloured Lego bricks
using HSV colour segmentation, running on a Raspberry Pi 3B communicating
with a Raspberry Pi Pico over I2C.

## Hardware
- Raspberry Pi 3B (vision processing, high-level control)
- Raspberry Pi Pico (motor/servo control via I2C)
- Camera module
- 4DOF robotic arm

## Repo structure
- `pi3b/` — HSV detection and control logic (runs on the Pi)
- `pico/` — MicroPython firmware for motor/servo control
- `models/` — trained/pretrained model files (if any)

## Setup

### Pi 3B
\`\`\`bash
cd pi3b
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
\`\`\`

### Pico
Flash MicroPython firmware, then copy `pico/main.py` and `pico/lib/` onto
the board (via Thonny or `mpremote`).

## Wiring / I2C
- SDA: GPIO _
- SCL: GPIO _
- I2C address: 0x__
