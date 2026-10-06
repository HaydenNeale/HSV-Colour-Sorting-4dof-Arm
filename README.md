# HSV-Colour-Sorting-4dof-Arm

Vision-guided 4DOF robotic arm that finds coloured blocks with HSV colour
segmentation, converts their pixel position to real-world coordinates, solves
inverse kinematics, and sends servo angles to a Raspberry Pi Pico over USB serial.

## How it works

1. **Vision** (`hsvtest.py`): converts the image to HSV, masks each colour, takes the
   largest contour per colour, and uses the bottom-centre of its bounding box as the
   table contact point. A homography from `calibration.json` maps that pixel to
   `(x, y, z)` in cm relative to the base rotation axis.
2. **Inverse kinematics** (`IKtest.py`): turns `(x, y, z)` into base, shoulder and
   elbow servo angles. `fk()` reverses this as a sanity check.
3. **Control** (`main.py`): builds, for each block, a hover pose (`z + 4 cm`) and a grip
   pose, then sends them to the Pico one at a time, waiting for a `DONE` reply before
   continuing.
4. **Firmware** (`pico/main.py`, MicroPython): reads angles from USB serial and moves the
   four servos with smoothed, per-servo step-limited motion.

## Hardware
- Raspberry Pi Pico (servo control)
- Desktop PC running the Python control code (`desktop code/`), connected to the Pico by
  USB (Windows `COM` port in the current config)
- Camera (currently a saved photo, `detect_photo.jpg`, is used as input)
- 4DOF robotic arm with 4 servos: claw, elbow, shoulder, base

### Arm dimensions (used by the IK solver)
| Link | Length | Description |
| :--- | :----- | :---------- |
| L0 | 4 cm | Base rotation axis to shoulder joint |
| L1 | 8 cm | Shoulder to elbow |
| L2 | 12 cm | Elbow to claw tip (7 cm link + 5 cm wrist/claw) |

Coordinate frame (from the base rotation axis): `x` forward, `y` left, `z` up.

## Repo structure
```
desktop code/      # runs on the desktop PC (demo setup)
  main.py            # control: vision -> IK -> serial to Pico
  hsvtest.py         # HSV block detection + pixel-to-world mapping
  IKtest.py          # inverse / forward kinematics
  calibration.json   # camera homography + table height (from camera_calibration.py)
  detect_photo.jpg   # test image used for detection
  requirements.txt
pico/
  main.py            # MicroPython servo firmware
```

## Setup

### Desktop PC
```bash
cd "desktop code"
python3 -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

`requirements.txt`:
```
opencv-python>=4.8.0
numpy>=1.24.0
pyserial>=3.5
```

Set the Pico's port at the top of `main.py`:
```python
Port = 'COM10'   # check the Pico's port in Device Manager / Thonny
baud = 115200
```
If the port can't be opened, `main.py` falls back to a simulation mode that only prints
the angles it would send.

### Camera calibration
Run `camera_calibration.py` to produce `calibration.json` (homography `H` and
`table_z_cm`). Redo this any time the camera moves.

### Pico
Flash MicroPython, then copy `pico/main.py` onto the board (Thonny or `mpremote`).
Optional editor type hints:
```bash
cd pico
pip install micropython-rp2-stubs --target ./typings
```

## Wiring

Servo signal pins on the Pico (50 Hz PWM):

| Servo    | Variable   | GPIO     | Physical Pin |
| :------- | :--------- | :------- | :----------- |
| Claw     | `Claw`     | **GP8**  | Pin 11       |
| Elbow    | `Elbow`    | **GP9**  | Pin 12       |
| Shoulder | `Shoulder` | **GP10** | Pin 14       |
| Base     | `base`     | **GP11** | Pin 15       |

Servos should be powered from a suitable external supply with a common ground to the Pico.

## Serial protocol (PC <-> Pico)

Connection: USB serial, 115200 baud.

| Direction | Message | Meaning |
| :-------- | :------ | :------ |
| PC -> Pico | `claw,elbow,shoulder,base\n` | Target angles in degrees, e.g. `100,90,0,0` |
| Pico -> PC | `DONE` | All servos reached the target |
| Pico -> PC | `LOG: ...` | Diagnostic message (e.g. invalid command format) |

Angle order everywhere is **Claw, Elbow, Shoulder, Base**.

## Pick sequence
For each of Red, Green, Blue: move to the hover pose above the block, move to the block,
then (planned) grip and move to the `Drop` pose. Yellow is detected but not sorted yet.

## Calibration constants (in `IKtest.py`)
`BASE_OFFSET_DEG`, `SHOULDER_FROM_VERTICAL_AT_ZERO`, `ELBOW_INTERIOR_AT_ZERO`,
`*_SIGN` multipliers, and `CLAW_Z_OFFSET` map IK angles to real servo commands.
Tune these on the physical arm (use the `fk()` round-trip check).

## Known issues
- `main.py` overwrites the claw angle with `0` when a block is found, so the intended
  `100` claw value in the offset arrays is lost.
- If a colour isn't detected, its IK is attempted on `(0, 0, 0)`, fails, and the
  remaining colours are skipped because all IK calls share one `try` block.
- The reachability check only tests that angles are numbers, not that IK succeeded.
- Gripping and drop moves are not yet in the send sequence.
- [ ] Add error handling for camera not detected
- [ ] Add debug LEDs for status/error reporting
- [ ] Write debug/report output (logging system state, errors, detection results)
