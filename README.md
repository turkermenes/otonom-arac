# Autonomous Person-Following Robot Car

A Raspberry Pi 5 robot car that uses a Pi Camera and YOLOv8 to find a person and follow them. A proportional (P) controller steers the car toward the target, a Kalman filter smooths the target's position, and when the person is lost the car spins in place toward where they were last seen. The annotated camera feed is streamed live to a browser over Flask.

## Features

- **Person detection with YOLOv8n** (`ultralytics`). Only the COCO `person` class is used, with a 0.25 confidence threshold; when several people are in view, the largest bounding box (usually the closest person) is chosen as the target.
- **Threaded pipeline.** Camera capture and motor control run in the main loop, and inference runs in its own thread. Both sides use single-slot queues, so the controller always works with the newest result and never waits on the model.
- **Kalman filtering.** A constant-velocity Kalman filter (OpenCV, 4 states / 2 measurements) smooths the target center and keeps predicting its position for a while after detections drop out.
- **P-control steering.** The horizontal offset of the target from the frame center is turned into a speed difference between the left and right motors.
- **Distance estimate.** The bounding-box width sorts the target into `UZAK` (far), `ORTA` (medium) or `YAKIN` (close). The car stops when the target is close.
- **Search (scan) mode.** If no person is detected, the car spins in place toward the side where the target was last seen. After `TOTAL_SEARCH_TIMEOUT` seconds the search cycle stops and resets.
- **Live MJPEG stream.** The annotated video (bounding box, Kalman prediction, trajectory trail, state text, FPS) is served at `http://<pi-ip>:5000`.
- **CSV logging.** The predicted target position, the driving decision and the data source are buffered and written to `takip_log.csv` about once per second.
- **Safe shutdown.** On exit (for example Ctrl+C), the detection thread is stopped, the remaining logs are written to disk, the camera is closed and the motors are stopped.

## Hardware

| Component | Notes |
|---|---|
| Raspberry Pi 5 | Main computer |
| Raspberry Pi Camera module | Used through `picamera2` |
| Dual H-bridge motor driver (L298N-style: IN1–IN4, ENA/ENB) | Direction pins plus PWM enable pins |
| 2 × DC motor groups (left / right) + chassis | Differential (tank-style) steering |
| Battery pack(s) | Separate supplies for the motors and the Pi are recommended |

### Wiring (BCM GPIO numbering, set in `config.py`)

| Driver pin | GPIO | Function |
|---|---|---|
| IN1 | 22 | Motor A (right side) direction |
| IN2 | 23 | Motor A (right side) direction |
| ENA | 12 | Motor A PWM speed |
| IN3 | 27 | Motor B (left side) direction |
| IN4 | 24 | Motor B (left side) direction |
| ENB | 13 | Motor B PWM speed |

Connect the motor driver's GND to a Raspberry Pi GND pin.

## How It Works

```
 Pi Camera ──► main loop ──(frame_queue)──► DetectionThread (YOLOv8n, imgsz=320)
                   ▲                                   │
                   └───────────(result_queue)◄─────────┘ largest "person" box
                   │
                   ├─► Kalman filter ─► P-controller ─► MotorController (gpiozero)
                   ├─► StateLogger ─► takip_log.csv
                   └─► web_server.set_global_frame() ─► Flask MJPEG stream :5000
```

1. **Capture.** A 640×480 frame is grabbed from the Pi Camera and mirrored horizontally.
2. **Detect.** The frame is put on `frame_queue` without blocking. The detection thread runs YOLOv8n on it and puts the best person box on `result_queue`.
3. **Track.** The box center is fed to the Kalman filter, and the predicted center `(pred_x, pred_y)` is used for control.
4. **Control.**
   - `dx = pred_x − frame_center_x`
   - If the box is wider than 200 px, the target is **close**, so the car **stops**.
   - If `|dx| ≤ DEADZONE`, the car drives **straight** at `BASE_SPEED`.
   - Otherwise it sets `turn = clamp(Kp · dx, ±BASE_SPEED)`, then `motor A = BASE_SPEED + turn` and `motor B = BASE_SPEED − turn`, each clamped to [0, 1].
5. **Search.** When no target is detected, the car spins in place (one side forward, the other backward at a lower speed) toward the last known side of the target. The Kalman filter keeps predicting during the search. After the timeout, the motors stop and the search timer restarts.
6. **Output.** The overlays are drawn, logs are flushed, and the frame is published to the web stream.

## Project Structure

| File | Description |
|---|---|
| `main.py` | Entry point: camera setup, thread startup, main control loop, safe shutdown |
| `config.py` | All tunable parameters: control gains, speeds, resolution, model path, GPIO pins, logging |
| `detector.py` | `DetectionThread`: loads YOLOv8, runs inference, selects the target person |
| `motor_control.py` | `MotorController`: forward/backward/stop and PWM speed control through `gpiozero` |
| `logger.py` | `StateLogger`: Kalman filter, trajectory buffer, buffered CSV logging, search state |
| `web_server.py` | Flask app that serves the annotated frames as an MJPEG stream |
| `requirements.txt` | Python dependencies |

## Installation

On Raspberry Pi OS (Bookworm, 64-bit):

```bash
# Picamera2 is best installed from apt
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv

# Create a venv that can see the system picamera2 package
python3 -m venv --system-site-packages venv
source venv/bin/activate

pip install -r requirements.txt
```

> `picamera2` is listed in `requirements.txt`, but on a Raspberry Pi the apt package is the reliable way to install it. If pip fails on it, remove it from the list and use the apt version.

### YOLOv8 weights

Set `YOLO_MODEL_PATH` in `config.py` to the location of `yolov8n.pt`. The default is `/home/pi/Desktop/otonom_arac/yolov8n.pt`. If you set it to just `"yolov8n.pt"`, Ultralytics downloads the weights automatically on the first run.

## Usage

```bash
python3 main.py
```

Then open `http://<raspberry-pi-ip>:5000` in a browser on the same network to watch the live annotated stream.

Press **Ctrl+C** to stop. The motors are always stopped on exit.

> ⚠️ Put the car on a stand or keep the wheels off the ground for the first test run, because it starts driving as soon as it sees a person.

## Configuration

All settings are in `config.py`:

| Parameter | Default | Description |
|---|---|---|
| `BASE_SPEED` | `1.0` | Forward speed when the target is centered (0.0–1.0) |
| `Kp` | `0.100` | Proportional steering gain (applied to the pixel error) |
| `DEADZONE` | `20` | Pixel tolerance around the center before the car steers |
| `SCAN_SPEED` | `0.9` | Spin speed in search mode |
| `TOTAL_SEARCH_TIMEOUT` | `30.0` | Seconds the car searches before resetting |
| `FRAME_WIDTH` / `FRAME_HEIGHT` | `640` / `480` | Camera resolution |
| `YOLO_MODEL_PATH` | `.../yolov8n.pt` | Path to the YOLOv8 weights |
| `DEVICE` | `'cuda'` | Inference device; falls back to `cpu` automatically on the Pi |
| `MOTOR_*` | see wiring | GPIO pins for the motor driver |
| `CSV_FILENAME` | `takip_log.csv` | Log file (overwritten on every start) |
| `LOG_WRITE_INTERVAL_SEC` | `1.0` | How often buffered logs are written to disk |

### Tuning tips

- `Kp` multiplies the error in **pixels**. With `Kp = 0.1` and `BASE_SPEED = 1.0`, the turn correction reaches its limit only about 10 px past the dead zone, so steering acts almost like on/off control. Lower `Kp` (for example to 0.003–0.01) for smoother, more gradual turns.
- `BASE_SPEED = 1.0` leaves no headroom to speed up the outer wheel, so turns only come from slowing the inner wheel. A lower base speed (for example 0.6–0.7) gives the controller room on both sides.
- If the car turns the wrong way, swap the motor wires or the `MOTOR_IN*` pin assignments. The image is mirrored (`cv2.flip`), which also affects left and right.
- The close/medium/far thresholds (200 px and 100 px box width) are set in `main.py` and depend on the camera's field of view.

## Log Format

`takip_log.csv` columns:

| Column | Description |
|---|---|
| `timestamp` | `HH:MM:SS` |
| `x`, `y` | Kalman-predicted target center (pixels) |
| `direction` | `MERKEZ` (center), `SAG` (right), `SOL` (left), `YAKIN` (close, stopped), `ARAMA_SAG` / `ARAMA_SOL` (searching right/left), `BEKLEME` (idle) |
| `source` | `YOLOv8+Kalman` (target visible) or `Kalman Search` (target lost) |

> Console output, on-screen labels and log values are in Turkish.

## Tech Stack

Python · Raspberry Pi 5 · Picamera2 · YOLOv8 (Ultralytics) · PyTorch · OpenCV · gpiozero · Flask · pandas · NumPy
