# NURobotics-London-Robot

## Project Overview  
This repository contains the hardware and software stack for the robot developed by the Northeastern University – London Robotics Team for the UniBots UK 2023 inter-collegiate robotics competition. The design uses a Raspberry Pi 4B controller, Python-based modules for motors, servos, ultrasonic sensors, and an IMU, and supports both autonomous and manual operation.  
For additional project documentation, media, and competition results, see: [https://www.aidanrc.com/unibots-uk-2023](https://www.aidanrc.com/unibots-uk-2023)
The technical document for this project is contained [here](https://docs.google.com/document/d/1x95HRRY2IjaurOZZFUa4TBfEYou3pGcw90neMIvEJIw/edit?usp=sharing).

## Objectives  
- Provide a reusable robotics platform combining hardware and software suitable for competition and research.  
- Enable autonomous navigation using sensor fusion (sonar + IMU + vision).  
- Allow manual control/override for testing and debugging.  
- Foster rapid prototyping of vision, control, and embedded systems.

## Hardware Architecture  
| Component                | Description                                  | Interface       |
|--------------------------|----------------------------------------------|-----------------|
| Raspberry Pi 4B          | Main compute and control board               | USB / GPIO      |
| Motor drivers            | Dual‐motor and servo control drivers         | PWM / GPIO      |
| Ultrasonic Sonar (Front) | Obstacle detection (forward direction)       | GPIO (Trig/Echo)|
| Ultrasonic Sonar (Rear)  | Obstacle detection (rear direction)          | GPIO (Trig/Echo)|
| BNO055 IMU               | Orientation and motion sensing (yaw/pitch/roll)| I²C           |
| Camera Module            | Front-facing vision for image processing     | CSI             |
| Chassis & Wheels         | Mobile drive base                            | —               |

## Pinout Summary  
| Device             | GPIO Pin(s)                                | Description                    |
|--------------------|--------------------------------------------|--------------------------------|
| HC-SR04            | TRIG → GPIO 17 · ECHO → GPIO 27            |         distance sensing       |
| HC-SR04            | TRIG → GPIO 22 · ECHO → GPIO 23            |      distance sensing          |
| Motor Driver A     | IN1 → GPIO 5 · IN2 → GPIO 6 · EN → GPIO 12 | Left wheel control             |
| Motor Driver B     | IN3 → GPIO 13 · IN4 → GPIO 19 · EN → GPIO 18| Right wheel control            |
| BNO055 IMU         | SDA → GPIO 2 · SCL → GPIO 3                | I²C bus orientation sensing    |
| Camera             | CSI connector                              | Pi camera interface            |


## Software Architecture  
- **Operating System:** Raspberry Pi OS (32-bit)  
- **Language:** Python 3.x  
- **Key Libraries:** `numpy`, `opencv‐python`, `RPi.GPIO`, `smbus2`, `adafruit-circuitpython-bno055`, `gpiozero`  
- **Modules:**
  - `motors.py` — motor and servo driver control  
  - `sonar.py` — ultrasonic sensor interface  
  - `imu.py` — IMU initialization and orientation reading  
  - `camera.py` — image capture and processing pipelines  
  - `autonomy.py` — autonomous decision logic  
  - `manual_control.py` — manual override and teleoperation  
  - `main.py` — entry point, orchestrates initialization, mode selection and control loop  

### Control Loop (Pseudocode)  
initialize all modules
select mode (manual or autonomous)

while robot_active:
imu_data = imu.read()
sonar_data = sonar.scan()
camera_data = camera.process()
if mode == "autonomous":
    drive_cmd = autonomy.decide(imu_data, sonar_data, camera_data)
else:
    drive_cmd = manual_control.input()

motors.drive(drive_cmd)
log.status()


## Setup & Installation  
### 1. Prepare the Raspberry Pi  
- Flash the Raspberry Pi OS (32-bit) onto the SD card.  
- Enable SSH, I2C, and camera via `raspi-config`.

### 2. Clone the Repository
```bash
git clone https://github.com/ArcKnight01/NURobotics-London-Robot.git
cd NURobotics-London-Robot
```

### 3. Install System Packages
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-rpi.gpio python3-opencv python3-picamera \
    python3-matplotlib python3-psutil python3-numpy pigpio
```

### 4. Install Python Dependencies
The autonomous controller, image processor, and camera utilities require the following Python packages:

- `numpy`
- `opencv-python`
- `adafruit-circuitpython-bno055`
- `gpiozero` (including TonalBuzzer support)
- `smbus2`
- `pyserial`
- `colorzero`
- `psutil`
- `picamera`
- `apriltag`
- `matplotlib`
- `simpleaudio`

Install them with `pip`:

```bash
pip3 install numpy opencv-python adafruit-circuitpython-bno055 gpiozero smbus2 pyserial \
    colorzero psutil picamera apriltag matplotlib simpleaudio
```

### 5. Additional Setup Notes
- Enable the Pi camera interface and I2C using `sudo raspi-config` (or the non-interactive `raspi-config nonint` commands) before running the software.
- Start the `pigpiod` daemon so `gpiozero` can provide hardware PWM for servos and the TonalBuzzer:
  ```bash
  sudo systemctl enable --now pigpiod
  ```
- If AprilTag detection is required on development machines without camera hardware, ensure the `apriltag` Python wheel is installed and OpenCV can access images from the `Frames/` directory.

### 6. Wire the Hardware

Follow the pinout summary above to connect sensors, motors, IMU and camera. A wiring diagram is contained in the technical document.
Ensure power supply is stable (typically 5 V logic + 12 V drive) and all grounds are common.

### 7. Calibrate Sensors

Run the IMU calibration routine if provided.

Verify sonar readings in a safe environment.

6. Run the System
python3 AutonomousController.py     # Runs the autonomous controller loop

The `AutonomousController.py` entry point enables an automatic start mode (`automatic_start=True`) that bypasses the physical
start button for bench testing, drives a predefined sequence via `driveForTime`, and continuously logs timing/power status.
Toggle ultrasound avoidance by setting `autonomousController.ultrasound_enabled = True` (or calling
`autonomousController.switch_ultrasound_enable()`), and gate motor motion via the `motor_enable` flag in the `__main__` loop.
Adjust logging chatter by editing the `verbose` flags used when instantiating `AutonomousController`, `ADCS`, and
`ImageProcessor` near the top of the script.

> **Future work:** If a dedicated CLI wrapper is desired, create a thin script (e.g., `run_robot.py`) that exposes these
> runtime toggles as command-line options; no such wrapper currently exists in the repository.


License

This project is released under the MIT License. See the LICENSE file for full details.

Acknowledgements

Thank you to the Northeastern London Robotics Team, competition organizers of UniBots UK, and open-source libraries and communities that support robotics development.



