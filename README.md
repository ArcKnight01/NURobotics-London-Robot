# NURobotics-London-Robot

This repository contains the hardware and software stack for the robot developed by the Northeastern University – London Robotics Team for the UniBots UK 2023 inter-collegiate robotics competition. The design uses a Raspberry Pi 4B controller and Python modules that are orchestrated by `AutonomousController.py` to coordinate drive motors, servos, ultrasonic sensors, the RGB status indicator, the camera gimbal, the ADCS (IMU fusion) stack, and onboard vision processing.
For additional project documentation, media, and competition results, see: [https://www.aidanrc.com/unibots-uk-2023](https://www.aidanrc.com/unibots-uk-2023)
The technical document for this project is contained [here](https://docs.google.com/document/d/1x95HRRY2IjaurOZZFUa4TBfEYou3pGcw90neMIvEJIw/edit?usp=sharing).

## Hardware Architecture  
| Component                | Description                                  | Interface       |
|--------------------------|----------------------------------------------|-----------------|
| Raspberry Pi 4B          | Main compute and control board               | USB / GPIO      |
| Motor drivers            | Dual‐motor and servo control drivers         | PWM / GPIO      |
| Ultrasonic Sonar (L) | Obstacle detection (left direction)       | GPIO (Trig/Echo)|
| Ultrasonic Sonar (R)  | Obstacle detection (right direction)          | GPIO (Trig/Echo)|
| BNO055 IMU               | Orientation and motion sensing (yaw/pitch/roll)| I²C           |
| Camera Module            | Front-facing vision for image processing     | CSI             |
| Chassis & Wheels         | Mobile drive base                            | —               |

## Pinout Summary

| Peripheral                | BCM Pin(s) (BCM numbering)              | Notes |
|---------------------------|-----------------------------------------|-------|
| Drive motor 1             | IN1 → 14 · IN2 → 15 · PWM/EN → 18       | Configured via `motor1_pins` |
| Drive motor 2             | IN1 → 8 · IN2 → 7 · PWM/EN → 12         | Configured via `motor2_pins` |
| Drive motor 3             | IN1 → 6 · IN2 → 5 · PWM/EN → 13         | Configured via `motor3_pins` |
| Drive motor 4             | IN1 → 20 · IN2 → 26 · PWM/EN → 19       | Configured via `motor4_pins` |
| Optional motor 5 / intake | IN1 → 9 · IN2 → 11 · PWM/EN → 10        | Disabled by default (`motor5_pins`) |
| Optional motor 6 / intake | IN1 → 27 · IN2 → 17 · PWM/EN → 22       | Disabled by default (`motor6_pins`) |
| Left HC-SR04 sonar        | Echo → 0 · Trigger → 1                  | Set with `distance_sensor_left_pin` |
| Right HC-SR04 sonar       | Echo → 21 · Trigger → 16                | Set with `distance_sensor_right_pin` |
| Camera pan servo          | Signal → 10                             | `top_servo_pin` argument |
| Camera tilt servo         | Signal → 9                              | `bottom_servo_pin` argument |
| RGB LED (common anode/cathode) | Red → 23 · Green → 24 · Blue → 25  | Configured via `rgb_pins` |
| User start button         | Signal → 4                              | Provided through `button_pin` |
| Buzzer                    | Signal → 11                             | Controlled through `buzzer_pin` |
| BNO055 IMU (I²C)          | SDA → 2 · SCL → 3                       | Uses Raspberry Pi I²C bus |
| Camera                    | CSI connector                           | Raspberry Pi camera interface |

Pins can be reconfigured in software by supplying alternative values to the constructor arguments of `AutonomousController` (for example `motor*_pins`, `distance_sensor_*_pin`, `rgb_pins`, `button_pin`, `*_servo_pin`, or `buzzer_pin`). Update your wiring to match any changes you make in code.

## Software Architecture
- **Operating System:** Raspberry Pi OS (32-bit)
- **Language:** Python 3.x
- **Key Libraries:** `numpy`, `opencv-python`, `RPi.GPIO`, `smbus2`, `adafruit-circuitpython-bno055`, `gpiozero`
- **Runtime Orchestration:** `AutonomousController.py` instantiates and coordinates all subsystems, runs the timed competition routine, and exposes helper methods for motion, perception, and telemetry logging.
- **Subsystem Modules:**
  - [`DCMotors.py`](./DCMotors.py) — wraps `gpiozero.Motor` objects into individually addressable drive motors and a `DriveMotors` helper that provides tank steering and stop routines.
  - [`Sonar.py`](./Sonar.py) — manages HC-SR04 ultrasonic sensors for left/right obstacle ranging and avoidance checks.
  - [`Servo_Motors.py`](./Servo_Motors.py) — provides a light abstraction over `gpiozero.Servo` with degree-based positioning and reset helpers.
  - [`RGB_Indicator.py`](./RGB_Indicator.py) — controls the tri-colour status LED to signal robot state or errors.
  - [`CameraMount.py`](./CameraMount.py) — couples two `ServoMotor` instances to aim the camera, including a sweep (`revolve`) routine for scanning.
  - [`ADCS_System.py`](./ADCS_System.py) — configures the BNO055 IMU, handles calibration, fuses acceleration/gyro/magnetometer readings, and logs telemetry to CSV for post-run analysis.
  - [`Image_Processor.py`](./Image_Processor.py) — captures frames from the PiCamera, performs buoy detection, triggers feedback via the buzzer, and logs frames for review.
  - [`Camera_Util.py`](./Camera_Util.py) — vision utility functions (AprilTag detection, buoy colour filtering, geometric helpers) consumed by the image processor.

### Control Loop (Pseudocode)
initialize `AutonomousController`

while robot_active:
    update ADCS, sonar, and image processing modules
    derive motion plan from current mission phase
    command drive motors and camera mount
    log telemetry


## Setup & Installation  
### 1. Prepare the Raspberry Pi  
- Flash the Raspberry Pi OS (32-bit) onto the SD card.  
- Enable SSH, I2C, and camera via `raspi-config`.
- To connect to the pi:
```bash
ssh pi@robotpi.local
fish
```

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

Run the IMU calibration routine within [`ADCS_System.py`](./ADCS_System.py) and record fresh offsets before each event.

Verify sonar readings in a safe environment.

6. Run the System
python3 AutonomousController.py     # Runs the autonomous controller loop

The `AutonomousController.py` entry point enables an automatic start mode (`automatic_start=True`) that bypasses the physical
start button for bench testing, drives a predefined sequence via `driveForTime`, and continuously logs timing/power status.
Toggle ultrasound avoidance by setting `autonomousController.ultrasound_enabled = True` (or calling
`autonomousController.switch_ultrasound_enable()`), and gate motor motion via the `motor_enable` flag in the `__main__` loop.
Adjust logging chatter by editing the `verbose` flags used when instantiating `AutonomousController`, `ADCS`, and
`ImageProcessor` near the top of the script.

### License

This project is released under the MIT License. See the LICENSE file for full details.

## Acknowledgements

Thank you to the Northeastern London Robotics Team, competition organizers of UniBots UK, and open-source libraries and communities that support robotics development.



