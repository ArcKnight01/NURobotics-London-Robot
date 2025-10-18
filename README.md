# NURobotics-London-Robot

## Project Overview  
This repository contains the hardware and software stack for the robot developed by the Northeastern University – London Robotics Team for the UniBots UK 2023 inter-collegiate robotics competition. The design uses a Raspberry Pi 4B controller and Python modules that are orchestrated by `AutonomousController.py` to coordinate drive motors, servos, ultrasonic sensors, the RGB status indicator, the camera gimbal, the ADCS (IMU fusion) stack, and onboard vision processing.
For additional project documentation, media, and competition results, see: [https://www.aidanrc.com/unibots-uk-2023](https://www.aidanrc.com/unibots-uk-2023)
The technical document for this project is contained [here](https://docs.google.com/document/d/1x95HRRY2IjaurOZZFUa4TBfEYou3pGcw90neMIvEJIw/edit?usp=sharing).

## Objectives  
- Provide a reusable robotics platform combining hardware and software suitable for competition and research.  
- Enable autonomous navigation using sensor fusion (sonar + IMU + vision).  
- Provide hooks for future manual control/override while prioritising an autonomous competition mode.
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
    log telemetry / persist frames


## Setup & Installation  
### 1. Prepare the Raspberry Pi  
- Flash the Raspberry Pi OS (32-bit) onto the SD card.  
- Enable SSH, I2C, and camera via `raspi-config`.

### 2. Clone the Repository
```bash
git clone https://github.com/ArcKnight01/NURobotics-London-Robot.git
cd NURobotics-London-Robot
```

### 3. Install Dependencies
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-rpi.gpio python3-opencv pigpio
pip3 install numpy opencv-python adafruit-circuitpython-bno055 gpiozero smbus2 pyserial
```

### 4. Wire the Hardware

Follow the pinout summary above to connect sensors, motors, IMU and camera. A wiring diagram is contained in the technical document.
Ensure power supply is stable (typically 5 V logic + 12 V drive) and all grounds are common.

### 5. Calibrate Sensors

Run the IMU calibration routine within [`ADCS_System.py`](./ADCS_System.py) and record fresh offsets before each event.

Verify sonar readings in a safe environment.

### 6. Run the System
`AutonomousController.py` is the entry point for deployments and handles the timed routine automatically once the start button is pressed. Manual teleoperation is not currently implemented in this repository, but the controller class exposes helpers for future expansion.

```bash
python3 AutonomousController.py
```

Use the onboard toggle button (GPIO 4) to start/stop the autonomous routine when running on the robot.

## Diagnostics & Calibration Utilities
- [`Tests/gpio_indiv_motor_test.py`](./Tests/gpio_indiv_motor_test.py) — exercise individual drive motors to verify wiring, direction, and PWM tuning.
- [`Tests/gpio_led_test.py`](./Tests/gpio_led_test.py) — validate RGB indicator wiring and colour mappings.
- [`Tests/image_test.py`](./Tests/image_test.py) — run vision routines against stored frames for tuning thresholds without deploying on hardware.
- [`ADCS_System.py`](./ADCS_System.py) — contains calibration routines (`calibrate_*` methods) for the BNO055 IMU; run on the robot to refresh offsets before competitions.


License

This project is released under the MIT License. See the LICENSE file for full details.

Acknowledgements

Thank you to the Northeastern London Robotics Team, competition organizers of UniBots UK, and open-source libraries and communities that support robotics development.



