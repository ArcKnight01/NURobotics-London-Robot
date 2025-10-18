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
The default BCM pin assignments mirror the tuples provided to `AutonomousController.__init__` and cover every onboard peripheral that the controller wires up out of the box.

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
3. Install Dependencies
sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-rpi.gpio python3-opencv pigpio
pip3 install numpy opencv-python adafruit-circuitpython-bno055 gpiozero smbus2 pyserial
```
4. Wire the Hardware

Follow the pinout summary above to connect sensors, motors, IMU and camera. A wiring diagram is contained in the technical document.
Ensure power supply is stable (typically 5 V logic + 12 V drive) and all grounds are common.

5. Calibrate Sensors

Run the IMU calibration routine if provided.

Verify sonar readings in a safe environment.

6. Run the System
python3 main.py --mode autonomous   # Autonomous mode
python3 main.py --mode manual       # Manual mode


License

This project is released under the MIT License. See the LICENSE file for full details.

Acknowledgements

Thank you to the Northeastern London Robotics Team, competition organizers of UniBots UK, and open-source libraries and communities that support robotics development.



