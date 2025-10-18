# NURobotics-London-UniBots-Robot
Code for NuLondon Robotics Team for Cambridge Uni's Unibots Competition.

# NURobotics-London-Robot

## Project Overview  
This repository contains the competition robot developed by the **Northeastern University – London Robotics Team** for the **UniBots Challenge**.  
It features a Raspberry Pi 4B controller with modular Python drivers for motors, servos, sonar sensors, and an IMU—enabling both manual and autonomous control modes.  

The robot is designed as a flexible base for research, education, and competition, emphasizing modularity and rapid iteration. View log, and instructions to duplicate here:
https://docs.google.com/document/d/1x95HRRY2IjaurOZZFUa4TBfEYou3pGcw90neMIvEJIw/edit#heading=h.e5y70zrwz3jn

---

## Objectives  
- Develop a **reusable robotics platform** combining Python-based software and simple hardware interfaces.  
- Implement **autonomous navigation** through sonar and IMU feedback loops.  
- Provide an extendable **camera vision pipeline** for future object-tracking and target-detection modules.  
- Maintain **manual override control** via remote controller or SSH interface for testing.

---

## Hardware Architecture  

| Component        | Description                                  | Interface              |
|------------------|----------------------------------------------|------------------------|
| **Raspberry Pi 4B** | Main compute and control board               | USB / GPIO             |
| **Motor drivers**   | Dual-motor and servo motor HATs             | GPIO / PWM             |
| **Ultrasonic sonar** | Obstacle detection (front & back)           | GPIO (trigger/echo)    |
| **IMU sensor**      | Orientation feedback (yaw, pitch, roll)     | I²C                    |
| **Camera module**   | Front-facing for image processing tasks     | CSI / USB              |
| **Chassis & wheels**| Mobile base platform with two drive motors  | —                      |

---

## Software Architecture  

- **Language:** Python 3.x  
- **Dependencies:** `numpy`, `opencv-python`, `RPi.GPIO`, `smbus2`, `time`, `math`  
- **Modules:**
  - `motors.py` — control for DC/servo motors  
  - `sonar.py` — distance measurement via ultrasonic sensors  
  - `imu.py` — orientation initialization and filtering  
  - `camera.py` — frame capture and object detection routines  
  - `autonomy.py` — decision-making loop (sensor fusion + navigation)  
  - `manual_control.py` — teleoperation and override functions  
  - `main.py` — orchestrator, initializes all subsystems and runs main control loop  

### Control Loop (Simplified Pseudocode)
```python
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
    log.update(status)


