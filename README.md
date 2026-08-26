# Autonomous Robot Vehicle Control & Navigation System

## Overview
This repository contains the software architecture, computer vision routines, sensor drivers, and hardware interface scripts for an autonomous robot car chassis powered by a **Raspberry Pi 5** single-board computer. 

The vehicle executes high-level computer vision routines, spatial array calculations, low-latency sensor acquisition, and direct hardware PWM actuation entirely on the Raspberry Pi 5. Spatial awareness and object detection rely on an integrated sensor array: a forward-facing **Raspberry Pi Camera Module** for optical road parsing, an **Inertial Measurement Unit (IMU)** for orientation and heading tracking, and **six HC-SR04 Ultrasonic Sensors** arranged around the vehicle perimeter for 360-degree proximity monitoring.

The system software leverages **OpenCV** (`cv2`) for image processing, **NumPy** (`numpy`) for high-performance spatial vector calculations, Python standard **threading** for concurrent non-blocking sensor acquisition, and **gpiozero** with the **LGPIOFactory** pin driver backend (`gpiozero.pins.lgpio`) to manage low-level GPIO pin interfaces (`DigitalOutputDevice`, `DigitalInputDevice`, `Servo`, `Button`).

Physical actuation consists of a mid-mounted **DC drive motor** controlled via pulse-width modulation for rear-wheel propulsion and a **steering servo** driving front tie-rod Ackermann steering geometry. Power is supplied by high-discharge **LiPo batteries** regulated down to logic levels, with physical **push-buttons** providing hardware state control and system shutdown. Vibration isolation and component mounting stability are achieved using adhesive **silicone pads**.

---

## System Architecture & Electromechanical Integration

The software solution centralizes execution on the Raspberry Pi 5 running Linux. High-frequency sensor polling (such as ultrasonic echo timing and optical frame acquisition) is offloaded into parallel Python execution threads using the `threading` library to prevent execution latency in the primary driving state machine.
```
+-----------------------------------------------------------------------------------+
|                                 Raspberry Pi 5                                    |
|                                                                                   |
|  [RPi Camera] ---------> cv2 Frame Capture Thread                                 |
|                                 │                                                 |
|  [6x HC-SR04] ---------> DigitalInput/Output Threads ──> numpy Matrix Calculation |
|                                 │                           │                     |
|  [I2C IMU] ------------> smbus2 Orientation Readout         │                     |
|                                                             ▼                     |
|  [2x Buttons] --------─> Button Interrupt Event ────> Servo & Motor Controllers   |
|                          (LGPIOFactory Backend)      (gpiozero PWM Outputs)       |
+-------------------------------------------------------------│---------------------+
                                                              │
                                      ┌───────────────────────┴───────────────────────┐
                                      ▼                                               ▼
                              [DC Drive Motor]                                 [Steering Servo]
                              (Rear Axle Drive)                                (Front Steering)
```
### Module Breakdown & Electromechanical Mapping

#### 1. Core Software Stack
* **`cv2` (OpenCV):** Handles real-time video frame capture from the Raspberry Pi Camera Module, applying color space transformations (HSV), edge detection algorithms (Canny), and contour extraction to detect lane boundaries and visual markers.
* **`numpy`:** Executes high-speed matrix calculations and polynomial curve fitting on extracted pixel coordinates, computing spatial trajectory offsets and velocity vectors.
* **`gpiozero` (`DigitalOutputDevice`, `DigitalInputDevice`, `Servo`, `Button`):** Provides object-oriented Python abstractions for interfacing with hardware GPIO pins.
* **`LGPIOFactory` (`gpiozero.pins.lgpio`):** Specifies the native Linux character device GPIO driver (`lgpio`) as the low-level pin factory backend required for high-precision hardware pulse management on the Raspberry Pi 5.
* **`threading`:** Spawns concurrent background threads for non-blocking distance sampling across all six ultrasonic sensors and real-time camera stream polling.
* **`time`:** Provides precise microsecond and millisecond timing routines for ultrasonic pulse width calculations and loop frequency throttling.

#### 2. Electromechanical Hardware Mapping

* **Raspberry Pi Camera Module:**
  * **Role:** Captures forward-facing visual data for `cv2` line detection and path tracking.
  * **Connection:** Connected via high-speed ribbon cable directly to the Pi 5 CSI/DSI port.
* **6x HC-SR04 Ultrasonic Sensors:**
  * **Role:** Continuous 360-degree obstacle detection (Front Left, Front Center, Front Right, Rear Left, Rear Center, Rear Right).
  * **Software Interface:** Driven using `DigitalOutputDevice` for the `TRIG` pins and `DigitalInputDevice` for the `ECHO` pins within dedicated `threading` worker loops.
  * **Wiring & Voltage Matching:** `TRIG` pins connect directly to Pi GPIO output pins. Each 5V `ECHO` pin output routes through a resistor voltage divider (1 kΩ and 2 kΩ) built on the breadboard to drop the voltage to 3.3V logic before connecting to Pi GPIO input pins.
* **IMU (Inertial Measurement Unit):**
  * **Role:** Tracks pitch, roll, and yaw (heading) to keep the vehicle driving straight and correct for mechanical drift or wheel slip.
  * **Connection:** Wired directly via DuPont jumper cables to **GPIO 2 (SDA)** and **GPIO 3 (SCL)** over the hardware I2C bus.
* **1x DC Motor (Propulsion):**
  * **Role:** Drives the rear solid axle via a gear-and-belt drive loop.
  * **Software Interface:** Controlled via `DigitalOutputDevice` / PWM output pins connected to an external DC motor driver board powered by the LiPo battery.
* **1x Steering Servo:**
  * **Role:** Adjusts front steering linkages using Ackermann steering geometry.
  * **Software Interface:** Controlled via the `Servo` class from `gpiozero` instantiated with specific min/max pulse width parameters.
  * **Connection:** Signal line connects to a hardware PWM pin on the Pi. Power (5V) and Ground connect to an external regulated power rail.
* **2x Push Buttons:**
  * **Role:** User interface buttons. **Button 1** arms the system and initiates the navigation loop. **Button 2** serves as an immediate emergency stop and software termination trigger.
  * **Software Interface:** Managed using the `Button` class from `gpiozero` with non-blocking event callbacks (`when_pressed`).
  * **Connection:** Wired directly between Pi GPIO pins and Ground on the breadboard using internal pull-up resistors.
* **LiPo Batteries:**
  * **Role:** Supplies high-discharge power to the DC motor driver and steering servo, with a voltage regulator supplying clean 5V power to the Raspberry Pi 5.
* **Breadboard & Female-Male DuPont Wires:**
  * **Role:** Provides common Ground and 5V power rails, houses the 6x voltage divider circuits for the ultrasonic echo lines, and routes signal lines between components.
* **4x Silicone Pads:**
  * **Role:** Installed underneath the chassis baseplate, Raspberry Pi 5, and IMU board to dampen mechanical vibrations caused by the drive motor and terrain.

---

## Detailed Component Interface Specifications

### 1. Multi-Threaded Ultrasonic Sensor Array (`gpiozero` + `threading`)
Each HC-SR04 distance calculation is offloaded into a non-blocking worker thread. A 10µs pulse is emitted using `DigitalOutputDevice.on()` and `off()`, while `DigitalInputDevice` registers the echo start and end timestamps using `time.time_ns()`:

$$\text{Distance (cm)} = \frac{(\text{Echo End Time} - \text{Echo Start Time}) \times 34300}{2}$$

The resulting distance values are continuously pushed into a shared `numpy` array, allowing the main navigation loop to read instantaneous 360-degree proximity data without blocking execution.

### 2. Servo & Motor PWM Calibration (`LGPIOFactory`)
To ensure jitter-free servo operation on Raspberry Pi 5 hardware, the pin factory is set to `LGPIOFactory` at runtime:

```python
from gpiozero import Servo, Device
from gpiozero.pins.lgpio import LGPIOFactory

# Set the pin factory backend for Raspberry Pi 5
Device.pin_factory = LGPIOFactory()

# Initialize steering servo with custom pulse bounds
steering_servo = Servo(
    pin=13, 
    initial_value=0, 
    min_pulse_width=1/1000, 
    max_pulse_width=2/1000
)
