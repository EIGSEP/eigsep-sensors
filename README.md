# EIGSEP Sensors

[![codecov](https://codecov.io/github/EIGSEP/eigsep-sensors/graph/badge.svg?token=WV88G15IG9)](https://codecov.io/github/EIGSEP/eigsep-sensors)

Code for sensors used: accelerometer, lidar, thermistors, peltiers.

## Quick Start

**For Python Development:**
```bash
git clone https://github.com/EIGSEP/eigsep-sensors.git
cd eigsep-sensors
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
python -m pytest tests/  # Run tests
```

**For Pico C Firmware:**
```bash
cd peltier_pico/
export PICO_SDK_PATH=/path/to/pico-sdk
chmod +x build_it.sh && ./build_it.sh
# Flash generated build/pico_T_ctrl.uf2 to Pico
```

**For Temperature Control:**
```python
from eigsep_sensors.peltier.temperature_controller import TemperatureController
controller = TemperatureController('/dev/ttyACM0')
data = controller.read_temperature()  # Returns 7 values: timestamp + 2 peltier readings
```

**Installation:**

Clone the repository and run `pip install .` in the root of the directory. This automatically installs all required dependencies. For developing, do `pip install .[dev]` instead to get additional dependencies related to testing of code.

**Set-up:**

Connect a Raspberry Pi Pico wired to the sensor used. Copy the script to run on the Pico (usually called "main.py") to the Pico root. If the script imports anything from this module, you must copy the whole package and the script. Below we show an example to set up the thermistor. The script to run on the Pico is scripts/thermistor/main.py.

From a terminal, copy the package and the script to the Pico root:
```shell
mpremote devs  # list the devices to see where the Pico is connected
mpremote connect /dev/ttyACM0 fs cp -r src/eigsep_sensors :/eigsep_sensors  # replace /dev/ttyACM0 with the output of the previous command
mpremote connect /dev/ttyACM0 fs cp -r scripts/thermistor/main.py :/main.py  # script to run
mpremote connect /dev/ttyACM0 reset   # rests the pico, so main.py stars automatically
```

Now main.py is running on the Pico and will start automatically on boot.

# Peltier Pico Setup - Linux
 This guide walks through setting up a host environment to build and flash the peltier control loop ".uf2" firmware files for the Raspberry Pi Pico 2 using the C software dev. kit (SDK). 

 **Dependencies:**
 Install cmake, make, gcc-arm-non-eabi, and build-essential. 
```bash
sudo apt update
sudo apt install cmake make gcc-arm-non-eabi build-essential git
```

**Clone the project repository:**
```bash
git clone https://github.com/EIGSEP/eigsep-sensors.git
```

**SDK Setup:**
There are two options here:
1. Clone pico SDK locally [github.com/pico-sdk](https://github.com/raspberrypi/pico-sdk) -- we'll opt for option 1.
2. Auto-fetch SDK (this is enabled in pico_sdk_import.cmake) -- not tested.

```bash
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init # pico-sdk includes other GitHub repos (submodules). 
cd ..
export PICO_SDK_PATH=YOUR_PATH_TO_PICO_SDK/pico-sdk
```

**Build peltier_pico:**

Inside peltier_pico/, compile the shell script "build_it.sh". This does three things:
1. creates a new directory called "build" (if it doesn't already exist, which it shouldn't yet!)
2. moves into said directory
3. runs CMake, telling it to configure the build for the Pico 2 (rp2350 is the chip used in the Raspberry Pi Pico 2). CMake will look for the CMakeLists.txt in the parent directory (peltier_pico/), if the CMake step is successful then -- runs "make" to actually build/compile the project.

```bash
chmod +x build_it.sh
./build_it.sh
```

NOTE: any changes to the .c or header (.h) scripts will requires recompilation in the build directory using "make". If successful, this will create a new .uf2 which can be flashed to the Pico.
```bash
cd build/
make
```

**Flash firmware to Pico:**
1. Hold the BOOTSEL button on the Pico while connecting it to USB
2. The Pico will appear as a USB drive (RPI-RP2)
3. Copy the generated `pico_T_ctrl.uf2` file to the Pico drive
4. The Pico will automatically reboot and start running the firmware

## Python Interface Usage

### Development Setup

Create and activate a virtual environment, then install dependencies:

```bash
# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install package with development dependencies
pip install -e ".[dev]"
```

### Running Tests

```bash
# Activate virtual environment
source .venv/bin/activate

# Run all tests
python -m pytest tests/

# Run specific test file
python -m pytest tests/test_temperature_controller.py -v

# Run tests with coverage
python -m pytest tests/ --cov=src --cov-report=html
```

### Code Formatting and Linting

```bash
# Format code
python -m black src/

# Check linting
python -m flake8 src/

# Check formatting without making changes
python -m black --check src/
```

## Temperature Controller Usage

### Dual Peltier Control System
The firmware now supports independent control of two Peltier elements with separate DS18B20 temperature sensors.

**Hardware Requirements:**
- Raspberry Pi Pico 2
- 2x DS18B20 temperature sensors (1-Wire)
- H-bridge driver (e.g., XY-160D) 
- 2x Peltier elements
- Pull-up resistor (4.7kΩ) on 1-Wire data line

**Wiring:**
- DS18B20 data line: GPIO 22 (with 4.7kΩ pull-up to 3.3V)
- Peltier 1 PWM: GPIO 16
- Peltier 1 Direction: GPIO 18, 19
- Peltier 2 PWM: GPIO 15  
- Peltier 2 Direction: GPIO 13, 12

### Python Temperature Controller

```python
from eigsep_sensors.peltier.temperature_controller import TemperatureController

# Connect to Pico
controller = TemperatureController('/dev/ttyACM0', baudrate=115200)

# Read single temperature measurement (returns 7 values)
timestamp, t1_now, t1_target, t1_drive, t2_now, t2_target, t2_drive = controller.read_temperature()

print(f"Peltier 1: {t1_now:.1f}°C -> {t1_target:.1f}°C (drive: {t1_drive:.2f})")
print(f"Peltier 2: {t2_now:.1f}°C -> {t2_target:.1f}°C (drive: {t2_drive:.2f})")

# Start/stop control loops
controller.start_control_loop()
controller.stop_control_loop()

# Continuous monitoring with callback
def data_callback(reading):
    timestamp, t1_now, t1_target, t1_drive, t2_now, t2_target, t2_drive = reading
    print(f"Time: {timestamp}, P1: {t1_now:.1f}°C, P2: {t2_now:.1f}°C")

controller.start_monitoring(interval=1.0, callback=data_callback)
# ... let it run ...
controller.stop_monitoring()

# Clean up
controller.close()
```

### Serial Protocol

The firmware communicates via USB serial at 115200 baud. Commands:

- `REQ` - Request temperature reading (returns 3 lines)
- `RESUME` - Start temperature control loops  
- `STOP` - Stop temperature control loops
- `END` - Stop data logging

**Response format for `REQ`:**
```
12345,
25.5,30.0,0.75,
32.0,35.0,0.5
```
Where:
- Line 1: Unix timestamp
- Line 2: Peltier 1 (current temp, target temp, drive level)
- Line 3: Peltier 2 (current temp, target temp, drive level)

### Configuration

Edit `main.c` to change default settings:

```c
// Peltier-1 parameters  
float T_target=30.0;    // °C, front-end target
float t_target=10.0;    // s
float gain=0.2;         // max allowed drive

// Peltier-2 parameters
float T_target2=32.0;   // °C, noise source target
```

### Data Logging Scripts

Use the provided Python scripts for data collection:

```bash
# Log temperature data to CSV
python scripts/peltier/host_logger.py

# Interactive Pico control
python scripts/peltier/picoctl.py

# Simple serial interface
python scripts/peltier/picoser.py
```

## Other Sensor Modules

### IMU (BNO08x/MMA8451)
```python
from eigsep_sensors.imu import IMU

# Initialize IMU
imu = IMU()
accel_data = imu.get_acceleration()
```

### Thermistor
```python
from eigsep_sensors.thermistor import Thermistor

thermistor = Thermistor()
temperature = thermistor.read_temperature()
```

## Troubleshooting

**Common Issues:**

1. **Build fails:** Ensure PICO_SDK_PATH is set correctly
2. **Serial connection fails:** Check device permissions (`sudo usermod -a -G dialout $USER`)
3. **Tests fail:** Ensure virtual environment is activated and dependencies installed
4. **Firmware doesn't respond:** Check baud rate (115200) and try pressing reset button

**Development Tips:**

- Use `mpremote` for Pico filesystem operations
- Monitor serial output: `screen /dev/ttyACM0 115200` or `picocom -b 115200 /dev/ttyACM0`
- Check Pico status LED for error indications

