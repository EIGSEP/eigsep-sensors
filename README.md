# EIGSEP Sensors

[![codecov](https://codecov.io/github/EIGSEP/eigsep-sensors/graph/badge.svg?token=WV88G15IG9)](https://codecov.io/github/EIGSEP/eigsep-sensors)

Code for sensors used: accelerometer, lidar, thermistors, peltiers.

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

