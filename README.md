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
