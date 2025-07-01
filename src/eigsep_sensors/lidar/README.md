# LIDAR Sensor Module

This module provides Python interfaces for LIDAR sensors used in the EIGSEP project.

## Supported Sensors

### TF-Luna LIDAR
- **Range**: 0.2m - 8m
- **Interface**: I2C
- **Default Address**: 0x10
- **Features**: Distance, signal strength, temperature

### GRF-250 LIDAR  
- **Range**: 0.05m - 250m
- **Interface**: I2C
- **Default Address**: 0x66
- **Features**: Distance, signal strength, temperature

## Quick Start

### Basic Usage
```python
from eigsep_sensors.lidar import Lidar

# Connect to LIDAR via serial port
lidar = Lidar('/dev/ttyACM0', timeout=2)

# Read sensor data
data = lidar.read()
if data:
    print(f"Distance: {data['distance']} m")
    print(f"Signal Strength: {data['strength']}")
    print(f"Temperature: {data['temperature']:.1f} °C")
```

### Continuous Monitoring
```python
import time
from eigsep_sensors.lidar import Lidar

lidar = Lidar('/dev/ttyACM0')

try:
    while True:
        data = lidar.read()
        if data:
            print(f"{data['unix_time']:.1f}: {data['distance']:.2f}m, {data['strength']}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Stopping...")
```

## Hardware Setup

### Raspberry Pi Pico Scripts

For hardware integration, use the provided Pico scripts:

- **TF-Luna**: `scripts/lidar/tfluna_main.py`
- **GRF-250**: `scripts/lidar/grf250_main.py`

### Wiring

#### TF-Luna (I2C)
```
TF-Luna    Pico
VCC   -->  3.3V
GND   -->  GND  
SDA   -->  GP0
SCL   -->  GP1
```

#### GRF-250 (I2C)
```
GRF-250    Pico
VCC   -->  3.3V or 5V
GND   -->  GND
SDA   -->  GP0  
SCL   -->  GP1
```

### Flashing Pico Scripts

1. Copy the appropriate script to your Pico as `main.py`
2. The script will automatically start on boot
3. Connect via serial at 115200 baud
4. Send "REQ" commands to get readings

```bash
# Copy TF-Luna script to Pico
mpremote connect /dev/ttyACM0 fs cp scripts/lidar/tfluna_main.py :main.py

# Or copy GRF-250 script
mpremote connect /dev/ttyACM0 fs cp scripts/lidar/grf250_main.py :main.py

# Reset Pico
mpremote connect /dev/ttyACM0 reset
```

## Data Format

All LIDAR sensors return data in this format:

```python
{
    'distance': 1.23,        # Distance in meters
    'strength': 856,         # Signal strength (sensor-specific units)
    'temperature': 24.5,     # Temperature in Celsius
    'unix_time': 1625097600  # Timestamp when reading was taken
}
```

### Serial Protocol

The Pico scripts respond to these commands:

- **REQ**: Request a single reading
- Returns: `"distance,strength,temperature"` (CSV format)

Example:
```
> REQ
< 1.23,856,24.5
```

## Troubleshooting

### Common Issues

1. **No response from sensor**
   - Check I2C wiring and addresses
   - Verify 3.3V/5V power requirements
   - Use `i2c.scan()` on Pico to detect sensors

2. **Serial connection fails**
   - Check baud rate (115200)
   - Verify correct serial port
   - Try resetting the Pico

3. **Inconsistent readings**
   - Check power supply stability
   - Verify I2C pull-up resistors (usually built-in on dev boards)
   - Ensure sensor is within operating range

### Testing I2C Connection

Add this to your Pico script for debugging:
```python
from machine import I2C, Pin

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
print("I2C devices found:", [hex(addr) for addr in i2c.scan()])
```

## API Reference

### Lidar Class

#### `__init__(port, timeout=1)`
Initialize LIDAR connection.

**Parameters:**
- `port` (str): Serial port (e.g., '/dev/ttyACM0')
- `timeout` (float): Read timeout in seconds

#### `read()`
Request and read sensor data.

**Returns:**
- `dict`: Sensor data with distance, strength, temperature, and timestamp
- `None`: If reading failed

#### `_request_data()`
Send data request to sensor. Override in subclasses for different protocols.

#### `_parse_line(line)`
Parse CSV response from Pico into dictionary format.

## Extending the Module

### Adding New LIDAR Sensors

1. Create a new Pico script following the pattern:
   ```python
   while True:
       line = sys.stdin.readline().strip()
       if line == "REQ":
           # Read your sensor
           distance, strength, temp = read_your_sensor()
           print(f"{distance},{strength},{temp}")
   ```

2. The base `Lidar` class will automatically work with your sensor if it follows the CSV response format.

3. For different data formats, override `_parse_line()`:
   ```python
   class CustomLidar(Lidar):
       def _parse_line(self, line):
           # Custom parsing logic
           return {"distance": x, "strength": y, "temperature": z}
   ```

## Error Handling

The module includes basic error handling:

- Parse errors are caught and logged
- Failed readings return `None`
- Serial timeouts are handled gracefully

For production use, consider adding:
- Retry logic for failed reads
- Connection health monitoring
- Data validation and filtering