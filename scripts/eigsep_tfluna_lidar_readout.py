import time
import eigsep_sensors as eig


lidar = eig.TFLuna()

try:
    while True:
        distance, strength, temperature = lidar.read_all()
        if distance is not None:
            print(f"Distance: {distance} cm | Strength: {strength} | Temp: {temperature:.2f} °C")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping.")
finally:
    lidar.close()
