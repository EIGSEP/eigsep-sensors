import serial
import time
import contextlib

@contextlib.contextmanager
def open_port(port=None, baud=115200):
    if port is None:
        import glob
        ports = glob.glob("/dev/cu.usbmodem*")
        if not ports:
            raise RuntimeError("no Pico found...")    
        port = ports[0]

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)

    try:
        yield ser
    finally:
        ser.close()

def send_cmd(ser, text):
    ser.write((text + "\n").encode())
    return ser.readline().decode(errors="replace").strip()