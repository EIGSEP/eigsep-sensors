import sys
from machine import I2C, Pin
import time
import struct

# === GRF-250 I2C Configuration ===
GRF250_ADDR = 0x66
CMD_ID_READ = 44        # Distance data in cm
CMD_ID_CONFIG = 27      # Distance output config
CONFIG_BITS = 0b01000101  # Enable bits 0, 2, 6 → 69

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100_000)

# === CRC16 checksum algorithm from manual ===
def create_crc(data):
    crc = 0
    for byte in data:
        code = crc >> 8
        code ^= byte
        code ^= code >> 4
        crc = (crc << 8) & 0xFFFF
        crc ^= code
        code = (code << 5) & 0xFFFF
        crc ^= code
        code = (code << 2) & 0xFFFF
        crc ^= code
    return crc

# === Build read packet ===
def build_read_packet(cmd_id):
    flags = 0x0002  # 1-byte payload, read = 0
    return build_packet(cmd_id, flags, is_write=False)

# === Build write packet with 4-byte payload ===
def build_write_packet(cmd_id, value):
    flags = (4 << 1) | 1  # 4 bytes, write = 1
    value_bytes = struct.pack("<I", value)
    return build_packet(cmd_id, flags, is_write=True, data=value_bytes)

# === Generic packet builder ===
def build_packet(cmd_id, flags, is_write, data=b''):
    start = 0xAA
    flags_high = (flags >> 8) & 0xFF
    flags_low = flags & 0xFF
    packet = bytearray([start, flags_high, flags_low, cmd_id]) + data
    crc = create_crc(packet)
    packet += bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
    return packet

# === Send Command 27 to enable distance + strength + temp ===
def configure_output():
    pkt = build_write_packet(CMD_ID_CONFIG, CONFIG_BITS)
    try:
        i2c.writeto(GRF250_ADDR, pkt)
        time.sleep(0.05)  # give the device time to apply
        print("Configured GRF-250 output: distance + strength + temperature")
    except Exception as e:
        print("Failed to configure output format:", e)

# === Read distance + strength + temp ===
def read_distance_plus():
    try:
        pkt = build_read_packet(CMD_ID_READ)
        i2c.writeto(GRF250_ADDR, pkt)
        time.sleep(0.01)

        resp = i2c.readfrom(GRF250_ADDR, 18)
        if resp[0] != 0xAA or resp[3] != CMD_ID_READ:
            print("Invalid header or command ID in response")
            return None

        data_wo_crc = resp[:-2]
        crc_recv = resp[-2] + (resp[-1] << 8)
        if create_crc(data_wo_crc) != crc_recv:
            print("CRC mismatch")
            return None

        distance_raw = struct.unpack("<i", resp[4:8])[0]
        strength = struct.unpack("<i", resp[8:12])[0]
        temp_raw = struct.unpack("<i", resp[12:16])[0]

        distance_cm = distance_raw / 10.0
        temperature_c = temp_raw / 100.0

        return distance_cm, strength, temperature_c

    except Exception as e:
        print("Read error:", e)
        return None

# === Main ===
configure_output()  # one-time setup on boot

while True:
    line = sys.stdin.readline().strip()
    if line == "REQ":
        result = read_distance_plus()
        if result:
            d, s, t = result
            print(f"{d:.2f},{s},{t:.2f}")
        time.sleep(0.1)
