import sys
from machine import I2C, Pin
import time
import struct

# GRF-250 I2C Configuration Addresses
GRF250_ADDR = 0x66
CMD_ID_READ = 44
CMD_ID_CONFIG = 27
CMD_ID_TOKEN = 10
CMD_ID_SAVE_PARAMS = 12
CMD_ID_PRODUCT_NAME = 0
CMD_ID_UPDATE_RATE = 74

CONFIG_BITS = 0b01000101  # Enable bits 0, 2, 6 → distance, strength, temp
UPDATE_HZ = 20

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100_000)

# CRC16 checksum algorithm from manual
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

def build_packet(cmd_id, is_write, data=b''):
    start = 0xAA
    flags = (len(data) << 1) | (1 if is_write else 0)
    flags_high = (flags >> 8) & 0xFF
    flags_low = flags & 0xFF
    packet = bytearray([start, flags_low, flags_high, cmd_id]) + data
    crc = create_crc(packet)
    crc_high = (crc >> 8) & 0xFF
    crc_low = crc & 0xFF
    packet += bytearray([crc_low, crc_high])
    return packet

def build_read_packet(cmd_id):
    return build_packet(cmd_id, is_write=False)

def build_write_packet(cmd_id, value):
    value_bytes = struct.pack("<I", value)
    return build_packet(cmd_id, is_write=True, data=value_bytes)

# Do a dummy read to initialize the I2C interface
def dummy_handshake():
    try:
        pkt = build_read_packet(CMD_ID_PRODUCT_NAME)
        i2c.writeto(GRF250_ADDR, pkt)
        time.sleep(0.05)
        pkt = build_read_packet(CMD_ID_PRODUCT_NAME)
        i2c.writeto(GRF250_ADDR, pkt)
        time.sleep(0.05)
        resp = i2c.readfrom(GRF250_ADDR, 22)
        if resp[0] == 0xAA:
            name = resp[4:-2].decode('ascii', 'ignore').strip('\x00')
            print("GRF-250 connected:", name)
    except Exception as e:
        print("Handshake failed:", e)

# Configure output + update rate and persist
def configure_and_save():
    try:
        # 1. Configure output format
        pkt = build_write_packet(CMD_ID_CONFIG, CONFIG_BITS)
        i2c.writeto(GRF250_ADDR, pkt)
        time.sleep(0.05)
        print("Configured: distance + strength + temperature")

        # 2. Set update rate
        pkt = build_write_packet(CMD_ID_UPDATE_RATE, UPDATE_HZ)
        i2c.writeto(GRF250_ADDR, pkt)
        time.sleep(0.05)
        print("Set update rate to", UPDATE_HZ, "Hz")

        # 3. Get safety token
        pkt = build_read_packet(CMD_ID_TOKEN)
        i2c.writeto(GRF250_ADDR, pkt)
        time.sleep(0.05)
        resp = i2c.readfrom(GRF250_ADDR, 8)
        token = struct.unpack("<H", resp[4:6])[0]
        print("Token:", token)

        # 4. Save to flash
        pkt = build_write_packet(CMD_ID_SAVE_PARAMS, token)
        i2c.writeto(GRF250_ADDR, pkt)
        print("Saved configuration to flash")

    except Exception as e:
        print("Configuration failed:", e)

# Read distance + strength + temp
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

dummy_handshake()
configure_and_save()

while True:
    line = sys.stdin.readline().strip()
    if line == "REQ":
        result = read_distance_plus()
        if result:
            d, s, t = result
            print(f"{d:.2f},{s},{t:.2f}")
        time.sleep(0.1)  # 10 Hz loop
