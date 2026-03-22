import serial
import struct
import json
import time

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03F056-if00-port0"
BAUD = 115200

# Sensor packets we want streamed
# (choose ones useful for mapping + odometry)
SENSOR_PACKETS = [
    7,   # Bumps and wheel drops
    19,  # Distance
    20,  # Angle
    21,  # Charging state
    22,  # Voltage
    23,  # Current
    24,  # Temperature
    25,  # Battery charge
    26   # Battery capacity
]

# Packet sizes from OI spec
PACKET_SIZES = {
    7: 1,
    19: 2,
    20: 2,
    21: 1,
    22: 2,
    23: 2,
    24: 1,
    25: 2,
    26: 2
}


def decode_packet(packet_id, data):

    if packet_id == 7:
        bumps = data[0]
        return {
            "bump_right": bool(bumps & 0x01),
            "bump_left": bool(bumps & 0x02),
            "wheel_drop_right": bool(bumps & 0x04),
            "wheel_drop_left": bool(bumps & 0x08),
        }

    elif packet_id == 19:
        return {"distance_mm": struct.unpack(">h", data)[0]}

    elif packet_id == 20:
        return {"angle_deg": struct.unpack(">h", data)[0]}

    elif packet_id == 21:
        return {"charging_state": data[0]}

    elif packet_id == 22:
        return {"voltage_mv": struct.unpack(">H", data)[0]}

    elif packet_id == 23:
        return {"current_ma": struct.unpack(">h", data)[0]}

    elif packet_id == 24:
        return {"temperature_c": struct.unpack("b", data)[0]}

    elif packet_id == 25:
        return {"battery_charge_mah": struct.unpack(">H", data)[0]}

    elif packet_id == 26:
        return {"battery_capacity_mah": struct.unpack(">H", data)[0]}

    return {"raw": list(data)}


def start_stream(ser):

    ser.write(bytes([128]))  # START
    time.sleep(0.1)

    ser.write(bytes([131]))  # SAFE MODE
    time.sleep(0.1)

    cmd = [148, len(SENSOR_PACKETS)] + SENSOR_PACKETS
    ser.write(bytes(cmd))


def read_stream(ser):

    while True:

        header = ser.read(1)

        if not header:
            continue

        if header[0] != 19:  # stream header
            continue

        length = ser.read(1)[0]
        payload = ser.read(length)
        checksum = ser.read(1)[0]

        frame = bytes([19, length]) + payload + bytes([checksum])

        if (sum(frame) & 0xFF) != 0:
            print("checksum error")
            continue

        i = 0
        parsed = {}

        while i < len(payload):

            packet_id = payload[i]
            i += 1

            size = PACKET_SIZES.get(packet_id)

            if size is None:
                print("unknown packet", packet_id)
                break

            data = payload[i:i+size]
            i += size

            parsed[str(packet_id)] = decode_packet(packet_id, data)

        parsed["timestamp"] = time.time()

        print(json.dumps(parsed))


def main():

    with serial.Serial(PORT, BAUD, timeout=1) as ser:

        print("Connected")
        start_stream(ser)

        print("Streaming...")
        read_stream(ser)


if __name__ == "__main__":
    main()