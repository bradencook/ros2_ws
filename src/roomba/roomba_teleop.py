import serial
import sys
import termios
import tty
import select
import time
import struct

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03F056-if00-port0"
BAUD = 115200
SPEED = 200

ser = serial.Serial(PORT, BAUD, timeout=0.1)


# ---------- Roomba Commands ----------

def send(cmd):
    ser.write(bytes(cmd))

def start():
    send([128])
    time.sleep(0.1)
    send([132])
    time.sleep(0.1)

def drive(left, right):
    send([
        145,
        (right >> 8) & 0xFF,
        right & 0xFF,
        (left >> 8) & 0xFF,
        left & 0xFF
    ])

def stop():
    drive(0,0)

# ---------- Sensor Reading ----------

def read_sensors():
    try:
        # Request sensor packet group 0 (basic sensors)
        send([142, 0])

        data = ser.read(26)

        if len(data) != 26:
            return None

        bumps = data[0]
        battery_voltage = struct.unpack(">H", data[17:19])[0] / 1000

        bump_right = bumps & 0x01
        bump_left = (bumps >> 1) & 0x01

        return {
            "voltage": battery_voltage,
            "bump_left": bump_left,
            "bump_right": bump_right
        }

    except:
        return None

# ---------- Keyboard Handling ----------

def get_key():
    dr,_,_ = select.select([sys.stdin],[],[],0)
    if dr:
        return sys.stdin.read(1)
    return None

def setup_terminal():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old

def restore_terminal(old):
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)

# ---------- Main ----------

def main():

    old_settings = setup_terminal()

    print("\nRoomba Teleop\n")
    print("Arrow keys = drive")
    print("space = stop")
    print("q = quit\n")

    start()

    last_sensor_time = 0

    try:
        while True:
            key = get_key()

            # --- Arrow keys ---
            if key == '\x1b':
                key += sys.stdin.read(2)

                if key == '\x1b[A':  # up
                    drive(SPEED, SPEED)

                elif key == '\x1b[B':  # down
                    drive(-SPEED, -SPEED)

                elif key == '\x1b[C':  # right
                    drive(SPEED, -SPEED)

                elif key == '\x1b[D':  # left
                    drive(-SPEED, SPEED)

            # --- WASD controls ---
            elif key in ['w', 'W']:
                drive(SPEED, SPEED)

            elif key in ['s', 'S']:
                drive(-SPEED, -SPEED)

            elif key in ['a', 'A']:
                drive(-SPEED, SPEED)

            elif key in ['d', 'D']:
                drive(SPEED, -SPEED)

            elif key == ' ':
                stop()

            elif key == 'q':
                break

            # Poll sensors at 5Hz
            if time.time() - last_sensor_time > 0.2:

                sensors = read_sensors()

                if sensors:
                    print(
                        f"\rVoltage: {sensors['voltage']:.2f}V | "
                        f"BumpL: {sensors['bump_left']} | "
                        f"BumpR: {sensors['bump_right']}      ",
                        end=""
                    )

                last_sensor_time = time.time()

    finally:
        stop()
        send([128])
        restore_terminal(old_settings)
        ser.close()
        print("\nStopped")

if __name__ == "__main__":
    send([150, 0])
    ser.reset_input_buffer()
    main()
