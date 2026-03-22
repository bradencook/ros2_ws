import serial
import struct
import json
import time

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03F056-if00-port0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

def close():
    start()
    ser.close()


def clear():
    ser.reset_input_buffer()


def send(cmd):
    """
    Encapsulates sending a command through the serial interface
    """
    ser.write(bytes(cmd))


def startup(full_mode=False):
    """
    Boot into safe mode unless a truthy value is given as an argument then boots to full mode
    """
    start()
    time.sleep(0.1) 
    full() if full_mode else safe()
    time.sleep(0.1)


def start():
    """
    This command starts the OI. You must always send the Start command before sending any other
    commands to the OI.
    - Serial sequence: [128].
    - Available in modes: Passive, Safe, or Full
    - Changes mode to: Passive. Roomba beeps once to acknowledge it is starting from “off” mode.
    """
    send([128])


def reset():
    """
    This command resets the robot, as if you had removed and reinserted the battery.
    - Serial sequence: [7].
    - Available in modes: Always available.
    - Changes mode to: Off. You will have to send start() again to re-enter Open Interface mode.
    """
    send([7])


def stop_OI():
    """
    This command stops the OI. All streams will stop and the robot will no longer respond to commands.
    Use this command when you are finished working with the robot.
    - Serial sequence: [173].
    - Available in modes: Passive, Safe, or Full
    - Changes mode to: Off. Roomba plays a song to acknowledge it is exiting the OI.
    """
    send([173])


def safe():
    """
    This command puts the OI into Safe mode, enabling user control of Roomba. It turns off all LEDs. The OI
    can be in Passive, Safe, or Full mode to accept this command. If a safety condition occurs (see above)
    Roomba reverts automatically to Passive mode.
    - Serial sequence: [131]
    - Available in modes: Passive, Safe, or Full
    - Changes mode to: Safe
    """
    send([131])


def full():
    """
    This command gives you complete control over Roomba by putting the OI into Full mode, and turning off
    the cliff, wheel-drop and internal charger safety features. That is, in Full mode, Roomba executes any
    command that you send it, even if the internal charger is plugged in, or command triggers a cliff or wheel
    drop condition.
    - Serial sequence: [132]
    - Available in modes: Passive, Safe, or Full
    - Changes mode to: Full  
    """
    send([132])


def passive():
    """
    Runs start() to change mode to passive.
    """
    start()


def drive(velocity, radius):
    """
    This command controls Roomba’s drive wheels. It takes four data bytes, interpreted as two 16-bit signed
    values using two’s complement. (http://en.wikipedia.org/wiki/Two%27s_complement) The first two bytes
    specify the average velocity of the drive wheels in millimeters per second (mm/s), with the high byte
    being sent first. The next two bytes specify the radius in millimeters at which Roomba will turn. The
    longer radii make Roomba drive straighter, while the shorter radii make Roomba turn more. The radius is
    measured from the center of the turning circle to the center of Roomba. A Drive command with a
    positive velocity and a positive radius makes Roomba drive forward while turning toward the left. A
    negative radius makes Roomba turn toward the right. Special cases for the radius make Roomba turn in
    place or drive straight, as specified below. A negative velocity makes Roomba drive backward.
    NOTE:
    Internal and environmental restrictions may prevent Roomba from accurately carrying out some drive
    commands. For example, it may not be possible for Roomba to drive at full speed in an arc with a large
    radius of curvature.
    - Serial sequence: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
    - Available in modes: Safe or Full
    - Changes mode to: No Change
    - Velocity (-500 – 500 mm/s)
    - Radius (-2000 – 2000 mm)
    Special cases:
    Straight = 32768 or 32767 = 0x8000 or 0x7FFF
    Turn in place clockwise = -1 = 0xFFFF
    Turn in place counter-clockwise = 1 = 0x0001
    Example:
    To drive in reverse at a velocity of -200 mm/s while turning at a radius of 500mm, send the following
    serial byte sequence:
    [137] [255] [56] [1] [244]
    Explanation:
    Desired value -> two’s complement and convert to hex -> split into 2 bytes -> convert to decimal
    Velocity = -200 = 0xFF38 = [0xFF] [0x38] = [255] [56]
    Radius = 500 = 0x01F4 = [0x01] [0xF4] = [1] [244]
    """
    # Clamp velocity and radius to their valid ranges
    velocity = max(-500, min(500, velocity))
    radius = max(-2000, min(2000, radius))
    
    # Convert velocity and radius to 16-bit two's complement representation
    velocity_16bit = velocity & 0xFFFF
    radius_16bit = radius & 0xFFFF
    
    # Extract high and low bytes from each value
    velocity_high = (velocity_16bit >> 8) & 0xFF
    velocity_low = velocity_16bit & 0xFF
    radius_high = (radius_16bit >> 8) & 0xFF
    radius_low = radius_16bit & 0xFF
    
    # Send the command: [137] [velocity_high] [velocity_low] [radius_high] [radius_low]
    send([137, velocity_high, velocity_low, radius_high, radius_low])


def drive_direct(left, right):
    """
    This command lets you control the forward and backward motion of Roomba’s drive wheels
    independently. It takes four data bytes, which are interpreted as two 16-bit signed values using two’s
    complement. The first two bytes specify the velocity of the right wheel in millimeters per second (mm/s),
    with the high byte sent first. The next two bytes specify the velocity of the left wheel, in the same
    format. A positive velocity makes that wheel drive forward, while a negative velocity makes it drive
    backward.
    - Serial sequence: [145] [Right velocity high byte] [Right velocity low byte] [Left velocity high byte]
    [Left velocity low byte]
    - Available in modes: Safe or Full
    - Changes mode to: No Change
    - Right wheel velocity (-500 – 500 mm/s)
    - Left wheel velocity (-500 – 500 mm/s)
    """
    # Clamp left and right velocity to their valid ranges
    left = max(-500, min(500, left))
    right = max(-500, min(500, right))   

    send([
        145,
        (right >> 8) & 0xFF,
        right & 0xFF,
        (left >> 8) & 0xFF,
        left & 0xFF
    ])


def stop():
    """
    Stops all wheel motion
    """
    drive_direct(0, 0)


def song(num, song):
    """
    This command lets you specify up to four songs to the OI that you can play at a later time. Each song is
    associated with a song number. The Play command uses the song number to identify your song selection.
    Each song can contain up to sixteen notes. Each note is associated with a note number that uses MIDI
    note definitions and a duration that is specified in fractions of a second. The number of data bytes varies,
    depending on the length of the song specified. A one note song is specified by four data bytes. For each
    additional note within a song, add two data bytes.
    - Serial sequence: [140] [Song Number] [Song Length] [Note Number 1] [Note Duration 1] [Note
    Number 2] [Note Duration 2], etc.
    - Available in modes: Passive, Safe, or Full
    - Changes mode to: No Change
    - Song Number (0 – 4)
    The song number associated with the specific song. If you send a second Song command, using the
    same song number, the old song is overwritten.
    - Song Length (1 – 16)
    The length of the song, according to the number of musical notes within the song.
    - Song data bytes 3, 5, 7, etc.: Note Number (31 – 127)
    The pitch of the musical note Roomba will play, according to the MIDI note numbering scheme. The
    lowest musical note that Roomba will play is Note #31. Roomba considers all musical notes outside
    the range of 31 – 127 as rest notes, and will make no sound during the duration of those notes.
    - Song data bytes 4, 6, 8, etc.: Note Duration (0 – 255)
    The duration of a musical note, in increments of 1/64th of a second. Example: a half-second long
    musical note has a duration value of 32.

    Number  Note    Frequency
    31      G       49.0 
    32      G#      51.9 
    33      A       55.0 
    34      A#      58.3 
    35      B       61.7 
    36      C       65.4 
    37      C#      69.3 
    38      D       73.4 
    39      D#      77.8 
    40      E       82.4 
    41      F       87.3 
    42      F#      92.5 
    43      G       98.0 
    44      G#      103.8 
    45      A       110.0 
    46      A#      116.5 
    47      B       123.5 
    48      C       130.8 
    49      C#      138.6 
    50      D       146.8 
    51      D#      155.6 
    52      E       164.8 
    53      F       174.6 
    54      F#      185.0 
    55      G       196.0 
    56      G#      207.7 
    57      A       220.0 
    58      A#      233.1 
    59      B       246.9 
    60      C       261.6 
    61      C#      277.2 
    62      D       293.7 
    63      D#      311.1 
    64      E       329.6 
    65      F       349.2 
    66      F#      370.0 
    67      G       392.0 
    68      G#      415.3 
    69      A       440.0 
    70      A#      466.2 
    71      B       493.9 
    72      C       523.3 
    73      C#      554.4 
    74      D       587.3 
    75      D#      622.3 
    76      E       659.3 
    77      F       698.5 
    78      F#      740.0 
    79      G       784.0 
    80      G#      830.6 
    81      A       880.0
    82      A#      932.4
    83      B       987.8
    84      C       1046.5
    85      C#      1108.8
    86      D       1174.7
    87      D#      1244.5
    88      E       1318.5
    89      F       1396.9
    90      F#      1480.0
    91      G       1568.0
    92      G#      1661.3
    93      A       1760.0
    94      A#      1864.7
    95      B       1975.6
    96      C       2093.1
    97      C#      2217.5
    98      D       2349.4
    99      D#      2489.1
    100     E       2637.1
    101     F       2793.9
    102     F#      2960.0
    103     G       3136.0
    104     G#      3322.5
    105     A       3520.1
    106     A#      3729.4
    107     B       3951.2 
    """
    if num > 4 or num < 0:
        print("song number must be 0-4")
        return
    if len(song) > 16:
        print("song cannot be longer than 16 notes")
    for note in song:
        if len(note) != 2:
            print("each note must be a (note_number, duration) tuple")
            return
        note_number, duration = note
        if not (31 <= note_number <= 127):
            print(f"note number {note_number} out of range (31-127)")
            return
        if not (0 <= duration <= 255):
            print(f"duration {duration} out of range (0-255)")
            return

    flat_song = [item for note in song for item in note]
    send([140, num, len(song)] + flat_song)
    send([])


def play(num):
    """
    This command lets you select a song to play from the songs added to Roomba using the Song command.
    You must add one or more songs to Roomba using the Song command in order for the Play command to
    work.
    - Serial sequence: [141] [Song Number]
    - Available in modes: Safe or Full
    - Changes mode to: No Change
    - Song Number (0 - 4)
    The number of the song Roomba is to play.
    """
    send([141, num])


def stream(packets):
    """
    This command starts a stream of data packets. The list of packets requested is sent every 15 ms, which
    is the rate Roomba uses to update data.
    This method of requesting sensor data is best if you are controlling Roomba over a wireless network
    (which has poor real-time characteristics) with software running on a desktop computer.
    - Serial sequence: [148] [Number of packets] [Packet ID 1] [Packet ID 2] [Packet ID 3] etc.
    - Available in modes: Passive, Safe, or Full
    - Changes mode to: No Change
    The format of the data returned is:
    [19][N-bytes][Packet ID 1][Packet 1 data…][Packet ID 2][Packet 2 data…][Checksum]
    N-bytes is the number of bytes between the n-bytes byte and the checksum.
    The checksum is a 1-byte value. It is the 8-bit complement of all of the bytes in the packet, excluding
    the checksum itself. That is, if you add all of the bytes in the packet, including the checksum, the low
    byte of the result will be 0.
    Example:
    To get data from Roomba's left cliff signal (packet 29) and virtual wall sensor (packet 13), send the
    following command string to Roomba:
    [148] [2] [29] [13]
    NOTE:
    The left cliff signal is a 2-byte packet and the virtual wall is a 1-byte packet.
    Roomba starts streaming data that looks like this:
    19 5 29 2 25 13 0 163
    header n-bytes packet ID 1 Packet data 1 (2 bytes) packet ID 2 packet data 2 (1 byte) Checksum
    NOTE:
    Checksum computation: (19 + 5 + 29 + 2 + 25 + 13 + 0 + 163) = 256 and (256 & 0xFF) = 0.
    In the above stream segment, Roomba’s left cliff signal value was 549 (0x0225) and there was no virtual
    wall signal.
    It is up to you not to request more data than can be sent at the current baud rate in the 15 ms time slot.
    For example, at 115200 baud, a maximum of 172 bytes can be sent in 15 ms:
    15 ms / 10 bits
    (8 data + start + stop) * 115200 = 172.8
    If more data is requested, the data stream will eventually become corrupted. This can be confirmed by
    checking the checksum.
    The header byte and checksum can be used to align your receiving program with the data. All data
    chunks start with 19 and end with the 1-byte checksum.
    """
    send([148, len(packets)] + packets)


def pause_stream():
    """
    Pause the current data stream
    """
    send([150, 0])


def resume_stream():
    """
    Resume a paused data stream
    """
    send([150, 1])


def read_stream(continuous=True):

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
        if not continuous: 
            break


# Sensor packets we want streamed
# (choose ones useful for mapping + odometry)
SENSOR_PACKETS = [
    7,   # Bumps and wheel drops
    17,  # IR omni
    19,  # Distance
    20,  # Angle
    21,  # Charging state
    22,  # Voltage
    23,  # Current
    24,  # Temperature
    25,  # Battery charge
    26,  # Battery capacity
    52,  # IR Left
    53   # IR Right
]

# Packet sizes from OI spec
PACKET_SIZES = {
    7: 1,
    17: 1,
    19: 2,
    20: 2,
    21: 1,
    22: 2,
    23: 2,
    24: 1,
    25: 2,
    26: 2,
    52: 1,
    53: 1
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
    elif packet_id == 17:
        return {"ir_omni": data[0]}

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