# UART link from the ESP32-C3 to the Zigbee MCU (ESP32-C6): COBS framing + CRC-8.
#
# Frames are COBS-encoded and terminated by a single 0x00 byte. COBS
# guarantees the payload contains no 0x00, so 0x00 is an unambiguous
# frame delimiter and the receiver can always resync after corruption.
# A CRC-8 over the raw payload lets the receiver reject bit-flips that
# COBS framing alone would not catch.

import struct


def _crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def _cobs_encode(data):
    out    = bytearray(1)  # index 0 holds the first code byte
    code_i = 0
    code   = 1
    for b in data:
        if b == 0:
            out[code_i] = code
            code_i = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_i] = code
                code_i = len(out)
                out.append(0)
                code = 1
    out[code_i] = code
    return out


# Payload (13 bytes, little-endian): presence(uint8), slot1/2/3(float32).
# None scores are sent as -1.0; the receiver maps any negative score to NaN.
# Must match the decoder in zigbee/peril.yaml.
def send_scores(uart, presence, slot_score):
    s = [v if v is not None else -1.0 for v in slot_score]
    payload = struct.pack('<Bfff', 1 if presence else 0, s[0], s[1], s[2])
    packet  = _cobs_encode(payload + bytes((_crc8(payload),)))
    packet.append(0)  # 0x00 frame delimiter
    uart.write(packet)
