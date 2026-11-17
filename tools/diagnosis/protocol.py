"""Host-side validation and DaMiao read-only helpers; no hardware access."""
import struct

MAGIC = 0x44494147
ABI = 1
WORDS = 1344
RESULTS = ['accepted', 'invalid request', 'busy / cancellation pending', 'hardware error', 'listen-only: select normal or loopback first']
KINDS = {1: 'RX', 2: 'TX_QUEUED', 3: 'TX_OK', 4: 'TX_ERROR', 5: 'TX_TIMEOUT'}
BUS_FIELDS = 'bitrate mode ready rx_frames rx_dropped rx_callbacks tx_queued tx_ok tx_error tx_timeout esr tsr rf0r last_tx_tsr pending last_rx_ms'.split()


def number(value):
    return int(value, 0) if isinstance(value, str) else int(value)


def frame(can_id, data, flags=0):
    can_id, flags = number(can_id), number(flags)
    if flags < 0 or flags > 3 or not 0 <= can_id <= (0x1fffffff if flags & 1 else 0x7ff):
        raise ValueError('CAN ID / flags out of range')
    payload = bytes.fromhex(data) if isinstance(data, str) else bytes(data)
    if len(payload) > 8:
        raise ValueError('Classic CAN supports at most 8 bytes')
    return can_id, payload, flags


def read_register(motor_id, register):
    motor_id, register = number(motor_id), number(register)
    if not 0 <= motor_id <= 0x7ff or not 0 <= register <= 255:
        raise ValueError('Motor ID / register out of range')
    # J3507 manual: parameter reads use ID 0x7FF and exactly four data bytes.
    return 0x7ff, struct.pack('<HBB', motor_id, 0x33, register), 0


def decode_event(words):
    seq, ms, kind, channel, can_id, flags, length, low, high, token = words
    data = struct.pack('<II', low, high)[:min(length, 8)]
    result = dict(seq=seq, ms=ms, kind=KINDS.get(kind, str(kind)), channel=channel,
                  id=f'0x{can_id:03X}', flags=flags, length=length,
                  data=data.hex(' ').upper(), token=token)
    # Classification stays explicit: register replies must not be interpreted
    # as packed motor feedback. Physical ranges are never guessed by model.
    if kind == 1 and flags == 0 and length == 8 and data[2] == 0x33:
        result['register_candidate'] = dict(motor_id=int.from_bytes(data[:2], 'little'),
            register=data[3], uint32=int.from_bytes(data[4:], 'little'),
            float32=struct.unpack('<f', data[4:])[0])
    return result


def decode_feedback(data, limits):
    if len(data) != 8:
        raise ValueError('Feedback requires 8 bytes')
    import math
    ranges = [float(limits[name]) for name in ('p_max', 'v_max', 't_max')]
    if any(not math.isfinite(v) or v <= 0 for v in ranges):
        raise ValueError('Read and verify positive P_MAX / V_MAX / T_MAX first')
    raw = [data[1] << 8 | data[2], data[3] << 4 | data[4] >> 4,
           (data[4] & 15) << 8 | data[5]]
    values = [(value / maximum * 2 - 1) * limit
              for value, maximum, limit in zip(raw, (65535, 4095, 4095), ranges)]
    return dict(state=data[0] >> 4, motor_id_low_nibble=data[0] & 15,
                position_rad=values[0], velocity_rad_s=values[1], torque_nm=values[2],
                mos_c=data[6], rotor_c=data[7])
