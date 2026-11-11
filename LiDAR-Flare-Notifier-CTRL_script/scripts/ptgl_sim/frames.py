FRAME_LEN = 8
HEADER0 = 0x55
HEADER1 = 0xAA

FUNC_SELF_TEST = 0x80
FUNC_SINGLE = 0x88
FUNC_STREAM = 0x89
FUNC_STOP = 0x8E

STA_OK = 1


def checksum_reply(frame: bytes) -> int:
    return sum(frame[:7]) & 0xFF


def checksum_command(func: int, data: bytes) -> int:
    return (func + sum(data)) & 0xFF


def build_reply(func: int, sta: int, distance_mm: int = 0) -> bytes:
    frame = bytearray(
        [
            HEADER0,
            HEADER1,
            func & 0xFF,
            sta & 0xFF,
            (distance_mm >> 16) & 0xFF,
            (distance_mm >> 8) & 0xFF,
            distance_mm & 0xFF,
            0,
        ]
    )
    frame[7] = checksum_reply(frame)
    return bytes(frame)


def build_command(func: int) -> bytes:
    data = b"\xFF\xFF\xFF\xFF"
    return bytes([HEADER0, HEADER1, func & 0xFF]) + data + bytes([checksum_command(func, data)])
