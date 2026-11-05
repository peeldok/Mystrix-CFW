#!/usr/bin/env python3
"""Pack ESP32-S3 app BIN for Mystrix UF2: 256 bytes/block, OTA_0-relative addresses."""
import argparse
import struct
from pathlib import Path

START0, START1, END = 0x0A324655, 0x9E5D5157, 0x0AB16F30
FAMILY = 0xC47E5767
PAYLOAD = 256
SECTOR = 512

def to_uf2(data):
    if not data or data[0] != 0xE9:
        raise ValueError("Application BIN is missing ESP-IDF magic 0xE9")
    n = (len(data) + PAYLOAD - 1) // PAYLOAD
    result = bytearray(n * SECTOR)
    for i in range(n):
        pos = i * SECTOR
        struct.pack_into("<8I", result, pos, START0, START1, 0x2000,
                         i * PAYLOAD, PAYLOAD, i, n, FAMILY)
        chunk = data[i * PAYLOAD : (i + 1) * PAYLOAD]
        result[pos+32:pos+32+len(chunk)] = chunk
        struct.pack_into("<I", result, pos+508, END)
    reconstructed = b"".join(result[i * SECTOR + 32 : i * SECTOR + 32 + PAYLOAD]
                             for i in range(n))[:len(data)]
    if reconstructed != data:
        raise ValueError("UF2 reconstruction mismatch")
    if struct.unpack_from("<I", result, 12)[0] != 0:
        raise ValueError("First UF2 target address is not zero")
    print(f"Verified {n} UF2 blocks, family 0x{FAMILY:08x}, 256-byte blocks, starting at 0x0")
    return result

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("app", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    args.output.write_bytes(to_uf2(args.app.read_bytes()))
    print(f"Created {args.output} ({args.output.stat().st_size} bytes)")
