#!/usr/bin/env python3
"""Validate a DialScroll UF2 image for the selected bootloader layout."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import struct


APPLICATION_RANGES = {"sd": (0x26000, 0xEC000), "nosd": (0x1000, 0xEA000)}
FAMILY_ID = 0xADA52840
BLOCK_SIZE = 512


@dataclass(frozen=True)
class Verification:
    start_address: int
    reset_vector: int
    block_count: int


def verify_uf2(path: Path, variant: str) -> Verification:
    start, limit = APPLICATION_RANGES[variant]
    data = path.read_bytes()
    if not data or len(data) % BLOCK_SIZE:
        raise ValueError("File size must be a positive multiple of 512 bytes")

    block_count = len(data) // BLOCK_SIZE
    numbers = set()
    segments = []
    reset_vector = None
    for offset in range(0, len(data), BLOCK_SIZE):
        block = data[offset:offset + BLOCK_SIZE]
        magic0, magic1, flags, address, size, number, total, family = struct.unpack_from("<8I", block)
        if (magic0, magic1, struct.unpack_from("<I", block, 508)[0]) != (
            0x0A324655, 0x9E5D5157, 0x0AB16F30
        ):
            raise ValueError(f"Invalid UF2 magic at file offset {offset}")
        if flags != 0x2000 or family != FAMILY_ID:
            raise ValueError(f"Expected an nRF52840 flash block with family ID 0x{FAMILY_ID:08X}")
        if not 0 < size <= 476 or size % 4 or address % 4:
            raise ValueError(f"Invalid payload size or alignment in block {number}")
        if total != block_count or number >= block_count or number in numbers:
            raise ValueError("Block numbering is incomplete, duplicated, or inconsistent")
        numbers.add(number)
        if address < start or address + size > limit:
            raise ValueError(
                f"Block {number} falls outside the {variant} application region "
                f"[0x{start:08X}, 0x{limit:08X})"
            )
        segments.append((address, address + size))
        if address == start:
            if size < 8:
                raise ValueError("The first payload must contain the stack pointer and reset vector")
            reset_vector = struct.unpack_from("<I", block, 36)[0]

    segments.sort()
    if segments[0][0] != start:
        raise ValueError(f"Expected application start 0x{start:08X}")
    if any(current[0] < previous[1] for previous, current in zip(segments, segments[1:])):
        raise ValueError("UF2 payloads overlap")
    if reset_vector is None or not reset_vector & 1 or not start <= (reset_vector & ~1) < limit:
        raise ValueError("Reset vector must be a Thumb address inside the application region")
    return Verification(start, reset_vector, block_count)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("--variant", choices=APPLICATION_RANGES, required=True)
    args = parser.parse_args()
    try:
        result = verify_uf2(args.image, args.variant)
    except (OSError, ValueError) as error:
        parser.exit(1, f"UF2 validation failed: {error}\n")
    print(f"Variant:      {args.variant}")
    print(f"UF2 start:    0x{result.start_address:08X}")
    print(f"Reset vector: 0x{result.reset_vector:08X}")
    print(f"Blocks:       {result.block_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
