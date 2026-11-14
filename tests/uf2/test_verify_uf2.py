"""Exercise bootloader-layout checks with complete and damaged UF2 images."""

from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from verify_uf2 import verify_uf2


def image(start=0x26000, reset=None):
    blocks = []
    for number in range(2):
        block = bytearray(512)
        struct.pack_into("<8I", block, 0, 0x0A324655, 0x9E5D5157, 0x2000,
                         start + number * 256, 256, number, 2, 0xADA52840)
        if number == 0:
            struct.pack_into("<2I", block, 32, 0x20010000,
                             reset if reset is not None else start + 0x101)
        struct.pack_into("<I", block, 508, 0x0AB16F30)
        blocks.append(block)
    return bytearray().join(blocks)


class VerifyUf2Test(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "firmware with spaces.uf2"

    def verify(self, data, variant="sd"):
        self.path.write_bytes(data)
        return verify_uf2(self.path, variant)

    def test_accepts_both_layouts_and_reordered_blocks(self):
        for variant, start in (("sd", 0x26000), ("nosd", 0x1000)):
            with self.subTest(variant=variant):
                data = image(start)
                result = self.verify(data[512:] + data[:512], variant)
                self.assertEqual((result.start_address, result.reset_vector, result.block_count),
                                 (start, start + 0x101, 2))

    def test_rejects_empty_partial_and_whole_block_truncation(self):
        for data in (b"", image()[:-1], image()[:512]):
            with self.subTest(size=len(data)):
                with self.assertRaises(ValueError):
                    self.verify(data)

    def test_checks_magic_in_every_block(self):
        for offset in (0, 4, 508, 512, 516, 1020):
            with self.subTest(offset=offset):
                data = image()
                data[offset] ^= 1
                with self.assertRaisesRegex(ValueError, "magic"):
                    self.verify(data)

    def test_rejects_wrong_family_and_flags(self):
        for offset, value in ((28, 0), (540, 0), (8, 0), (520, 0x2001)):
            with self.subTest(offset=offset, value=value):
                data = image()
                struct.pack_into("<I", data, offset, value)
                with self.assertRaisesRegex(ValueError, "family ID"):
                    self.verify(data)

    def test_rejects_wrong_layout_and_missing_vector_block(self):
        with self.assertRaises(ValueError):
            self.verify(image(), "nosd")
        data = image()
        struct.pack_into("<I", data, 12, 0x26200)
        with self.assertRaisesRegex(ValueError, "application start"):
            self.verify(data)

    def test_rejects_reset_outside_layout_or_without_thumb_bit(self):
        for variant, start, limit in (("sd", 0x26000, 0xEC000), ("nosd", 0x1000, 0xEA000)):
            for reset in (start - 1, start + 0x100, limit + 1):
                with self.subTest(variant=variant, reset=hex(reset)):
                    with self.assertRaisesRegex(ValueError, "Reset vector"):
                        self.verify(image(start, reset), variant)

    def test_rejects_invalid_payload_and_overlapping_ranges(self):
        for size in (0, 4, 255, 480):
            with self.subTest(size=size):
                data = image()
                struct.pack_into("<I", data, 16, size)
                with self.assertRaises(ValueError):
                    self.verify(data)
        data = image()
        struct.pack_into("<I", data, 524, 0x26080)
        with self.assertRaisesRegex(ValueError, "overlap"):
            self.verify(data)

    def test_rejects_payload_outside_application(self):
        for address in (0x25F00, 0xEBF04):
            with self.subTest(address=hex(address)):
                data = image()
                struct.pack_into("<I", data, 524, address)
                with self.assertRaisesRegex(ValueError, "application region"):
                    self.verify(data)

    def test_rejects_duplicate_or_out_of_range_block_numbers(self):
        for number in (0, 2):
            with self.subTest(number=number):
                data = image()
                struct.pack_into("<I", data, 532, number)
                with self.assertRaisesRegex(ValueError, "numbering"):
                    self.verify(data)

    def test_cli_handles_paths_with_spaces_and_reports_failure(self):
        script = Path(__file__).resolve().parents[2] / "tools" / "verify_uf2.py"
        self.path.write_bytes(image())
        command = [sys.executable, str(script), str(self.path), "--variant", "sd"]
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("0x00026000", result.stdout)
        self.path.write_bytes(b"truncated")
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(result.returncode, 1)
        self.assertIn("UF2 validation failed", result.stderr)


if __name__ == "__main__":
    unittest.main()
