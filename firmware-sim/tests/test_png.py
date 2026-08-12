import pathlib
import struct
import unittest
import zlib

from ..png import PngError, decode, encode_grey


def _chunk(typ, payload):
    body = typ + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))


def _make_png(width, height, colour_type, rows, filter_type=0):
    """rows: list of bytes, one per scanline, already in raw (unfiltered) form"""
    ihdr = struct.pack(">IIBBBBB", width, height, 8, colour_type, 0, 0, 0)
    raw = b"".join(bytes([filter_type]) + r for r in rows)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(raw))
        + _chunk(b"IEND", b"")
    )


class TestPng(unittest.TestCase):
    def test_greyscale_filter_none(self):
        img = decode(_make_png(3, 2, 0, [bytes([0, 128, 255]), bytes([10, 20, 30])]))
        self.assertEqual((img.width, img.height, img.channels), (3, 2, 1))
        self.assertEqual(img.grey(0, 0), 0)
        self.assertEqual(img.grey(2, 0), 255)
        self.assertEqual(img.grey(1, 1), 20)

    def test_rgb(self):
        img = decode(_make_png(2, 1, 2, [bytes([255, 0, 0, 0, 0, 255])]))
        self.assertEqual(img.channels, 3)
        self.assertGreater(img.grey(0, 0), img.grey(1, 0))  # red brighter than blue

    def test_rgba(self):
        img = decode(_make_png(1, 1, 6, [bytes([255, 255, 255, 255])]))
        self.assertEqual(img.channels, 4)
        self.assertEqual(img.grey(0, 0), 255)

    def test_grey_alpha(self):
        img = decode(_make_png(1, 1, 4, [bytes([77, 255])]))
        self.assertEqual(img.channels, 2)
        self.assertEqual(img.grey(0, 0), 77)

    def test_sub_filter(self):
        # Sub: each byte is the delta from the pixel to its left
        png = _make_png(3, 1, 0, [bytes([10, 5, 5])], filter_type=1)
        img = decode(png)
        self.assertEqual([img.grey(x, 0) for x in range(3)], [10, 15, 20])

    def test_up_filter(self):
        ihdr = struct.pack(">IIBBBBB", 2, 2, 8, 0, 0, 0, 0)
        raw = bytes([0, 10, 20]) + bytes([2, 5, 5])  # row0 None, row1 Up
        png = (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(raw))
            + _chunk(b"IEND", b"")
        )
        img = decode(png)
        self.assertEqual([img.grey(x, 1) for x in range(2)], [15, 25])

    def test_average_filter(self):
        ihdr = struct.pack(">IIBBBBB", 2, 2, 8, 0, 0, 0, 0)
        # row0 None -> [10, 20]; row1 Average -> a = left, b = above
        raw = bytes([0, 10, 20]) + bytes([3, 5, 5])
        png = (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(raw))
            + _chunk(b"IEND", b"")
        )
        img = decode(png)
        # x=0: 5 + (0 + 10) // 2 = 10 ; x=1: 5 + (10 + 20) // 2 = 20
        self.assertEqual([img.grey(x, 1) for x in range(2)], [10, 20])

    def test_paeth_filter(self):
        ihdr = struct.pack(">IIBBBBB", 2, 2, 8, 0, 0, 0, 0)
        raw = bytes([0, 10, 20]) + bytes([4, 0, 0])
        png = (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(raw))
            + _chunk(b"IEND", b"")
        )
        img = decode(png)
        # Paeth with zero deltas reproduces the predictor exactly
        self.assertEqual([img.grey(x, 1) for x in range(2)], [10, 20])

    def test_rejects_interlaced(self):
        ihdr = struct.pack(">IIBBBBB", 1, 1, 8, 0, 0, 0, 1)
        png = (
            b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(bytes([0, 0])))
            + _chunk(b"IEND", b"")
        )
        with self.assertRaises(PngError):
            decode(png)

    def test_rejects_non_png(self):
        with self.assertRaises(PngError):
            decode(b"not a png at all")

    def test_decodes_real_photo(self):
        p = pathlib.Path("Maze Overhead 1.png")
        if not p.exists():
            self.skipTest("reference photo not present")
        img = decode(p.read_bytes())
        self.assertEqual((img.width, img.height, img.channels), (1018, 574, 4))


class TestEncode(unittest.TestCase):
    def test_encode_decode_roundtrip(self):
        pixels = bytearray([0, 64, 128, 255, 10, 20, 30, 40])
        img = decode(encode_grey(4, 2, pixels))
        self.assertEqual((img.width, img.height, img.channels), (4, 2, 1))
        self.assertEqual([img.grey(x, 0) for x in range(4)], [0, 64, 128, 255])
        self.assertEqual([img.grey(x, 1) for x in range(4)], [10, 20, 30, 40])

    def test_encode_rejects_wrong_pixel_count(self):
        with self.assertRaises(PngError):
            encode_grey(4, 2, bytearray([0, 0]))


if __name__ == "__main__":
    unittest.main()
