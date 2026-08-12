"""Minimal PNG/PGM reader and writer built on the standard library.

Exists because the target interpreter has no Pillow and the project carries a
zero-dependency constraint. Supports bit depth 8, colour types 0/2/4/6,
non-interlaced -- which covers anything a drawing program exports for a
black-and-white maze image.
"""

import pathlib
import struct
import zlib
from dataclasses import dataclass

_SIGNATURE = b"\x89PNG\r\n\x1a\n"

_CHANNELS = {0: 1, 2: 3, 4: 2, 6: 4}


class PngError(Exception):
    pass


@dataclass
class Image:
    width: int
    height: int
    channels: int
    pixels: bytearray

    def grey(self, x, y):
        """Luminance 0-255 at (x, y). Alpha is ignored: a maze image's
        transparent regions are treated as whatever colour they carry."""
        i = (y * self.width + x) * self.channels
        p = self.pixels
        if self.channels <= 2:
            return p[i]
        return (p[i] * 299 + p[i + 1] * 587 + p[i + 2] * 114) // 1000


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def decode(data):
    """Decode PNG bytes into an Image."""
    if not data.startswith(_SIGNATURE):
        raise PngError("not a PNG file (bad signature)")

    header = None
    idat = bytearray()
    i = len(_SIGNATURE)

    while i + 8 <= len(data):
        (length,) = struct.unpack(">I", data[i : i + 4])
        typ = data[i + 4 : i + 8]
        payload = data[i + 8 : i + 8 + length]

        if typ == b"IHDR":
            header = struct.unpack(">IIBBBBB", payload[:13])
        elif typ == b"IDAT":
            idat += payload
        elif typ == b"IEND":
            break

        i += 12 + length

    if header is None:
        raise PngError("PNG has no IHDR chunk")

    width, height, bit_depth, colour_type, compression, filter_method, interlace = header

    if bit_depth != 8:
        raise PngError(f"unsupported bit depth {bit_depth} (only 8 is supported)")
    if colour_type not in _CHANNELS:
        raise PngError(f"unsupported colour type {colour_type}")
    if interlace != 0:
        raise PngError("interlaced PNGs are not supported; re-export without Adam7")
    if compression != 0 or filter_method != 0:
        raise PngError("unsupported compression or filter method")

    channels = _CHANNELS[colour_type]
    stride = width * channels

    try:
        raw = zlib.decompress(bytes(idat))
    except zlib.error as exc:
        raise PngError(f"corrupt image data: {exc}") from exc

    expected = (stride + 1) * height
    if len(raw) < expected:
        raise PngError(f"truncated image data: got {len(raw)} bytes, need {expected}")

    out = bytearray(stride * height)
    prev = bytearray(stride)

    pos = 0
    for row in range(height):
        ftype = raw[pos]
        pos += 1
        line = bytearray(raw[pos : pos + stride])
        pos += stride

        if ftype == 0:
            pass
        elif ftype == 1:  # Sub
            for n in range(channels, stride):
                line[n] = (line[n] + line[n - channels]) & 0xFF
        elif ftype == 2:  # Up
            for n in range(stride):
                line[n] = (line[n] + prev[n]) & 0xFF
        elif ftype == 3:  # Average
            for n in range(stride):
                left = line[n - channels] if n >= channels else 0
                line[n] = (line[n] + ((left + prev[n]) >> 1)) & 0xFF
        elif ftype == 4:  # Paeth
            for n in range(stride):
                left = line[n - channels] if n >= channels else 0
                upleft = prev[n - channels] if n >= channels else 0
                line[n] = (line[n] + _paeth(left, prev[n], upleft)) & 0xFF
        else:
            raise PngError(f"unknown filter type {ftype} on row {row}")

        out[row * stride : (row + 1) * stride] = line
        prev = line

    return Image(width, height, channels, out)


def _read_pnm(data):
    """Read a netpbm P2 (ASCII grey) or P5 (binary grey) image."""
    if data[:2] not in (b"P2", b"P5"):
        raise PngError("not a netpbm P2/P5 file")
    binary = data[:2] == b"P5"

    fields = []
    pos = 2
    while len(fields) < 3:
        while pos < len(data) and data[pos : pos + 1].isspace():
            pos += 1
        if data[pos : pos + 1] == b"#":
            while pos < len(data) and data[pos : pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos : pos + 1].isspace():
            pos += 1
        fields.append(int(data[start:pos]))

    width, height, maxval = fields
    pos += 1  # single whitespace byte after maxval

    if binary:
        pixels = bytearray(data[pos : pos + width * height])
    else:
        pixels = bytearray(int(t) for t in data[pos:].split()[: width * height])

    if len(pixels) != width * height:
        raise PngError("truncated netpbm data")
    if maxval != 255:
        pixels = bytearray(min(255, p * 255 // maxval) for p in pixels)

    return Image(width, height, 1, pixels)


def read(path):
    """Read a PNG or netpbm image from disk, dispatching on extension."""
    p = pathlib.Path(path)
    data = p.read_bytes()
    if p.suffix.lower() in (".pgm", ".pbm", ".pnm"):
        return _read_pnm(data)
    return decode(data)


def _chunk(typ, payload):
    body = typ + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))


def encode_grey(width, height, pixels):
    """Encode an 8-bit greyscale image to PNG bytes. Used by the placeholder
    maze generator; filter type 0 throughout, which compresses fine for the
    large flat regions a maze image is made of."""
    if len(pixels) != width * height:
        raise PngError(f"expected {width * height} pixels, got {len(pixels)}")

    raw = bytearray()
    for row in range(height):
        raw.append(0)
        raw += bytes(pixels[row * width : (row + 1) * width])

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (
        _SIGNATURE
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(bytes(raw), 9))
        + _chunk(b"IEND", b"")
    )
