"""Dump the string table of a DEX file. No Java needed.

Usage: python -I dexstr.py <classes.dex> [...]
Prints one string per line as "<file>\t<string>".
"""
import struct
import sys

# -I implies -E, so PYTHONIOENCODING is ignored: set it here.
sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def uleb128(buf, i):
    result = 0
    shift = 0
    while True:
        b = buf[i]
        i += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, i
        shift += 7


def strings(path):
    with open(path, "rb") as f:
        buf = f.read()
    if buf[:4] != b"dex\n":
        raise SystemExit("%s: not a dex file" % path)
    n_ids, off_ids = struct.unpack_from("<II", buf, 0x38)
    out = []
    for k in range(n_ids):
        (data_off,) = struct.unpack_from("<I", buf, off_ids + 4 * k)
        _, i = uleb128(buf, data_off)          # utf16 length, then MUTF-8 bytes
        end = buf.index(b"\x00", i)
        out.append(buf[i:end].decode("utf-8", "replace"))
    return out


for p in sys.argv[1:]:
    name = p.replace("\\", "/").rsplit("/", 1)[-1]
    for s in strings(p):
        sys.stdout.write("%s\t%s\n" % (name, s.replace("\t", " ").replace("\n", " ")))
