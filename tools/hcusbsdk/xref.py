"""Find AArch64 BL/B call sites targeting an address, and name the enclosing function.

Usage: python -I xref.py <lib.so> <0xTARGET>
"""
import struct
import sys

from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

path = sys.argv[1]
target = int(sys.argv[2], 0)

with open(path, "rb") as fh:
    elf = ELFFile(fh)
    funcs = []
    for sec in elf.iter_sections():
        if isinstance(sec, SymbolTableSection):
            for s in sec.iter_symbols():
                if s["st_info"]["type"] == "STT_FUNC" and s["st_value"]:
                    funcs.append((s["st_value"], s["st_size"], s.name))
    funcs.sort()

    text = None
    for sec in elf.iter_sections():
        if sec.name == ".text":
            text = (sec["sh_addr"], sec.data())
    if text is None:
        raise SystemExit("no .text")

base, data = text


def owner(addr):
    lo, hi = 0, len(funcs) - 1
    best = None
    while lo <= hi:
        mid = (lo + hi) // 2
        if funcs[mid][0] <= addr:
            best = funcs[mid]
            lo = mid + 1
        else:
            hi = mid - 1
    if best and (best[1] == 0 or addr < best[0] + best[1]):
        return best[2], addr - best[0]
    return "?", 0


found = 0
for off in range(0, len(data) - 3, 4):
    (w,) = struct.unpack_from("<I", data, off)
    op = w >> 26
    if op not in (0x25, 0x05):          # BL = 100101, B = 000101
        continue
    imm = w & 0x03FFFFFF
    if imm & 0x02000000:
        imm -= 0x04000000
    addr = base + off
    if addr + imm * 4 != target:
        continue
    name, delta = owner(addr)
    print("%#010x  %-4s  in %s +%#x" % (addr, "BL" if op == 0x25 else "B", name, delta))
    found += 1
print("-- %d call site(s) --" % found)
