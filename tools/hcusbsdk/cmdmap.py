"""Extract the (control selector, sub-function) pair each CUsbCommandConfig<ID> encodes.

Each instantiation builds an 11-byte prefix on the stack:
    +0 bmRequestType  +1 bRequest  +4 wValue(=CS<<8)  +6 wIndex  +8 3  +9 flag  +10 sub-function
so the selector is the halfword stored at prefix+4 and the sub-function is the byte at prefix+10.
Both are plain `mov wN, #imm` followed by a store, which is what this tracks.

Usage: python -I cmdmap.py <lib.so>
"""
import re
import struct
import sys

from capstone import CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN, Cs
from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

SYM = re.compile(r"CUsbCommandConfigILi(\d+)E(.*?)6ExcuteEv$")

with open(sys.argv[1], "rb") as fh:
    elf = ELFFile(fh)
    syms = {}
    for sec in elf.iter_sections():
        if isinstance(sec, SymbolTableSection):
            for s in sec.iter_symbols():
                if s["st_info"]["type"] == "STT_FUNC" and s["st_value"]:
                    syms[s.name] = (s["st_value"], s["st_size"])

    sections = [(s["sh_addr"], s.data()) for s in elf.iter_sections()
                if s["sh_addr"] and s["sh_type"] != "SHT_NOBITS"]

    def read(addr, size):
        for a, d in sections:
            if a <= addr < a + len(d):
                return d[addr - a:addr - a + size]
        return b""

    md = Cs(CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN)
    rows = []
    for name, (addr, size) in syms.items():
        m = SYM.search(name)
        if not m:
            continue
        cid = int(m.group(1))
        imms = {}
        cs_sel = sub = off_h = None
        for ins in md.disasm(read(addr, size or 0x200), addr):
            if ins.mnemonic == "mov" and ins.op_str.count(",") == 1:
                a, b = (x.strip() for x in ins.op_str.split(","))
                if b.startswith("#"):
                    try:
                        imms[a] = int(b[1:], 0)
                    except ValueError:
                        pass
            elif ins.mnemonic in ("strh", "strb"):
                mm = re.match(r"(w\w+), \[sp, #(0x[0-9a-f]+|\d+)\]", ins.op_str)
                if not mm:
                    continue
                reg, o = mm.group(1), int(mm.group(2), 0)
                val = imms.get(reg)
                if val is None:
                    continue
                if ins.mnemonic == "strh" and val and (val & 0xFF) == 0 and cs_sel is None:
                    cs_sel, off_h = val >> 8, o
                elif ins.mnemonic == "strb" and off_h is not None and o == off_h + 6:
                    if sub is None:
                        sub = val
        if cs_sel is not None:
            rows.append((cid, cs_sel, sub, m.group(2)))

rows.sort()
print("%-6s %-3s %-5s %s" % ("ID", "CS", "sub", "signature"))
for cid, cs_sel, sub, sig in rows:
    print("%-6d %-3d %-5s %s" % (cid, cs_sel, "0x%02x" % sub if sub is not None else "?", sig[:70]))
print("-- %d commands --" % len(rows))
