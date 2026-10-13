"""Disassemble named functions out of an AArch64 shared object.

Usage: python -I disas.py <lib.so> <symbol> [symbol ...]
       python -I disas.py <lib.so> --list [filter]
"""
import sys

from capstone import CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN, Cs
from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

path = sys.argv[1]
args = sys.argv[2:]

with open(path, "rb") as fh:
    elf = ELFFile(fh)

    syms = {}
    for sec in elf.iter_sections():
        if isinstance(sec, SymbolTableSection):
            for s in sec.iter_symbols():
                if s["st_info"]["type"] == "STT_FUNC" and s["st_value"]:
                    syms[s.name] = (s["st_value"], s["st_size"])

    if args and args[0] == "--list":
        needle = args[1].lower() if len(args) > 1 else ""
        for n in sorted(syms):
            if needle in n.lower():
                print("%-60s %#010x %6d" % (n, syms[n][0], syms[n][1]))
        raise SystemExit

    # Map a virtual address to file data via the section containing it.
    def read(addr, size):
        for sec in elf.iter_sections():
            a = sec["sh_addr"]
            if a and a <= addr < a + sec["sh_size"] and sec["sh_type"] != "SHT_NOBITS":
                off = addr - a
                return sec.data()[off:off + size]
        return b""

    md = Cs(CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN)
    for name in args:
        if name not in syms:
            print("!! %s not found" % name)
            continue
        addr, size = syms[name]
        size = size or 0x200
        print("\n==== %s  @%#x  (%d bytes) ====" % (name, addr, size))
        for ins in md.disasm(read(addr, size), addr):
            print("  %#08x  %-8s %s" % (ins.address, ins.mnemonic, ins.op_str))
