#!/usr/bin/env python3
"""Fail unless the .noinit section of the image is safe (T77).

The generated linker script has no .noinit rule, so the linker places it as an orphan section.
Checks on the linked ELF (readelf -S / nm): .noinit exists, is NOBITS (no flash image), starts
at or above _ebss (the startup zeroes .bss only up to _ebss, so .noinit survives a reset) and
ends at or below _sstack (it cannot alias the stack region).
"""
import argparse
import re
import subprocess
import sys


def symbol(nm, elf, name):
    out = subprocess.run([nm, elf], check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == name:
            return int(parts[0], 16)
    return None


def noinit_section(readelf, elf):
    out = subprocess.run([readelf, "-S", "-W", elf], check=True, capture_output=True, text=True).stdout
    for line in out.splitlines():
        m = re.search(r"\]\s+(\.noinit)\s+(\S+)\s+([0-9a-f]+)\s+[0-9a-f]+\s+([0-9a-f]+)", line)
        if m:
            return m.group(2), int(m.group(3), 16), int(m.group(4), 16)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf")
    ap.add_argument("--nm", default="arm-none-eabi-nm")
    ap.add_argument("--readelf", default="arm-none-eabi-readelf")
    args = ap.parse_args()

    sec = noinit_section(args.readelf, args.elf)
    ebss = symbol(args.nm, args.elf, "_ebss")
    sstack = symbol(args.nm, args.elf, "_sstack")
    if sec is None or ebss is None or sstack is None:
        print(f"error: .noinit, _ebss or _sstack not found in {args.elf}", file=sys.stderr)
        return 2
    kind, addr, size = sec
    bad = 0
    if kind != "NOBITS":
        print(f"error: .noinit is {kind}, expected NOBITS (it would take flash and be re-initialised)", file=sys.stderr)
        bad += 1
    if addr < ebss:
        print(f"error: .noinit starts at {addr:#x}, below _ebss {ebss:#x} (the startup would zero it)", file=sys.stderr)
        bad += 1
    if addr + size > sstack:
        print(f"error: .noinit ends at {addr + size:#x}, above _sstack {sstack:#x} (aliases the stack)", file=sys.stderr)
        bad += 1
    if bad:
        return 1
    print(f"ok: .noinit NOBITS {addr:#x}..{addr + size:#x}, _ebss {ebss:#x}, _sstack {sstack:#x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
