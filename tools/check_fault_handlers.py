#!/usr/bin/env python3
"""Fail unless every fault handler of the image is stack-free up to the MSP reload (T77).

Disassembles each handler with `arm-none-eabi-objdump -d` and requires: (1) the first
instruction is an `ldr` (the _estack literal), (2) an `msr MSP` follows within the first three
instructions, (3) no `push`/`stmdb sp!`/`sub sp` and no `bl`/`blx` (a call would push LR)
appear in the handler body. A compiler change that adds a prologue (a lost `naked`) trips this.
"""
import argparse
import re
import subprocess
import sys

HANDLERS = ["NMI_Handler", "HardFault_Handler", "MemManage_Handler", "BusFault_Handler", "UsageFault_Handler"]
FORBIDDEN = re.compile(r"\b(push|stmdb|stmfd|vpush|bl|blx)\b|\bsub(?:\.w|w)?\s+sp\b")


def body(objdump, elf, name):
    out = subprocess.run([objdump, "-d", f"--disassemble={name}", elf], check=True, capture_output=True, text=True).stdout
    insns = []
    started = False
    for line in out.splitlines():
        if re.match(r"^[0-9a-f]+ <" + re.escape(name) + r">:", line):
            started = True
            continue
        if started:
            if not line.strip():
                break
            parts = line.split("\t")
            if len(parts) >= 3:
                text = " ".join(parts[2:]).split("@")[0].strip()
                if text and not text.startswith(".word") and not text.startswith(".short"):
                    insns.append(text)
    return insns


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf")
    ap.add_argument("--objdump", default="arm-none-eabi-objdump")
    args = ap.parse_args()
    bad = 0
    for name in HANDLERS:
        insns = body(args.objdump, args.elf, name)
        if not insns:
            print(f"error: {name}: not found in {args.elf}", file=sys.stderr)
            bad += 1
            continue
        if not insns[0].startswith("ldr"):
            print(f"error: {name}: first instruction is '{insns[0]}', expected the _estack ldr", file=sys.stderr)
            bad += 1
        if not any(re.match(r"msr\s+msp\b", i, re.I) for i in insns[:3]):
            print(f"error: {name}: no 'msr MSP' in the first three instructions", file=sys.stderr)
            bad += 1
        for i in insns:
            if FORBIDDEN.search(i):
                print(f"error: {name}: stack use / call in handler body: '{i}'", file=sys.stderr)
                bad += 1
    if bad:
        return 1
    print(f"ok: {len(HANDLERS)} fault handlers reload MSP first and use no stack ({args.elf})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
