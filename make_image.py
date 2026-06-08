#!/usr/bin/env python3
"""Build the 32 KiB NKCEmu boot image for the USB-BASIC setup.

Layout (the lower 32 KiB the real machine has in EPROM):
    $0000  Grundprogramm 3.1 (USB-enabled)   ../roms/MONITOR.rom
    $2000  free                              ($FF fill)
    $4000  RDK 8K-BASIC 1.3                  ../roms/BASIC.rom
    $6000  BASIC<->USB SAVE/LOAD EPROM       ../basicusb.rom

Run from the nkcemu/ directory after `make` has produced ../basicusb.rom:
    python3 make_image.py
-> writes resources/nkc_usb.bin, loadable with:  ./build/nkcemu -bresources/nkc_usb.bin
"""
import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

SLOTS = [
    (0x0000, os.path.join(ROOT, "roms", "MONITOR.rom")),
    (0x4000, os.path.join(ROOT, "roms", "BASIC.rom")),
    (0x6000, os.path.join(ROOT, "basicusb.rom")),
]
OUT = os.path.join(HERE, "resources", "nkc_usb.bin")

def main():
    img = bytearray(b"\xff" * 0x8000)
    for base, path in SLOTS:
        if not os.path.exists(path):
            sys.exit(f"missing {path} (run `make` in {ROOT} first?)")
        data = open(path, "rb").read()
        if len(data) != 0x2000:
            sys.exit(f"{path}: expected 8192 bytes, got {len(data)}")
        img[base:base + len(data)] = data
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, "wb").write(img)
    print(f"wrote {OUT} ({len(img)} bytes)")

if __name__ == "__main__":
    main()
