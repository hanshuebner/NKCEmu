# NKCEmu — SDL2 + emulated VDIP1 USB stick

A fork of Torsten Evers' [NKCEmu](https://github.com/Nightwulf/NKCEmu)
(NDR-Klein-Computer emulator: Z80 + GDP64 + KEY + ROA64), with two additions
for working on the `basicusb` BASIC⇄USB project (the `nkc-load-save`
repository, expected as a sibling directory `../nkc-load-save`):

1. **Ported from SDL 1.2 to SDL2** (SDL 1.2 no longer builds on current
   macOS/Linux). Rendering now goes through an `SDL_Window` / `SDL_Renderer` /
   streaming `SDL_Texture`; keyboard input uses `SDL_TEXTINPUT` + `SDL_KEYDOWN`.
2. **An emulated VDIP1 (FTDI VNC1L) USB card on I/O port `$30`** — the piece
   the upstream emulator lacked — so the Grundprogramm's USB routines, and
   hence `CALL 24576/24579` from BASIC, actually transfer files.

## Build

Needs SDL2 and cmake (`brew install sdl2 cmake`):

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Run

```sh
./build/nkcemu -bresources/full.bin        # stock ROM set, windowed
```

Windowed is the default now; pass `-F` for fullscreen. **F1** toggles
fullscreen at runtime, **F10** toggles mouse grab, **F12** is NMI-to-$0000.

### The USB-BASIC image

`resources/nkc_usb.bin` is the 32 KiB boot image with the USB-enabled
Grundprogramm 3.1, RDK 8K-BASIC 1.3, and the `basicusb` EPROM at `$6000`.
Rebuild it after assembling `basicusb.rom` (`make` in `../nkc-load-save`):

```sh
python3 make_image.py            # reads ../nkc-load-save (or $NKC_BASIC_DIR)
./build/nkcemu -bresources/nkc_usb.bin
```

From the NKC monitor start BASIC, then in BASIC:

```
CALL 24576    save current program  (prompts FILE:)
CALL 24579    load a program        (prompts FILE:)
```

## The emulated USB stick is a local directory

Files SAVEd from BASIC become real files on the host, and LOAD reads them
back. The "stick" is just a directory:

* default: `./usb/` (created on first SAVE),
* override with the `NKC_USB_DIR` environment variable.

```sh
NKC_USB_DIR=/path/to/myfiles ./build/nkcemu -bresources/nkc_usb.bin
```

Set `NKC_USB_DEBUG=1` to trace VDAP commands (OPW/WRF/CLF/RDF) on stderr.

## How the card works (`vdip.c`)

Port `$30` is bit-banged SPI: `bit0`=MISO, `bit1`=MOSI, `bit2`=SCLK (idle
high, one pulse per bit on the rising edge), `bit6`=/RESET, `bit7`=/CS. Each
transfer clocks a start bit, an R/W bit, a register-select bit, 8 data bits
(MSB first) and an ack bit. On top of that `vdip.c` implements the VNC1L's
VDAP short command set the monitor uses — `OPW`/`WRF`/`CLF` for writing and
the streaming read command for loading — mapping each to `fopen`/`fread`/
`fwrite` against the directory above. See the header comment in `vdip.c` for
the exact wire protocol, which was reverse-engineered from the Grundprogramm
disassembly (`$1992..$1B13`).

## Testing without the GUI

The bundled z80pack monitor (`>>>` prompt on stdin) is a full debugger and
makes the card scriptable without touching the keyboard. For example, driving
the real monitor SAVE routine and checking the resulting host file:

```sh
NKC_USB_DEBUG=1 ./build/nkcemu <<'EOF'
e resources/nkc_usb.bin
m 87ec
54
45
53
54
00
.
b 602a
g 6009
q
EOF
```
