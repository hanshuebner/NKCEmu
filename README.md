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
3. **An emulated SER2 R2 serial card (XR88C681 DUART) at base `$90`** — see
   *Serial (SER2)* below.

## Serial (SER2)

The SER2 R2 card (a 2681/68681-type dual UART) is emulated at I/O base `$90`
(ports `$90..$9F`). Its two channels are each exposed as a **TCP listening
socket on localhost**, so you can attach a terminal with e.g. `nc 127.0.0.1
2681`:

| Channel | Default TCP port | Override env var      |
|---------|------------------|-----------------------|
| A       | 2681             | `NKCEMU_SER2_PORTA`   |
| B       | 2682             | `NKCEMU_SER2_PORTB`   |

* **Buffering:** while no client is connected, bytes the NKC transmits are
  held (up to 1 MiB, oldest-preserved) and flushed when you connect — so
  output produced right after start-up is not lost before you attach.
* **Interrupts:** the DUART's `INTRN` is modelled as wired to the Z80 `/INT`
  line, so an interrupt-driven driver (unmask RxRDY in IMR, `IM 1`) works; the
  guest vectors to `$0038` on a received byte. This needs `WANT_INT`, now
  enabled in `sim.h`.
* The register model is faithful enough to run `SER2.BAS` (the probe in the
  `nkc-bbc-basic` repo): counter liveness, MR read-back, IVR scratch,
  on-chip local loopback, and the ISR/IMR interrupt-logic test.

The card lives in `ser2.c`; it is serviced (sockets + interrupt line) every
~2 ms from the CPU loop in `sim1.c`.

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
* override with the `NKC_USB_DIR` environment variable,
* or one or more `-u DIR` options.

**Merging directories.** Pass `-u` more than once and the stick is the *union*
of those directories — reads search them in order (first match wins), writes
(`SAVE`) and the directory listing's new files go to the first. This lets a
project keep its build output and its example programs separate yet present them
as one stick, with no copying or symlinks:

```sh
./build/nkcemu -b ../nkc-bbc-basic/build/boot.rom \
               -u ../nkc-bbc-basic/build \
               -u ../nkc-bbc-basic/examples
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
