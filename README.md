# p2-viewer

A pocket thermal camera made from a USB thermal module and a cheap ESP32 board with a
touchscreen. Plug the camera in and you get a live thermal image, a colour palette, a crosshair
that tells you how hot the thing you're pointing at is, and a few buttons down the side. No
phone, no app, no cloud account that stops working in two years.

It started as a viewer for the InfiRay P2 Pro, which is where the name comes from. Then I
plugged in a borescope to see what would happen, and it turned into a viewer for whatever camera
I own. It now knows about five of them.

## It takes almost any camera you plug into it

Thermal cameras get the full instrument: palette, auto-ranging temperature scale, flat-field
correction, crosshair readout.

- **InfiRay P2 Pro** over USB-C, at its full 25 fps
- **GOYOJO GW192A** over USB, which turned out to be a rebadged HIKMICRO module

Anything else gets a plain full-screen picture, which is enough to be useful:

- **any USB webcam or borescope**, using MJPEG and the P4's hardware JPEG decoder where it can
- **an NK3 "earpick" otoscope over WiFi**, which has no USB connector at all - it serves video
  from its own access point, so the viewer finds the network, joins it and starts streaming
  without being asked. It also reports its own orientation, so the image is rotated upright as
  you twist it.
- **the little OV5647 camera** on the board's ribbon connector, as a fallback, so the screen is
  never just blank

They're tried in that order: USB first, then WiFi, then the onboard camera. Swap USB cameras
while it's running and it follows along without a reset.

Nothing is matched on USB IDs - a camera is thermal because of what it advertises about itself.
So a thermal camera I've never seen stands a decent chance of just working, and if it doesn't,
you get a normal picture instead of garbage.

It boots to a live image in about 1.7 seconds, which took some doing and is written up in the
notes.

## Getting it on the device

Grab the `.bin` from the [latest release](https://github.com/stanelie/p2-viewer/releases/latest)
and write it at offset 0, using the **UART** Type-C port so the other one stays free for a
camera:

```bash
esptool.py --chip esp32p4 -b 460800 write_flash 0x0 p2-viewer-v1.3.1-esp32p4.bin
```

That one file is the bootloader, partition table and app together. For a clean device, run
`esptool.py --chip esp32p4 erase_flash` first.

You'll want a **Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5**: 480x320 touchscreen, USB host, battery
charger, and an ESP32-P4, which is the part that makes this possible at all - it can act as a USB
host, which most hobbyist microcontrollers can't.

## Using it

Eight buttons down the side:

| button | does |
|---|---|
| `FFC` | point at something uniform, like a wall, and press. Takes about a second |
| `CLR` | the same button, once a correction is loaded - throws it away |
| `AS1` / `AS0` | let the camera do its own periodic shutter calibration, or stop it |
| `IRON` / `GRAY` | palette |
| `AUTO` / `LOCK` | follow the scene's temperature range, or hold it still |
| `MAX±` `MIN±` | nudge the ends of the scale while it's locked |

Hold the power button for a second to shut down.

Your flat-field correction is saved and comes back after a power cycle, which is worth doing:
the camera's own shutter sits *behind* the lens, so it can't see dust on the window or the lens's
own vignetting. Pointing at a wall can.

## Building it yourself

ESP-IDF v5.5 or newer - not optional, v5.3.x refuses to flash this board's chip revision.

```bash
idf.py set-target esp32p4
./tools/patch_managed_components.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Don't skip the patch script; it fixes four genuine bugs in Espressif's components that this
firmware needs fixed, and it explains each one as it goes. It's all one C file against ESP-IDF,
no LVGL, with the display driven straight through `esp_lcd`, because what I cared about was boot
time, framerate and latency in that order.

## Before you build one

Three things I'd want to know in your position:

- **The camera can't run off the internal battery** without adding a 3.7 -> 5 V boost module.
  Both USB-C VBUS pins feed the PMIC's charger input, and that PMIC has no boost, so there's
  simply no 5 V to hand out.
- **The screen tears, and it can't be fixed on this board.** There's no sync signal to wait for,
  and the tear forms inside the panel's own memory, so buffering on our side does nothing.
- The thermal image is about 8% vertically stretched, because the scaler works in sixteenths.

## The notes

Most of what's interesting is in **[docs/](docs/)**. A fair amount of this hardware was
reverse-engineered, so those files are mostly measurements, and they include the things that
didn't work - the point of writing a dead end down is that nobody walks into it twice. There's a
nine-second cold start cut to five, a camera that turned out to be a different brand underneath,
a USB fault I chased through the SoC's analogue PHY registers before finding it in a connector,
and a sensor mode that's both more pixels and 2.7x faster than the one that looked right.

[docs/README.md](docs/README.md) is the index.

## Credit

The P2 Pro's command transport was worked out by
[LeoDJ/P2Pro-Viewer](https://github.com/LeoDJ/P2Pro-Viewer), and a copy of the reference
implementation lives in [`docs/P2Pro_cmd_reference.py`](docs/P2Pro_cmd_reference.py). The NK3
work builds on haxko/NE3-Scope, with corrections from this particular unit noted in the spec.
