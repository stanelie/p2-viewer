# p2-viewer

A handheld thermal camera, built out of a USB thermal module and a cheap ESP32 board with a
touchscreen. Plug the camera into the board, and you get a live thermal image with a colour
palette, a crosshair temperature readout and a few touch buttons, with no phone and no app in
the middle.

It started as a viewer for the InfiRay P2 Pro, which is where the name comes from. It has since
grown into a general-purpose camera viewer: if it recognises a thermal camera you get the full
thermal interface, and if you plug in something else - a webcam, a borescope, an otoscope - you
get a plain picture instead. There is also a WiFi otoscope that it finds and connects to on its
own, and the little camera on the board itself as a last resort, so the screen is never blank.

The whole thing is one C file against ESP-IDF. No LVGL, no RTOS abstractions over the top; the
display is driven straight through `esp_lcd`, because the three things I cared about were boot
time, framerate and latency, in that order. It boots to a picture in about 1.7 seconds and runs
the P2 Pro at its full 25 fps.

## Hardware

- **Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5** - 480x320 SPI touchscreen, USB OTG host, battery
  charger. The P4 is the part that matters: it can be a USB host, which is what makes any of
  this possible.
- A camera (see below). The USB one goes in the OTG port.
- Optionally the OV5647 module that ships on the board's MIPI-CSI connector.

One warning before you build one: **the camera cannot run off the internal battery without a
hardware modification.** Both Type-C VBUS pins share a net that feeds the PMIC's charger input,
and that PMIC has no boost converter, so there is no 5 V to give the camera. On battery you need
an added 3.7 -> 5 V boost module.

## Cameras it supports

| camera | connection | what you get |
|---|---|---|
| **InfiRay P2 Pro** | USB-C | full thermal UI, 25 fps, absolute temperatures, shutter control |
| **GOYOJO GW192A** | USB | same thermal UI, 96x96 sensor, temperatures good to about ±1.5 °C |
| **Any UVC webcam or borescope** | USB | plain full-screen preview, MJPEG where the camera offers it |
| **NK3 / HNDEC "earpick" otoscope** | WiFi | auto-joins its access point, rotates the image upright |
| **OV5647 on the board** | MIPI-CSI | fallback preview when nothing else is attached |

They are tried in that order of priority: a USB camera wins, then the WiFi otoscope, then the
board's own camera. You can swap between USB cameras without resetting the device.

Thermal cameras are recognised by what they advertise, not by their USB IDs, so a thermal camera
I have never seen may well just work - and if it does not, it degrades to a normal picture
rather than to garbage.

## Installing it

Grab the `.bin` from the [latest release](https://github.com/stanelie/p2-viewer/releases/latest)
and write it at offset 0:

```bash
esptool.py --chip esp32p4 -b 460800 write_flash 0x0 p2-viewer-v1.3.1-esp32p4.bin
```

Use the **UART** Type-C port, not the OTG one, so the camera port stays free. That single file
contains the bootloader, partition table and app. For a clean device, run
`esptool.py --chip esp32p4 erase_flash` first.

Coming from a release older than v1.3.0, your stored flat-field table will be discarded once,
because the app partition grew and everything after it moved. Just capture it again.

## Building it yourself

You need **ESP-IDF v5.5 or newer**. Not optional: this board's P4 is silicon revision v3.2 and
v5.3.x refuses to flash it outright.

```bash
idf.py set-target esp32p4
./tools/patch_managed_components.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Don't skip the patch script. It fixes four real bugs in Espressif's managed components - an
uninitialised struct that breaks auto white balance on our chip revision, and three limits in
the UVC host that block vendor control transfers. `managed_components/` is gitignored and gets
recreated on every fresh clone, so the script is the honest way to carry those fixes rather than
hand-editing files that will silently come back. It is idempotent and it explains each patch as
it applies it.

## Using it

Eight buttons down the side of the thermal view:

| button | does |
|---|---|
| `MAX+` `MAX-` `MIN+` `MIN-` | nudge the temperature scale when it is locked |
| `FFC` | flat-field correction: aim at something uniform, press, wait about a second |
| `CLR` | the same button once a correction is active - clears it |
| `AS1` / `AS0` | let the camera run its own periodic shutter calibration, or suppress it |
| `IRON` / `GRAY` | colour palette |
| `AUTO` / `LOCK` | auto-range the scale to the scene, or hold it where it is |

Holding the power button for one second shuts the device down.

The FFC table is saved to its own flash partition and restored at boot, so a correction survives
a power cycle. This is worth doing because the camera's own shutter sits behind the lens and
therefore cannot see dust on the window or lens vignetting, which a wall-based correction can.

## Notes and findings

The detail lives in [`docs/`](docs/). Most of it is measurement rather than documentation - a
good deal of this was reverse-engineered, and the notes exist so neither I nor anyone else
repeats a dead end.

| | |
|---|---|
| [The thermal path](docs/thermal_cameras.md) | how two different thermal cameras share one pipeline, flat-field correction, the shutter toggle |
| [P2 Pro protocol](docs/p2pro_protocol.md) | stream format, the three cold-start phases, cutting the startup from 9 s to 5 s, the vendor command channel |
| [GW192A frame layout](docs/GW192A_frame_layout.md) | the rebadged HIKMICRO module, its frame map, its temperature calibration, the extension unit I recovered but cannot yet get data out of |
| [Generic USB cameras](docs/generic_uvc_cameras.md) | mode selection, hardware JPEG decode, hot-swap |
| [The WiFi otoscope](docs/wifi_camera.md) | joining its AP, reassembling its UDP frames, auto-rotation from its accelerometer, latency |
| [NK3 camera protocol](docs/NK3_camera_protocol.md) | the device protocol itself, as a standalone spec |
| [The built-in camera](docs/builtin_camera.md) | OV5647 through the P4's ISP, why the sensor mode matters more than anything host-side |
| [Board and platform](docs/platform.md) | boot time, flash layout, the power button, the display bug that bit every path |
| [The borescope USB saga](docs/usb_link_investigation.md) | a long hunt that ended at a connector, with the PHY measurements it produced on the way |

Two known limits worth knowing before you build one. **The screen tears and it cannot be fixed
on this board** - there is no TE signal to sync to, and the tear forms in the panel's own memory,
so no amount of buffering on our side helps. And the thermal image is about 8% vertically
stretched, because the PPA's scale factors land on sixteenths.

## Credit

The P2 Pro's vendor command transport was worked out by
[LeoDJ/P2Pro-Viewer](https://github.com/LeoDJ/P2Pro-Viewer); a copy of the reference
implementation is kept in [`docs/P2Pro_cmd_reference.py`](docs/P2Pro_cmd_reference.py). The NK3
protocol work builds on haxko/NE3-Scope, with corrections from this hardware noted in the spec.
