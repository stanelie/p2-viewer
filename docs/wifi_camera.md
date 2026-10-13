# The WiFi earpick camera (NK3 / HNDEC, NE7)

What it took to drive this camera from the ESP32-P4. The device protocol itself is a separate
document: **[NK3_camera_protocol.md](NK3_camera_protocol.md)**.

An NK3 "earpick" otoscope: an OPEN 2.4 GHz access point serving JPEG over UDP. Preferred over the
built-in camera, used only when no USB camera is attached.

**Working: 13 fps at 28.7 ms end to end, zero frames dropped at any stage.** The radio comes up on
the onboard ESP32-C6 via `esp_wifi_remote` + `esp_hosted` (the P4 has no radio of its own), the AP
is found by SSID prefix, and the join gives `ip 192.168.1.100 gw 192.168.1.1` - the **gateway is
what identifies NE7**, since the brand string reports NE3 and is wrong. Frames are reassembled from
the 4-byte chunk headers, `FF D9` is appended because the camera omits it, and the P4's hardware
JPEG decoder feeds the existing PPA path.

```
wifi displayed=26  avg_e2e=28715us | frames=639  drops: reasm=0 busy=0 info=0 decode=0
```

## There is NO 5-byte trailer on this unit

The spec describes discarding the last 5 bytes of the final chunk - "a trailer, not image data",
"purpose unknown (plausibly a checksum)" - but tags it **[source]**, read from the upstream
implementation rather than verified on this hardware. On this unit those 5 bytes are **real scan
data**. Stripping them truncated every frame and the hardware decoder rejected ~70% with:

```
jpeg.decoder: the number of data units obtained after decoding a frame of image is different
from the number of data units calculated based on the image resolution configured by the software
```

~30% still decoded, because JPEG can complete when the shortfall happens to land in padding - so a
partially working stream was the misleading symptom. `WIFI_CAM_TRAILER` is 0.

**What found it:** splitting one `dropped` counter into four - `reasm` / `busy` / `info` / `decode`.
One run then read `reasm=0 busy=0 info=0 decode=N`, which cleared the link, the hand-off and the
JPEG header simultaneously and pointed at the decoder, whose own log named the cause. A single
lumped counter cannot distinguish a lossy link from a slow renderer, and those need opposite fixes.

## A dead session never ended - the real cause of "it will not connect"

Switching the camera off and on again appeared to break connection. It was not a connection
problem at all. The session's give-up test read:

```c
if (!s_wifi_frames) {        /* LIFETIME counter - non-zero forever after frame 1 */
    static int giveup = 0;   /* and static: persists across sessions */
    if (++giveup >= 5) break;
}
```

`s_wifi_frames` counts frames for the life of the process, so once one frame had arrived the
branch was **unreachable**. With the camera powered off the session sat re-sending `20 36` into
the void every 2.1 s forever, never returning to the outer loop, so nothing was ever scanning for
the camera when it came back. From the outside that is indistinguishable from a failure to
connect. The `static` on `giveup` was a second latent bug - it would have made a *later*, healthy
session abort almost immediately.

Fixed two ways: a **per-session** stall counter that gives up after `WIFI_CAM_MAX_STALLS` (4)
regardless of history and resets on any traffic, and **direct detection of lost association** -
powering the camera off raises `STA_DISCONNECTED` (beacon timeout), so the session now exits
within ~2 s rather than waiting out stalls.

**Worth recording about the process:** everything in the section below - static IP, join retries,
channel hints, reason codes - was aimed at the *join*, and the join was never broken. Those are
real improvements but they were not the fix, because the failure had not been reproduced first.
One power-cycle reproduced it instantly. Reproduce before fixing.

## The strip fallback must be permanent

The fallback originally re-probed the 307 KB allocation every 600 frames "in case the heap
recovers". It cannot: the fragmentation comes from esp-hosted's SDIO init and persists for the
life of the boot. So the retry could only ever fail, and each failure emitted three driver errors
(`spi_master`, `lcd_panel.io.spi`, `st7796_general`) about twice a minute - which reads as a fault
and is not one. Now permanently sticky: one burst at the moment it engages, then silence. With a
USB camera attached the radio never starts, the heap is never fragmented, and the flag is never
set at all.

## Handover clears the panel

The WiFi branch only wiped the panel when the decoded geometry *changed*, so moving from the
built-in camera's full-screen 480x320 to WiFi's 297x297 circle left the previous image in the
margins. It now clears the whole panel once on entry, re-armed whenever WiFi stops being the
active source, and stripped because the heap is fragmented whenever this path runs.

## Connecting: faster, and failures now say why

Four changes, after "it sometimes just would not connect":

**Static IP instead of DHCP.** The camera's AP is always 192.168.1.0/24 with the camera on .1 -
that is how NE7 is identified in the first place - and its DHCP server is a BK7231U running on a
250 mAh battery. Measured **3.6 s** to issue a lease, on the critical path, with no recourse if it
simply does not answer. Assigning ourselves .100 removes the delay and the failure mode together.
`s_wifi_use_dhcp` flips on permanently if a session ever associates but receives no frames, so a
wrong assumption self-corrects rather than bricking the feature.

**The disconnect reason is logged.** Previously the code polled for an address and, on timeout,
blamed DHCP - which made an association failure indistinguishable from a slow lease, and those
need opposite fixes. It now waits on the `STA_CONNECTED` event and decodes the reason code on
failure. `ASSOC_TOOMANY` in particular is called out as *"AP full - one client only!"*, which is
the most likely explanation for a refused join: the camera accepts exactly one client, so a phone
on the AP locks us out.

**Channel-targeted scan.** The channel is remembered after the first success and scanned alone,
falling back to a full sweep on a miss. The camera does move - observed on ch2, and once ch1 - so
the hint cannot be trusted blindly, but it is several times faster when it holds.

**Join retries before rescanning.** Three attempts, 500 ms apart, before going back to scanning. A
refused association is often transient, and a rescan costs far more than another attempt.

Measured after: camera found and **joined in 2636 ms with no DHCP step**, against a previous path
that spent 3.6 s on the lease alone.

**Honest limit:** the intermittent failure was never reproduced here, so this is not a confirmed
fix for it. What it does is remove one known failure mode, add retries, and make the next failure
diagnosable - the reason code will name it.

## Latency: 37-43 ms of ours, and two stalls removed

Measured per stage, which is the only way the two serialisations showed up:

| stage | before | after |
|---|---|---|
| reassembly (first chunk -> last) | 1-9 ms | 5-11 ms |
| hand-off queue wait | 0.3 - **49 ms** | **0.2 ms** |
| memcpy + JPEG decode | 2.8 ms | 2.8 ms |
| rotate + mask + byte swap | 7.5 ms | 7.5 ms |
| **wait for previous blit** | **~21 ms** | **0.006 ms** |
| SPI blit | 21 ms | 21 ms |
| **total, ours** | **60-113 ms** | **37-43 ms** |

The ~21 ms stall was the same single-buffer serialisation already fixed on the built-in path:
with one output buffer the rotate pass cannot start until the previous blit finishes. Two
alternating buffers remove it, and removing it collapsed the queue wait too, because render_task
now comes back for the next frame before one has finished assembling.

What remains is near-irreducible: 21 ms is SPI wire time for 176,418 bytes at 80 MHz (120 MHz causes
boot hangs - see [platform.md](platform.md)), and the 5-11 ms of reassembly is the
camera spreading one frame over 4-10 datagrams. Against an eyeballed ~300 ms total, **~250 ms is
upstream** - the camera's own exposure, encode and WiFi stack, plus the 78 ms frame interval.

**A caution on measuring this:** the first instrumentation attempt timestamped before the
semaphore take, so a 21 ms stall was being attributed to "rotate" (reported as 31 ms). The stall
was real but mislocated, and splitting `wait` out from `rotate` is what made it obvious.

## There is no faster video mode - established from the vendor APK

Decompiled `com.xiaozhen.beauty.hnd`. It has **no native library**; the whole protocol is Java in
`T0/d.smali`, so nothing is hidden. It sends exactly four payloads:

| payload | port | purpose |
|---|---|---|
| `66 30 01 01` | 46526 broadcast, x2 | discovery (note `0x30`, not the `0x39` the spec had) |
| `86 06 01` | 52219 | start IMU |
| `20 36` | 44506 | start video |
| `20 37` | 44506 | **stop video** - the app logs "send stop rev img cmd." |

Searching that class for `resolution`, `width`, `height`, `fps`, `quality`, `bitrate` or `mode`
returns nothing. **The vendor app cannot change the video mode either**, so 432x432 at ~13 fps is
the hardware's limit.

The APK also carries the NE3 sibling (`T0/f.smali`, port 7080) whose `0x20 0x3X` family is larger,
so the four opcodes NE7 never uses were probed against a live stream 5 s apart, with the decoded
dimensions logged throughout:

| probe | frames in | result |
|---|---|---|
| `20 35` | 61 | 432x432, ~13 fps, no drops, no stall |
| `20 38` | 120 | unchanged |
| `30 66` | 181 | unchanged |
| `30 67` | 247 | unchanged |

All ignored, stream undisturbed. So the ceiling is confirmed from both directions - the app has no
mode control, and the opcodes it does not use do nothing. **Do not re-probe this.**

Two findings from the APK were worth adopting: the discovery byte is now the vendor's `0x30`
(`0x39` also works - the camera is lenient on it), and `20 37` is sent on session teardown to
release the camera, since it accepts only one client and otherwise a phone waits for a timeout. Four corrections were fed back into
[`docs/NK3_camera_protocol.md`](NK3_camera_protocol.md), including that port 58090 does not exist and that
the app does not write `cam_brightness`/`calibrated` as that document had assumed.

## Auto-rotation: the accelerometer reading is the correct one

The camera reports orientation on the IMU channel (port 52219, 24 bytes) and the spec flags which
field carries roll as **disputed**. Both candidates were computed and logged side by side, and
three independent samples settle it:

| `xyz` (int16 BE, bytes 0/2/4) | magnitude | g at 2^14/g | `ANGLE` (u16 BE, 18-19) |
|---|---|---|---|
| -251, -3745, -14644 | 15117 | **0.92** | 75 |
| -130, -9476, 13566 | 16548 | **1.01** | 307 |
| -305, 17027, -1771 | 17122 | **1.05** | 175 |

Every sample reads **~1 g** on the int16 big-endian interpretation, which is what a stationary
3-axis accelerometer must report - confirming the spec's hypothesis on three samples rather than
the one it had. `ANGLE` meanwhile jumps 75 -> 307 -> 175 with no coherent progression, so it is not
image roll. `P2_WIFI_ROTATE` defaults to `WIFI_ROT_ACCEL` (`atan2(x, y)`); set it to
`WIFI_ROT_ANGLE` to use the other field, or `WIFI_ROT_OFF`.

Smoothed with a 15% low-pass and a 2-degree deadband, taking the shortest way round so 359 -> 1
does not spin the long way, plus a gimbal guard that holds the last angle when the lens points
near-vertical and roll becomes undefined.

**PPA cannot rotate by an arbitrary angle** (90-degree steps only), so rotation, scaling and the
ST7796 byte swap are done in one CPU pass, inverse-mapped with nearest sampling and 16.16 fixed
point stepped incrementally along each row. That pass **replaces both PPA transfers** this path
used, so it is partly self-funding. Cost: decode+rotate 13.4 ms, end to end 32.7 ms against
28.7 ms with PPA - still the camera's full 13 fps, with no frames dropped.

The app partition grew 1MB -> 3MB for the WiFi stack, which moves `ffc` and `storage`: a stored
flat-field table is discarded once on update.

