# GOYOJO GW192A over UVC — measured protocol

Everything here was measured on the real device: a GW192A on the OTG port of the
Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5, dumped by the `P2_GW_PROBE` block in
`main/p2pro_uvc.c` (set it to 1, flash, read the console). No vendor documentation
exists for this camera, and the only prior public work
([kuczy/GOYOJO-GW192A-Thermal-Camera](https://github.com/kuczy/GOYOJO-GW192A-Thermal-Camera))
reports a preview only, with no temperature path — "no direct temperature reading".

The short version: **it is a plain UVC device with no vendor control channel, and one of
its six advertised modes carries a 96x96 plane of 16-bit radiometric counts.** That is the
same shape of data as the P2 Pro's bottom half, which is why it reuses the P2 Pro's whole
render path rather than getting one of its own.

## What it advertises

Six formats, all at exactly 25 fps (`dwFrameInterval` 400000, a single discrete entry):

| # | format | size | opens? | what is in it |
|---|---|---|---|---|
| 0 | YUY2 | 96x176 | **yes** | radiometric + telemetry + NV12 preview — the one we use |
| 1 | YUY2 | 96x100 | yes | radiometric + telemetry, no preview |
| 2 | YUY2 | 96x194 | yes | as 0, plus a header block — but does not frame stably |
| 3 | YUY2 | 96x216 | no | `ESP_ERR_NOT_FOUND` from `uvc_host_stream_control()` |
| 4 | YUY2 | 96x322 | no | same |
| 5 | NV12 | 96x96 | no | same |

Modes 3, 4 and 5 are advertised but cannot be negotiated: the UVC component fails to find
them in the streaming descriptors even when asked for the device's own default interval
(`fps = 0`), so this is not the usual "you asked for a rate it does not have" failure. They
are not needed, so they have not been chased further.

"96 pixels wide" is a fiction of the YUY2 wrapper. YUY2 is 2 bytes per pixel, so every mode
is really **192 bytes per row**, and the payload is 16-bit words or 8-bit pixels depending
on which part of the frame you are in. Nothing in the frame is actually YUV.

## Mode 0 (96x176) layout

192 bytes per row, 176 rows. Measured `data_len` is 33826 against the 33792 the geometry
implies — 34 bytes of slack, harmless, and the plane we read sits at the very front.

| rows | bytes | content |
|---|---|---|
| 0..96 | 0..18449 | **96x96 radiometric plane, uint16 little-endian**, word-stuffed (see below) |
| 96..99 | 18450..19199 | telemetry block, not yet decoded |
| 100..171 | 19200..33023 | NV12 96x96 visible image: Y plane (48 rows of 192B = 96x96), then UV at 0x8080 (24 rows) |
| 172..175 | 33024..33791 | zero padding |

Note that the plane is 9225 words — 18450 bytes — because of the stuffing described below,
so it runs 18 bytes past row 95 and into the start of row 96. Reading it as a clean 18432-byte
block ending on a row boundary is wrong by exactly nine pixels.

The NV12 preview is **deliberately unused**. It is the camera's own auto-contrast picture;
palette-mapping the 16-bit plane ourselves gives the same resolution with a scale we
control, and it is exactly what the P2 Pro path already does.

Mode 1 (96x100) is this frame with the preview and padding removed — rows 0..95 plus the
same 4 telemetry rows. Useful if the bus ever needs the 14KB back.

### Stuffed marker words

The plane is **word-stuffed**. Every 1001st word is a marker holding `0x8002` rather than a
pixel, and it is *inserted* into the stream rather than written over a pixel:

```
1000 pixels, 1 marker, 1000 pixels, 1 marker, ...
```

So a 96x96 plane spans **9225 words, not 9216**, and the nine markers land at word indices
1000, 2001, 3002, 4003, 5004, 6005, 7006, 8007 and 9008 — those with
`i % 1001 == 1000` — which read as:

```
r10,c40  r20,c81  r31,c26  r41,c67  r52,c12  r62,c53  r72,c94  r83,c39  r93,c80
```

They have to be **removed and the rest closed up** (`destuff_plane()`), not masked in place.
Masking was the first implementation and it looked almost right, which is the trap: every
marker pushes the pixels after it one place along, so the image comes out in horizontal
bands each shifted one pixel right of the band above. A vertical edge renders as a staircase
with a step every ~10.7 sensor rows — about 32 rows on the 3x panel. A one-pixel-per-1000
shift is invisible in a byte dump and obvious on the glass, so this was caught by looking at
the picture, not the numbers.

De-stuffing is nine straight `memcpy` runs of 2000 bytes rather than a modulo test per pixel,
which costs microseconds; everything downstream then indexes a plain dense 96x96 image and
needs no knowledge of the stuffing. With it in place the firmware's own marker count reports
`0 of 9216 pixels >= 0x8000`, which is the regression test for this.

The `invalid_at` value mask (32768 counts = 2048 K at 16 counts/K, so unreachable) is kept as
a belt-and-braces guard in case a future camera firmware moves the stride. Left in *without*
de-stuffing it is actively harmful in a second way too: the markers pinned the auto-scale's
maximum at 32770 for ever, squashing the real scene into the bottom fifth of the palette and
rendering it almost black.

## Mode 2 (96x194) — why it is only a fallback

Mode 2 is mode 0 with a 24-row block added, and its plane reads about **420 counts lower**
than mode 0's on an unchanged scene: 4674..4692 against 5168..5176, which is the difference
between a believable 19-20 °C indoor reading and an impossible 50 °C one. That made it look
like the calibrated mode.

It is not usable as it stands. On hardware the frame does not assemble at a fixed offset:

- the `0x8002` marker words move between frames rather than holding their stride;
- zeros appear inside what should be the plane, pinning the scene minimum at 0;
- the scene maximum swings frame to frame — 5219, 32141, 9278, 5788, 14981, 28334;
- the probe saw `data_len` 36932 against an expected 37248, so frames arrive short.

Its row 0 does carry what looks like a real header — `73 77 82 70 1c 12 00 00 06 00 00 00
04 90 00 00` and, in a later run, the magic-looking `aabb ccdd c179 e1bb` — so there is
probably a documented structure here worth decoding. Until the framing is understood, a
believable number from an unstable frame is worth less than a steady picture, so the
firmware prefers mode 0 and fixes the reading with the `counts_bias` term instead.

## Temperature scale

Counts are linear in temperature:

```
celsius = (raw - counts_bias) / counts_per_kelvin - 273.15
```

Both constants are **measured on hardware** against a reference thermal camera:

| target | reference reading | GW192A centre 3x3 |
|---|---|---|
| matte wall | 23.0 °C | 5053 counts (steady to ±2) |
| human palm | 33.5 °C | 5596 counts (steady to ±4) |

543 counts over 10.5 K:

```
counts_per_kelvin = 543 / 10.5      = 51.714
counts_bias       = 5053 - 51.714 * 296.15 = -10262      (0 °C sits at 3864 counts)
```

Verified back on the panel: the same wall then reads 23.7 °C.

### Getting this wrong first is instructive

Earlier builds carried `counts_per_kelvin = 16`, inferred from magnitude alone — an indoor
scene reads ~5170, and 5170/16 = 323 K, which is the right *order* for a camera whose counts
are a fixed multiple of Kelvin. It was wrong by more than 3x, and reported a 23 °C wall as
50 °C. Order of magnitude is not a calibration; only two known temperatures are.

Note also that the 96x194 mode's apparently more plausible readings (19-20 °C where 96x176
said 50 °C) were a **coincidence of the wrong scale**, not evidence that mode 2 was the
calibrated one. Under the real 51.714 counts/K, 96x176 was always the right plane.

### Accuracy caveat

A 10.5 K baseline is short. Half a degree of error on either reference moves the slope by
about 7% — 47 to 57 counts/K — which is roughly ±1.5 °C at body temperature and worse further
out. The true slope may well be a round 51.2, or 64 as on the P2 Pro.

To tighten it, build with `P2_GW_CALIB 1` (the default today), aim the camera at a reference
far from these two — ice water at 0 °C, or just-boiled water near 100 °C — and read the
`GWCAL centre=` counts off the console. Then with any two well-separated points
`(raw_1, T_1)` and `(raw_2, T_2)` in °C:

```
counts_per_kelvin = (raw_2 - raw_1) / (T_2 - T_1)
counts_bias       = raw_1 - counts_per_kelvin * (T_1 + 273.15)
```

Put both in the model table and set `P2_GW_CALIB` to 0. Note that emissivity matters for the
references: water and matte paint are both ~0.95-0.96 and fine, bare metal is not.

The `GWCAL` line scans its own min/max/mean over the current frame rather than reusing the
render pass's cached auto-scale range. That cache is refreshed every `MINMAX_PERIOD` frames,
which is right for driving a palette and wrong for calibration — against the 23 °C wall it
reported a range of 5068..5491 while the centre read 5053, a centre *below* the frame
minimum, which cannot happen within one frame and made the numbers untrustworthy.

## What this camera does not have

No vendor control channel is known. The InfiRay opcodes are not fired at it — the model
table's `vendor_cmds` flag is false, and `p2_camera_init()` is skipped — which matters for
more than tidiness: aiming those requests at a camera that does not implement them makes it
STALL EP0 on every one, and the retry loop turns into a control-transfer storm that drowns
the stream (observed for real against an iMac webcam, see the generic-camera notes).

So on the GW192A there is **no** shutter scheduling, no forced flat-field, and no device
temperature readout. The camera runs its own shutter on its own schedule and we take what it
gives. Everything that does not need the vendor channel works: palette, manual and automatic
scale, crosshair, min/max/centre readouts, the touch OSD, and the firmware's own
flat-field capture, which is computed from the frames themselves and is re-keyed per sensor
geometry so a 256x192 table is never applied to a 96x96 plane.
