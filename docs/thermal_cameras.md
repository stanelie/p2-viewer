# The thermal path: two cameras, one pipeline

Both supported thermal cameras hand over a 16-bit radiometric plane inside an ordinary UVC
frame, so everything downstream of the parse is shared. This covers that shared design, the
per-model table, the flat-field correction and the auto-shutter toggle.

## Measured performance (P2 Pro)

Streams 256x384 YUY2 at a stable **25 fps** (the sensor's ceiling), **~36.9 ms**
arrival-to-on-screen, **~1.71 s** from power-on to first image and **~6.4 s** to a fully
calibrated one (down from ~11.8 s). Time-to-calibrated is reported on the console every stream
open.

## Two thermal cameras, one path

The GW192A is supported through the *same* code as the P2 Pro, not a parallel implementation.
Both deliver a 16-bit radiometric plane inside an ordinary UVC frame, so everything downstream
of the parse — flat-field, min/max scaling, palettes, crosshair, readouts, touch OSD, manual
scale — is shared, and only the four numbers that actually differ live in a table
(`s_thermal_models` in `main/p2pro_uvc.c`):

| | InfiRay P2 Pro | GOYOJO GW192A |
|---|---|---|
| stream format | YUY2 256x384 | YUY2 96x176 |
| radiometric plane | 256x192 at byte 98304 (bottom half) | 96x96 at byte 0 |
| counts | Kelvin x 64 | 51.714 counts/K, intercept -10262 (measured) |
| panel image | 416x312 (1.625x) | 288x288 (3x) |
| vendor control channel | yes (InfiRay, driven) | HIKMICRO extension unit, found but not yet driven |

Detection stays capability-based, never VID:PID: a camera is thermal because it advertises one
of those stacked formats. Anything else still gets the generic preview.

Two consequences for the GW192A, both inherent to the device rather than this firmware:

- **No shutter control, no forced flat-field, no device temperatures — yet.** The InfiRay
  opcodes are not fired at it, which is a safety measure as much as a limitation: aiming them at
  a camera that does not implement them makes it STALL EP0 on every request, and the retry loop
  becomes a control-transfer storm that drowns the stream. The firmware's own frame-derived
  flat-field still works, re-keyed per sensor geometry.

  Decompiling the vendor app showed the GW192A is a **rebadged HIKMICRO MiniE/P09-class module**
  (`37b4:0102`), and the camera does expose a HIKMICRO UVC **extension unit** — unit 10, GUID
  `{A29E7641-DE04-47E3-8B2B-F4341AFF003B}`, ten control selectors — which would give a forced
  flat-field, emissivity and distance correction, the device's own temperature scaling and its
  model string. **The protocol is now recovered** by disassembling `libHCUSBSDK.so`: control
  selector 5 is a `{selector, sub-function}` switch, CS 6 is a 1-byte status register, and all
  415 of the SDK's commands have been mapped to their selector and sub-function pair — forced
  flat-field is CS 2 sub 0x04, thermometry params are CS 3 sub 0x01. The transport is
  **implemented and runs** (`P2_GW_XU_READ`, default 0): the latch, the length query, the read and
  the status register all succeed and the camera reports success. It still returns no data —
  every one of nine commands across five selectors answers with the same two bytes — so one
  framing step is still missing, and a USB capture of the vendor app would settle it. Full
  write-up in [`GW192A_frame_layout.md`](GW192A_frame_layout.md).
- **Absolute temperatures are calibrated from two measured points, over a short baseline.**
  A matte wall at 23.0 °C reads 5053 counts and a palm at 33.5 °C reads 5596, giving 51.714
  counts/K with the intercept at -10262; the wall then reads back as 23.7 °C on the panel. The
  10.5 K baseline is the weak part — half a degree of error on either reference moves the slope
  ~7%, so expect roughly ±1.5 °C at body temperature and worse at the extremes. A third
  reference far from these (ice or boiling water) would tighten it;
  [`GW192A_frame_layout.md`](GW192A_frame_layout.md) has the procedure and the arithmetic.

A square sensor costs panel width: 288 is the only multiple of 32 a 96-pixel source can reach
at or below the 320-row panel, so the GW192A gets a 192px control bar against the P2 Pro's
64px. The layout is searched now rather than hardcoded, and reproduces the P2 Pro's hand-tuned
416x312 exactly.

Full measured protocol, including the frame map, the marker words and the modes that will not
negotiate: **[GW192A_frame_layout.md](GW192A_frame_layout.md)**.

## Flat-field correction button

Aim at something thermally uniform and press FFC. It averages 32 frames (~1.3 s) and applies each
pixel's deviation from the frame mean.

**The table is persisted** to a dedicated 64 KB `ffc` partition (48 KB of `int8` offsets plus magic,
version, pixel count and CRC32) and restored at boot. It is rejected if the CRC fails or it came
from a different geometry, so a corrupt table cannot silently degrade the image, and `CLR` erases
it. The side-bar button reads `CLR` whenever a correction is active - including a restored one - so
it is never applied invisibly.

This is worth persisting because a wall-based correction fixes something the camera structurally
cannot. The shutter sits *behind* the lens, so the camera's own FFC is blind to anything in the
optical path - a speck on the window, vignetting - which therefore survives every shutter cycle. A
wall-based capture sees the whole path. That component is a physical property of the camera and is
identical on every power-up.

**The camera's own shutter is NOT used as the reference, although it was tried.** Mechanically it
is the obvious choice - `shutter_manual_switch` (`0x420c`) closes it and it presents a field with
only ~110-130 counts of spread. But the shutter has a thermal gradient of its own, so a table
captured from it encodes that gradient and corrects visibly worse than a plain uniform surface at a
distance. Reverted after testing on hardware.

**Cost, and why it is built the way it is.** Applying the offset is per-pixel work inside a 40 ms
frame budget that already has only ~3 ms spare:

| implementation | convert | e2e |
|---|---|---|
| separate corrected-copy pass (PSRAM) | 13.9 ms | 90-110 ms, runaway backlog |
| folded into the existing passes, `int16` | 12.0 ms | 39.4 ms, stable |
| folded, `int8` offsets (current) | 11.3 ms | 38.3 ms, stable |

Materialising a corrected copy costs read-raw + read-offset + write-copy + re-read-copy, all PSRAM.
Folding the offset into the loops that already read each pixel removes three of those four streams.
The table is `int8` because 1 count is 1/64 K, so +/-127 covers +/-2 C - ample for FPN residual -
and halving the table halves what the per-pixel loop reads.

**Do not move these buffers to internal SRAM.** It makes the apply pass cheaper but starves the USB
stack, which needs internal DMA memory for its ISOC transfers: the camera drops out with a
continuous `Frame buffer underflow`. Measured, not theorised.

**The 5x7 font only carries the characters the UI happens to need.** A missing glyph renders as a
hollow box (deliberately - it used to render blank, which made `CALIBRATING` silently appear as
`CALI RATING` and `FFC` as two squares). Adding a label means checking its characters exist.

## Auto-shutter toggle (AS1 / AS0)

The camera runs its own flat-field every `MAX_INTERVAL` (60 s), closing the shutter and briefly
freezing the image. The **AS1 / AS0** button suppresses that: `PROP_SWITCH` (auto-shutter param 0)
written to 0. Verified by read-back - unlike `MIN_INTERVAL`, which the camera refuses, this write
is accepted.

**It is only ever applied after the camera's first calibration completes.** That startup
calibration is itself an auto-shutter event (the `PREVIEW_START` delays in [p2pro_protocol.md](p2pro_protocol.md)), so disabling it any
earlier means the camera never runs a flat-field and the image stays permanently uncorrected. The
clean-frame detector is what marks the right moment.

Defaults to **ON** (the camera's own behaviour) and is not persisted, so every boot returns to the
camera keeping fixed-pattern noise corrected. Either state is applied only after that first
calibration.

**The trade-off is real.** Bolometer fixed-pattern noise drifts as the sensor warms, which is
exactly why the camera re-corrects periodically. With AS0 the image will slowly degrade over
minutes, and temperature readings with it. The FFC button is the manual compensation, and AS1 puts
the camera back in charge.

