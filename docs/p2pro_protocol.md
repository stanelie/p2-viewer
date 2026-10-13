# InfiRay P2 Pro: stream format, cold start and command channel

How the P2 Pro behaves on the wire, what its three startup phases look like, how the
nine-second cold start was cut to five, and the vendor command channel behind UVC.

The P2 Pro is a plain UVC device — no vendor driver needed to stream. It advertises YUY2 256x192
and 256x384; the 384-row format is **preview on top, raw radiometric in the bottom half**. Raw
counts are **Kelvin x 64**:

```
celsius = raw / 64.0 - 273.15
```

## Cold-start behaviour (measured)

Plugging in a cold camera goes through three phases, timed from stream open:

| phase | window | signature |
|---|---|---|
| flat sentinel | 0 – 2.6 s | every radiometric pixel is `0x8000` |
| uncorrected | 2.9 – 8.75 s | real data, ~53x spatially noisier, ~54 °C spread |
| clean | 8.75 s on | flat-field applied, ~11 °C spread |

Phase 2 is uncorrected bolometer output (per-pixel fixed-pattern noise, no flat-field yet). It is
full-length and inside a plausible temperature band, so neither a frame-length check nor the
`0x8000` check catches it — it just looks garbled.

The firmware labels these honestly, which took a correction: it originally showed **CALIBRATING**
through phase 1, when the camera is merely booting and the shutter has not fired, and then let the
genuinely-uncalibrated phase 2 show through as a garbled picture. Now phase 1 shows **STARTING**
and phase 2 shows **CALIBRATING**, which is the phase the flat-field actually lands at the end of.
The phase-2 test is bounded to the startup window and stops once a clean frame is seen, so a
high-contrast scene can never trigger a spurious overlay later.

A useful metric for telling these apart is the mean absolute difference between horizontally
adjacent raw pixels: ~0 flat, **~320 uncorrected, ~6 clean**.

## Cutting the cold start (the auto-shutter delays)

The uncorrected phase is **not** a missing FFC command - it is configuration. The camera schedules
its own shutter events, and `get/set_prop_auto_shutter_params` (`0x8214` / `0xc214`) exposes the
schedule. Factory defaults as read from the camera:

| id | name | default |
|---|---|---|
| 0 | `PROP_SWITCH` | 1 (auto-shutter on) |
| 1 | `MIN_INTERVAL` | 5 |
| 2 | `MAX_INTERVAL` | 60 |
| **9** | **`PREVIEW_START_1ST_DELAY`** | **5** |
| **10** | **`PREVIEW_START_2ND_DELAY`** | **4** |
| 11 | `CHANGE_GAIN_1ST_DELAY` | 5 |
| 12 | `CHANGE_GAIN_2ND_DELAY` | 4 |

Units are seconds, and 5 + 4 = 9 matches the measurement exactly. The firmware writes both to 1 as
soon as the command channel opens, which brings a usable image forward from **~9.35 s to ~5.4 s
after device connect** (9349/9366 ms vs 5350/5425 ms over repeated cold plugs). The two early
shutter clicks the official phone app produces are these same two events with shorter delays
configured - there is no manual-FFC opcode, which is why other projects looking for one did not
find it.

Measure this from **device connect**, not from stream open: the channel-up time varies by ~900 ms
run to run, so the "after stream open" figure moves around while the total stays within ~75 ms.

The schedule must be rewritten **on every stream open**. The camera forgets it whenever it loses
power, so latching "already configured" after the first success silently restores the stock 5s+4s
on every subsequent replug.

Verified cold-plug timing with the schedule rewritten: **5216 / 5218 ms** from device connect to a
calibrated image, against **9349 / 9366 ms** stock.

Three things that do **not** work, all measured:
- **Configuring before preview starts.** The delays count from preview start, so writing earlier
  should move the first shutter - but the command channel does not answer until preview is
  running, so there is no window in which to do it.
- **Lowering `MIN_INTERVAL`.** The camera rejects the write with a status error and it reads back
  5 unchanged.
- **Forcing the shutter** with `shutter_manual_switch` (`0x420c`, 0=OPEN / 1=CLOSE) the moment the
  channel answers, instead of waiting for the auto logic. 5337/5281 ms vs 5350/5425 ms baseline:
  no gain. The ~1.5 s after the trigger is the flat-field operation itself, not scheduling
  latency, so there is nothing to bring forward — and it blanks the image for ~800 ms.

  The actuator does work (close gives a uniform field, spread 132; open restores it), so it is
  available if a manual FFC is ever wanted. **A closed shutter reads as a smooth, low-roughness
  frame** — roughness ~7, indistinguishable from a calibrated image on roughness alone — so any
  "is it calibrated yet" test must also require a realistic min/max spread. Without that, the
  detector fires mid-shutter and reports a calibration that has not happened.

**These values do not survive the camera losing power** - a cold camera reads back 5/4 - so the
firmware reapplies them on every connect. Nothing is permanently written to the camera.

## Where the remaining time goes

Measured on a true cold boot (board and camera both unpowered), timestamps are ms since power-on:

| segment | cost | whose |
|---|---|---|
| power-on -> camera enumerated | 1009 ms | ours |
| -> first image on screen | 700 ms | camera (SET_INTERFACE) |
| -> command channel answers | 2832 ms | **camera** |
| -> schedule written | 140 ms | ours |
| -> flat-field complete | 1520 ms | **camera** |
| **total** | **6209 ms** | |

The command channel answers **~4.5 s after the camera receives power** - 4549 ms from power-on here,
and ~4.2 s after plug-in across the replug tests, despite very different host timings in the two
cases. So it is gated by the camera's own boot, not by when the host starts preview.

**Further boot optimisation will not improve time-to-calibrated.** The first image is already on
screen at ~1.7 s, 2.8 s before the camera will talk to us at all. Cutting host boot further makes
the first frame appear sooner but leaves the usable image at ~6.2 s. About 4.35 s of that 6.2 s is
the camera's own boot plus its flat-field.

Opcodes and parameter ids were extracted from `libircmd.so` in the official Android APK:
`readelf --dyn-syms` lists ~155 JNI exports whose names are the SDK API, disassembly gives the
opcode and packing, and `baksmali` on `classes.dex` gives the enum semantics
(`CommonParams$PropAutoShutterParameter`).

## Vendor command channel

Beyond UVC, the camera carries an InfiRay command interface behind two vendor control requests.
Transport reverse-engineered by [LeoDJ/P2Pro-Viewer](https://github.com/LeoDJ/P2Pro-Viewer); a copy
of the reference implementation is in [`P2Pro_cmd_reference.py`](P2Pro_cmd_reference.py).

- write `bmRequestType 0x41, bRequest 0x45, wValue 0x78`; read `0xC1, 0x44, 0x78`
- **ready check** at `wIndex 0x200` — must be polled to idle after *every* command
- 8-byte header at `wIndex 0x1d00`: cmd LE(2) + param LE(4) + length **big-endian**(2)

Reads work and are safe (`get_device_info`, `cur_vtemp`, `prop_tpd_params`). The channel opens
~2.93 s after stream start on a cold camera, and instantly on a warm one.

> **Do not send start/stop mode commands speculatively.** `y16_preview_start` (`0x010a`) and
> `preview_start` (`0xc10f`) are state-changing. Sending them left the camera emitting raw clustered
> just above `0x8000` at 27.5 fps — a compressed bottom half and an orange top half — and the mode
> **survived a replug**. `y16_preview_stop` does not undo it (returns a status error); only
> physically unplugging the camera recovered it.
>
> Never send `spi_transfer` (`0x8201`, writes camera flash) or `sys_reset_to_rom` (`0x0805`).

Note `preview_start 0xc10f` already contains the `0x4000` SET bit — don't blindly OR it. Invalid
commands do **not** USB-stall; they return a camera status error in the ready byte.

