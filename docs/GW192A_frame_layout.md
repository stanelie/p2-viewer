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

## What the camera really is, and the vendor channel it has

Decompiling the GOYOJO vendor app (`com.guardExpert.thgStartGoogle`, "THG Start") settled what
this device is. Its device table groups the GW192A with HIKMICRO hardware:

```
F0:   "MiniE", "P09", "HM-TJ10-1ARF", "HM-TJ10G-1ARF",
      "HM-TJ33-10G-1ARF", "Finder S1", "Finder S1 Pro", "GW192A"
F2V2: ... "Finder S2", "Finder S2 Pro", "GW340"
```

So the **GW192A is a rebadged HIKMICRO MiniE/P09-class module** (HIKMICRO is Hikvision's thermal
brand), and the app drives it with the Hikvision `HCUSBSDK`. The MiniE is a 96x96 core, which
matches the plane measured above and settles the 192x192 marketing claim: it is interpolated.

The device identifies as **`37b4:0102`**, USB 2.0 high speed, class 239 (misc/IAD).

### It does have a vendor control channel

The earlier conclusion in this document — that no vendor channel exists — was **wrong**. The
camera exposes a UVC **Extension Unit**, read straight off its own descriptors by
`probe_dump_xu()` (build with `P2_USB_PROBE 1`):

```
XU unit_id=10 on interface 0  bNumControls=15  bNrInPins=1
GUID  {A29E7641-DE04-47E3-8B2B-F4341AFF003B}
bControlSize=4  bmControls=ff 03 00 00     -> control selectors 1..10 implemented
```

That is the same mechanism the SDK uses: `libHCUSBSDK.so` exports
`AccessDeviceDriver::ParseExtensionUnit(... _USB_GUID ...)`, which finds the unit by GUID and
returns its ID, and `CUsbDeviceVideoBase::ControlTransfer` / `SwitchCommand`, which take a
`tagXU_COMMAND_HEADER`. The SDK's own log format string names the fields:

```
byRequestType[0x%02x], byRequest[0x%02x], wValue[0x%04x],
Sub-function[0x%02x], wIndex[0x%04x], dwDataLen[%d], iLen[%d]
```

So: ordinary UVC XU addressing (`wValue = CS << 8`, `wIndex = (unit << 8) | interface`, which is
`0x0a00` here) wrapping a command header that carries a command ID and a sub-function byte. The
command IDs are plain integers in the Java bindings, and the ones that matter to a viewer are:

| ID | command | what it would buy us |
|---|---|---|
| 2025 | `USB_SET_IMAGE_MANUAL_CORRECT` | **forced flat-field** — the P2 Pro's shutter equivalent |
| 2030 / 2031 | `USB_GET/SET_THERMOMETRY_BASIC_PARAM` | emissivity, distance, reflective temperature, range, **and `byFloatTransMode`/`byShiftLength`: the device's own value scaling** |
| 2110 / 2111 | `USB_GET/SET_THERMAL_STREAM_CTRL` | enable/disable the radiometric stream |
| 2038 / 2039 | `USB_GET/SET_THERMAL_STREAM_PARAM` | stream coding type |
| 2131 / 2132 / 2133 | `USB_GET/SET_THERMAL_STREAM_YUV_RESOLUTION(_CAPABILITIES)` | possibly the modes that will not negotiate (96x216, 96x322, NV12 96x96) |
| 2047 | `USB_GET_ROI_MAX_TEMPERATURE_SEARCH` | device-side hotspot search |
| 2011 | `USB_GET_SYSTEM_DEVICE_INFO` | model string — real identification instead of inferring it from advertised formats |
| 2026 / 2027 | `USB_GET/SET_IMAGE_ENHANCEMENT` | device-side contrast and denoise |
| 2115 | `USB_SET_IMAGE_PALETTE_DATA` | upload a palette |

`USB_THERMOMETRY_BASIC_PARAM` is the interesting one for this document, because it carries
`byShiftLength` and `byFloatTransMode` — the HIKMICRO convention for sending floats as scaled
integers. If that can be read, the camera states its own scaling, and the measured
51.714 counts/K above stops being a two-point fit and becomes a cross-check.

### What stopped us, and what is fixed

Our own stack was the first wall. `usb_host_uvc` 2.6.0 allocates the endpoint-0 transfer at
64 bytes and then validates every `uvc_host_usb_ctrl()` call as
`data_buffer_size >= wLength + sizeof(usb_setup_packet_t)`, so the largest vendor payload it
could carry was **56 bytes**. Every selector on this camera reports a length of 512, so a
`GET_CUR` failed inside the component before a packet was ever sent. `GET_INFO` (1 byte) and
`GET_LEN` (2 bytes) worked, which is what made the limit visible rather than mysterious.

`tools/patch_managed_components.sh` now raises that allocation to 576 bytes. It is harmless for
the P2 Pro, whose commands are a few bytes, and it is a prerequisite for anything in this
section. **Run the patch script after any component re-download**, or XU work fails confusingly.

That is as far as this has been taken. With the bigger buffer the read-only enumeration returns
**unstable** answers — on one pass CS 5 reported `info=0x03, len=512` and on the next
`info=0x32, len=2`, while selectors 7..10 moved from `0x00` to `0x07` — which is the expected
behaviour of a stateful mailbox: reading a command buffer that has had no command written into it
is not a defined operation. The numbers from that enumeration are therefore **not** a usable map
of the channel, and should not be quoted as one. The disassembly below explains exactly why —
selector and sub-function are latched by a separate "switch" control and the SDK even caches
them, so an unswitched read answers for whatever command was selected last.

### The protocol, recovered from libHCUSBSDK.so

Disassembling the SDK's AArch64 build (`tools/hcusbsdk/disas.py`, capstone + pyelftools) recovered
the whole exchange, and it has since been **implemented and run against the camera**. The
addressing, the selector latch, the length query and the status register all behave as described
below. The request framing does not yet produce data — see "Implemented, and exactly where it
stops" at the end of this section before building on any of it.

#### Addressing

Plain UVC extension-unit control transfers:

```
wValue = control_selector << 8
wIndex = (unit_id << 8) | interface      /* 0x0A00 on this camera */
GET:  bmRequestType 0xA1, bRequest 0x81 (GET_CUR)
SET:  bmRequestType 0x21, bRequest 0x01 (SET_CUR)
```

`CUsbDeviceVideoBase::ControlTransfer` fills `wIndex` itself from the unit it cached at
`ParseExtensionUnit` time; only `wValue` and the direction come from the command.

#### Two selectors are reserved

| CS | role |
|---|---|
| 5 | **command switch** — a 2-byte mailbox, `{ target_CS, sub_function }` |
| 6 | **status / last error** — 1 byte: `1` = busy, `0` = success, anything else = error |

Across all 415 command objects in the SDK, **no command ever targets CS 5 or CS 6**, which
corroborates that they are infrastructure rather than data. Our probe measured CS 6 as a 1-byte
control, matching `GetUVCLastError`.

#### The sequence

`ProtocolStrategyUvc20::ReadDataStrategy` / `WriteDataStrategy` do this:

1. **Switch** (`CUsbDeviceVideoBase::SwitchCommand`) — `SET_CUR` on **CS 5**, 2 bytes
   `{ target_CS, sub_function }`.

   This is **cached**: `HoldCommand(wValue, sub_function)` compares against the last pair it
   selected and returns 0 — skipping the transfer entirely — when nothing changed. That is
   exactly why our own read-only enumeration gave different answers on consecutive passes: with
   no switch written, every selector was still answering for whatever command had been selected
   before, so the numbers drifted with history rather than describing the selectors.

2. **Transfer** — `GET_CUR` or `SET_CUR` on the **target CS**, same `wIndex`, carrying the
   payload.

3. **Poll** (`ProtocolStrategyUvc20::CheckDeviceStatus`) — `GET_CUR` on **CS 6**, 1 byte. While
   it reads `1`, sleep 10 ms and retry, up to the 5 s timeout the SDK passes.

Reads longer than one control transfer are chunked, and each chunk comes back with a **5-byte
header**: the SDK requests `chunk + 5`, checks that exactly that arrived, and copies from offset
5 onward. So usable payload per transfer is `wLength - 5`.

#### The 11-byte command prefix

Internally the SDK passes commands around as a buffer whose first 11 bytes are the prefix and
whose remainder is the payload (`ReadDataStrategy` writes `pCmd + 11` with length
`cmdLen - 11`, and rejects any `cmdLen <= 10`):

```
+0  u8   bmRequestType      0xA1 get / 0x21 set
+1  u8   bRequest           0x81 GET_CUR / 0x01 SET_CUR
+2  u16  (reserved, zero)
+4  u16  wValue             control_selector << 8
+6  u16  wIndex             filled in by the device layer
+8  u8   3                  constant in every command inspected
+9  u8   flag               0 on the get path, 0x88 on the set path
+10 u8   sub_function       what gets written to CS 5
+11 ...  payload
```

That 8-byte head is `tagXU_COMMAND_HEADER`, and it is memcpy'd straight into the driver's
transfer descriptor at offset 0x18, where the fields line up one-for-one with
`libusb_control_transfer(handle, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout)`.

#### Command map

The SDK implements each command as a template instantiation
`CUsbCommandConfig<ID, InputStruct, OutputStruct>::Excute()`, and each one stores its own
constants into that prefix. `tools/hcusbsdk/cmdmap.py` walks every instantiation and reads the
selector and sub-function back out — 415 commands in total. GET and SET of the same item share a
`(CS, sub)` pair and differ only in direction, which is the design the prefix implies.

The ones a thermal viewer wants:

| ID | command | CS | sub |
|---|---|---|---|
| 2011 | `GET_SYSTEM_DEVICE_INFO` | 1 | 0x01 |
| 2025 | `SET_IMAGE_MANUAL_CORRECT` (forced FFC) | 2 | 0x04 |
| 2026 / 2027 | `GET/SET_IMAGE_ENHANCEMENT` | 2 | 0x05 |
| 2080 | `GET_IMAGE_ENHANCEMENT_EX` | 2 | 0x05 |
| 2105 / 2115 | `GET/SET_IMAGE_PALETTE_DATA` | 2 | 0x15 |
| 2119 | `SET_IMAGE_MULTI_PALETTE_DATA` | 2 | 0x1a |
| 2030 / 2031 | `GET/SET_THERMOMETRY_BASIC_PARAM` | 3 | 0x01 |
| 2032 / 2033 | `GET/SET_THERMOMETRY_MODE` | 3 | 0x02 |
| 2038 / 2039 | `GET/SET_THERMAL_STREAM_PARAM` | 3 | 0x05 |
| 2047 | `GET_ROI_MAX_TEMPERATURE_SEARCH` | 3 | 0x0a |
| 2048 | `GET_P2P_PARAM` | 3 | 0x0b |
| 2063 / 2064 | `GET/SET_ENVIROTEMPERATURE_CORRECT` | 3 | 0x13 |
| 2110 / 2111 | `GET/SET_THERMAL_STREAM_CTRL` | 3 | 0x14 |
| 2131 / 2132 | `GET/SET_THERMAL_STREAM_YUV_RESOLUTION` | 3 | 0x1f |
| 2087 | `GET_SYSTEM_DEVICE_INFO_CAPABILITIES` | 8 | 0x11 |
| 2133 | `GET_THERMAL_STREAM_YUV_RESOLUTION_CAPABILITIES` | 8 | 0x3c |
| 2068 / 2069 | `GET/SET_VIDEO_CODE` (max frame rate) | 10 | 0x01 |

All of those selectors are inside the 1..10 this camera advertises. `tools/hcusbsdk/command_map.txt` has
the full 415. Two cautions about that file: the SDK serves card readers and CDC devices as well
as cameras, so plenty of entries are irrelevant here; and a handful of extracted selectors come
out above 10 (11, 23, 24, 25 and one absurd 415), which are either other device families or
mis-parses of the heuristic, and should not be trusted without checking the function by hand.

#### Implemented, and exactly where it stops

The transport is in `main/p2pro_uvc.c` (`xu_ctrl`, `xu_select`, `xu_data_len`, `xu_wait_ready`,
`xu_read`) behind `P2_GW_XU_READ`, which **defaults to 0**. It works, and it is not enough.

**What works, measured on the camera:**

- the selector latch — `SET_CUR` on CS 5 with `{ target_CS, sub_function }` — is accepted;
- `GET_LEN` on the target selector answers 512;
- `GET_CUR` on the target selector completes;
- the status register on CS 6 reads **0, success**, after every exchange.

**What does not work:** the camera answers *every* command with the same two bytes, `00 02`, and
nothing behind them. That was checked across nine commands on five different selectors —
`SYSTEM_DEVICE_INFO`, `DEVICE_INFO_CAPABILITIES`, `THERMOMETRY_BASIC`, `THERMOMETRY_MODE`,
`THERMAL_STREAM_CTRL`, `THERMAL_STREAM_YUV_RES`, `YUV_RES_CAPABILITIES`, `VIDEO_CODE`,
`IMAGE_ENHANCEMENT` — and all nine returned byte-for-byte the same thing. A response that does
not vary with the command is not a response.

Note `0x0200` is 512, the same number `GET_LEN` reports, so the likeliest reading is that the
device is handing back a length rather than a payload: the command is latched and acknowledged,
but the body was never produced. The missing step is almost certainly a `SET_CUR` of a request
block on the target selector *before* the read — `CUsbDeviceVideoBase::CommonDeviceConfig` sets
up two 512-byte buffers and branches on whether the command has an input structure, and that
branch is where the request layout lives. It was not chased further.

#### Two wrong turns worth recording

Both cost a build each and both looked like the camera's fault when they were ours:

1. **Guessing the read length.** Asking for the advertised 512 bytes failed, and the natural
   reading was "the device stalled". It had not. `GetDataLen()` shows the SDK never picks a read
   length — it asks the device with `GET_LEN` and assigns the answer straight into the transfer.
   Worth knowing even though, on this camera, the answer is always 512.

2. **A short IN read is not an error.** `usb_host_uvc` enforces
   `actual_num_bytes == num_bytes` and reports anything less as `ESP_ERR_INVALID_RESPONSE`,
   which is indistinguishable from a real fault at the call site. A short control IN is legal
   USB; libusb returns the real count, which is why the vendor SDK never had to care, and why
   its 4-byte `GET_LEN` read works there and fails here against a device that answers the
   standard 2 bytes. `tools/patch_managed_components.sh` now relaxes that check for IN transfers
   only — a short *write* is still a fault — and zeroes the payload area first so a caller never
   sees stale bytes past what arrived.

   This one matters beyond the GW192A: it applies to any vendor control request on any camera.

#### Finishing it

The cheapest remaining route is unchanged and now better motivated: **capture the Android app
talking to the camera** (USBPcap on Windows, or `usbmon` on a rooted phone). One known-good
exchange would show the request block the device is waiting for, and the transport to replay it
is already written and tested. The alternative is to keep reading
`CommonDeviceConfig` and the `tagUSB_DEVICE_CONFIG_CONTEXT` it takes.

Worth saying plainly: it is also possible the GW192A implements none of this. It is a consumer
camera carrying an SDK written for industrial modules, the thermometry API it exposes is a
superset of anything a £40 phone accessory needs, and "acknowledges the command, returns no
data" is consistent with a module that answers the plumbing and implements nothing behind it.
The capture would settle that too — if the vendor app gets no more out of it than we do, there
is nothing there to get.

### Reproducing the decompile

The app is not on F-Droid and the APK has to be supplied. With a JDK and
[jadx](https://github.com/skylot/jadx) on the path:

```bash
unzip -o app.apk 'classes*.dex' 'lib/arm64-v8a/lib*.so' -d apk_x
jadx --no-res --no-debug-info -d out apk_x/classes2.dex   # HCUSBSDK + f1/f2 module bindings
jadx --no-res --no-debug-info -d out3 apk_x/classes3.dex  # the device model tables
```

`classes2.dex` holds `com.hcusbsdk.*` (command IDs, structures) and `com.hik.f2module.*` (the
F0/F2 wrapper the GW192A goes through); `classes3.dex` holds the model-to-module-type table
quoted above. The native side worth reading is `libHCUSBSDK.so` for the transport and
`libusbCam_host.so`, a much smaller HAL carrying `hal_usbcam_host_shutter_ctrl`,
`hal_usbcam_host_get_cavity_temperature` and `hal_usbcam_host_pass_through_read/write`.

## What this camera does not have

No *InfiRay* control channel. The InfiRay opcodes are not fired at it — the model
table's `vendor_cmds` flag is false, and `p2_camera_init()` is skipped — which matters for
more than tidiness: aiming those requests at a camera that does not implement them makes it
STALL EP0 on every one, and the retry loop turns into a control-transfer storm that drowns
the stream (observed for real against an iMac webcam, see the generic-camera notes).

The camera does have a vendor channel of its own — a HIKMICRO extension unit — which this
firmware does not yet drive. See the section above.

So **as the firmware stands today** there is no shutter scheduling, no forced flat-field, and no device
temperature readout. The camera runs its own shutter on its own schedule and we take what it
gives. Everything that does not need the vendor channel works: palette, manual and automatic
scale, crosshair, min/max/centre readouts, the touch OSD, and the firmware's own
flat-field capture, which is computed from the frames themselves and is re-keyed per sensor
geometry so a 256x192 table is never applied to a 96x96 plane.
