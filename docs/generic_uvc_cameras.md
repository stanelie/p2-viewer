# Generic USB cameras (preview mode)

Mode selection, the MJPEG path through the hardware JPEG decoder, the measured steady state, and
hot-swapping between camera types.

Any UVC camera that is not a P2 Pro gets a plain full-screen preview instead of the thermal
interface. Detection is on **capability, not VID:PID**: the P2's signature is that it advertises
**256x384** YUY2 - preview stacked on raw radiometric, which no ordinary webcam offers. Anything
else is treated as a generic camera, which also means a P2 left in image-only mode degrades to a
picture rather than a garbled thermal parse.

The generic path costs almost no CPU: one PPA pass converts YUY2 to RGB565 *and* scales
(`PPA_SRM_COLOR_MODE_YUV422_YUYV` is a supported input mode), and a second does the byte swap the
ST7796 needs. The swap cannot be folded into the first pass because PPA's `byte_swap` affects how
it *reads*.

Resolution is chosen as the largest YUY2 mode that fits the panel. The panel is only 480x320, so a
larger capture buys no visible detail and costs isochronous bandwidth. Both axes share one 1/16
scale factor so the aspect ratio is exact - picking each axis's own best fit would stretch the
picture, since PPA quantizes them independently.

Worked example, the camera tested here (Sunplus `1bcf:2085`, an auricular scope):

| | |
|---|---|
| advertises | MJPEG and YUY2, each at 640x480 / 640x360 / 320x240 / 1280x720 |
| speed | high speed (480 Mbps), largest isoc endpoint MPS 1020 -> **~8.16 MB/s** |
| chosen | **320x240 YUY2 @ 25 fps** = 3.84 MB/s |
| rejected | 640x360 needs 11.5 MB/s, 640x480 needs 15.4 MB/s, 1280x720 needs 46 MB/s |
| scaled to | 420x315 at (30,2), aspect 1.3333 = exactly 4:3 |

Check the **negotiated speed** before doing this arithmetic. At full speed the packet rate is
1000/s rather than 8000/s, giving ~1 MB/s, and no YUY2 mode would be viable at 25 fps - the design
would have to go through MJPEG and the P4's hardware JPEG decoder instead.

**Verified working** against an iMac webcam: `352x288 -> 374x306`, **5.3 ms** convert and
**29.0 ms** end-to-end, no PPA errors and no underflows. The conversion is cheaper than the
thermal path (5.3 ms vs 11.3 ms) because the PPA does the colour conversion in hardware.

Two things had to be fixed before any frame appeared, and both are easy to misdiagnose:

**1. `CONFIG_UVC_CHECK_PAYLOAD_HEADER_EOH=n`.** Some cameras do not set the End-of-Header bit in
the UVC payload header, and the component rejects every such payload (`uvc-frame: EOH bit not
set`), so no frame ever completes even though the stream opened and data is arriving. It is a
Kconfig option, so this is not a patch to `managed_components` and survives a component upgrade.
The P2 sets the bit correctly and is unaffected.

**2. `out.buffer_size` must be cache-line aligned, not just the buffer address.** PPA validates
the size too, and declaring the *used extent* fails whenever `w*h*2` is not a multiple of 64 -
374x306 gives 228888, which is 24 over, and every pass failed with `ESP_ERR_INVALID_ARG` giving a
**black screen**. The thermal path only ever worked because 416x312x2 = 259584 happens to divide
by 64. `ppa_out_bytes()` rounds the extent up, which leaves the thermal sizes byte-identical.

The driver names the cause exactly - `E ppa_core: out.buffer addr or out.buffer_size not aligned
to cache line size` - so read the PPA tag's own errors before theorising about alignment of the
input.

**P2 vendor commands must be thermal-only.** `p2_camera_init()` ran after every stream open, so it
fired InfiRay vendor control requests at the webcam, which correctly stalled every one
(`USBH: Dev 1 EP 0 STALL`) - and its channel-availability probe retries for 15 s at 25 ms
intervals, a control-transfer storm that drowns the stream it is meant to precede.


## Generic cameras use MJPEG with hardware JPEG decode

A generic (non-thermal) camera is now asked for **MJPEG** when it advertises it, and the frames go
through the ESP32-P4's hardware JPEG decoder (`esp_driver_jpeg`, `SOC_JPEG_DECODE_SUPPORTED`) and
then the existing PPA scale and byte-swap passes. YUY2 is the fallback. The thermal path is pinned
to YUY2 - the raw radiometric half must never be compressed.

Measured on the borescope at 320x240, 15-20 frames per mode:

| format | bytes/frame |
|---|---|
| MJPEG 320x240 | **22,824** (19,184 - 24,468) |
| MJPEG 640x480 | 77,600 |
| YUY2 320x240 | 153,600 |
| YUY2 640x480 | 614,400 |

~6.7x less bus traffic for the same picture, decoded in hardware rather than on the CPU. Set
`P2_GENERIC_PREFER_MJPEG 0` to go back to YUY2.

Two traps worth recording. The decoder must be configured
`JPEG_DEC_RGB_ELEMENT_ORDER_BGR`: its "RGB" order writes components in the opposite byte order to
the RGB565 PPA produces from YUY2, and with `..._RGB` the picture arrives with red and blue
traded. And a frame that lost packets mid-stream is still delivered (the component only validates
the SOI, and we have the payload error check disabled), so it turns up as a JPEG with **no EOI** -
checking for the `FF D9` terminator separates link losses (`truncated`) from real decoder failures
(`decode_err`), which is what made the diagnosis above possible.


## Generic cameras: mode choice is oversampled, not fitted

The mode is picked as **the largest advertised mode up to `P2_GEN_OVERSAMPLE` (2) x the panel in
each axis**, not the largest that fits inside it. On the 480x320 panel that lands on 640x480,
downscaled to 400x300.

Why, measured on hardware with the two candidates cycling on the panel:

| preset | source | on screen | result |
|---|---|---|---|
| 320x240 scale | 22.8 KB/frame | 420x315 (**upscaled**) | softer |
| **640x480 scale** | 77.6 KB/frame | 400x300 (**downscaled**) | **chosen - visibly sharper, no discernible fps or latency difference** |
| 640x480 crop | 77.6 KB/frame | 480x320 1:1 | **rejected - clearly worse latency** |

The old rule took the largest mode that *fit* the panel, which meant upscaling 320x240 and
discarding detail the camera was happy to supply. Downscaling from a larger capture is sharper at
the same framerate, and on this camera it costs nothing measurable.

The cap is the important half of the rule. Past ~2x the panel the extra pixels cannot be seen and
are not free: this camera's 1280x720 MJPEG wants 46 MB/s and 4x the JPEG decode of 640x480 for no
visible gain, so the cap rejects it (1280 > 960) while accepting 640x480 (640 <= 960, 480 <= 640).

Two notes on geometry. 400x300 rather than something closer to full panel is PPA's 1/16 scale grid:
10/16 gives 400x300 and 11/16 would give 440x330, whose 330 rows do not fit 320. Filling the panel
completely from a 4:3 source would mean cropping it to 3:2 first (640x427 at 12/16 = 480x320
exactly), which loses ~11% of the top and bottom - bad for a borescope's circular image, so it is
not done.

**`640x480 crop` was built, measured and removed.** 1:1 centre crop has identical bus traffic and
decode cost to `640x480 scale`, but blits 480x320 = 153600 px against the letterbox's 132300, and
the SPI blit is this board's bottleneck. Cropping to a *smaller* 1:1 box (420x315) would equalise
SPI and decode while keeping native pixels, but narrows the field of view to 66% per axis - that is
a zoom feature, not a quality setting.

**Framerate has no headroom to find.** Every mode this camera offers is capped at 25 fps
(`dwFrameInterval = 400000`); only 1280x720 MJPEG offers alternatives and they all go *downward*
(25/20/15/10/5), and YUY2 1280x720 is 10 fps only. There is also a hard 40 ms floor on end-to-end
latency, because at 25 fps a frame only exists every 40 ms.


## Measured: generic preview, steady state

Borescope on the USB-A path, 640x480 MJPEG, over ~4 minutes:

```
generic displayed=51 avg_convert=13915us avg_e2e=38784us (640x480 MJPEG -> 400x300)
mjpeg ok=51 truncated=0 decode_err=0 (100% complete)
```

| metric | value | note |
|---|---|---|
| framerate | **25.4 fps** | 51 frames / 2.01 s - the camera's 25 fps ceiling |
| end-to-end latency | **38.8 ms** | inside the 40 ms frame period |
| decode + 2x PPA | 13.9 ms | hardware JPEG at 640x480, plus scale and byte-swap |
| truncated frames | **0 of ~6000** | the USB-A path is clean, not merely working |


## Camera hot-swap (no reset needed)

Swapping between the P2 Pro and a generic camera used to need a reset. That was not a missing
feature - it was a **use-after-free** being papered over by a reboot.

`s_frame_queue` holds up to 3 frames and was never drained. `uvc_host_stream_close()` frees the
driver's frame buffers on disconnect, but those three pointers survived in the queue, so
render_task dequeued them and read `frame->data` from freed memory - and on a swap it did so with
the *new* camera's geometry and format.

Three fixes on that path:

1. **Epoch-stamped frames.** Every queue entry carries a counter bumped on each open and each
   disconnect; render_task drops any entry whose stamp does not match, *without dereferencing it*.
2. **Drain before close.** The disconnect handler hands queued frames back while the stream handle
   is still valid, then closes. Closing with frames outstanding is what freed buffers under
   render_task.
3. **Never reopen blind.** `s_cam_kind` is reset to `CAM_NONE` on disconnect and the streaming task
   waits for fresh detection. It previously called `stream_open` with the *previous* camera's
   format - asking a webcam for 256x384 YUY2 - and burned the 5 s timeout before the retry used
   the right values.

Also, the JPEG decoder is released when the new camera does not need it, reclaiming ~1.2 MB of
PSRAM at 640x480. Done on render_task, so it cannot free buffers mid-decode.

