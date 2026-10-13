# The built-in OV5647 camera

Shown when no USB camera is attached and no WiFi camera is reachable, so the device is never a
blank screen. MIPI-CSI, raw Bayer through the P4's ISP.

The board carries an OV5647 on its MIPI-CSI connector, and it answers at SCCB address **0x36 on
the same I2C bus** already used for the FT6336 touch and the AXP2101 - our own bus scan has been
logging it all along. So `esp_video` is handed the existing bus handle with `init_sccb = false`,
and `reset_pin` / `pwdn_pin` are -1: there is no GPIO to drive. It is a raw Bayer sensor, so the
P4's ISP demosaics and we ask V4L2 for RGB565 - the same format the generic USB path already hands
to PPA, so the back half of that pipeline is reused.

**Measured: 1280x960 binning, cropped to 640x960, rotated and scaled to 480x320 - full panel at
32.0 fps.** `esp_video 0.9.0`, which also pulls `esp_cam_sensor 1.0.0` and `esp_ipa 0.3.0`.

## The sensor mode is everything

This is the single most important finding, and it is nothing to do with the host:

| mode | pixels | nominal | **actual** |
|---|---|---|---|
| RAW8 800x1280 | 1.02 Mpx | "50 fps" | **16.1 fps** - 32% of nominal |
| RAW8 800x640 | 0.51 Mpx | "50 fps" | 35.8 fps - 72% |
| **RAW10 1280x960 binning** | 1.23 Mpx | "45 fps" | **44.0 fps - 98%** |

The binning mode is *more* pixels and runs **2.7x faster** than 800x1280. Measured with a rate
test that dequeues and requeues frames with no PPA and no blit, so the camera path is isolated
from rendering. Nothing host-side moved the 800x1280 figure - it sat at exactly 62.186 ms per
frame through every attempt - because its sensor timing, not our pipeline, is the limit.

**Things tried against that 16 fps and measured to do nothing:** a 256 KB L2 cache with 128-byte
lines (the vendor's own setting, kept anyway); `esp_video` 0.8.0 -> 0.9.0;
`ESP_VIDEO_ENABLE_DATA_PREPROCESSING`; and YUV420 instead of RGB565 - accepted by both ISP and
PPA, 25% fewer bytes per frame, **identical** framerate, which is what proved the limit was not
bandwidth. A "~17 Mpx/s ISP ceiling" model derived from the two slow modes was simply wrong; the
binning mode does 54 Mpx/s.

## Geometry: crop to the panel's aspect before rotating

The sensor is mounted 90 degrees off the panel, so the frame is rotated for display. Cropping
640x960 out of the 1280x960 frame gives the panel's 3:2 aspect *before* rotation, so the result
fills 480x320 exactly at 8/16 scale - on PPA's 1/16 grid, no quantization loss - and halves PPA's
read to 614,400 pixels. The cost is field of view: sensor x becomes screen y, so taking half the
columns costs half the vertical view. The full frame would keep all of it but fill only 240x320.

Orientation is **sensor H-flip + `ROTATION_ANGLE_90`**, and getting there needed three corrections:

- **PPA's `mirror_x`/`mirror_y` do nothing here** - tried in both passes and both axes, no visible
  effect, although the driver demonstrably programs the registers. That is why `R90` alone left
  the horizontal wrong and `R270` alone left the vertical wrong.
- **`VIDIOC_S_CTRL` is not implemented** by esp_video, only `VIDIOC_S_EXT_CTRLS`. The first flip
  attempt was silently refused, and only a warning log caught it.
- **The control named `V4L2_CID_HFLIP` flips this sensor VERTICALLY.** The OV5647 driver's register
  tables comment 0x3820 as horizontal and 0x3821 as vertical, transposed from the datasheet. So
  the upright pairing is a *vertical* source flip with a 90 degree rotation.

## Two buffers, not three

`DQBUF` returns the **oldest** queued buffer, so the queue depth *is* the display lag. With 3
buffers and the camera faster than the renderer (44 fps in, 32 out) the queue stayed full and
frames were one to two capture periods old - and since those rates do not divide evenly the age
drifted, which looked like old frames interspersed with new ones at a regular interval. Two
buffers cap the camera at one frame ahead.

3 was correct while the camera was the *slow* side at 16 fps, where it removed a ~16 ms stall;
the binning mode reversed which side waits. The better fix - drain to the newest frame - is
impossible here: esp_video's `DQBUF` hardcodes `portMAX_DELAY` and ignores `O_NONBLOCK`, so a
drain loop blocks forever and renders nothing.

## Pipeline

The SPI blit and the PPA conversion used to run in series because the completion semaphore was
taken before the conversion. With two alternating output buffers the semaphore moves to just
before the blit, so the next frame's conversion overlaps the current blit. Stage timings at
480x320: `ppa_scale` 10.4 ms, `ppa_swap` 3.1 ms, `blit_wait` 14.5 ms - the 307,200-byte blit at
80 MHz is the remaining ceiling, which is the right place to be bound.

## The built-in camera must never compete with the instrument

Two regressions came out of the first version of this, both now fixed, both worth keeping in mind
for anything else that shares the render task:

**A use-after-free on teardown.** `builtin_cam_stop()` frees the two output buffers, and carried a
comment asserting the last blit had already completed. That was true while the blit was
synchronous; pipelining it made render_task issue `draw_bitmap` *without* holding the completion
semaphore afterwards, so the SPI could still be reading `s_bi_tx[i]` as it was freed - firing
exactly when a USB camera arrived and the preview was torn down. Presented as "worked, then froze
after a while". The stop path now waits for the blit before freeing.

**Eager start cost the thermal path latency.** The preview started the instant `s_cam_kind` was
`CAM_NONE`, which includes all of boot before the P2 is detected and every brief disconnect or
stream renegotiation - so it ran *concurrently* with the thermal stream, taking PSRAM bandwidth
and PPA time. It now waits `BUILTIN_IDLE_WAIT_US` (3 s) of no USB camera, re-armed whenever one is
present. A camera about to enumerate clears `CAM_NONE` well inside that window.

**Known residual risk:** esp_video's `DQBUF` blocks with `portMAX_DELAY`, so while render_task
waits for a built-in frame it cannot notice anything else - and render_task is also what services
the USB frame queue. Normally that wait is ~23 ms. If the CSI ever stalls the display would hang;
the proper fix is a helper task feeding render_task through a queue with a timeout.

## Auto white balance needs a JSON section AND a patched component

Two separate things had to be fixed, and neither is obvious:

**1. The algorithm is inert without configuration.** `CONFIG_ESP_IPA_AWB_ALGORITHM` defaults to
`y`, but AWB does nothing unless the sensor's IPA JSON has an `"awb"` section - and the
component's `ov5647_default.json` ships `adn`/`aen`/`ian`/`acc` only. That is exactly why exposure
worked and white balance did not. `main/ov5647_ipa.json` is that default plus an AWB block,
selected with `CONFIG_CAMERA_OV5647_CUSTOMIZED_IPA_JSON_CONFIGURATION_FILE`.

**The `range` sub-object is not optional**, even though the shipped `sc2336_default.json` omits
it. esp_video computes the white patch as
`luminance.max = green_max * (1 + rg_max + bg_max)`, and the AWB control defaults to all zeros, so
without `range` the white patch collapses, no pixel can ever qualify as a white point, and the
gains settle somewhere arbitrary - observed on hardware as a fixed purple cast that never
corrects. Values come from the esp_ipa README's worked example. `green_luma_env` / `_init` /
`_step_ratio` are deliberately omitted: they name a variable the OV5647's `ian` section does not
define (it has `luma.ae` only).

**2. An upstream bug that only bites chip rev >= 3.0.** `isp_start_awb()` and
`isp_reconfigure_awb()` declare `esp_isp_awb_config_t awb_config;` **uninitialised**, and
`isp_init_awb_param()` fills only `.window` - never `.subwindow`. `esp_driver_isp` validates the
subwindow against the window *only* on rev >= 3.0, so on our rev v3.2 the check runs against stack
garbage, AWB fails to start and takes `STREAMON` down with it:

```
ISP_AWB: s_esp_isp_awb_config_hardware(90): subwindow exceeds window range
isp_video: isp_start_awb(622): failed to new AWB
builtin: STREAMON failed
```

Still present in 0.9.1. Because `managed_components/` is gitignored and gets recreated, the fix is
**not** a silent hand-edit: run `./tools/patch_managed_components.sh` after a fresh clone or
whenever dependencies are re-fetched. It is idempotent and explains itself.

Current state: colour is much better, but adaptation is slow. `min_red_gain_step` /
`min_blue_gain_step` (0.034) gate how readily gains move; Waveshare ship a hand-tuned
`ov5647_custom.json` in their `12_esp32-p4-eye` example if it ever needs to be better.

## Also

Exposure works (AGC drives the sensor) but only in the binning mode. Note the OV5647 driver
implements no *getter* for exposure or gain, so a readback is always -1 even while AE is actively
writing them - do not read that as AE being inert.

`esp_video`'s ISP pipeline controller calls `esp_log_level_set(TAG, ESP_LOG_DEBUG)` **on itself**
in shipped code, gated on `LOG_LOCAL_LEVEL` which our `CONFIG_LOG_MAXIMUM_LEVEL=DEBUG` satisfies.
It then logs AE/AWB statistics from interrupt context every frame: ~10k lines in 40 s, which at
115200 baud starved the ISRs into repeated `Interrupt wdt timeout` panics and SW_CPU_RESET loops.
Fixed at runtime with `esp_log_level_set("ISP", ESP_LOG_WARN)` rather than by patching the
component.

PPA validates `out.buffer_size` against the **cache line**, so `P2_CACHE_LINE` now derives from
`CONFIG_CACHE_L2_CACHE_LINE_SIZE` instead of hardcoding 64 - changing the cache config would
otherwise have silently reproduced an earlier black-screen bug.

