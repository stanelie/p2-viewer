/*
 * Step 3 test: pull the raw radiometric half of the P2 Pro's 256x384 YUY2
 * frame, palette-map it to RGB565, and blit it to the onboard ST7796 SPI
 * display - no LVGL, direct esp_lcd calls, to keep the render path as short
 * as possible. Reports per-frame render time and end-to-end pipeline fps.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"   /* esp_ptr_internal() */
#include "esp_partition.h"
#include "esp_crc.h"
#include "esp_check.h"
#include "esp_rom_sys.h"      /* esp_rom_delay_us() in the port tracer */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ppa.h"
#include "driver/jpeg_decode.h"   /* P4 hardware JPEG: the MJPEG generic path */
#include "esp_wifi.h"        /* WiFi earpick camera; the radio lives on the onboard C6 */
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"   /* the WiFi stack needs NVS for calibration data */
#include "lwip/sockets.h"
/* Built-in OV5647 on the MIPI-CSI connector, via esp_video's V4L2 layer. */
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "sys/mman.h"   /* esp_video ships its own mmap shim */
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "linux/videodev2.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7796.h"

#include "usb/usb_host.h"
#include "usb/usb_helpers.h"   /* usb_parse_next_descriptor(), for the XU walk */
#include "soc/usb_dwc_struct.h"   /* USB_DWC_HS */
#include "soc/usb_utmi_struct.h"  /* USB_UTMI: analogue PHY trims (termination, squelch, swing) */
#include "usb/uvc_host.h"
/* Declared rather than included: esp_private/uvc_control.h pulls in the component's private
 * usb_types_uvc.h, which is not on the public include path. The symbol itself is exported and is
 * documented for exactly this use ("device drivers that use custom/vendor specific commands"). */
esp_err_t uvc_host_usb_ctrl(uvc_host_stream_hdl_t stream_hdl, uint8_t bmRequestType,
                            uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
                            uint16_t wLength, uint8_t *data);

#include "driver/i2c_master.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_ft5x06.h"

#define USB_HOST_PRIORITY   (15)
#define STREAM_H_RES         256
#define STREAM_V_RES         384 /* top 192 rows = preview, bottom 192 rows = raw radiometric */
#define RAW_H_RES            256
#define RAW_V_RES             192
#define STREAM_FPS            25.0f

/* ---- camera kind ----------------------------------------------------------------------------
 * The P2 Pro's signature is that it advertises 256x384 YUY2 - preview stacked on top of raw
 * radiometric, which no ordinary webcam offers. Anything else gets a plain preview instead of
 * having its frames parsed as though the bottom half were temperatures.
 *
 * Detecting on capability rather than VID:PID means any other UVC camera works too, and a P2
 * that has somehow been left in image-only mode falls back to a picture rather than garbage. */
typedef enum {
    CAM_NONE = 0,
    CAM_THERMAL,     /* a stacked radiometric format: full thermal interface */
    CAM_GENERIC,     /* any other USB camera: just show the image */
} cam_kind_t;

/* ---- thermal camera models -------------------------------------------------------------------
 * Two cameras present the same SHAPE of data - a 16-bit radiometric plane carried inside an
 * ordinary UVC frame - and everything downstream of the parse is identical for both: flat-field,
 * min/max scaling, palette mapping, crosshair, readouts, touch OSD, the manual scale. Only four
 * numbers differ, so they live in a table and the rest of the firmware reads them through
 * s_therm instead of the old RAW_H_RES/RAW_V_RES constants.
 *
 *   InfiRay P2 Pro   YUY2 256x384: preview in the top half, raw radiometric in the bottom.
 *                    Counts are Kelvin x 64. Has the InfiRay vendor control channel (shutter
 *                    schedule, forced FFC, device temperatures).
 *
 *   GOYOJO GW192A    YUY2 96x176, measured on this hardware (docs/GW192A_frame_layout.md):
 *                      rows   0..95   the 96x96 radiometric plane, uint16 LE, at offset 0
 *                      rows  96..99   a telemetry block, contents not yet decoded
 *                      rows 100..171  an NV12 96x96 visible copy - unused, because palette
 *                                     mapping the radiometric plane is both sharper and the
 *                                     same thing the P2 path already does
 *                      rows 172..175  zero padding
 *                    Counts are Kelvin x 16. No vendor channel is known for it, so there is no
 *                    shutter control and no device-temperature readout - the camera runs its own
 *                    schedule and we take what it gives.
 *
 * Detection is by advertised format, never VID:PID, for the reason given above. */
typedef enum {
    THERMAL_P2PRO = 0,
    THERMAL_GW192A,
    THERMAL_GW192A_FULL,
} thermal_model_t;

typedef struct {
    const char *name;
    uint16_t stream_w, stream_h;   /* the UVC format to request */
    uint16_t raw_w, raw_h;         /* geometry of the radiometric plane */
    size_t   raw_offset;           /* its byte offset within the frame */
    float    counts_per_kelvin;
    /* Counts reported at 0 K, i.e. the intercept of the camera's linear count/temperature law.
     * The P2 Pro is a pure scale (0), so its arithmetic is untouched. Kept separate from the
     * scale because the two are measured differently: the scale comes from the DIFFERENCE
     * between two reference temperatures and the bias from where that line crosses zero. */
    float    counts_bias;
    /* Pixels at or above this read as markers, not temperatures, and are excluded from the
     * scene. 0 disables the test. The GW192A sprinkles ~0x8002 words through its radiometric
     * plane - roughly one per row, at a position that moves - and left in they pin the auto
     * scale's maximum at 32770 forever, which squashes the whole real scene into the bottom
     * fifth of the palette and renders it nearly black. Measured on hardware.
     *
     * The threshold is physics, not a magic number: 32768 counts is 2048 K at 16 counts/K, so
     * nothing this camera can legitimately see reaches it. The P2 Pro keeps 0 deliberately -
     * its 0x8000 is a whole-frame "not calibrated yet" sentinel that the STARTING overlay
     * already handles, and masking it per pixel would leave that frame with no pixels at all. */
    uint16_t invalid_at;
    /* Non-zero: the radiometric plane is WORD-STUFFED. Every marker_stride-th word (those with
     * i % marker_stride == marker_stride - 1) is a marker rather than a pixel, and it is
     * INSERTED into the stream, not written over a pixel - so it has to be removed and the rest
     * closed up, not masked in place.
     *
     * Masking in place was the first implementation and it looked almost right, which is the
     * trap: every marker pushed the pixels after it one place along, so the picture came out in
     * horizontal bands each shifted one pixel right of the band above - a vertical edge rendered
     * as a staircase with a step every ~10.7 sensor rows. Reported from the panel, which is the
     * only place a one-pixel-per-1000 shift is visible at all.
     *
     * On the GW192A the stride is 1001: 1000 pixels, one 0x8002 marker, repeat. A 96x96 plane
     * therefore spans 9225 words, not 9216. */
    uint16_t marker_stride;
    /* UVC Extension Unit carrying this camera's vendor channel; 0 = none, and then the two
     * fields below are meaningless. Read off the GW192A's own descriptors on hardware
     * (probe_dump_xu(), behind P2_USB_PROBE) rather than assumed:
     *     XU unit_id=10 on interface 0, GUID {A29E7641-DE04-47E3-8B2B-F4341AFF003B},
     *     bmControls=ff 03 -> control selectors 1..10 implemented.
     * Kept here instead of re-walking the descriptors at runtime because the UVC component owns
     * the device and does not expose them, and a second USB host client purely to re-read a
     * constant is not worth the resources. The numbers are not trusted blindly either: every
     * exchange is checked against the device's own status register and the returned dwSize, so a
     * wrong unit shows up as a failed read and not as silently wrong data. */
    uint8_t  xu_unit;
    uint8_t  xu_interface;
    bool     vendor_cmds;          /* the InfiRay control channel is available */
} thermal_model_info_t;

static const thermal_model_info_t s_thermal_models[] = {
    [THERMAL_P2PRO]       = { "P2 Pro",     256, 384, 256, 192, 256u * 192u * 2u,
                              64.0f, 0.0f, 0,     0,    0,  0, true  },
    /* Table order is preference order. 96x176 wins because it is the one that FRAMES cleanly:
     * its marker words land at a fixed stride every frame and its min/max are steady, which is
     * what a stable picture needs.
     *
     * 96x194 is kept last and is effectively a fallback. It was tried as the preferred mode
     * because its plane reads ~420 counts lower - a plausible 19-20 C against 96x176's
     * implausible 50 C - but on hardware the frame does not assemble at a fixed offset: the
     * marker words move between frames, zeros appear inside the plane, and the scene maximum
     * swings from 5219 to 32141 frame to frame. A believable number from an unstable frame is
     * not worth a flickering image, and the offset below (bias) is the honest way to fix the
     * reading on the mode that actually works. */
    /* Scale and intercept MEASURED on hardware against a reference thermal camera, not guessed:
     *     matte wall  23.0 C  ->  5053 counts   (centre 3x3, steady to +-2)
     *     human palm  33.5 C  ->  5596 counts   (steady to +-4)
     * 543 counts over 10.5 K gives 51.71 counts/K, and the wall point puts the intercept at
     * -10262 counts, i.e. 0 C sits at 3864 counts.
     *
     * Earlier builds carried 16.0 counts/K, which was inferred from magnitude alone (5170/16 is
     * about room temperature in Kelvin) and was wrong by more than 3x - it reported a 23 C wall
     * as 50 C. Order of magnitude is not a calibration.
     *
     * Accuracy caveat: a 10.5 K baseline is short. Half a degree of error on either reference
     * moves the slope by about 7% (47 to 57 counts/K), which is +-1.5 C at body temperature and
     * worse further out, and the true slope may well be a round 51.2 or 64. A third reference far
     * from these two - ice water or just-boiled water - would tighten it; docs has the procedure. */
    [THERMAL_GW192A]      = { "GW192A",      96, 176,  96,  96, 0u,
                              51.714f, -10262.0f, 32768, 1001, 10, 0, false },
    [THERMAL_GW192A_FULL] = { "GW192A/194",  96, 194,  96,  96, 24u * 192u,
                              51.714f, -10262.0f, 32768, 1001, 10, 0, false },
};
#define THERMAL_MODEL_COUNT (sizeof(s_thermal_models) / sizeof(s_thermal_models[0]))

/* Written by the connect callback, read by render_task and the OSD - hence volatile. It always
 * points into the table above, so a stale read is a wrong camera, never a wild pointer. */
static const thermal_model_info_t *volatile s_therm = &s_thermal_models[THERMAL_P2PRO];

/* ---- built-in OV5647 ------------------------------------------------------------------------
 * The board carries an OV5647 on its MIPI-CSI connector, and it answers at SCCB address 0x36 on
 * the SAME I2C bus we already own for the FT6336 touch and the AXP2101 (SDA 7 / SCL 8) - our own
 * bus scan has been logging it as "0x36" all along. So esp_video is handed our existing bus
 * handle with init_sccb = false, and reset_pin / pwdn_pin are -1: there is no GPIO to drive.
 *
 * It is a raw Bayer sensor, so the P4's ISP does the demosaic and we ask V4L2 for RGB565 - the
 * same pixel format the generic USB path already hands to PPA, so the second half of that
 * pipeline is reused verbatim.
 *
 * The sensor is mounted 90 degrees off the panel, so the frame is rotated before display - which
 * makes the sensor's PORTRAIT mode the one that suits a LANDSCAPE panel. 800x1280 (RAW8, 50 fps)
 * rotates to 1280x800 and fits 480x320 at 6/16 in both axes, exactly on PPA's 1/16 grid, giving
 * 480x300: the full panel width with 10 black rows top and bottom.
 *
 * 800x640 was tried first and rotated to 640x800, which could only fill 240x300 - half the width,
 * with wide bars either side. That is unavoidable for a landscape sensor mode here, not a bug.
 *
 * Shown only while no USB camera is attached. Everything is opened and closed around that, so
 * the buffers are not sitting on PSRAM that the MJPEG path wants. */
/* ---- WiFi earpick camera -------------------------------------------------------------------
 * An NK3 / "HNDEC_55-*" otoscope: an OPEN 2.4 GHz access point serving JPEG over UDP. It accepts
 * exactly ONE client, so a phone sitting on the same AP starves us. Its protocol is NE7, chosen
 * by gateway address (192.168.1.1) and NOT by the brand string, which claims NE3 and is wrong.
 *
 * Preferred over the built-in camera, and only looked for when no USB camera is attached.
 *
 * Phase 1 is scan-only on purpose: the P4 has no radio, so the first thing to establish is
 * whether the onboard C6 answers at all. */
/* Diagnostic: render_task sets s_render_stage as it moves through the loop, and a separate
 * low-priority task logs it every 2 s. render_task has several UNBOUNDED waits - esp_video's
 * DQBUF hardcodes portMAX_DELAY and the SPI completion semaphore has no timeout - so when it
 * stops producing output the only question that matters is which one it is sitting in.
 * Observed: all rendering stopped at 4.1 s, right as esp-hosted finished its SDIO init. */
#define P2_RENDER_STAGE_TRACE 0

#define P2_WIFI_CAM        1
#define WIFI_CAM_PREFIX    "HNDEC"
/* Protocol constants, from the NE7 spec in ~/Documents/nk3-scope/NK3_CAMERA_PROTOCOL.md. */
#define WIFI_CAM_LOCAL_PORT  36123   /* one socket, all traffic; replies demuxed by source port */
#define WIFI_CAM_DISC_PORT_A 58090   /* silent on this unit, probed anyway - other revisions may */
#define WIFI_CAM_DISC_PORT_B 46526   /* this is the one that answers */
#define WIFI_CAM_VIDEO_PORT  44506
#define WIFI_CAM_IMU_PORT    52219
#define WIFI_CAM_JPEG_MAX    (64 * 1024)  /* frames measured at 4.5-14 KB; ample */
#define WIFI_CAM_STALL_MS    2000    /* no video for this long -> re-send the start packet */
#define WIFI_CAM_MAX_STALLS  4       /* ...and after this many, abandon the session and rescan */
/* Bytes to discard from the final chunk. The spec calls this a trailer but tags it [source] -
 * read from the upstream implementation, not verified on this unit - and "purpose unknown
 * (plausibly a checksum)". Stripping 5 made the hardware decoder reject ~70% of frames with
 * "the number of data units obtained after decoding is different from ... the image resolution",
 * i.e. the scan data was SHORT, which is exactly what cutting real bytes would cause. */
#define WIFI_CAM_TRAILER     0

/* ---- auto-rotation ---------------------------------------------------------------------------
 * The camera reports its orientation on the IMU channel and the vendor app uses it to keep the
 * picture upright. Which FIELD carries roll is disputed in the spec, so both candidates are
 * computed and logged and a flag picks the one that is applied:
 *
 *   ANGLE  - uint16 big-endian at bytes 18-19, used directly. Tagged [source].
 *   ACCEL  - atan2 of int16 big-endian axes at bytes 0 and 2. The spec argues for this: the
 *            at-rest sample gives |(x,y,z)| = 1.037 g at 2^14 counts/g, which is what a
 *            stationary accelerometer should read, while the uint8 reading has no such
 *            interpretation. Also tagged as unconfirmed, on one sample.
 *
 * Rotation is applied as -angle about the image centre. The frames are square, so rotation never
 * exposes empty corners inside a square viewport. */
#define WIFI_ROT_OFF    0
#define WIFI_ROT_ANGLE  1
#define WIFI_ROT_ACCEL  2
#define P2_WIFI_ROTATE  WIFI_ROT_ACCEL

/* WHICH AXIS PAIR carries image roll is not derivable from the spec - it depends on how the
 * sensor die sits relative to the pixel array. Measured on this unit: x stays small (-251, -130,
 * -305) while y and z both swing widely, so x is near the axis being twisted about and carries
 * no roll. atan2(x, y) therefore sat at ~0 or ~180 and flipped in one jump, which is exactly the
 * reported symptom: nothing, then a sudden large rotation, symmetric on the way back.
 * All three pairs are computed and logged; this selects the one applied. */
#define WIFI_AXES_XY  0
#define WIFI_AXES_YZ  1
#define WIFI_AXES_XZ  2
#define P2_WIFI_ROT_AXES  WIFI_AXES_YZ

/* NO SMOOTHING, deliberately. A low pass and a deadband were implemented and removed: with the
 * correct axis pair the raw signal is steady enough, and stanelie's call is that the response
 * should be as direct as possible - lag on an instrument you are aiming by hand is worse than
 * a little jitter. 100 = pass the raw angle straight through.
 *
 * The gimbal guard is off for the same reason. It only matters when the lens points near the
 * twist axis, where roll is mathematically undefined and will swing; set it to 1 if that ever
 * becomes annoying in use. */
#define WIFI_ROT_LPF_NUM   100     /* new sample weight /100 */
#define WIFI_ROT_DEADBAND  0.0f    /* degrees */
#define WIFI_ROT_GIMBAL_GUARD 0

/* Gain 1. The job is to CANCEL the rotation already present in the raw frame, not to add to it:
 * the scene is fixed and the sensor turns, so the frame arrives pre-rotated and the display needs
 * exactly the opposite rotation. Anything other than 1 makes the scene move with the twist.
 *
 * Observed while establishing this, with gain 1 in both cases:
 *   applied = -angle  ->  "works correctly, but opposite direction"
 *   applied = +angle  ->  "twice as slow as it should"
 * A sign flip cannot change a magnitude, so those two are only consistent if what is seen is
 * (raw + applied) - which is the proof that the raw frame rotates and that the fix is
 * cancellation at unity gain, not a multiplier. Gain 2 then measured 360 degrees of image for
 * 180 of camera, exactly as that model predicts.
 *
 * The angle itself is sound: y +2086 -> -14902 with z -15618 -> -2278 is a ~90 degree move in the
 * y-z plane and the angle moved 172 -> -99, i.e. 89 degrees, with |y,z| steady at ~1.05 g. */
#define WIFI_ROT_GAIN  1.0f

/* Fixed mounting correction, added to the measured twist: the sensor sits 90 degrees off the
 * display, so the scene needs a constant 90 degrees CCW on top of the roll cancellation.
 *
 * Sign derived from the renderer rather than guessed: with rad = +90 the inverse map sends
 * output-right to source-above, i.e. top content lands on the right, which is 90 CW. CCW
 * therefore needs rad = -90, and since rad = -(angle + offset), the offset is +90.
 *
 * Not multiplied by the gain - the gain scales the measured twist, this is a constant. */
/* 95, not 90: the extra 5 degrees CCW is a mounting trim, levelled by eye on this unit. The
 * sensor die is not square to the probe body, so the nominal 90 leaves the image slightly off
 * level. Expect this figure to differ on another camera. */
#define WIFI_ROT_OFFSET_DEG  95.0f

/* Circular mask, diameter = the output width. Rotating a square source inside a square target
 * sweeps black wedges through the corners; masking to the inscribed circle hides that and looks
 * like the otoscope view it is.
 *
 * It costs nothing - it SAVES work. The span is computed per row as sqrt(r^2 - dy^2), so the
 * inner loop samples only inside the circle and the rest is a black fill: one sqrt per row
 * against ~88,000 avoided source reads, and the skipped pixels are the expensive part (a strided
 * PSRAM read each). pi/4 of the area is kept, so roughly 21% of the sampling disappears. */
#define P2_WIFI_CIRCLE_MASK  1

/* Per-stage latency breakdown (reassembly / queue / decode / rotate / blit-wait / blit). How the
 * two serialisation stalls were found; off now that they are gone. */
#define P2_WIFI_LATENCY_TRACE 0

/* Probe an undocumented command once, 10 s into a session, and log what the stream does.
 *
 * The vendor APK's NE3 sibling (T0/f.smali) carries a LARGER 0x20 0x3X family - 35, 36, 37, 38 -
 * plus 30 66 and 30 67. On NE7 the app only ever sends 36 (start) and 37 (stop), so 35 and 38
 * have never been tried on this camera. They may be nothing; they may be the mode control the
 * app does not expose.
 *
 * Safe to try because the stall keepalive re-sends 20 36 after 2 s of silence, so a probe that
 * kills the stream recovers by itself rather than needing a power cycle. 0 disables. */
/* TRIED AND DEAD - do not re-probe. All four undocumented opcodes from the NE3 sibling were sent
 * to a live NE7 stream, 5 s apart, with the JPEG dimensions logged throughout:
 *
 *   20 35 (frames=61)   20 38 (frames=120)   30 66 (frames=181)   30 67 (frames=247)
 *
 * Result for every one: 432x432 unchanged, ~13 fps unchanged, no drops, no stall. The camera
 * ignores them and keeps streaming. Combined with the vendor app having only four commands and
 * no mode control at all, 432x432 at ~13 fps is this hardware's ceiling. */
#define P2_WIFI_PROBE_CMD  0
#define P2_WIFI_PROBE_SEQ  0

#define P2_BUILTIN_CAM 1
/* TRIED AND REJECTED - kept so it is not tried again. YUV420 out of the ISP is 1.5 bytes/px
 * instead of RGB565's 2, so 25% fewer bytes per frame. It was accepted by the ISP and PPA and
 * changed the framerate by exactly nothing:
 *
 *   800x640  RGB565   512,000 px   35.8 fps   18.3 Mpx/s
 *   800x1280 RGB565 1,024,000 px   16.1 fps   16.5 Mpx/s
 *   800x1280 YUV420 1,024,000 px   16.1 fps   16.5 Mpx/s   <- 25% fewer bytes, same fps
 *
 * The limit is the ISP's PIXEL rate, ~17 Mpx/s, not its byte rate - which is why cutting bytes
 * did nothing and why only a smaller sensor mode can raise the framerate. Left at 0: it costs
 * chroma resolution, its planar layout against PPA's expectation was never verified, and it buys
 * nothing. */
#define P2_BUILTIN_YUV420 0

/* Diagnostic: dequeue and requeue frames with NO PPA and NO blit, and report the rate. That
 * isolates what the CSI+ISP path can actually deliver from what our rendering costs - the
 * difference tells us whether to attack the camera pipeline or our own. Set to 0 for normal use. */
#define P2_BI_RATE_TEST 0

/* Per-stage timings (ppa_scale / ppa_swap / blit_wait). Off: they were how the pipeline was
 * characterised, they are cheap, and they are noisy once it is settled. */
#define P2_BI_STAGE_TIMING 0
/* Logged the sensor's exposure and gain every 2 s while chasing "exposure is stuck". Off, and
 * of limited use: AE turned out to work once the binning mode was in (the 800x1280 mode either
 * does not drive exposure or converged too slowly at 16 fps to notice), and the OV5647 driver
 * implements no GETTER for these controls, so the readback is always -1 even while AGC is
 * actively writing them. */
#define P2_BI_AE_TRACE 0

#define BUILTIN_CAM_W  1280
#define BUILTIN_CAM_H  960
/* 2, deliberately. DQBUF returns the OLDEST queued buffer, so the queue depth IS the display
 * lag: with 3 buffers and a camera running faster than we render (44 fps in, ~32 fps out) the
 * queue stayed full and we showed frames one to two capture periods old - and because the rates
 * do not divide evenly that age oscillated, which looked like old frames interspersed with new
 * ones at a regular interval. Two buffers bound the camera to at most one frame ahead, so the
 * lag is both small and steady. esp_video gives no way to drain to the newest frame: its DQBUF
 * hardcodes portMAX_DELAY and ignores O_NONBLOCK.
 *
 * (3 was right when the camera was the SLOW side, at 800x1280/16 fps, where it removed a ~16 ms
 * stall. The binning mode reversed which side waits.) */
#define BUILTIN_BUFS   2

#if P2_BUILTIN_CAM
static i2c_master_bus_handle_t s_i2c_bus;   /* shared with touch, PMIC and the camera's SCCB */
static int      s_bi_fd = -1;
static void    *s_bi_buf[BUILTIN_BUFS];
static size_t   s_bi_len[BUILTIN_BUFS];
static int      s_bi_w, s_bi_h, s_bi_x, s_bi_y;   /* on-panel geometry */
/* Two output buffers, alternating. Taking the SPI completion semaphore BEFORE the PPA work (as
 * the USB paths do, sharing one s_rgb_tx) serialises the blit and the conversion: measured
 * 28.8 ms of blit - 480x300x2 bytes at 80 MHz - plus 16.5 ms of PPA = the 45 ms e2e seen. With
 * two buffers the semaphore moves to just before the blit, so the next frame's PPA overlaps the
 * current blit and the period becomes max(blit, convert) instead of their sum. */
static uint16_t *s_bi_tx[2];
static int       s_bi_tx_i;
/* Source sub-rectangle PPA reads, so the picture fills the panel without stretching. */
static int       s_bi_sw, s_bi_sh, s_bi_sx, s_bi_sy;
static bool     s_bi_video_inited;
static uint32_t s_bi_frames;
static int64_t  s_bi_idle_since;   /* when CAM_NONE began; 0 = not idle */
/* How long no USB camera must be present before the built-in one takes over. Long enough to
 * cover enumeration and a stream renegotiation, short enough not to feel broken. */
#define BUILTIN_IDLE_WAIT_US  3000000
#endif

static volatile cam_kind_t s_cam_kind = CAM_NONE;
static volatile uint16_t s_cam_w = STREAM_H_RES;   /* format actually requested */
static volatile uint16_t s_cam_h = STREAM_V_RES;
static volatile enum uvc_host_stream_format s_cam_fmt = UVC_VS_FORMAT_YUY2;

/* ---- why a generic camera is asked for MJPEG ------------------------------------------------
 * Isochronous packets are never retried, so a frame only arrives if EVERY one of its packets
 * does: completion goes as (1-p)^N in the packet count N, and N is set by bytes-per-frame, not
 * by the endpoint size. Measured on the Sunplus borescope, 320x240:
 *
 *     YUY2   153600 B/frame  ~151 loss units of 1020 B
 *     MJPEG   22824 B/frame   ~22   (mean of 20 frames, 19184..24468)
 *
 * That is the whole reason an earlier MAX_MPS_IN patch (3060 -> 944 MPS) changed nothing: it
 * cut packet SIZE and left the count alone. Only the format moves the count, and ~7x fewer
 * loss units is the difference between (1-p)^151 and (1-p)^22.
 *
 * Set to 0 to go back to YUY2 for every generic camera. */
#define P2_GENERIC_PREFER_MJPEG 1

/* Override the generic-camera mode for A/B measurement. 0 = auto (largest advertised mode of the
 * preferred format that fits the panel). The borescope caps at 25 fps in EVERY mode, so framerate
 * cannot be bought with resolution - but the auto rule picks 320x240 and UPSCALES it to 420x315,
 * where 640x480 downscaled to the same box is sharper for the same 25 fps. Which one actually
 * sustains 25 fps end to end is a measurement, not a guess. */
#define P2_GENERIC_FORCE_W 0
#define P2_GENERIC_FORCE_H 0

/* ---- generic-preview benchmark ---------------------------------------------------------------
 * Cycles the three candidate presets automatically, ~12 s each, so one flash produces all the
 * numbers instead of three reflashes and three cable swaps. The preset name and the live stats
 * are both on the panel.
 *
 * What differs between them, and what cannot:
 *  - Bus traffic and JPEG decode cost are set by the CAMERA'S mode. Cropping cannot reduce
 *    either: the camera sends the whole frame regardless, and the P4's decoder has no ROI
 *    option, so a 640x480 JPEG is fully decoded whether we use all of it or not.
 *  - Only PPA read/write volume and the SPI blit size change. At full-panel 1:1 the crop blits
 *    480x320 = 153600 px against the letterbox's 420x315 = 132300, i.e. ~16% MORE SPI - and SPI
 *    is the bottleneck here. So crop should cost throughput and buy sharpness. Measure it.
 *  - 25 fps is the camera's ceiling in every mode, so no preset can beat that. */
/* How much larger than the panel a capture may be, per axis. 2 was chosen on measurement:
 * 640x480 (1.33x / 1.5x the 480x320 panel) is sharper than 320x240 at no measurable cost, while
 * 1280x720 is past the cap and would cost 46 MB/s and 4x the decode for nothing visible. */
#define P2_GEN_OVERSAMPLE 2

#define P2_GEN_BENCH 0
#define P2_GEN_BENCH_MS 15000

#if P2_GEN_BENCH
typedef struct {
    const char *name;
    uint16_t cam_w, cam_h;   /* mode requested from the camera */
    bool     crop;           /* true: 1:1 centre crop; false: fit-and-scale */
} gen_preset_t;

static const gen_preset_t s_gen_presets[] = {
    { "320X240 SCALE", 320, 240, false },   /* least bus + decode, upscaled - softest */
    { "640X480 SCALE", 640, 480, false },   /* same output box, downscaled - sharper */
    /* "640X480 CROP" (1:1 centre, full panel) was measured and REMOVED: visibly worse latency,
     * as predicted. Identical bus traffic and decode cost to 640X480 SCALE, but it blits
     * 480x320 = 153600 px against the letterbox's 420x315 = 132300, and the SPI blit is the
     * bottleneck on this board. Cropping to a SMALLER 1:1 box (420x315) would equalise SPI and
     * decode while keeping native pixels, but it narrows the field of view to 66% per axis -
     * that is a zoom feature, not a quality setting, so it is not the default's business. */
};
static volatile int s_gen_preset = 0;
#endif /* P2_GEN_BENCH */

/* The advertised mode list, cached at connect. A preset change has to re-select from it, and
 * the connect callback's own copy is long gone by then. */
#define P2_MAX_ADVERTISED 16
static uvc_host_frame_info_t s_adv[P2_MAX_ADVERTISED];
static size_t s_adv_n;

/* Hardware JPEG decoder, brought up lazily the first time an MJPEG frame arrives so the
 * thermal path never pays for it. */
static jpeg_decoder_handle_t s_jpeg_dec;
static uint8_t  *s_jpeg_in;        /* DMA-capable copy of the compressed frame */
static size_t    s_jpeg_in_cap;
static uint8_t  *s_jpeg_out;       /* decoded RGB565 at the camera's native size */
static size_t    s_jpeg_out_cap;
static bool      s_jpeg_failed;    /* bring-up failed once: stop retrying every frame */
static int       s_jpeg_w, s_jpeg_h;   /* resolution the buffers were sized for */

/* Diagnostics: on a marginal link these three counters ARE the measurement - they separate
 * "the frame never completed" from "the decoder could not handle it". */
static uint32_t s_mj_ok, s_mj_truncated, s_mj_decode_err;
static uint32_t s_port_disables __attribute__((unused));   /* survives a stream teardown, unlike the mjpeg counters */
/* Latest 2-second window, snapshotted for the on-screen readout. */
static uint32_t s_stat_fps_x10, s_stat_convert_us, s_stat_e2e_us;

/* Generic-preview output geometry, computed once per connect. */
static int s_gen_w, s_gen_h, s_gen_x, s_gen_y;
/* Source sub-rectangle PPA reads. Equal to the full frame unless a preset crops. */
static int s_src_w, s_src_h, s_src_x, s_src_y;
static bool s_gen_crop;
static size_t s_fb_bytes;   /* allocated size of s_rgb_buf/s_rgb_tx, cache-line aligned */

/* --- Display (ST7796 over SPI, pins from the Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5 BSP) --- */
#define LCD_H_RES            320
#define LCD_V_RES            480
#define LCD_SPI_MOSI         GPIO_NUM_20
#define LCD_SPI_CLK          GPIO_NUM_21
#define LCD_SPI_CS           GPIO_NUM_23
#define LCD_DC               GPIO_NUM_26
#define LCD_BACKLIGHT        GPIO_NUM_28
#define LCD_RST              GPIO_NUM_27
#define LCD_PIXEL_CLOCK_HZ   (80 * 1000 * 1000) /* Waveshare's validated default - 120MHz reproduced boot hangs, reverted */

/* --- Touch (FT5x06 over I2C, pins from the same BSP) --- */
#define TOUCH_I2C_PORT       I2C_NUM_1
#define TOUCH_I2C_SDA        GPIO_NUM_7
#define TOUCH_I2C_SCL        GPIO_NUM_8
#define TOUCH_RST            GPIO_NUM_29
#define TOUCH_INT            GPIO_NUM_50

static const char *TAG = "p2pro_uvc";

/* Boot-phase timing. esp_timer_get_time() is microseconds since startup. */
#define BOOT_MARK(what) ESP_LOGW(TAG, "boot %5d ms | %s", (int)(esp_timer_get_time()/1000), what)

/* ---------------- Display orientation (OSD-adjustable in the future) ----------------
 * rotation_deg: one of 0/90/180/270 - 90 is the only one verified on real hardware so far.
 * mirror_h/mirror_v: additional flip on top of the rotation, applied in the panel's
 * logical (post-rotation) axes - this is what a future on-screen menu would toggle. */
typedef struct {
    int  rotation_deg;
    bool mirror_h;
    bool mirror_v;
} display_orientation_t;

static display_orientation_t s_orientation = {
    .rotation_deg = 90,
    .mirror_h = true,  /* flip along the long edge, per on-hardware feedback */
    .mirror_v = false,
};

static int s_disp_w, s_disp_h; /* full logical panel size for the current orientation, set by display_init() */
/* Layout: the image sits flush top-left; the remaining width is one side panel and the
 * remaining height is the status bar.
 *
 * 416x312 is exactly 1.625x the native 256x192 on BOTH axes, so the picture is geometrically
 * correct and fills all the height PPA can reach (output heights from a 192-row source are
 * always multiples of 12, so 320 itself is unreachable; 312 is the closest fit).
 * Budget note: 259584 bytes/frame is near the practical ceiling - full-screen 480x320
 * (307200 bytes, ~31.5ms of SPI @80MHz) overran the 40ms frame and produced frame-buffer
 * underflow with visible corruption. */
#define SIDE_PANEL_W 64
/* The status bar is now the vertical side panel: s_bar_w x s_bar_h at (s_bar_x, s_bar_y).
 * The image takes the largest PPA-exact height that fits (312 of 320), leaving an 8-row dead
 * strip under it - heights from a 192-row source are always multiples of 12, so 320 itself is
 * unreachable. */
static int s_bar_w, s_bar_h, s_bar_x, s_bar_y;
#define BTN_W 56
#define BTN_H 38
/* Readouts moved onto the image, so the bar is all controls now and the -/+ pairs stack
 * vertically - one per slot, full button width, instead of two squeezed side by side. */
#define ADJ_BTN_W BTN_W
#define ADJ_BTN_H 38
/* 8 slots over a 320px bar is 40px each, which keeps BTN_H at 38 - the gaps shrink to 2px but
 * the touch targets do not. */
#define BAR_SLOTS 8    /* top to bottom */
#define SLOT_MAXPLUS 0
#define SLOT_MAXMINUS 1
#define SLOT_FFC 2
#define SLOT_SHUT 3
#define SLOT_PAL 4
#define SLOT_SCL 5
#define SLOT_MINPLUS 6
#define SLOT_MINMINUS 7

/* Software flat-field correction. Aim at something thermally uniform and press.
 *
 * This deliberately does NOT use the camera's own shutter as the reference, although the shutter
 * is mechanically the obvious choice and was tried: the shutter has a thermal gradient of its own,
 * so a table captured from it encodes that gradient and corrects less well than a plain uniform
 * surface at a distance. Tested on hardware and reverted.
 *
 * Never persisted - RAM only, so a power cycle always starts from the camera's own correction. */
#define FFC_FRAMES 32          /* ~1.3s at 25fps - averages the temporal noise down */
/* int8, not int16: halves the PSRAM read this table costs in the per-pixel loop. 1 count is
 * 1/64 K, so +/-127 covers +/-2 C of offset - ample for FPN residual (a closed shutter measured
 * 112 counts of total spread, i.e. deviations within about +/-56). Saturation is counted and
 * reported, so if a camera ever needs more range we will know rather than silently clip. */
static int8_t   *s_ffc_offset;    /* per-pixel deviation from the frame mean, counts */
static uint32_t *s_ffc_accum;
static volatile bool s_ffc_active = false;
static volatile bool s_ffc_req = false;
static int s_ffc_remaining = 0;

/* ---- persisted flat-field table -------------------------------------------------------------
 * A wall-based FFC sees the WHOLE optical path, so it corrects things the camera's own shutter FFC
 * structurally cannot: the shutter sits behind the lens, so anything in the optics (a speck on the
 * window, vignetting) is invisible to it and survives every shutter cycle. That part of the table
 * is a physical property of the camera, identical on every power-up, so it is worth keeping.
 *
 * Stored raw in its own 64KB partition: 48KB does not fit the 24KB NVS partition, and a raw write
 * avoids NVS blob overhead. CLR erases it, so this stays reversible. */
#define FFC_STORE_MAGIC   0x50324646u   /* 'P2FF' */
#define FFC_STORE_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t npix;
    uint32_t crc;      /* esp_crc32_le over the offset bytes */
} ffc_store_hdr_t;

static const esp_partition_t *ffc_part(void)
{
    static const esp_partition_t *p;
    static bool looked;
    if (!looked) {
        looked = true;
        p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, "ffc");
        if (!p) {
            ESP_LOGW(TAG, "FFC: no 'ffc' partition - correction will not persist");
        }
    }
    return p;
}

static void ffc_store_save(size_t npix)
{
    const esp_partition_t *part = ffc_part();
    if (!part) {
        return;
    }
    ffc_store_hdr_t h = {
        .magic = FFC_STORE_MAGIC,
        .version = FFC_STORE_VERSION,
        .npix = (uint32_t)npix,
        .crc = esp_crc32_le(0, (const uint8_t *)s_ffc_offset, npix),
    };
    esp_err_t e = esp_partition_erase_range(part, 0, (sizeof(h) + npix + 0xFFF) & ~0xFFFU);
    if (e == ESP_OK) { e = esp_partition_write(part, 0, &h, sizeof(h)); }
    if (e == ESP_OK) { e = esp_partition_write(part, sizeof(h), s_ffc_offset, npix); }
    ESP_LOGI(TAG, "FFC: %s stored table (%u bytes)",
             (e == ESP_OK) ? "saved" : esp_err_to_name(e), (unsigned)npix);
}

static void ffc_store_erase(void)
{
    const esp_partition_t *part = ffc_part();
    if (!part) {
        return;
    }
    esp_err_t e = esp_partition_erase_range(part, 0, 0x1000);  /* drop the header -> invalid */
    ESP_LOGI(TAG, "FFC: stored table %s", (e == ESP_OK) ? "erased" : esp_err_to_name(e));
}

/* Returns true if a valid table was loaded into s_ffc_offset. */
static bool ffc_store_load(size_t npix)
{
    const esp_partition_t *part = ffc_part();
    if (!part) {
        return false;
    }
    ffc_store_hdr_t h;
    if (esp_partition_read(part, 0, &h, sizeof(h)) != ESP_OK) {
        return false;
    }
    if (h.magic != FFC_STORE_MAGIC || h.version != FFC_STORE_VERSION || h.npix != npix) {
        return false;                      /* nothing stored, or from a different build */
    }
    if (esp_partition_read(part, sizeof(h), s_ffc_offset, npix) != ESP_OK) {
        return false;
    }
    const uint32_t crc = esp_crc32_le(0, (const uint8_t *)s_ffc_offset, npix);
    if (crc != h.crc) {
        ESP_LOGW(TAG, "FFC: stored table failed CRC - ignoring");
        return false;
    }
    ESP_LOGI(TAG, "FFC: restored stored table (%u bytes)", (unsigned)npix);
    return true;
}

/* The stored flat-field table belongs to one sensor geometry, so swapping cameras has to re-key
 * it: a 256x192 table applied to a 96x96 plane stays in bounds and is silently wrong, which is
 * the worst of both worlds. ffc_store_load() already rejects a mismatched pixel count from its
 * header, so loading unconditionally here is the whole check. */
static void ffc_reset_for_camera(void)
{
    s_ffc_active = false;
    s_ffc_remaining = 0;
    if (ffc_store_load((size_t)s_therm->raw_w * s_therm->raw_h)) {
        s_ffc_active = true;   /* the side-bar button shows CLR, so this is visible and undoable */
    }
}

/* One press = 2 C, expressed in whatever counts the attached camera reports (Kelvin x 64 on the
 * P2 Pro, x 16 on the GW192A), so the button means the same thing on both. */
static int adj_step_raw(void)
{
    return (int)(2.0f * s_therm->counts_per_kelvin);
}
/* Touch posts steps; render_task applies them, so all scale state stays owned by one task. */
static volatile int s_adj_min_steps = 0;
static volatile int s_adj_max_steps = 0;
static int s_img_w, s_img_h;   /* scaled image size (PPA output) */
/* The frame buffer covers the FULL panel height, not just the image: the readouts are drawn
 * low enough to straddle the image bottom and the leftover rows beneath it, so those rows have
 * to be inside the buffer we transfer. Costs ~6.6KB/frame more over the SPI bus. */
static int s_fb_h;
static int s_img_off_x, s_img_off_y; /* image offset within the panel (off_y should be 0: top) */

/* ---------------- OSD / status bar ---------------- */
static volatile bool s_camera_connected = false; /* false until a stream is open and running */
/* Set when the stream is alive but wrong or stalled - a knock on the USB-C connector can brown
 * the camera out and bring it back in its DEFAULT 256x192 image-only mode, which the driver
 * happily commits while uvc_host_stream_open() still reports success. Recovered by tearing the
 * stream down and renegotiating rather than requiring a human to unplug the camera. */
static volatile bool s_stream_restart_req = false;
static volatile bool s_osd_enabled = true;
static volatile bool s_osd_dirty = true; /* set by touch_task on toggle, or initially, to force a redraw */
static uint16_t *s_status_buf; /* s_disp_w * s_bar_h, white-on-black text, swap-invariant colors */
static esp_lcd_touch_handle_t s_touch;

/* The status bar uses the same esp_lcd_panel_io (one callback per io, not per draw_bitmap call)
 * as the main image, so its completion would otherwise satisfy s_trans_done_sem too - letting
 * render_task start overwriting s_rgb_buf while the MAIN image's transfer is still queued
 * behind the status bar's in the SPI hardware FIFO. Set right before the status-bar draw call;
 * the SPI device queue is strictly FIFO, so its completion always fires before the main image's
 * (queued after it), making this safe to just clear-and-return on. */
static volatile bool s_expect_status_bar_completion = false;

static esp_lcd_panel_handle_t s_panel = NULL;
static SemaphoreHandle_t s_trans_done_sem; /* given when the in-flight SPI transfer to the panel completes */
static int64_t s_pending_arrival_us;       /* arrival timestamp of the frame currently being transferred */
static QueueHandle_t s_frame_queue;
static uvc_host_stream_hdl_t s_stream = NULL;
/* Given once app_main has allocated the frame buffers. USB enumeration (~550ms) runs on its own
 * task and can complete before that, so the streaming task must wait rather than race it. */
static SemaphoreHandle_t s_buffers_ready = NULL;
/* Mirrors of the per-frame raw range, so the diagnostic watcher can see the calibration
 * state. A cold camera reports a constant 0x8000 in every radiometric pixel. */
static volatile uint16_t s_dbg_min_v = 0, s_dbg_max_v = 0;
static volatile int64_t  s_stream_open_us = 0;  /* when the current stream opened */
/* Spatial roughness of the radiometric half. ~320 on uncorrected cold data, ~6 once the
 * camera applies its flat-field - so a collapse here IS the FFC-completed signal. */
static volatile uint32_t s_dbg_neigh_mad = 0;
static volatile bool s_clean_logged = false;  /* one-shot: time-to-calibrated per stream */
static volatile bool s_autoshutter_req = false; /* first calibration done - settle the schedule */
static volatile uint32_t s_startup_roughness = 0; /* only meaningful until s_clean_logged */
static volatile bool s_shutter_cfg_done = false; /* set when the schedule write verified */
static volatile int64_t s_connect_us = 0;  /* device-connect time: what the user actually waits */
#define P2_RAW_UNCAL 32768u   /* 0x8000 - flat sentinel until the camera calibrates */

/* Characterises what is actually in the radiometric half, to explain the ~6s of garbled image that
 * follows calibration. A full-length frame with a plausible min/max can still be the wrong CONTENT.
 *
 * Real radiometric data is spatially smooth and sits in a narrow band around ambient; preview/YUY2
 * bytes reinterpreted as uint16 are high-variance and spread across the whole range. So log the
 * neighbour-difference magnitude and how many pixels fall outside a plausible thermal band. */
/* Logs a single line when the camera's flat-field lands, so time-to-calibrated is always
 * visible without the full per-frame diagnostic. Subsamples every 7th adjacent pair and stops
 * computing anything at all once it has fired, so the steady-state cost is one bool test. */
static void p2_log_first_clean(const uint16_t *raw16, size_t npix, int64_t t_open_us)
{
    /* s_dbg_min_v is published late in the render loop, so it is stale here on the first frames -
     * test the data itself. During the flat phase every pixel is the sentinel, so one sample is
     * enough. A real thermal image always has some spatial variation, so a roughness of exactly
     * zero means "not real data", never "perfectly calibrated". */
    if (s_clean_logged || raw16[0] == P2_RAW_UNCAL) {
        return;
    }
    uint64_t adiff = 0;
    size_t n = 0;
    for (size_t i = 1; i < npix; i += 7) {
        const int d = (int)raw16[i] - (int)raw16[i - 1];
        adiff += (uint64_t)(d < 0 ? -d : d);
        n++;
    }
    const uint32_t mad = n ? (uint32_t)(adiff / n) : 0;
    s_startup_roughness = mad;
    /* Low roughness alone is NOT enough: a CLOSED shutter is a uniform field and scores just as
     * smooth as a calibrated image (measured: roughness ~7, spread 132). Without the spread test
     * this fired mid-shutter and reported a calibration that had not happened yet. A real scene
     * measured 532-730 counts of spread. */
    const int spread = (int)s_dbg_max_v - (int)s_dbg_min_v;
    if (mad > 0 && mad < 100 && spread > 300) {
        s_clean_logged = true;
        /* The camera's first flat-field has landed. Raising a request rather than acting here:
         * this function is defined well above the command transport, and an #if referencing a
         * macro defined further down silently evaluates to 0 - which is exactly how the first
         * attempt at this compiled away to nothing. */
        s_autoshutter_req = true;
        /* Report from device connect, not from stream open: anything done before stream_start
         * would otherwise be invisible in this number and make a change look better than it is. */
        const int64_t now = esp_timer_get_time();
        ESP_LOGW(TAG, "camera calibrated: %d ms after connect (%d ms after stream open, roughness %u)",
                 s_connect_us ? (int)((now - s_connect_us) / 1000) : -1,
                 (int)((now - t_open_us) / 1000), (unsigned)mad);
    }
}

static void p2_frame_diag(const uint16_t *raw16, size_t npix, size_t data_len, int64_t t_open_us)
{
#if !P2_DIAG
    (void)raw16; (void)npix; (void)data_len; (void)t_open_us;
    return;
#else
    static int64_t last_log_us = 0;
    const int64_t now = esp_timer_get_time();
    if ((now - t_open_us) > 25000000) {
        return;                                  /* only the startup window */
    }
    if (now - last_log_us < 250000) {
        return;
    }
    last_log_us = now;

    uint16_t mn = 0xFFFF, mx = 0;
    uint64_t sum = 0, adiff = 0;
    uint32_t outside = 0;
    for (size_t i = 0; i < npix; i++) {
        const uint16_t v = raw16[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
        /* plausible band: roughly -40C..150C in Kelvin*64 */
        if (v < 14900u || v > 27100u) outside++;
        if ((i % s_therm->raw_w) != 0) {
            const int d = (int)v - (int)raw16[i - 1];
            adiff += (uint64_t)(d < 0 ? -d : d);
        }
    }
    s_dbg_neigh_mad = (uint32_t)(adiff / (npix ? npix : 1));
    ESP_LOGW(TAG, "diag t=%5dms len=%u min=%u max=%u mean=%u neigh_mad=%u outside=%u%%",
             (int)((now - t_open_us) / 1000), (unsigned)data_len, mn, mx,
             (unsigned)(sum / npix), (unsigned)(adiff / (npix ? npix : 1)),
             (unsigned)(outside * 100u / (npix ? npix : 1)));
#endif
}

/* ---- P2 Pro vendor command transport ---------------------------------------------------------
 * The camera carries an InfiRay command interface behind two vendor control requests, on top of
 * being an ordinary UVC device. Transport reverse-engineered by LeoDJ/P2Pro-Viewer from traces of
 * the official app (P2Pro/P2Pro_cmd.py); the command codes are the SDK's.
 *
 * Every command must be followed by polling a status byte until the camera reports idle - the SDK
 * calls this i2c_usb_check_access_done(). Skipping the poll is the usual reason a naive attempt
 * appears to work and then wedges.
 *
 * Byte order is genuinely inconsistent and is reproduced here rather than tidied: the command and
 * its parameter go out little-endian, but the length field is big-endian.
 */
#define P2_VC_OUT        0x41    /* vendor | device | host->dev */
#define P2_VC_IN         0xC1    /* vendor | device | dev->host */
#define P2_VC_REQ_WRITE  0x45
#define P2_VC_REQ_READ   0x44
#define P2_VC_VALUE      0x78

#define P2_IDX_STATUS    0x0200  /* 1 byte: bits 0,1 busy; bits 2-7 error */
#define P2_IDX_CMD       0x1d00  /* 8-byte command header */
#define P2_IDX_DATA      0x1d08  /* payload / read-back window */

#define P2_CMD_SET       0x4000

/* Auto-shutter configuration. Extracted from libircmd.so in the official Android app: the camera
 * schedules its own shutter events, and SHUTTER_PREVIEW_START_1ST/2ND_DELAY are the delays before
 * the first and second auto-shutter after preview starts. That is what the phone's two early
 * clicks are - the app is not triggering FFC manually, it configures when the camera does it. */
/* shutter_manual_switch, from libircmd.so. Already carries the 0x4000 SET bit (0x4000|0x020c),
 * so do NOT OR it again. Short form: the switch value is the parameter byte at header[2].
 * CommonParams$ShutterManualSwitchType: SHUTTER_OPEN = 0, SHUTTER_CLOSE = 1. */
#define P2_CMD_SHUTTER_MANUAL    0x420c
#define P2_SHUTTER_OPEN          0
#define P2_SHUTTER_CLOSE         1

#define P2_CMD_GET_AUTO_SHUTTER  0x8214
#define P2_CMD_SET_AUTO_SHUTTER  0xc214   /* 0x8214 | SET */

#define P2_ASP_PROP_SWITCH            0
#define P2_ASP_MIN_INTERVAL           1
#define P2_ASP_MAX_INTERVAL           2
#define P2_ASP_TEMP_THRESHOLD_OOC     3
#define P2_ASP_TEMP_THRESHOLD_B       4
#define P2_ASP_PROTECT_SWITCH         5
#define P2_ASP_ANY_INTERVAL           6
#define P2_ASP_PROTECT_THR_HIGH_GAIN  7
#define P2_ASP_PROTECT_THR_LOW_GAIN   8
#define P2_ASP_PREVIEW_START_1ST_DELAY 9
#define P2_ASP_PREVIEW_START_2ND_DELAY 10
#define P2_ASP_CHANGE_GAIN_1ST_DELAY  11
#define P2_ASP_CHANGE_GAIN_2ND_DELAY  12

#define P2_IDX_CMD_LONG  0x9d00
#define P2_IDX_DATA_LONG 0x1d10

#define P2_CMD_GET_DEVICE_INFO   0x8405
#define P2_CMD_SHUTTER_VTEMP     0x840c
#define P2_CMD_CUR_VTEMP         0x8b0d
#define P2_CMD_Y16_PREVIEW_START 0x010a
#define P2_CMD_Y16_PREVIEW_STOP  0x020a
#define P2_CMD_PREVIEW_START     0xc10f

static int s_p2_wait_ms = 1000;   /* lowered while hunting for channel availability */

static esp_err_t p2_wait_ready(int timeout_ms)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    do {
        uint8_t st = 0;
        esp_err_t err = uvc_host_usb_ctrl(s_stream, P2_VC_IN, P2_VC_REQ_READ,
                                          P2_VC_VALUE, P2_IDX_STATUS, 1, &st);
        if (err != ESP_OK) {
            return err;
        }
        if ((st & 0x03) == 0) {
            return ESP_OK;             /* idle */
        }
        if (st & 0xFC) {
            ESP_LOGW(TAG, "vdcmd status error 0x%02X", st);
            return ESP_FAIL;
        }
        vTaskDelay(1);
    } while (esp_timer_get_time() < deadline);
    return ESP_ERR_TIMEOUT;
}

/* Builds the 8-byte header both the write and read paths start with. */
static void p2_pack_header(uint8_t *h, uint16_t cmd, uint32_t param, uint16_t len)
{
    h[0] = (uint8_t)(cmd & 0xFF);          /* cmd, little-endian */
    h[1] = (uint8_t)(cmd >> 8);
    h[2] = (uint8_t)(param & 0xFF);        /* param, little-endian */
    h[3] = (uint8_t)(param >> 8);
    h[4] = (uint8_t)(param >> 16);
    h[5] = (uint8_t)(param >> 24);
    h[6] = (uint8_t)(len >> 8);            /* length, BIG-endian */
    h[7] = (uint8_t)(len & 0xFF);
}

/* Payload-less command. Returns ESP_ERR_NOT_SUPPORTED when the camera stalls it, which is how we
 * tell a command that does not exist from one that does. */
static esp_err_t p2_cmd(uint16_t cmd, uint32_t param)
{
    uint8_t h[8];
    p2_pack_header(h, cmd, param, 0);
    esp_err_t err = uvc_host_usb_ctrl(s_stream, P2_VC_OUT, P2_VC_REQ_WRITE,
                                      P2_VC_VALUE, P2_IDX_CMD, sizeof(h), h);
    if (err != ESP_OK) {
        return err;
    }
    return p2_wait_ready(s_p2_wait_ms);
}

/* "Long command" form, as libircmd.so builds it: an 8-byte header to 0x9d00 carrying cmd + a
 * big-endian 16-bit p1, a second 8-byte block to 0x1d08 whose last 4 bytes are the read length,
 * then the payload read back from 0x1d10. Note p1 is big-endian here while the short-form
 * parameter is little-endian - the firmware is not self-consistent, so follow it exactly. */
static esp_err_t p2_long_cmd_read(uint16_t cmd, uint16_t p1, uint8_t *out, uint16_t len)
{
    uint8_t b1[8] = {0}, b2[8] = {0};
    b1[0] = (uint8_t)(cmd & 0xFF);
    b1[1] = (uint8_t)(cmd >> 8);
    b1[2] = (uint8_t)(p1 >> 8);      /* p1, BIG-endian */
    b1[3] = (uint8_t)(p1 & 0xFF);
    /* b1[4..7] = p2 = 0 */
    b2[4] = (uint8_t)(len >> 24);    /* length, BIG-endian 32 */
    b2[5] = (uint8_t)(len >> 16);
    b2[6] = (uint8_t)(len >> 8);
    b2[7] = (uint8_t)(len & 0xFF);

    esp_err_t err = uvc_host_usb_ctrl(s_stream, P2_VC_OUT, P2_VC_REQ_WRITE,
                                      P2_VC_VALUE, P2_IDX_CMD_LONG, sizeof(b1), b1);
    if (err != ESP_OK) {
        return err;
    }
    err = uvc_host_usb_ctrl(s_stream, P2_VC_OUT, P2_VC_REQ_WRITE,
                            P2_VC_VALUE, P2_IDX_DATA, sizeof(b2), b2);
    if (err != ESP_OK) {
        return err;
    }
    err = p2_wait_ready(s_p2_wait_ms);
    if (err != ESP_OK) {
        return err;
    }
    return uvc_host_usb_ctrl(s_stream, P2_VC_IN, P2_VC_REQ_READ,
                             P2_VC_VALUE, P2_IDX_DATA_LONG, len, out);
}

/* Long-form write, as libircmd.so builds it: header to 0x9d00, then eight ZERO bytes to 0x1d08
 * (p3/p4 both unused here), then poll ready. */
static esp_err_t p2_long_cmd_write(uint16_t cmd, uint16_t p1, uint32_t p2)
{
    uint8_t b1[8], b2[8] = {0};
    b1[0] = (uint8_t)(cmd & 0xFF);
    b1[1] = (uint8_t)(cmd >> 8);
    b1[2] = (uint8_t)(p1 >> 8);       /* p1, BIG-endian */
    b1[3] = (uint8_t)(p1 & 0xFF);
    b1[4] = (uint8_t)(p2 >> 24);      /* p2, BIG-endian 32 */
    b1[5] = (uint8_t)(p2 >> 16);
    b1[6] = (uint8_t)(p2 >> 8);
    b1[7] = (uint8_t)(p2 & 0xFF);

    esp_err_t err = uvc_host_usb_ctrl(s_stream, P2_VC_OUT, P2_VC_REQ_WRITE,
                                      P2_VC_VALUE, P2_IDX_CMD_LONG, sizeof(b1), b1);
    if (err != ESP_OK) {
        return err;
    }
    err = uvc_host_usb_ctrl(s_stream, P2_VC_OUT, P2_VC_REQ_WRITE,
                            P2_VC_VALUE, P2_IDX_DATA, sizeof(b2), b2);
    if (err != ESP_OK) {
        return err;
    }
    return p2_wait_ready(s_p2_wait_ms);
}

static const char *p2_err(esp_err_t e)
{
    /* ESP_ERR_NOT_SUPPORTED specifically means the device STALLed, i.e. no such command. */
    return (e == ESP_ERR_NOT_SUPPORTED) ? "STALL (no such command)" : esp_err_to_name(e);
}

static esp_err_t p2_cmd_read(uint16_t cmd, uint32_t param, uint8_t *out, uint16_t len)
{
    uint8_t h[8];
    p2_pack_header(h, cmd, param, len);
    esp_err_t err = uvc_host_usb_ctrl(s_stream, P2_VC_OUT, P2_VC_REQ_WRITE,
                                      P2_VC_VALUE, P2_IDX_CMD, sizeof(h), h);
    if (err != ESP_OK) {
        return err;
    }
    err = p2_wait_ready(s_p2_wait_ms);
    if (err != ESP_OK) {
        return err;
    }
    return uvc_host_usb_ctrl(s_stream, P2_VC_IN, P2_VC_REQ_READ,
                             P2_VC_VALUE, P2_IDX_DATA, len, out);
}

/* One-shot startup probe. The camera only calibrates on its own schedule (~6s), and the image is
 * uncalibrated until it does - confirmed on hardware: the picture comes good exactly when the
 * shutter cycles. The official app gets a click immediately on connect, so it commands something
 * we do not.
 *
 * Each step is logged with a wide gap so an audible click can be attributed to a specific command.
 * Everything here is either a documented read or a documented action; spi_transfer (can write the
 * camera's flash) and sys_reset_to_rom are deliberately absent. */
/* Send the candidate commands at startup.
 *
 * KEEP THIS OFF. y16_preview_start/preview_start are mode-setting commands, not passive probes:
 * sending them on a live camera left it emitting raw clustered just above 0x8000 at 27.5fps, which
 * renders as a compressed bottom half and an orange top half, and the mode survived a replug. Any
 * future sweep needs an undo path worked out FIRST. */
#define P2_PROBE_STARTUP 0

/* Attempted one-shot undo for the above. It does NOT work: y16_preview_stop (0x020a) comes back
 * ESP_FAIL (camera status error), and what actually recovered the camera was physically
 * unplugging and replugging it. Left off - there is no known software undo for the mode change. */
#define P2_RECOVER_Y16_MODE 0

/* Shorten the camera's own auto-shutter schedule at startup. Factory defaults are 1ST=5s, 2ND=4s,
 * which is the measured ~8.75s before a usable image. To put the camera back to stock, set these
 * to 5 and 4 and reflash. */
/* Master switch for the investigation instrumentation: the auto-shutter dump, the vtemp
 * watcher, the per-frame content diagnostic and the UVC component's DEBUG logs. ESP_LOG to
 * UART blocks, and at 115200 these cost ~200ms of boot on their own - so they are off unless
 * something is being investigated. BOOT_MARK stays on; it is ~12 lines and worth it. */
#define P2_DIAG 0

/* Experiment, answered: installing the host on the P4's full-speed OTG controller
 * (peripheral_map = BIT1) succeeds - no error from usb_host_install - but sees NO DEVICE at all,
 * not even a root port reset. The FS controller's pins are not routed to this board's OTG
 * connector, so full speed is not available as a way to dodge high-speed disconnect detection.
 * Kept as a record; do not retry on this board. */
#define P2_FORCE_FULL_SPEED 0

/* Experiment, answered: masking the OTG disconnect interrupt does NOT stop the spurious teardown
 * (5 enumerations, 0 stream opens, unchanged). Wrong register - `disconnint` is the OTG-level
 * disconnect, while the host port state machine detects removal via HPRT.prtconndet and the port
 * interrupt. Masking that would mask connect events too and break enumeration. No software lever
 * exists here. Kept as a record; do not retry. */
#define P2_MASK_HS_DISCONNECT 0

/* Override the UTMI PHY analogue trims. Values are the raw register encodings, and the ones below
 * are the hardware defaults. IDF never writes these, so with P2_UTMI_TRIM 0 they stay at default.
 *
 *   adj_res_hs  0x0 = 40 ohm, 0x4 = 45 ohm (default), 0x6 = 50 ohm   HS termination trim
 *   adj_vref_sq 0x0 = 92 mV,  0x2 = 124 mV (default), 0x3 = 152 mV   squelch threshold
 *   adj_vsw_hs  0x0 = 320 mV, 0x4 = 400 mV (default), 0x7 = 460 mV   TX eye / swing
 *
 * MEASURED FACTS, and the header's documented defaults are NOT all correct:
 *
 *   fc_00 = 0x00000080  -> adj_res_hs  = 0x4 (45 ohm)   as documented
 *   fc_01 = 0x000000f8  -> adj_vref_sq = 0x8            header documents 0x2 - IT IS 0x8
 *                          adj_pw_hs   = 0xF (400 mV)   as documented
 *   fc_02 = 0x00000047  -> adj_iref_res= 0x7, adj_vsw_hs = 0x4 (400 mV)
 *
 * Writing a register is harmless; writing a WRONG VALUE is not. Writing adj_vref_sq = 0x2 (the
 * header's "default") actually cuts the squelch threshold hard and kills enumeration: HUB errors
 * 42-43 and 0 enumerations, against 20-31 and 2-10 at the real 0x8. An earlier conclusion that
 * "the write itself breaks it" was wrong - it was the value.
 *
 * Squelch sweep: 0x9, 0xA, 0xC, 0xF all make the device invisible (no hub activity at all), so
 * 0x8 is already at the edge of detection and there is no headroom upward. ONE run at 0xA did
 * produce enum=3 AND streamopen=3 - the only stream opens ever observed on this camera - but it
 * was not reproducible and the camera went absent shortly after, so it is unconfirmed and worth
 * retrying.
 *
 * VALIDITY RULE for any future sweep: a run with HUB == 0 AND enum == 0 means nothing was
 * attached - discard it, do not score it as "this setting failed". Several of my sweep results
 * were exactly that. And use "does a stream open" as the metric; enumeration counts are noisy
 * (identical runs give 2-10).
 *
 * Write them between usb_host_install() and powering the root port (P2_MANUAL_PORT_POWER), which
 * is verified harmless on its own: HUB 25-27, enum 4-6, same as baseline. */
#define P2_UTMI_TRIM     0
#define P2_UTMI_W_FC00   1   /* write fc_00 (adj_res_hs)  */
#define P2_UTMI_W_FC01   1   /* write fc_01 (adj_vref_sq) */
#define P2_UTMI_W_FC02   1   /* write fc_02 (adj_vsw_hs)  */
#define P2_UTMI_RES_HS   0x4   /* 45 ohm, the default */
#define P2_UTMI_VREF_SQ  0x8   /* measured hardware value (header wrongly documents 0x2).
                                * 0xA measurably improves the borescope link - HUB errors 20-43 -> 2-7
                                * and stream opens 0 -> 7-11 - but UNTESTED against the P2 Pro and the
                                * webcam, so it is NOT the default. See the README. */
#define P2_UTMI_VSW_HS   0x4   /* 400 mV, the measured hardware value */

/* ---- HS bus turn-around sweep ---------------------------------------------------------------
 * fc_06.hs_tx2rx_dly_cnt_sel is "PHY High-Speed bus turn-around time select", 3 bits, hardware
 * default 0b100. It sets how long the PHY waits after transmitting before it starts receiving.
 *
 * Why this knob and not another: the borescope's failure is NOT corrupted data. Every control
 * transfer over the same cable succeeds, repeatedly. What happens is that ~20ms after isochronous
 * traffic starts the core raises DISCONNINT - a purely analogue decision, taken by sampling the
 * differential amplitude in a narrow window after each SOF's EOP, with no handshake and no retry.
 * Sparse control traffic leaves the bus idle around that window; continuous isoc does not. If
 * ringing from a metre of thin unterminated cable is still settling when the PHY turns around,
 * a longer turn-around is exactly the fix.
 *
 * Swept in firmware rather than one value per build: each data point otherwise costs a physical
 * replug, and this camera wedges off the bus after repeated failures. */
#define P2_TX2RX_SWEEP 0

/* ---- port-register tracer ------------------------------------------------------------------
 * Stops the guessing. Every explanation so far (packets per frame, burst size, bus turn-around)
 * predicted DEGRADED data; the sweep measured ok=0 truncated=0 decode_err=0 across all eight
 * turn-around values, i.e. the frame callback never fires at all. So read the hardware.
 *
 * The decisive bit is HPRT.prtconnsts sampled at the instant of the "disconnect": if the device
 * is still electrically connected, the core raised DISCONNINT on a device that never left, and
 * this is a detector problem. If it really is 0, the device is genuinely dropping off and no
 * host-side register will help.
 *
 * GINTSTS is sampled too, for incompisoin / incompip - those are periodic-scheduling faults
 * rather than analogue ones, and would point somewhere else entirely. Note the ISR clears
 * GINTSTS, so these samples under-report; a bit seen set is meaningful, a bit seen clear is not. */
#define P2_PORT_TRACE 0

/* ---- on-screen USB diagnostics ---------------------------------------------------------------
 * The two USB sockets on this board are too close together to have the serial adapter and the
 * borescope's USB-A adapter plugged in at once, so the ONE case that still needs measuring - the
 * working plain-USB-2.0-hub case - cannot be captured over serial. Put the numbers on the panel
 * instead and read them off the screen.
 *
 * What matters is read straight out of the controller rather than echoed from the driver's log:
 * HCCHAR.mps is the per-transaction max packet size the core was actually programmed with, and
 * HCCHAR.ec is MULTCNT - transactions per microframe for a high-bandwidth periodic endpoint.
 * ESP-IDF never writes ec (verified: no write anywhere in the tree, and USB_EP_DESC_GET_MULT is
 * unused by hcd_dwc.c), so if the hub case streams with ec=0 at mps=1020 then high-bandwidth
 * isoc works here and MULTCNT is NOT the cause. That is the comparison this overlay exists for.
 *
 * Glyph set has no lowercase and no J, hence FMT1 rather than MJPEG. */
#define P2_USB_OSD 0

/* ---- microframe period check -----------------------------------------------------------------
 * Babble means "the device was still transmitting at end of frame", and HFIR.frint is where the
 * core believes end of frame IS. IDF sets it in usb_dwc_ll_hfir_set_frame_interval():
 *
 *     hfir.frint = (speed == USB_DWC_SPEED_FULL) ? 48000 : 6000;
 *
 * 6000 over a 125us microframe implies a 48 MHz PHY clock, while the same file sets phyif = 1
 * (16-bit UTMI), which is normally 30 MHz. That source is copy-pasted across esp32s2/s3/p4 and
 * S2/S3 are full-speed-only parts, where 48000 per 1 ms frame is correct - so the high-speed
 * branch may be a constant that was never checked against P4 silicon.
 *
 * HFNUM.frnum increments once per microframe, so this needs no camera and no replug: count it
 * against esp_timer. 8000/s is correct. Anything else is a frame-interval bug, and a SHORT
 * interval would make the core flag babble on any device that fills its microframe - which is
 * exactly the asymmetry we measured (P2 and webcam single-transaction: fine; borescope at
 * 3x1020: dead). */
#define P2_UFRAME_CHECK 0
#define P2_TRACE_N    384

/* Diagnostic: registers a second USB host client and logs each device's negotiated speed and
 * descriptors as it enumerates - independent of the UVC layer, so it reports even when the UVC
 * driver cannot claim the device. Off by default; it costs a task and a client slot. */
#define P2_USB_PROBE 0
/* Diagnostic: raise the USB enumerator/hub log levels so each enumeration stage is named as it
 * passes or fails. This is what showed that the borescope completes EVERY stage and is then
 * reported "device gone" by the hub - a disconnect, not an enumeration fault. Off by default;
 * it is very chatty. */
#define P2_USB_ENUM_TRACE 0
/* Experiment, kept as a record: skipping stream_open entirely for a generic camera made no
 * difference - the scope still cycled through 13 enumerations in 40s - which is how we know our
 * streaming code is not what destabilises it. */
#define P2_NO_OPEN_GENERIC 0


/* Whether the camera runs its own periodic flat-field (every MAX_INTERVAL = 60s), which closes
 * the shutter and briefly freezes the image. Toggled by the AS1/AS0 button; AS0 suppresses it.
 *
 * Defaults to ON - the camera's own behaviour, which keeps fixed-pattern noise corrected as the
 * sensor warms. AS0 trades that away for an uninterrupted image.
 *
 * Either way the state is only applied AFTER the camera's first calibration has completed: that
 * startup calibration is itself an auto-shutter event (the PREVIEW_START delays), so touching
 * this any earlier can leave the image permanently uncorrected.
 *
 * Not persisted: every boot returns to the camera's own behaviour. */
static volatile bool s_autoshutter_on = true;

#define P2_SET_SHUTTER_DELAYS 1
#define P2_SHUTTER_1ST_DELAY  1
#define P2_SHUTTER_2ND_DELAY  1
/* MIN_INTERVAL (default 5s) may gate how soon a second shutter is allowed after whatever the
 * camera does during its flat phase. Lower it too, same risk class, default recorded. */
#define P2_SHUTTER_MIN_INTERVAL 1
/* Try configuring BEFORE preview starts. The delays are counted from preview start, so writing
 * after stream_start (~3s, once the channel answers) may already be too late to move the first
 * shutter. The device handle exists after stream_open, so the channel can be tried there. */
/* Disproved: the command channel does not answer until preview is running, so the schedule
 * cannot be written before stream_start. The probe loop also cost ~1s of time-to-first-image.
 * Kept behind the flag only as a record that it was tried. */
#define P2_EARLY_CONFIG 0

/* Force a shutter cycle the moment the command channel answers, rather than waiting ~1.5s for the
 * camera's auto logic to act on the rewritten schedule. This is a direct actuator, not a mode
 * setting, so it is reversible by definition - but a shutter left closed would blind the camera,
 * hence the unconditional reopen and the uniform-field check after it. */
/* Disproved: forcing the shutter the moment the channel answers gave 5337/5281ms vs 5350/5425ms
 * baseline - no gain. The ~1.5s after the trigger is the flat-field operation itself, not
 * scheduling latency, so there is nothing to bring forward. It also blanks the image for ~800ms
 * and risks leaving the shutter shut. Kept behind the flag as a record that it was measured. */
#define P2_FORCE_SHUTTER 0
/* One-off validation: fire the cycle even when the image is already clean. Set back to 0. */
#define P2_SHUTTER_SELFTEST 0

/* Poll the shutter/vtemp registers after start and log them, so the calibration event shows up in
 * the log instead of depending on hearing the click. Costs two control transfers per tick. */
#define P2_WATCH_VTEMP   P2_DIAG
#define P2_WATCH_MS      20000
#define P2_WATCH_TICK_MS 100

#if P2_WATCH_VTEMP
static void p2_vtemp_watch_task(void *arg)
{
    (void)arg;
    const int64_t t0 = esp_timer_get_time();
    uint8_t last_sh[2] = {0xFF, 0xFF};
    bool last_uncal = true;
    ESP_LOGW(TAG, "p2: watching shutter_vtemp/cur_vtemp for %d ms", P2_WATCH_MS);
    while (s_stream && (esp_timer_get_time() - t0) < (int64_t)P2_WATCH_MS * 1000) {
        uint8_t sh[2] = {0}, cur[2] = {0};
        esp_err_t e1 = p2_cmd_read(P2_CMD_SHUTTER_VTEMP, 0, sh, sizeof(sh));
        esp_err_t e2 = p2_cmd_read(P2_CMD_CUR_VTEMP, 0, cur, sizeof(cur));
        const bool uncal = (s_dbg_min_v == P2_RAW_UNCAL && s_dbg_max_v == P2_RAW_UNCAL);
        const bool sh_changed = (e1 == ESP_OK) && (sh[0] != last_sh[0] || sh[1] != last_sh[1]);
        if (sh_changed || uncal != last_uncal) {
            ESP_LOGW(TAG, "p2: [t=%5dms] raw[%u..%u]%s shutter_vtemp %s%02x%02x cur_vtemp %02x%02x",
                     (int)((esp_timer_get_time() - t0) / 1000),
                     s_dbg_min_v, s_dbg_max_v,
                     uncal ? " UNCALIBRATED" : " calibrated  ",
                     (e1 == ESP_OK) ? "" : "(err) ", sh[0], sh[1],
                     (e2 == ESP_OK) ? cur[0] : 0, (e2 == ESP_OK) ? cur[1] : 0);
            last_sh[0] = sh[0];
            last_sh[1] = sh[1];
            last_uncal = uncal;
        }
        vTaskDelay(pdMS_TO_TICKS(P2_WATCH_TICK_MS));
    }
    ESP_LOGW(TAG, "p2: vtemp watch done");
    vTaskDelete(NULL);
}
#endif

/* Writes the shutter schedule. Returns ESP_OK only if every write read back as requested, so the
 * caller can tell a real success from a camera that is not listening yet. */
#if P2_FORCE_SHUTTER
/* Close the shutter, give the camera time to sample and compute its flat-field, then reopen.
 * The reopen is unconditional: if the camera already reopened by itself it is a harmless no-op,
 * and if it treats the command as a dumb actuator it is what prevents a blind camera. */
static void p2_force_shutter_cycle(void)
{
    ESP_LOGW(TAG, "p2: forcing shutter cycle (0x%04x)", P2_CMD_SHUTTER_MANUAL);
    esp_err_t ce = p2_cmd(P2_CMD_SHUTTER_MANUAL, P2_SHUTTER_CLOSE);
    ESP_LOGW(TAG, "p2:   close -> %s", (ce == ESP_OK) ? "OK" : p2_err(ce));
    if (ce != ESP_OK) {
        /* Never leave it ambiguous - try an open anyway in case the close partly took. */
        p2_cmd(P2_CMD_SHUTTER_MANUAL, P2_SHUTTER_OPEN);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(800));          /* long enough to sample and compute */
    esp_err_t oe = p2_cmd(P2_CMD_SHUTTER_MANUAL, P2_SHUTTER_OPEN);
    ESP_LOGW(TAG, "p2:   open  -> %s", (oe == ESP_OK) ? "OK" : p2_err(oe));

    /* A closed shutter presents a near-uniform field, which the clean-image detector would
     * happily call "calibrated". Spread is the giveaway: a real scene was ~730 counts, the
     * uncorrected phase ~3500. Measured with the shutter actually closed: 132 - so the original
     * threshold of 50 would have let a stuck shutter through. 300 errs the safe way; a redundant
     * reopen costs nothing, a missed one blinds the camera. */
    for (int i = 0; i < 12; i++) {
        vTaskDelay(pdMS_TO_TICKS(200));
        const int spread = (int)s_dbg_max_v - (int)s_dbg_min_v;
        if (spread > 300) {
            ESP_LOGW(TAG, "p2:   field spread %d - shutter is open", spread);
            return;
        }
    }
    ESP_LOGE(TAG, "p2: field still uniform (spread %d) after reopen - retrying open",
             (int)s_dbg_max_v - (int)s_dbg_min_v);
    p2_cmd(P2_CMD_SHUTTER_MANUAL, P2_SHUTTER_OPEN);
}
#endif

static esp_err_t p2_write_shutter_schedule(const char *when)
{
    static const struct { const char *name; uint16_t id; uint16_t val; } cfg[] = {
        /* MIN_INTERVAL is rejected by the camera - the write returns a status error and it
         * reads back 5 unchanged. Left out rather than retried every connect. */
        { "PREVIEW_START_1ST_DELAY", P2_ASP_PREVIEW_START_1ST_DELAY, P2_SHUTTER_1ST_DELAY },
        { "PREVIEW_START_2ND_DELAY", P2_ASP_PREVIEW_START_2ND_DELAY, P2_SHUTTER_2ND_DELAY },
    };
    esp_err_t worst = ESP_OK;
    for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
        esp_err_t we = p2_long_cmd_write(P2_CMD_SET_AUTO_SHUTTER, cfg[i].id, cfg[i].val);
        uint8_t rb[2] = {0};
        esp_err_t re = p2_long_cmd_read(P2_CMD_GET_AUTO_SHUTTER, cfg[i].id, rb, sizeof(rb));
        const unsigned got = (re == ESP_OK) ? (unsigned)((rb[0] << 8) | rb[1]) : 0xFFFFu;
        if (we != ESP_OK || re != ESP_OK || got != cfg[i].val) {
            worst = (we != ESP_OK) ? we : (re != ESP_OK ? re : ESP_FAIL);
        }
        ESP_LOGW(TAG, "p2: [%s] %-24s := %u -> %s, reads %u", when, cfg[i].name, cfg[i].val,
                 (we == ESP_OK) ? "OK" : p2_err(we), got);
    }
    if (worst == ESP_OK) {
        s_shutter_cfg_done = true;
    }
    return worst;
}

static void p2_camera_init(void)
{
    const int64_t t0 = esp_timer_get_time();
#define P2_MS_SINCE_START ((int)((esp_timer_get_time() - t0) / 1000))

    /* Transport check first: if these reads fail, nothing below means anything. */
#if P2_WATCH_VTEMP
    /* Start the watcher FIRST. Previously it was spawned after the retry below, so it never saw
     * the uncalibrated->calibrated transition it exists to catch. */
    xTaskCreate(p2_vtemp_watch_task, "p2vtemp", 4096, NULL, 3, NULL);
#endif

    /* Find when the command channel first answers, at fine granularity. A 1000ms wait_ready per
     * failed attempt swamped this measurement before: "3 attempts / 2844ms" was mostly our own
     * timeouts. Probe with a 60ms budget and a 2-byte read instead of a 48-byte one. */
    s_p2_wait_ms = 60;
    uint8_t vt[2] = {0};
    esp_err_t err = ESP_FAIL;
    int attempts = 0;
    while ((esp_timer_get_time() - t0) < 15000000) {
        attempts++;
        err = p2_cmd_read(P2_CMD_CUR_VTEMP, 0, vt, sizeof(vt));
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    s_p2_wait_ms = 1000;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "p2: command channel never came up (%d attempts, %dms): %s",
                 attempts, P2_MS_SINCE_START, p2_err(err));
        return;
    }
    ESP_LOGW(TAG, "p2: command channel UP at t=%dms (%d attempts)", P2_MS_SINCE_START, attempts);

    uint8_t pn[48] = {0};
    err = p2_cmd_read(P2_CMD_GET_DEVICE_INFO, 6 /* DEV_INFO_GET_PN */, pn, sizeof(pn));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "p2: device-info read failed after channel came up: %s", p2_err(err));
        return;
    }
    for (size_t i = 0; i < sizeof(pn); i++) {
        if (pn[i] && (pn[i] < 32 || pn[i] > 126)) {
            pn[i] = '.';
        }
    }
    ESP_LOGI(TAG, "p2: transport OK, part number '%s'", (char *)pn);

    /* Read-only dump of the camera's auto-shutter schedule. Params 9 and 10 are the delays before
     * the 1st and 2nd auto-shutter after preview start - the ~8.75s we are trying to shorten. */
    static const char *asp_names[13] = {
        "PROP_SWITCH", "MIN_INTERVAL", "MAX_INTERVAL", "TEMP_THRESHOLD_OOC",
        "TEMP_THRESHOLD_B", "PROTECT_SWITCH", "ANY_INTERVAL", "PROTECT_THR_HIGH_GAIN",
        "PROTECT_THR_LOW_GAIN", "PREVIEW_START_1ST_DELAY", "PREVIEW_START_2ND_DELAY",
        "CHANGE_GAIN_1ST_DELAY", "CHANGE_GAIN_2ND_DELAY",
    };
#if P2_DIAG
    ESP_LOGW(TAG, "p2: ---- auto-shutter params (read-only) ----");
    for (int i = 0; i < 13; i++) {
        uint8_t r[2] = {0};
        esp_err_t ae = p2_long_cmd_read(P2_CMD_GET_AUTO_SHUTTER, (uint16_t)i, r, sizeof(r));
        if (ae == ESP_OK) {
            ESP_LOGW(TAG, "p2:   [%2d] %-24s = %5u (0x%02x%02x)%s",
                     i, asp_names[i], (unsigned)((r[0] << 8) | r[1]), r[0], r[1],
                     (i == P2_ASP_PREVIEW_START_1ST_DELAY ||
                      i == P2_ASP_PREVIEW_START_2ND_DELAY) ? "  <== target" : "");
        } else {
            ESP_LOGW(TAG, "p2:   [%2d] %-24s -> %s", i, asp_names[i], p2_err(ae));
        }
    }
#endif

#if P2_SET_SHUTTER_DELAYS
    /* Late path: runs once the channel answers after stream_start. If the early attempt already
     * succeeded this is a no-op re-write, which is harmless and keeps the camera correct after a
     * renegotiation. */
    if (!s_shutter_cfg_done) {
        p2_write_shutter_schedule("late");
    }
#if P2_FORCE_SHUTTER
    /* Don't wait ~1.5s for the auto logic to act on the new schedule - trigger it now.
     * P2_SHUTTER_SELFTEST forces one cycle even on an already-clean camera, to prove the
     * actuator closes and reopens against a known-good baseline before relying on it cold. */
    if (!s_clean_logged || P2_SHUTTER_SELFTEST) {
        p2_force_shutter_cycle();
    }
#endif
#endif

    uint8_t v[2] = {0};
    if (p2_cmd_read(P2_CMD_CUR_VTEMP, 0, v, sizeof(v)) == ESP_OK) {
        ESP_LOGI(TAG, "p2: cur_vtemp    %02x %02x", v[0], v[1]);
    }
    if (p2_cmd_read(P2_CMD_SHUTTER_VTEMP, 0, v, sizeof(v)) == ESP_OK) {
        ESP_LOGI(TAG, "p2: shutter_vtemp %02x %02x", v[0], v[1]);
    }

#if P2_RECOVER_Y16_MODE
    /* Undo the mode change this firmware caused by sending y16_preview_start speculatively.
     * 0x020a is its documented counterpart. preview_stop (0x020f) is deliberately NOT sent - it
     * could halt the video feed outright, and only the Y16 mode needs reverting. */
    ESP_LOGW(TAG, "p2: sending y16_preview_stop (0x%04x) to undo the mode change",
             P2_CMD_Y16_PREVIEW_STOP);
    esp_err_t rec = p2_cmd(P2_CMD_Y16_PREVIEW_STOP, 0);
    ESP_LOGW(TAG, "p2: y16_preview_stop -> %s", (rec == ESP_OK) ? "ACCEPTED" : p2_err(rec));
#endif

#if P2_PROBE_STARTUP
    /* Highest prior first: the phone's instant click may just be the camera calibrating as a side
     * effect of entering Y16 mode, which we have never requested. */
    static const struct {
        const char *name;
        uint16_t    cmd;
    } candidates[] = {
        { "y16_preview_start",       P2_CMD_Y16_PREVIEW_START              },
        { "y16_preview_start|SET",   P2_CMD_Y16_PREVIEW_START | P2_CMD_SET },
        { "preview_start",           P2_CMD_PREVIEW_START                  },
        { "preview_start|SET",       P2_CMD_PREVIEW_START     | P2_CMD_SET },
        { "shutter_vtemp|SET",       P2_CMD_SHUTTER_VTEMP     | P2_CMD_SET },
    };

    /* Only meaningful while the camera is still uncorrected - that is the window we are trying
     * to shorten. If the data is already clean there is nothing to trigger. */
    if (s_dbg_neigh_mad < 100) {
        ESP_LOGW(TAG, "p2: data already clean (neigh_mad=%u) - skipping probe, camera was warm",
                 (unsigned)s_dbg_neigh_mad);
    } else {
        ESP_LOGW(TAG, "p2: ==== probe (neigh_mad=%u, uncorrected) ====",
                 (unsigned)s_dbg_neigh_mad);
        for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
            const uint32_t before = s_dbg_neigh_mad;
            ESP_LOGW(TAG, "p2: [t=%4dms] sending %-22s (0x%04x) neigh_mad=%u",
                     P2_MS_SINCE_START, candidates[i].name, candidates[i].cmd, (unsigned)before);
            esp_err_t e = p2_cmd(candidates[i].cmd, 0);
            ESP_LOGW(TAG, "p2: [t=%4dms]   -> %s", P2_MS_SINCE_START,
                     (e == ESP_OK) ? "ACCEPTED" : p2_err(e));
            vTaskDelay(pdMS_TO_TICKS(1200));   /* an FFC needs ~1s to close, sample and reopen */
            if (before >= 100 && s_dbg_neigh_mad < 100) {
                ESP_LOGE(TAG, "p2: *** FFC TRIGGERED by %s (0x%04x) - neigh_mad %u -> %u ***",
                         candidates[i].name, candidates[i].cmd,
                         (unsigned)before, (unsigned)s_dbg_neigh_mad);
                break;
            }
        }
        ESP_LOGW(TAG, "p2: ==== probe done, neigh_mad=%u ====", (unsigned)s_dbg_neigh_mad);
    }
#endif
#undef P2_MS_SINCE_START
}
typedef enum {
    PALETTE_IRONBOW = 0,
    PALETTE_WHITEHOT,
    PALETTE_COUNT,
} palette_t;
static uint16_t s_palettes[PALETTE_COUNT][256]; /* raw-normalized (0-255) -> RGB565 */
static volatile int s_palette = PALETTE_IRONBOW;
static const char *PALETTE_NAME[PALETTE_COUNT] = { "IRON", "GRAY" };



/* Palette scaling: auto re-ranges every frame, manual freezes the range captured when the
 * button was pressed. Out-of-range pixels clamp to the palette's own end colours (the existing
 * [0,255] clamp on the index already does this), so a locked scale reads consistently. */
static volatile bool s_scale_manual = false;
static volatile bool s_scale_latch_req = false;
static uint16_t s_manual_min = 0, s_manual_max = 1, s_manual_range = 1;

static uint16_t *s_small_rgb; /* RAW_H_RES * RAW_V_RES, native-res converted image (PPA input) */
/* Destination for destuff_plane(). Sized for the largest sensor so a camera swap never reallocs. */
static uint16_t *s_raw_destuff;
static uint16_t *s_rgb_buf;   /* PPA scale target + overlays, plain RGB565 */
static uint16_t *s_rgb_tx;    /* byte-swapped copy, DMA-capable - this is what gets blitted */
static ppa_client_handle_t s_ppa_client;

static volatile int s_frames_rendered = 0;
static volatile int64_t s_last_report_us = 0;
static volatile int64_t s_e2e_time_sum_us = 0;

/* ---------------- Palette ---------------- */

static void build_palettes(void)
{
    /* White hot: plain luminance ramp. */
    for (int i = 0; i < 256; i++) {
        uint8_t r5 = (uint8_t)(i * 31 / 255);
        uint8_t g6 = (uint8_t)(i * 63 / 255);
        uint8_t b5 = (uint8_t)(i * 31 / 255);
        s_palettes[PALETTE_WHITEHOT][i] = (r5 << 11) | (g6 << 5) | b5;
    }

    /* Simple black -> purple -> red -> orange -> yellow -> white ramp. */
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f;
        float r, g, b;
        if (t < 0.25f) {            /* black -> purple */
            float k = t / 0.25f;
            r = 0.3f * k; g = 0.0f; b = 0.5f * k;
        } else if (t < 0.5f) {      /* purple -> red */
            float k = (t - 0.25f) / 0.25f;
            r = 0.3f + 0.7f * k; g = 0.0f; b = 0.5f * (1 - k);
        } else if (t < 0.75f) {     /* red -> orange/yellow */
            float k = (t - 0.5f) / 0.25f;
            r = 1.0f; g = 0.8f * k; b = 0.0f;
        } else {                    /* yellow -> white */
            float k = (t - 0.75f) / 0.25f;
            r = 1.0f; g = 0.8f + 0.2f * k; b = 1.0f * k;
        }
        uint8_t r8 = (uint8_t)(r * 31);
        uint8_t g8 = (uint8_t)(g * 63);
        uint8_t b8 = (uint8_t)(b * 31);
        /* Plain RGB565 - PPA needs real channel values to scale/blend correctly. The
         * byte-swap for the panel's big-endian transfer requirement happens after PPA,
         * on its output (see render_task) - baking it in here fed PPA scrambled bits and
         * produced the garbled palette; letting PPA "un-swap" pre-swapped input (tried via
         * its .byte_swap input flag) only affects how PPA reads data, not what it writes,
         * which is why that attempt came out looking inverted instead of correct. */
        s_palettes[PALETTE_IRONBOW][i] = (r8 << 11) | (g8 << 5) | b8;
    }
}

/* ---------------- Tiny 5x7 bitmap font, just the characters the status bar needs ---------------- */

typedef struct {
    char c;
    uint8_t rows[7]; /* bits 4..0 = columns left..right */
} glyph_t;

static const glyph_t FONT_5X7[] = {
    {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}},
    {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}},
    {'3', {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
    {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}},
    {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
    {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}},
    {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}},
    {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
    {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}},
    {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}},
    {':', {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}},
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}},
    {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'N', {0x11, 0x19, 0x15, 0x15, 0x13, 0x11, 0x11}},
    {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
    {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}},
    {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}},
    {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}},
    {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
    {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
    {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}},
    {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}},
    {'Y', {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}},
    {'D', {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}},
    {'+', {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}},
    {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
    {'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}},
};
#define FONT_W 5
#define FONT_H 7

static const uint8_t *find_glyph(char c)
{
    for (size_t i = 0; i < sizeof(FONT_5X7) / sizeof(FONT_5X7[0]); i++) {
        if (FONT_5X7[i].c == c) {
            return FONT_5X7[i].rows;
        }
    }
    /* A hollow box, not a blank. This font only carries the characters the UI happens to need, so
     * a missing glyph is an easy mistake - "CALIBRATING" silently rendered as "CALI RATING"
     * because there was no 'B'. Make the gap visible instead of looking like a render glitch. */
    static const uint8_t missing[7] = {0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F};
    return missing;
}

/* Draws into s_status_buf. White-on-black (0xFFFF/0x0000) is byte-swap-invariant, so this
 * doesn't need the post-PPA swap step the camera image does - see render_task. */
static void draw_text_to(uint16_t *buf, int bw, int bh, int x0, int y0, int scale,
                         uint16_t colour, const char *text)
{
    int x = x0;
    for (const char *p = text; *p; p++) {
        const uint8_t *rows = find_glyph(*p);
        for (int row = 0; row < FONT_H; row++) {
            for (int col = 0; col < FONT_W; col++) {
                if (!(rows[row] & (1 << (FONT_W - 1 - col)))) {
                    continue;
                }
                for (int sy = 0; sy < scale; sy++) {
                    int py = y0 + row * scale + sy;
                    if (py < 0 || py >= bh) {
                        continue;
                    }
                    for (int sx = 0; sx < scale; sx++) {
                        int px = x + col * scale + sx;
                        if (px < 0 || px >= bw) {
                            continue;
                        }
                        buf[py * bw + px] = colour;
                    }
                }
            }
        }
        x += (FONT_W + 1) * scale;
    }
}

static void fill_rect_to(uint16_t *buf, int bw, int bh, int x0, int y0, int w, int h, uint16_t colour)
{
    for (int y = y0; y < y0 + h; y++) {
        if (y < 0 || y >= bh) continue;
        for (int x = x0; x < x0 + w; x++) {
            if (x < 0 || x >= bw) continue;
            buf[y * bw + x] = colour;
        }
    }
}

static void draw_text(int x0, int y0, int scale, const char *text)
{
    draw_text_to(s_status_buf, s_bar_w, s_bar_h, x0, y0, scale, 0xFFFF, text);
}

/* Raw P2 Pro counts are Kelvin*64 (reverse-engineered, confirmed by P2Pro-Viewer/thermal-cat/
 * p2pro-rs and verified sane against indoor-scene readings on this hardware). The GW192A uses
 * the same linear-in-Kelvin form at a different scale, which is why this is one function and
 * not two: see s_thermal_models. */
static float raw_to_celsius(uint16_t raw)
{
    return (raw - s_therm->counts_bias) / s_therm->counts_per_kelvin - 273.15f;
}

static int slot_h(void)      { return s_bar_h / BAR_SLOTS; }
static int slot_top(int i)   { return i * slot_h(); }
static int slot_text_y(int i, int scale) { return slot_top(i) + (slot_h() - FONT_H * scale) / 2; }
static int slot_btn_y(int i) { return slot_top(i) + (slot_h() - BTN_H) / 2; }
static int btn_x(void)       { return (s_bar_w - BTN_W) / 2; }
static int adj_x(void)       { return (s_bar_w - ADJ_BTN_W) / 2; }
static int adj_y(int slot)   { return slot_top(slot) + (slot_h() - ADJ_BTN_H) / 2; }

static int text_width(const char *s, int scale)
{
    return (int)strlen(s) * (FONT_W + 1) * scale;
}

/* Renders one line into the status bar and queues its transfer; text == NULL blanks the bar.
 * Only ever called from render_task - the completion-disambiguation flag and s_trans_done_sem
 * both assume a single task issues panel draws. */
static void draw_status_bar(const char *text)
{
    memset(s_status_buf, 0, (size_t)s_bar_w * s_bar_h * sizeof(uint16_t));
    if (text) {
        /* '\n' splits lines - the bar is only 64px wide, which fits 5 characters at this
         * size, so anything longer has to wrap or it gets clipped. */
        const int scale = 2;
        int nlines = 1;
        for (const char *p = text; *p; p++) {
            if (*p == '\n') nlines++;
        }
        int lh = FONT_H * scale + 2;
        int y = (s_bar_h - nlines * lh) / 2;
        for (const char *p = text; *p; ) {
            char line[16];
            size_t n = 0;
            while (*p && *p != '\n' && n < sizeof(line) - 1) {
                line[n++] = *p++;
            }
            line[n] = '\0';
            if (*p == '\n') p++;
            draw_text((s_bar_w - text_width(line, scale)) / 2, y, scale, line);
            y += lh;
        }
    }
    s_expect_status_bar_completion = true;
    esp_lcd_panel_draw_bitmap(s_panel, s_bar_x, s_bar_y,
                               s_bar_x + s_bar_w, s_bar_y + s_bar_h, s_status_buf);
}

static void draw_rect(int x0, int y0, int w, int h, uint16_t colour)
{
    for (int x = x0; x < x0 + w; x++) {
        if (x < 0 || x >= s_bar_w) continue;
        if (y0 >= 0 && y0 < s_bar_h)                 s_status_buf[y0 * s_bar_w + x] = colour;
        if (y0 + h - 1 >= 0 && y0 + h - 1 < s_bar_h) s_status_buf[(y0 + h - 1) * s_bar_w + x] = colour;
    }
    for (int y = y0; y < y0 + h; y++) {
        if (y < 0 || y >= s_bar_h) continue;
        if (x0 >= 0 && x0 < s_bar_w)                 s_status_buf[y * s_bar_w + x0] = colour;
        if (x0 + w - 1 >= 0 && x0 + w - 1 < s_bar_w) s_status_buf[y * s_bar_w + x0 + w - 1] = colour;
    }
}

/* Vertical side bar: min, palette button, centre, scale button, max - top to bottom, same
 * order as the old horizontal bar. No labels on the numbers, so position disambiguates. */
static void update_status_bar(uint16_t min_v, uint16_t max_v, uint16_t centre_v)
{
    if (!s_osd_enabled) {
        draw_status_bar(NULL);
        return;
    }
    const int scale = 2;
    memset(s_status_buf, 0, (size_t)s_bar_w * s_bar_h * sizeof(uint16_t));

    /* Flat-field correction button */
    const char *ffc = s_ffc_remaining ? "WAIT" : (s_ffc_active ? "CLR" : "FFC");
    int fby = slot_btn_y(SLOT_FFC);
    draw_rect(btn_x(), fby, BTN_W, BTN_H, 0xFFFF);
    draw_text(btn_x() + (BTN_W - text_width(ffc, scale)) / 2,
              fby + (BTN_H - FONT_H * scale) / 2, scale, ffc);

    /* Auto-shutter button. AS1 = the camera runs its own flat-field every 60s, closing the
     * shutter and briefly freezing the image. AS0 = suppressed. */
    const char *shut = s_autoshutter_on ? "AS1" : "AS0";
    int shby = slot_btn_y(SLOT_SHUT);
    draw_rect(btn_x(), shby, BTN_W, BTN_H, 0xFFFF);
    draw_text(btn_x() + (BTN_W - text_width(shut, scale)) / 2,
              shby + (BTN_H - FONT_H * scale) / 2, scale, shut);

    /* Palette button (label shows the palette currently in use) */
    const char *pal = PALETTE_NAME[s_palette];
    int pby = slot_btn_y(SLOT_PAL);
    draw_rect(btn_x(), pby, BTN_W, BTN_H, 0xFFFF);
    draw_text(btn_x() + (BTN_W - text_width(pal, scale)) / 2,
              pby + (BTN_H - FONT_H * scale) / 2, scale, pal);

    /* Scale lock button (label shows the mode currently in use) */
    const char *scl = s_scale_manual ? "LOCK" : "AUTO";
    int sby = slot_btn_y(SLOT_SCL);
    draw_rect(btn_x(), sby, BTN_W, BTN_H, 0xFFFF);
    draw_text(btn_x() + (BTN_W - text_width(scl, scale)) / 2,
              sby + (BTN_H - FONT_H * scale) / 2, scale, scl);

    /* Scale endpoint nudges - only meaningful while the scale is frozen, so they're hidden
     * in AUTO where the next recompute would overwrite any adjustment. */
    if (s_scale_manual) {
        const int slots[4] = { SLOT_MAXPLUS, SLOT_MAXMINUS, SLOT_MINPLUS, SLOT_MINMINUS };
        const char *labels[4] = { "+", "-", "+", "-" };
        for (int i = 0; i < 4; i++) {
            int ay = adj_y(slots[i]);
            draw_rect(adj_x(), ay, ADJ_BTN_W, ADJ_BTN_H, 0xFFFF);
            draw_text(adj_x() + (ADJ_BTN_W - text_width(labels[i], scale)) / 2,
                      ay + (ADJ_BTN_H - FONT_H * scale) / 2, scale, labels[i]);
        }
    }

    s_expect_status_bar_completion = true;
    esp_lcd_panel_draw_bitmap(s_panel, s_bar_x, s_bar_y,
                               s_bar_x + s_bar_w, s_bar_y + s_bar_h, s_status_buf);
}

/* 3x3 average at the native sensor centre: a single pixel is noisy enough to make the reading
 * jitter by a degree or more frame to frame. */
/* Applies the flat-field offset to one pixel. Folded into the passes that already read the
 * pixel rather than materialising a corrected copy: the separate pass cost ~4.8ms/frame (read
 * raw + read offset + write copy, then re-read the copy, all PSRAM) which exceeded the ~3ms of
 * headroom in the 40ms budget and tipped the pipeline into backlog. */
static inline uint16_t ffc_px(const uint16_t *raw, size_t i)
{
    if (!s_ffc_active) {
        return raw[i];
    }
    int32_t v = (int32_t)raw[i] - s_ffc_offset[i];
    if (v < 0) { v = 0; }
    if (v > 65535) { v = 65535; }
    return (uint16_t)v;
}

/* Words the camera actually sends to deliver npix pixels, stuffing included. */
static size_t raw_plane_words(size_t npix)
{
    const size_t stride = s_therm->marker_stride;
    if (!stride || npix == 0) {
        return npix;
    }
    return npix + (npix - 1) / (stride - 1);
}

/* Copies the plane out, dropping the stuffed marker words so that pixel n lands at index n.
 *
 * Done as a handful of memcpy runs rather than a per-pixel test: there are only nine markers in
 * a 96x96 plane, so this is ~9 straight copies of 2000 bytes and costs microseconds, where a
 * modulo per pixel would run 9216 times. Everything downstream - flat-field, min/max, palette,
 * centre patch - then indexes a plain dense image and needs no knowledge of the stuffing. */
static const uint16_t *destuff_plane(const uint16_t *src, size_t npix)
{
    const size_t stride = s_therm->marker_stride;
    size_t out = 0, in = 0;
    while (out < npix) {
        /* How many pixels remain before the next marker, from wherever we are in the pattern. */
        size_t run = stride - 1 - (in % stride);
        size_t n = npix - out;
        if (n > run) {
            n = run;
        }
        memcpy(&s_raw_destuff[out], &src[in], n * sizeof(uint16_t));
        out += n;
        in  += n;
        if ((in % stride) == stride - 1) {
            in++;                      /* step over the marker */
        }
    }
    return s_raw_destuff;
}

/* True for a word that is a marker rather than a measurement - see invalid_at. */
static inline bool raw_invalid(uint16_t v)
{
    const uint16_t lim = s_therm->invalid_at;
    return lim && v >= lim;
}

static uint16_t centre_raw(const uint16_t *raw16)
{
    uint32_t sum = 0;
    int n = 0;
    const int w = s_therm->raw_w;
    const int cx = w / 2, cy = s_therm->raw_h / 2;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            const uint16_t v = ffc_px(raw16, (size_t)(cy + dy) * w + (cx + dx));
            if (!raw_invalid(v)) {
                sum += v;
                n++;
            }
        }
    }
    /* A marker landing in the 3x3 centre patch must not drag the reading down towards zero;
     * averaging only the real pixels keeps the crosshair honest. All nine invalid is possible
     * in principle, so fall back to the raw centre rather than dividing by zero. */
    return n ? (uint16_t)(sum / n) : ffc_px(raw16, (size_t)cy * w + cx);
}

/* Readouts overlaid along the bottom of the image. Drawn into s_rgb_buf before the byte swap
 * like the crosshair, so they cost no extra SPI transfer at all - and unlike the side bar they
 * are not throttled, so they track the scene at the full frame rate. Each gets a black backing
 * box because white text is invisible over the hot end of either palette. */
static void draw_readouts(uint16_t min_v, uint16_t max_v, uint16_t centre_v)
{
    const int scale = 2;
    char lo[16], mid[16], hi[16];
    snprintf(lo,  sizeof(lo),  "%.1f", raw_to_celsius(min_v));
    snprintf(mid, sizeof(mid), "%.1f", raw_to_celsius(centre_v));
    snprintf(hi,  sizeof(hi),  "%.1f", raw_to_celsius(max_v));

    int th = FONT_H * scale;
    int ty = s_fb_h - th - 1; /* as low as the buffer allows */
    const char *txt[3] = { lo, mid, hi };
    int tx[3];
    tx[0] = 6;
    tx[1] = (s_img_w - text_width(mid, scale)) / 2;
    tx[2] = s_img_w - text_width(hi, scale) - 6;

    for (int i = 0; i < 3; i++) {
        int w = text_width(txt[i], scale);
        fill_rect_to(s_rgb_buf, s_img_w, s_fb_h, tx[i] - 3, ty - 2, w + 6, th + 4, 0x0000);
        draw_text_to(s_rgb_buf, s_img_w, s_fb_h, tx[i], ty, scale, 0xFFFF, txt[i]);
    }
}

/* Drawn into s_rgb_buf AFTER the PPA scale but BEFORE the byte swap: post-scale keeps the lines
 * 1px crisp (drawing pre-scale would let PPA stretch them 1.75x/1.5x), and pre-swap means the
 * existing swap loop handles endianness, so this is plain RGB565. Gapped reticle so the pixels
 * actually being measured aren't covered by the marker reporting them. */
static void draw_crosshair(void)
{
    const uint16_t colour = 0x07FF; /* cyan - the ironbow ramp has no cyan anywhere, so it stays
                                     * readable over black, red, orange, yellow and white alike */
    const int arm = 14, gap = 5, thick = 2;
    const int cx = s_img_w / 2, cy = s_img_h / 2;

    for (int t = 0; t < thick; t++) {
        int yy = cy + t - thick / 2;
        if (yy >= 0 && yy < s_img_h) {
            for (int d = gap; d <= gap + arm; d++) {
                if (cx - d >= 0)      s_rgb_buf[yy * s_img_w + (cx - d)] = colour;
                if (cx + d < s_img_w) s_rgb_buf[yy * s_img_w + (cx + d)] = colour;
            }
        }
        int xx = cx + t - thick / 2;
        if (xx >= 0 && xx < s_img_w) {
            for (int d = gap; d <= gap + arm; d++) {
                if (cy - d >= 0)      s_rgb_buf[(cy - d) * s_img_w + xx] = colour;
                if (cy + d < s_img_h) s_rgb_buf[(cy + d) * s_img_w + xx] = colour;
            }
        }
    }
}

/* ---------------- Display bring-up (raw esp_lcd, no LVGL) ---------------- */

/* Runs in ISR context (SPI DMA completion) - keep it to counters only, no logging/printf. */
/* Strips remaining in the frame currently on the bus; 0 means "single draw". */
static volatile int s_blit_strips_left;
static bool s_blit_force_strips;   /* a full-frame transfer has failed: never retry it */

static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)io; (void)edata; (void)user_ctx;
    if (s_expect_status_bar_completion) {
        s_expect_status_bar_completion = false;
        return false;
    }
    /* A frame may be sent as several strips (see blit_strips). Only the last one ends the frame:
     * counting or signalling per strip would inflate the stats and hand the buffer back while
     * later strips are still queued. */
    if (s_blit_strips_left > 0 && --s_blit_strips_left > 0) {
        return false;
    }
    int64_t now = esp_timer_get_time();
    s_e2e_time_sum_us += (now - s_pending_arrival_us);
    s_frames_rendered++;

    BaseType_t high_task_wakeup = pdFALSE;
    xSemaphoreGiveFromISR(s_trans_done_sem, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

/* Translate rotation_deg + mirror_h/mirror_v into the swap_xy/mirror_x/mirror_y calls the
 * ST7796 (and MADCTL-style controllers in general) actually expose. Only rotation_deg=90 has
 * been checked against real hardware; 0/180/270 follow the standard swap_xy+mirror convention
 * but are untested - worth re-verifying here first if a future OSD adds those options. */
static void display_compute_logical_size(const display_orientation_t *o, int *w, int *h)
{
    bool swapped = (o->rotation_deg == 90 || o->rotation_deg == 270);
    *w = swapped ? LCD_V_RES : LCD_H_RES;
    *h = swapped ? LCD_H_RES : LCD_V_RES;
}

/* Shared by the display and the touch controller, which must agree on this transform or taps
 * land on the wrong thing relative to what's drawn. */
static void compute_orientation_transform(const display_orientation_t *o,
                                           bool *swap_xy, bool *mirror_x, bool *mirror_y)
{
    bool base_mirror_x, base_mirror_y;
    switch (o->rotation_deg) {
    case 90:  *swap_xy = true;  base_mirror_x = true;  base_mirror_y = false; break;
    case 180: *swap_xy = false; base_mirror_x = true;  base_mirror_y = true;  break;
    case 270: *swap_xy = true;  base_mirror_x = false; base_mirror_y = true;  break;
    default:  *swap_xy = false; base_mirror_x = false; base_mirror_y = false; break; /* 0 */
    }
    *mirror_x = base_mirror_x ^ o->mirror_h;
    *mirror_y = base_mirror_y ^ o->mirror_v;
}

static esp_err_t display_apply_orientation(const display_orientation_t *o)
{
    bool swap_xy, mirror_x, mirror_y;
    compute_orientation_transform(o, &swap_xy, &mirror_x, &mirror_y);

    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_panel, swap_xy), TAG, "panel swap_xy");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, mirror_x, mirror_y), TAG, "panel mirror");
    return ESP_OK;
}

/* PPA's scale factor is quantized to 1/16 steps (PPA_SR_SCAL_X_FRAG_V == 0xF), and the driver
 * derives the number of output pixels it writes as int*in + frag*in/16. If we size a buffer from
 * an un-quantized ratio, PPA writes FEWER pixels than the buffer holds and the remainder keeps
 * whatever stale PSRAM was there - which showed up on hardware as a few rows of garbage along the
 * bottom edge of the image (292/192 quantizes down to 1.5, writing only 288 of 292 rows). So ask
 * for a target size and return what PPA will actually produce, mirroring the driver's arithmetic. */
/* Largest PPA-exact box that fits the display while preserving the source aspect ratio. Both
 * axes must share ONE 1/16 scale factor, otherwise the picture is subtly stretched - PPA
 * quantizes each axis independently and picking each one's "best fit" does not give equal
 * factors. Tries height-limited first, falls back to width-limited for a wide source. */
static void fit_preserving_aspect(int in_w, int in_h, int max_w, int max_h,
                                  int *out_w, int *out_h)
{
    int s16 = (int)(((float)max_h / in_h) * 16.0f);
    int w = (s16 / 16) * in_w + ((s16 % 16) * in_w) / 16;
    int h = (s16 / 16) * in_h + ((s16 % 16) * in_h) / 16;
    if (w > max_w) {                       /* source is wider than the panel - limit on width */
        s16 = (int)(((float)max_w / in_w) * 16.0f);
        w = (s16 / 16) * in_w + ((s16 % 16) * in_w) / 16;
        h = (s16 / 16) * in_h + ((s16 % 16) * in_h) / 16;
    }
    *out_w = w;
    *out_h = h;
}

/* PPA validates out.buffer_size against the cache line, not just the address, so the size handed
 * to it must be a multiple of 64. Rounding the USED extent up keeps each pass asking PPA to write
 * back only what it needs: the thermal path's 416x312x2 = 259584 is already a multiple of 64 and
 * is unchanged, while 374x306x2 = 228888 (24 over) becomes 228928. The buffers are allocated to
 * the largest rounded-up case, so the extra bytes are always inside the allocation. */
/* PPA validates out.buffer_size against the CACHE LINE, not just the address, so this has to
 * track the cache configuration rather than assume 64. Hardcoding it is what produced a black
 * screen once already; deriving it from Kconfig means changing CONFIG_CACHE_L2_CACHE_LINE_*
 * cannot silently break PPA again. */
#ifdef CONFIG_CACHE_L2_CACHE_LINE_SIZE
#define P2_CACHE_LINE CONFIG_CACHE_L2_CACHE_LINE_SIZE
#else
#define P2_CACHE_LINE 64
#endif

static inline size_t ppa_out_bytes(int w, int h)
{
    return ((size_t)w * h * sizeof(uint16_t) + (P2_CACHE_LINE - 1)) & ~(size_t)(P2_CACHE_LINE - 1);
}

/* Bring up the hardware JPEG decoder and its two DMA buffers for a w*h MJPEG stream.
 *
 * The input buffer gets the uncompressed size as its ceiling: a JPEG larger than its own
 * raw bitmap is not a thing, so this cannot be undersized, and it saves having to trust
 * dwMaxVideoFrameBufferSize. Both buffers come from jpeg_alloc_decoder_mem, which applies the
 * alignment the peripheral needs - the same class of requirement that cost us a black screen
 * on PPA's out.buffer_size. */
static bool jpeg_decoder_ensure(int w, int h)
{
    if (s_jpeg_dec && s_jpeg_w == w && s_jpeg_h == h) {
        return true;
    }
    if (s_jpeg_failed) {
        return false;
    }
    if (s_jpeg_dec) {
        /* Resolution changed under us (a mode switch). The buffers are sized from w*h, so
         * reusing them would silently undersize the input and overflow the output. */
        ESP_LOGI(TAG, "jpeg: resizing %dx%d -> %dx%d", s_jpeg_w, s_jpeg_h, w, h);
        jpeg_del_decoder_engine(s_jpeg_dec);
        s_jpeg_dec = NULL;
        free(s_jpeg_in);  s_jpeg_in  = NULL;
        free(s_jpeg_out); s_jpeg_out = NULL;
        s_jpeg_in_cap = s_jpeg_out_cap = 0;
    }

    const size_t want = (size_t)w * h * sizeof(uint16_t);

    const jpeg_decode_engine_cfg_t eng = {
        .intr_priority = 0,
        .timeout_ms    = 100,   /* a 25fps frame budget is 40ms; 100 flags a real stall */
    };
    esp_err_t err = jpeg_new_decoder_engine(&eng, &s_jpeg_dec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg: engine init failed: %s", esp_err_to_name(err));
        s_jpeg_dec = NULL;
        s_jpeg_failed = true;
        return false;
    }

    const jpeg_decode_memory_alloc_cfg_t in_cfg  = { .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER };
    const jpeg_decode_memory_alloc_cfg_t out_cfg = { .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER };
    s_jpeg_in  = jpeg_alloc_decoder_mem(want, &in_cfg,  &s_jpeg_in_cap);
    s_jpeg_out = jpeg_alloc_decoder_mem(want, &out_cfg, &s_jpeg_out_cap);
    if (!s_jpeg_in || !s_jpeg_out) {
        ESP_LOGE(TAG, "jpeg: buffer alloc failed (wanted %u each, in=%p out=%p)",
                 (unsigned)want, s_jpeg_in, s_jpeg_out);
        free(s_jpeg_in);  s_jpeg_in  = NULL;
        free(s_jpeg_out); s_jpeg_out = NULL;
        jpeg_del_decoder_engine(s_jpeg_dec);
        s_jpeg_dec = NULL;
        s_jpeg_failed = true;
        return false;
    }

    s_jpeg_w = w;
    s_jpeg_h = h;
    ESP_LOGI(TAG, "jpeg: decoder up for %dx%d (in %u B, out %u B)",
             w, h, (unsigned)s_jpeg_in_cap, (unsigned)s_jpeg_out_cap);
    return true;
}

/* Send one frame as a series of row strips.
 *
 * Why this exists: esp_lcd never sets SPI_TRANS_DMA_USE_PSRAM, so when the source buffer is in
 * PSRAM the SPI driver allocates an INTERNAL bounce buffer the size of the whole transfer
 * (spi_master.c: need_malloc is true for an external pointer without that flag, with mem_cap
 * MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL). A full-screen 480x320 blit therefore wanted 307,200 bytes
 * of contiguous internal RAM on EVERY frame. That silently worked only while internal RAM was
 * near-empty; bringing up esp-hosted for WiFi fragmented it to a 32 KB largest block and every
 * blit then failed with ESP_ERR_NO_MEM - and because a failed draw produces no completion
 * callback, render_task blocked on the semaphore forever.
 *
 * Splitting the frame caps that bounce at BLIT_STRIP_BYTES regardless of panel size, which also
 * removes the latent fragility from the thermal and USB paths.
 *
 * The caller must already hold s_trans_done_sem; the completion callback releases it once, after
 * the last strip. */
#define BLIT_STRIP_BYTES 16384

static esp_err_t blit_strips(int x0, int y0, int x1, int y1, const uint16_t *buf)
{
    const int w = x1 - x0;
    const int h = y1 - y0;
    if (w <= 0 || h <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Try the whole frame in one transfer first. When internal RAM has a large enough
     * contiguous block that is strictly better - measured 32.0 fps / 23.7 ms for a single
     * 480x320 draw against 20.7 fps / 49.5 ms for 16 strips, the difference being 16 bounce
     * mallocs and 16 SPI setups rather than one. Stripping is the fallback, not the default. */
    if (!s_blit_force_strips) {
        s_blit_strips_left = 1;
        esp_err_t one = esp_lcd_panel_draw_bitmap(s_panel, x0, y0, x1, y1, buf);
        if (one == ESP_OK) {
            return ESP_OK;
        }
        if (one != ESP_ERR_NO_MEM) {
            s_blit_strips_left = 0;
            xSemaphoreGive(s_trans_done_sem);
            return one;
        }
        /* Sticky, and PERMANENTLY so. Retrying the big allocation every frame measured worse
         * than just stripping (16 fps against 20.7). It was briefly re-probed every 600 frames
         * "in case the heap recovers", which was wishful: the fragmentation comes from
         * esp-hosted's SDIO init and never goes away for the life of the boot, so the re-probe
         * could only ever fail - emitting three driver errors each time (spi_master,
         * lcd_panel.io.spi, st7796_general) roughly twice a minute, which looks like a fault and
         * is not one. If no USB camera is attached the radio never starts, the heap is never
         * fragmented, and this flag is never set in the first place. */
        s_blit_force_strips = true;
        ESP_LOGW(TAG, "blit: full-frame transfer needs %d bytes of contiguous internal RAM "
                      "(largest is %u) - switching to strips",
                 w * h * 2, (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    }

    const size_t row_bytes = (size_t)w * sizeof(uint16_t);
    int rows = (int)(BLIT_STRIP_BYTES / row_bytes);
    if (rows < 1) {
        rows = 1;
    }
    if (rows > h) {
        rows = h;
    }
    const int strips = (h + rows - 1) / rows;

    /* Set before the first draw: a completion can fire while we are still queueing. */
    s_blit_strips_left = strips;
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < strips; i++) {
        const int sy = y0 + i * rows;
        const int sh = (sy + rows > y1) ? (y1 - sy) : rows;
        esp_err_t e = esp_lcd_panel_draw_bitmap(s_panel, x0, sy, x1, sy + sh,
                                                buf + (size_t)(sy - y0) * w);
        if (e != ESP_OK) {
            /* Nothing more will complete, so account for the strips that will never report. */
            ret = e;
            s_blit_strips_left -= (strips - i);
            if (s_blit_strips_left <= 0) {
                s_blit_strips_left = 0;
                xSemaphoreGive(s_trans_done_sem);
            }
            break;
        }
    }
    return ret;
}

#if P2_WIFI_CAM
/* Rotate + scale + byte-swap a decoded RGB565 frame straight into the transmit buffer.
 *
 * PPA can only rotate in 90-degree steps, so an arbitrary roll angle has to be done here. Doing
 * it in one pass also replaces BOTH PPA passes this path used to need, so the extra CPU work is
 * partly paid for by the two PPA transfers and the intermediate buffer it removes.
 *
 * Inverse-mapped (output -> source) with nearest sampling and 16.16 fixed point, stepping
 * incrementally along each row so the inner loop is two adds, a bounds test and a load. Pixels
 * that map outside the source are black - with a square source inside a square target that only
 * happens at the very corners. */
static void wifi_rotate_blit(const uint16_t *src, int sw, int sh,
                             uint16_t *dst, int dw, int dh, float deg)
{
    /* Negated: this cancels the rotation the raw frame already carries. (The spec's own
     * "apply as -angle" is about ITS `angle` field, not a roll we derive ourselves from
     * atan2(y, z) - the two share no sign convention, and assuming they did cost a round trip.) */
    const float rad = -(deg * WIFI_ROT_GAIN + WIFI_ROT_OFFSET_DEG) * 0.01745329252f;
    const float cs = cosf(rad), sn = sinf(rad);
    const float scale = (float)sw / (float)dw;  /* source pixels per output pixel */

    const float cu = dw * 0.5f, cv = dh * 0.5f;
    const float cx = sw * 0.5f, cy = sh * 0.5f;

    const int32_t dxdu = (int32_t)(cs * scale * 65536.0f);
    const int32_t dydu = (int32_t)(-sn * scale * 65536.0f);

#if P2_WIFI_CIRCLE_MASK
    const float r = (dw < dh ? dw : dh) * 0.5f;
    const float r2 = r * r;
#endif
    for (int v = 0; v < dh; v++) {
        const float dv = (float)v - cv;
        uint16_t *row = dst + (size_t)v * dw;

        int u0 = 0, u1 = dw - 1;
#if P2_WIFI_CIRCLE_MASK
        /* Span of the inscribed circle on this row. Outside it nothing is sampled at all. */
        const float dy2 = dv * dv;
        if (dy2 >= r2) {
            memset(row, 0, (size_t)dw * sizeof(uint16_t));
            continue;
        }
        const float hw = sqrtf(r2 - dy2);
        u0 = (int)(cu - hw);
        u1 = (int)(cu + hw);
        if (u0 < 0) { u0 = 0; }
        if (u1 > dw - 1) { u1 = dw - 1; }
        if (u0 > 0) { memset(row, 0, (size_t)u0 * sizeof(uint16_t)); }
        if (u1 < dw - 1) { memset(row + u1 + 1, 0, (size_t)(dw - 1 - u1) * sizeof(uint16_t)); }
#endif
        /* Start the incremental walk at u0, not 0, since the leading pixels were skipped. */
        int32_t x = (int32_t)((cx + (((float)u0 - cu) * cs + dv * sn) * scale) * 65536.0f);
        int32_t y = (int32_t)((cy + (-((float)u0 - cu) * sn + dv * cs) * scale) * 65536.0f);
        for (int u = u0; u <= u1; u++) {
            const int sx = x >> 16, sy = y >> 16;
            if ((unsigned)sx < (unsigned)sw && (unsigned)sy < (unsigned)sh) {
                const uint16_t px = src[(size_t)sy * sw + sx];
                row[u] = (uint16_t)((px >> 8) | (px << 8));   /* the ST7796's byte order */
            } else {
                row[u] = 0;
            }
            x += dxdu;
            y += dydu;
        }
    }
}
#endif

static int ppa_quantized_out(int in_dim, int desired_out)
{
    int sixteenths = (int)(((float)desired_out / in_dim) * 16.0f); /* floor, as the driver casts */
    return (sixteenths / 16) * in_dim + ((sixteenths % 16) * in_dim) / 16;
}

/* Panel layout for the attached thermal camera: the image flush top-left at the largest size
 * that keeps the sensor's own aspect ratio, with the leftover width as the control bar.
 *
 * This used to be two hand-tuned expressions that happened to land on 416x312 for a 256x192
 * sensor - exactly 1.625x on both axes. A 96x96 sensor does not land on the same numbers, and
 * reusing them stretched it 1.2:1, so the rule is now searched rather than assumed.
 *
 * Two hard constraints:
 *  - Width must be a multiple of 32, so each row is exactly 64 bytes (one cache line) of RGB565.
 *    PPA validates its output buffer size against the cache line, and a whole number of lines
 *    per row makes any height legal without further rounding.
 *  - PPA can only produce outputs of in_dim * k/16, so a 96-wide source reaches multiples of 6
 *    and a 256-wide one reaches multiples of 16. Any width that is not exactly reachable is
 *    rejected rather than silently scaled to something else.
 *
 * Largest first, so the first candidate that satisfies both wins:
 *   256x192 -> 416x312 at 1.625x, 64px bar (unchanged, bit for bit)
 *    96x96  -> 288x288 at 3x,    192px bar. 288 is the only multiple of 32 that a 96-pixel
 *              source can reach at or below the 320-row panel, so a square sensor costs a wide
 *              bar. Geometry correct beats filling the glass. */
static void thermal_layout(void)
{
    const int max_w = ((s_disp_w - SIDE_PANEL_W) / 32) * 32;
    int img_w = 0, img_h = 0;
    for (int w = max_w; w >= 32; w -= 32) {
        if (ppa_quantized_out(s_therm->raw_w, w) != w) {
            continue;                      /* PPA cannot land on this width exactly */
        }
        const int want_h = (int)((float)w * s_therm->raw_h / s_therm->raw_w + 0.5f);
        const int h = ppa_quantized_out(s_therm->raw_h, want_h);
        if (h > 0 && h <= s_disp_h) {
            img_w = w;
            img_h = h;
            break;
        }
    }
    if (!img_w) {
        /* No aspect-correct layout exists for this sensor on this panel. Fall back to the old
         * independent-axis rule, which always produces something displayable. */
        img_w = ppa_quantized_out(s_therm->raw_w, max_w);
        img_h = ppa_quantized_out(s_therm->raw_h, s_disp_h);
        ESP_LOGW(TAG, "layout: no square-scale fit for %ux%u, falling back to %dx%d",
                 s_therm->raw_w, s_therm->raw_h, img_w, img_h);
    }
    s_img_w = img_w;
    s_img_h = img_h;
    /* REVERTED to the image height: covering all 320 rows cost ~0.7ms of transfer plus a
     * bigger byte-swap pass, which tipped the frame over 40ms and sent latency climbing
     * (39 -> 52 -> 73ms as frames backed up). Readouts sit as low as the image allows. */
    s_fb_h = s_img_h;
    s_bar_x = s_img_w;
    s_bar_y = 0;
    s_bar_w = s_disp_w - s_img_w;
    s_bar_h = s_disp_h;
    /* Flush left, so the leftover width consolidates into one usable panel on the right
     * instead of being split into two useless slivers. */
    s_img_off_x = 0;
    s_img_off_y = 0;
    ESP_LOGI(TAG, "layout: %s %ux%u -> image %dx%d (%.2fx), bar %dx%d at (%d,%d)",
             s_therm->name, s_therm->raw_w, s_therm->raw_h, s_img_w, s_img_h,
             (float)s_img_w / s_therm->raw_w, s_bar_w, s_bar_h, s_bar_x, s_bar_y);
}

static esp_err_t display_init(void)
{
    display_compute_logical_size(&s_orientation, &s_disp_w, &s_disp_h);
    thermal_layout();

    gpio_config_t bk_cfg = {
        .pin_bit_mask = 1ULL << LCD_BACKLIGHT,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bk_cfg), TAG, "backlight gpio cfg");
    gpio_set_level(LCD_BACKLIGHT, 0); /* off until panel is initialized */

    const spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SPI_CLK,
        .mosi_io_num = LCD_SPI_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = s_disp_w * s_disp_h * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus init");

    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_SPI_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 3,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle),
                         TAG, "panel io spi");

    s_trans_done_sem = xSemaphoreCreateBinary();
    assert(s_trans_done_sem);
    xSemaphoreGive(s_trans_done_sem); /* first frame's wait should pass immediately */
    const esp_lcd_panel_io_callbacks_t io_cbs = {
        .on_color_trans_done = on_color_trans_done,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_register_event_callbacks(io_handle, &io_cbs, NULL), TAG, "register cbs");

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    BOOT_MARK("lcd: spi bus + panel_io ready");
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7796(io_handle, &panel_config, &s_panel), TAG, "new st7796");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel reset");
    BOOT_MARK("lcd: panel_reset done");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel init");
    BOOT_MARK("lcd: panel_init done");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_panel, true), TAG, "panel invert");
    ESP_RETURN_ON_ERROR(display_apply_orientation(&s_orientation), TAG, "panel orientation");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "panel on");

    /* Fill whole screen black once so stale GRAM content doesn't show before the first real
     * frame. Safe to fire-and-forget here: the same static all-zero buffer is reused unmodified
     * across every call, so there's no overwrite race like the one in render_task. */
    uint16_t *blank = heap_caps_calloc(s_disp_w * 8, sizeof(uint16_t), MALLOC_CAP_DMA);
    for (int y = 0; y < s_disp_h; y += 8) {
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, s_disp_w, y + 8, blank);
    }
    /* Make sure all of those queued transfers drain (and re-sync the semaphore to "free")
     * before render_task starts using it for the real frame buffer. */
    xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
    xSemaphoreGive(s_trans_done_sem);
    heap_caps_free(blank);

    BOOT_MARK("lcd: full-screen black fill drained");
    gpio_set_level(LCD_BACKLIGHT, 1);
    ESP_LOGI(TAG, "Display initialized (logical %dx%d, rotation=%d mirror_h=%d mirror_v=%d, %dMHz SPI)",
             s_disp_w, s_disp_h, s_orientation.rotation_deg, s_orientation.mirror_h, s_orientation.mirror_v,
             LCD_PIXEL_CLOCK_HZ / 1000000);
    ESP_LOGI(TAG, "Geometry: image %dx%d at (%d,%d) [PPA-quantized], status bar %d rows at y=%d",
             s_img_w, s_img_h, s_img_off_x, s_img_off_y, s_bar_h, s_img_h);
    ESP_LOGI(TAG, "Side bar: %dx%d at (%d,%d); unused strip %dx%d under the image",
             s_bar_w, s_bar_h, s_bar_x, s_bar_y, s_img_w, s_disp_h - s_img_h);
    ESP_LOGI(TAG, "Frame buffer %dx%d (image %d rows + %d rows under it)",
             s_img_w, s_fb_h, s_img_h, s_fb_h - s_img_h);
    return ESP_OK;
}

/* ---------------- Touch (OSD toggle) ---------------- */

/* ---- power button: 1s hold to power off -------------------------------------------------------
 * The AXP2101's own PWROFF long-press (REG 0x27 bits 3:2) only offers 4/6/8/10s, and shipped
 * configured for 6s. 2s is therefore not reachable by configuration - but the chip can raise a
 * long-press IRQ at 1/1.5/2/2.5s (bits 5:4), so we take that at 1s and command the shutdown.
 *
 * Observed on hardware, status register 0x49: bit 1 = press, bit 2 = long press at IRQLEVEL,
 * bit 0 = release. Those latch even with the IRQ masked in 0x41, so polling is enough and the
 * interrupt-enable registers are left alone.
 *
 * OFFLEVEL is also dropped to its 4s minimum. That is deliberately LONGER than our 1s software
 * path, so it acts as a backstop: if this task ever wedges, holding the button still cuts power
 * in hardware. */
#define PMIC_ADDR            0x34
#define PMIC_REG_COMMON_CFG  0x10   /* bit 0: soft power off */
#define PMIC_REG_PWRON_CFG   0x27
#define PMIC_REG_IRQ_STATUS2 0x49
#define PMIC_IRQ_PRESS       0x02
#define PMIC_IRQ_LONGPRESS   0x04
#define PMIC_IRQ_RELEASE     0x01
/* IRQLEVEL=00 (1s), OFFLEVEL=00 (4s), ONLEVEL=00 (128ms).
 * IRQLEVEL mapping confirmed on hardware from two points: 01 produced a measured 1.48s gap
 * between the press and long-press interrupts, and 10 produced a verified 2s power-off. */
#define PMIC_PWRON_CFG_VALUE 0x00

static i2c_master_dev_handle_t s_pmic;

static esp_err_t pmic_rd(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_pmic, &reg, 1, val, 1, 100);
}

static esp_err_t pmic_wr(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_pmic, buf, 2, 100);
}

static void pmic_power_task(void *arg)
{
    (void)arg;
    uint8_t before = 0;
    pmic_rd(PMIC_REG_PWRON_CFG, &before);
    /* A 1s hold is short enough that a careless press powers the device down - that is the
     * requested behaviour, not an oversight. */
    if (pmic_wr(PMIC_REG_PWRON_CFG, PMIC_PWRON_CFG_VALUE) != ESP_OK) {
        ESP_LOGW(TAG, "power: could not configure PWRON timing - leaving hardware default");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "power: PWRON cfg 0x%02x -> 0x%02x (irq 1s, hw off 4s backstop)",
             before, PMIC_PWRON_CFG_VALUE);

    /* Clear anything latched from the power-on press itself, then require a FRESH press before
     * honouring a long press. Without that, holding the button to switch the device ON leaves a
     * long-press bit set and we would shut straight back down. */
    for (uint8_t r = 0x48; r <= 0x4A; r++) {
        uint8_t st = 0;
        if (pmic_rd(r, &st) == ESP_OK && st) {
            pmic_wr(r, st);
        }
    }

    bool armed = false;
    while (true) {
        uint8_t st = 0;
        if (pmic_rd(PMIC_REG_IRQ_STATUS2, &st) == ESP_OK && st) {
            if (st & PMIC_IRQ_PRESS) {
                armed = true;
            }
            if (st & PMIC_IRQ_RELEASE) {
                armed = false;
            }
            if (armed && (st & PMIC_IRQ_LONGPRESS)) {
                ESP_LOGW(TAG, "power: 1s hold - powering off");
                uint8_t cc = 0;
                if (pmic_rd(PMIC_REG_COMMON_CFG, &cc) == ESP_OK) {
                    pmic_wr(PMIC_REG_COMMON_CFG, (uint8_t)(cc | 0x01));
                }
                vTaskDelay(pdMS_TO_TICKS(500));   /* should not return */
            }
            pmic_wr(PMIC_REG_IRQ_STATUS2, st);    /* write-1-to-clear */
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static esp_err_t touch_init(void)
{
    i2c_master_bus_handle_t i2c_bus = NULL;
    const i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .i2c_port = TOUCH_I2C_PORT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_bus_conf, &i2c_bus), TAG, "i2c bus");
#if P2_BUILTIN_CAM
    s_i2c_bus = i2c_bus;   /* the OV5647's SCCB lives on this same bus, at 0x36 */
#endif

    /* The AXP2101 PMIC shares this bus (scanned: 0x18, 0x34 PMIC, 0x36, 0x38 touch). */
    const i2c_device_config_t pmic_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PMIC_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(i2c_bus, &pmic_cfg, &s_pmic) == ESP_OK) {
        xTaskCreate(pmic_power_task, "pmicpwr", 3072, NULL, 4, NULL);
    } else {
        ESP_LOGW(TAG, "power: PMIC not reachable - power button keeps its hardware timing");
    }




    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    tp_io_config.scl_speed_hz = 400000;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io_handle), TAG, "touch io i2c");

    /* Start from the SAME transform as the display (compute_orientation_transform), since a
     * tap otherwise lands on a different spot than what's visually under the finger. The touch
     * sensor and the LCD panel are separate physical parts, though, and don't necessarily agree
     * on which way is "up" even when both claim the same rotation - confirmed on hardware: the
     * OSD toggle zone (bottom of the image) was landing at the top, a straight Y inversion. */
    bool swap_xy, mirror_x, mirror_y;
    compute_orientation_transform(&s_orientation, &swap_xy, &mirror_x, &mirror_y);
    /* esp_lcd_touch applies mirror BEFORE swap (x = x_max - x; y = y_max - y; then swap x/y), so
     * with swap_xy the reported logical Y comes from the NATIVE X axis - meaning the flag that
     * flips up/down here is mirror_x, not mirror_y. Confirmed on hardware: toggling mirror_y had
     * no visible effect (it flips the reported X, invisible since the button spans full width),
     * while the toggle zone stayed at the top instead of the bottom. */
    mirror_x = !mirror_x;
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_RST,
        .int_gpio_num = TOUCH_INT,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = swap_xy, .mirror_x = mirror_x, .mirror_y = mirror_y },
    };
    return esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &s_touch);
}

static void touch_task(void *arg)
{
    bool was_pressed = false;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_err_t rd_err = esp_lcd_touch_read_data(s_touch);
        if (rd_err != ESP_OK) {
            static int fail_count = 0;
            if (++fail_count % 40 == 1) { /* ~every 2s, so a persistent failure is visible but not spammy */
                ESP_LOGW(TAG, "esp_lcd_touch_read_data failed: %s (count=%d)", esp_err_to_name(rd_err), fail_count);
            }
            continue;
        }
        uint16_t x, y, strength;
        uint8_t point_num;
        bool pressed = esp_lcd_touch_get_coordinates(s_touch, &x, &y, &strength, &point_num, 1);
        if (pressed && point_num > 0) {
            ESP_LOGI(TAG, "touch raw: x=%u y=%u (panel logical %dx%d, status bar y>=%d)",
                     x, y, s_disp_w, s_disp_h, s_img_off_y + s_img_h);
        }

        /* The status bar is the touch target. Inside it, the palette button gets its own hit
         * box; anywhere else toggles the OSD. While the OSD is off the bar is blank, so there's
         * no visible button - any tap there just brings the OSD back. */
        /* The side bar is the touch target now. */
        bool in_bar = pressed && point_num > 0 && x >= (uint16_t)s_bar_x;
        int bx = s_bar_x + btn_x();
        int ffc_y = s_bar_y + slot_btn_y(SLOT_FFC);
        int shut_y = s_bar_y + slot_btn_y(SLOT_SHUT);
        int pal_y = s_bar_y + slot_btn_y(SLOT_PAL);
        int scl_y = s_bar_y + slot_btn_y(SLOT_SCL);
        bool in_ffc = in_bar && s_osd_enabled &&
                      x >= (uint16_t)bx && x < (uint16_t)(bx + BTN_W) &&
                      y >= (uint16_t)ffc_y && y < (uint16_t)(ffc_y + BTN_H);
        bool in_shut = in_bar && s_osd_enabled &&
                       x >= (uint16_t)bx && x < (uint16_t)(bx + BTN_W) &&
                       y >= (uint16_t)shut_y && y < (uint16_t)(shut_y + BTN_H);
        bool in_pal = in_bar && s_osd_enabled &&
                      x >= (uint16_t)bx && x < (uint16_t)(bx + BTN_W) &&
                      y >= (uint16_t)pal_y && y < (uint16_t)(pal_y + BTN_H);
        bool in_scl = in_bar && s_osd_enabled &&
                      x >= (uint16_t)bx && x < (uint16_t)(bx + BTN_W) &&
                      y >= (uint16_t)scl_y && y < (uint16_t)(scl_y + BTN_H);

        /* The small -/+ pairs only exist while the scale is locked. */
        /* Hit boxes are deliberately larger than the drawn boxes: the whole slot row counts,
         * split down the middle. 28x36 drawn is only ~4x5mm on this panel, which is below a
         * comfortable fingertip target, so the row absorbs the misses. */
        /* Each -/+ now owns a whole slot row across the full bar width, so the hit target is
         * the entire row rather than a 28px-wide box. */
        bool adj_live = in_bar && s_osd_enabled && s_scale_manual;
        #define IN_SLOT_ROW(sl) (y >= (uint16_t)(s_bar_y + slot_top(sl)) && \
                                 y <  (uint16_t)(s_bar_y + slot_top(sl) + slot_h()))
        bool in_max_plus  = adj_live && IN_SLOT_ROW(SLOT_MAXPLUS);
        bool in_max_minus = adj_live && IN_SLOT_ROW(SLOT_MAXMINUS);
        bool in_min_plus  = adj_live && IN_SLOT_ROW(SLOT_MINPLUS);
        bool in_min_minus = adj_live && IN_SLOT_ROW(SLOT_MINMINUS);

        if (in_bar && !was_pressed) {
            if (in_ffc) {
                if (s_ffc_active || s_ffc_remaining) {
                    s_ffc_active = false;
                    s_ffc_remaining = 0;
                    ffc_store_erase();
                    ESP_LOGI(TAG, "FFC cleared");
                } else {
                    s_ffc_req = true;
                    ESP_LOGI(TAG, "FFC capture requested - aim at a uniform surface");
                }
            } else if (in_shut) {
                s_autoshutter_on = !s_autoshutter_on;
                s_autoshutter_req = true;   /* render_task issues the write next frame */
                ESP_LOGI(TAG, "auto-shutter -> %s", s_autoshutter_on ? "ON" : "OFF");
            } else if (in_pal) {
                s_palette = (s_palette + 1) % PALETTE_COUNT;
                ESP_LOGI(TAG, "palette -> %s", PALETTE_NAME[s_palette]);
            } else if (in_max_minus) {
                s_adj_max_steps--;
            } else if (in_max_plus) {
                s_adj_max_steps++;
            } else if (in_min_minus) {
                s_adj_min_steps--;
            } else if (in_min_plus) {
                s_adj_min_steps++;
            } else if (in_scl) {
                if (!s_scale_manual) {
                    s_scale_latch_req = true; /* render_task captures the live range next frame */
                }
                s_scale_manual = !s_scale_manual;
                ESP_LOGI(TAG, "scale -> %s", s_scale_manual ? "LOCK" : "AUTO");
            } else {
                s_osd_enabled = !s_osd_enabled;
                ESP_LOGI(TAG, "OSD %s", s_osd_enabled ? "enabled" : "disabled");
            }
            s_osd_dirty = true;
        }
        was_pressed = in_bar;
    }
}

#if P2_PORT_TRACE
typedef struct {
    uint32_t t_us;
    uint32_t hprt;
    uint32_t gintsts;
} port_sample_t;

static port_sample_t s_trace[P2_TRACE_N];
static volatile uint32_t s_trace_head;     /* monotonic; index is head % P2_TRACE_N */
static volatile bool     s_trace_run;

static void port_trace_task(void *arg)
{
    while (true) {
        if (!s_trace_run) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        port_sample_t *e = &s_trace[s_trace_head % P2_TRACE_N];
        e->t_us    = (uint32_t)esp_timer_get_time();
        e->hprt    = USB_DWC_HS.hprt_reg.val;
        e->gintsts = USB_DWC_HS.gintsts_reg.val;
        s_trace_head++;
        /* ~100us. The failure window is ~20ms, so 384 samples cover it with room to spare. */
        esp_rom_delay_us(100);
    }
}

static void port_trace_dump(const char *why)
{
    const uint32_t head = s_trace_head;
    const uint32_t n    = (head < P2_TRACE_N) ? head : P2_TRACE_N;
    ESP_LOGW(TAG, "trace[%s]: %u samples (newest last)", why, (unsigned)n);

    /* Only transitions: 200 identical lines would bury the one that matters. */
    uint32_t last_h = 0xFFFFFFFFu, last_g = 0xFFFFFFFFu;
    int shown = 0;
    for (uint32_t i = head - n; i < head && shown < 40; i++) {
        const port_sample_t *e = &s_trace[i % P2_TRACE_N];
        /* sof/rxflvl/hchint toggle constantly - ignore them when deciding "changed". */
        const uint32_t gmask = ~(uint32_t)((1u << 3) | (1u << 4) | (1u << 25));
        if (e->hprt == last_h && (e->gintsts & gmask) == (last_g & gmask)) {
            continue;
        }
        last_h = e->hprt; last_g = e->gintsts;
        usb_dwc_hprt_reg_t h = { .val = e->hprt };
        usb_dwc_gintsts_reg_t g = { .val = e->gintsts };
        ESP_LOGW(TAG, "  %10u hprt=%08x connsts=%u conndet=%u ena=%u enachng=%u ovrcur=%u "
                      "lnsts=%u spd=%u | gintsts=%08x disconn=%u incompisoin=%u incompip=%u",
                 (unsigned)e->t_us, (unsigned)e->hprt,
                 (unsigned)h.prtconnsts, (unsigned)h.prtconndet, (unsigned)h.prtena,
                 (unsigned)h.prtenchng, (unsigned)h.prtovrcurract,
                 (unsigned)h.prtlnsts, (unsigned)h.prtspd,
                 (unsigned)e->gintsts, (unsigned)g.disconnint,
                 (unsigned)g.incompisoin, (unsigned)g.incompip);
        shown++;
    }
    /* The live register, read now - after the stack has already torn the port down. */
    usb_dwc_hprt_reg_t now = { .val = USB_DWC_HS.hprt_reg.val };
    ESP_LOGW(TAG, "  live: hprt=%08x connsts=%u ena=%u lnsts=%u",
             (unsigned)now.val, (unsigned)now.prtconnsts,
             (unsigned)now.prtena, (unsigned)now.prtlnsts);
}
#endif /* P2_PORT_TRACE */

#if P2_USB_OSD || P2_PORT_TRACE
/* Scan the 16 host channels for the isochronous IN one and report what the core was actually
 * programmed with. eptype 1 = isochronous (usb_dwc_xfer_type_t), epdir 1 = IN. */
static bool usb_isoc_chan_info(unsigned *mps, unsigned *ec, unsigned *epnum)
{
    for (int i = 0; i < 16; i++) {
        usb_dwc_hcchar_reg_t c = { .val = USB_DWC_HS.host_chans[i].hcchar_reg.val };
        if (c.eptype == 1 && c.epdir == 1 && c.mps != 0) {
            *mps = c.mps; *ec = c.ec; *epnum = c.epnum;
            return true;
        }
    }
    return false;
}
#endif

#if P2_USB_OSD
/* Drawn into the scaled RGB565 image BEFORE the byte-swap pass, so the text gets swapped along
 * with the picture and comes out the right way round. */
static void usb_osd_draw(uint16_t *buf, int bw, int bh)
{
    char l1[48], l2[48], l3[48];
    unsigned mps = 0, ec = 0, epnum = 0;
    const bool have = usb_isoc_chan_info(&mps, &ec, &epnum);

    snprintf(l1, sizeof(l1), "FMT%d %dX%d",
             (s_cam_fmt == UVC_VS_FORMAT_MJPEG) ? 1 : 2, (int)s_cam_w, (int)s_cam_h);
    if (have) {
        snprintf(l2, sizeof(l2), "MPS %u EC %u EP %u", mps, ec, epnum);
    } else {
        snprintf(l2, sizeof(l2), "MPS -");
    }
    snprintf(l3, sizeof(l3), "%u.%u FPS  C%u E%u MS",
             (unsigned)(s_stat_fps_x10 / 10), (unsigned)(s_stat_fps_x10 % 10),
             (unsigned)((s_stat_convert_us + 500) / 1000),
             (unsigned)((s_stat_e2e_us + 500) / 1000));

    char l4[48];
    snprintf(l4, sizeof(l4), "OK %u TR %u PDIS %u",
             (unsigned)s_mj_ok, (unsigned)s_mj_truncated, (unsigned)s_port_disables);

    const int sc = 2;
    const int lh = (FONT_H + 2) * sc;
    const int w  = 23 * (FONT_W + 1) * sc;
    fill_rect_to(buf, bw, bh, 0, 0, w < bw ? w : bw, lh * 4 + 4, 0x0000);
    draw_text_to(buf, bw, bh, 2, 2,          sc, 0xFFFF, l1);
    draw_text_to(buf, bw, bh, 2, 2 + lh,     sc, 0x07FF, l2);
    draw_text_to(buf, bw, bh, 2, 2 + lh * 2, sc, 0x07E0, l3);
    draw_text_to(buf, bw, bh, 2, 2 + lh * 3, sc, 0xFFE0, l4);
}
#endif /* P2_USB_OSD */

#if P2_UFRAME_CHECK
/* Runs once the port is enabled and SOFs are flowing. frnum is 16 bits (wraps every 65536
 * microframes = 8.192 s at spec rate), so a 500 ms window cannot alias. */
static void uframe_check_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        usb_dwc_hprt_reg_t h = { .val = USB_DWC_HS.hprt_reg.val };
        if (!h.prtena) {
            continue;   /* no SOFs while the port is down */
        }
        const uint32_t f0 = USB_DWC_HS.hfnum_reg.frnum;
        const int64_t  t0 = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(500));
        const uint32_t f1 = USB_DWC_HS.hfnum_reg.frnum;
        const int64_t  t1 = esp_timer_get_time();

        const uint32_t ticks = (f1 - f0) & 0xFFFF;
        const int64_t  us    = t1 - t0;
        if (!ticks || us <= 0) {
            ESP_LOGW(TAG, "uframe: no SOF ticks (port enabled but frnum static)");
            continue;
        }
        /* period in nanoseconds, to avoid a float just for a log line */
        const uint32_t period_ns = (uint32_t)((us * 1000) / ticks);
        ESP_LOGW(TAG, "uframe: %u ticks in %lld us -> %u.%03u us each (spec 125.000), "
                      "rate %u/s (spec 8000) | HFIR.frint=%u rld=%u phyif=%u spd=%u",
                 (unsigned)ticks, us, (unsigned)(period_ns / 1000), (unsigned)(period_ns % 1000),
                 (unsigned)((uint64_t)ticks * 1000000 / (uint64_t)us),
                 (unsigned)USB_DWC_HS.hfir_reg.frint,
                 (unsigned)USB_DWC_HS.hfir_reg.hfirrldctrl,
                 (unsigned)USB_DWC_HS.gusbcfg_reg.phyif,
                 (unsigned)h.prtspd);
        vTaskDelay(pdMS_TO_TICKS(8000));
    }
}
#endif

#if P2_BUILTIN_CAM
/* Bring up CSI + ISP + sensor and start streaming RGB565. Everything here is undone by
 * builtin_cam_stop(), so the ~2 MB of frame buffers only exist while the picture is on screen. */
static bool builtin_cam_start(void)
{
    if (s_bi_fd >= 0) {
        return true;
    }
    if (!s_i2c_bus) {
        return false;
    }

    if (!s_bi_video_inited) {
        /* esp_video_init registers the video devices and must run exactly once. */
        esp_video_init_csi_config_t csi[] = {{
            .sccb_config = { .init_sccb = false, .i2c_handle = s_i2c_bus, .freq = 100000 },
            .reset_pin = -1,
            .pwdn_pin  = -1,
        }};
        const esp_video_init_config_t cfg = { .csi = csi };
        esp_err_t err = esp_video_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "builtin: esp_video_init failed: %s", esp_err_to_name(err));
            return false;
        }
        s_bi_video_inited = true;
        /* esp_video's ISP pipeline controller does esp_log_level_set(TAG, ESP_LOG_DEBUG) on
         * itself (esp_video_isp_pipeline.c, gated on LOG_LOCAL_LEVEL which our
         * CONFIG_LOG_MAXIMUM_LEVEL=DEBUG satisfies). It then logs AE/AWB statistics from
         * interrupt context every frame: ~10k lines in 40s, which at 115200 baud blocks ISRs
         * long enough to trip the interrupt watchdog - measured as repeated
         * "Interrupt wdt timeout on CPU1" panics and SW_CPU_RESET loops. Put it back. */
        esp_log_level_set("ISP", ESP_LOG_WARN);
        esp_log_level_set("esp_ipa", ESP_LOG_WARN);
    }

    /* Blocking. esp_video's DQBUF hardcodes portMAX_DELAY and ignores O_NONBLOCK entirely
     * (esp_video_ioctl.c: "uint32_t ticks = portMAX_DELAY"), so there is no way to poll for
     * "is a newer frame waiting" and drain to the freshest one - an attempt to do that simply
     * blocked in the drain loop forever and rendered nothing. Staleness is bounded by the buffer
     * count instead; see BUILTIN_BUFS. */
    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "builtin: open %s failed", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        return false;
    }

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {
            .width       = BUILTIN_CAM_W,
            .height      = BUILTIN_CAM_H,
#if P2_BUILTIN_YUV420
            /* The ISP's own format check accepts YUV420 only with a known encoding and
             * quantization, so state them rather than relying on 0 meaning DEFAULT. */
            .pixelformat = V4L2_PIX_FMT_YUV420,
            .ycbcr_enc   = V4L2_YCBCR_ENC_601,
            .quantization = V4L2_QUANTIZATION_LIM_RANGE,
#else
            .pixelformat = V4L2_PIX_FMT_RGB565,   /* the ISP demosaics the sensor's Bayer */
#endif
        },
    };
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "builtin: S_FMT %dx%d failed", BUILTIN_CAM_W, BUILTIN_CAM_H);
        close(fd);
        return false;
    }

    struct v4l2_requestbuffers req = {
        .count  = BUILTIN_BUFS,
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "builtin: REQBUFS failed");
        close(fd);
        return false;
    }

    for (int i = 0; i < BUILTIN_BUFS; i++) {
        struct v4l2_buffer buf = {
            .index  = i,
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "builtin: QUERYBUF %d failed", i);
            close(fd);
            return false;
        }
        s_bi_buf[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                           fd, buf.m.offset);
        s_bi_len[i] = buf.length;
        /* esp_video's mmap shim documents NULL on failure, not MAP_FAILED - it does not even
         * define MAP_FAILED. */
        if (s_bi_buf[i] == NULL) {
            ESP_LOGE(TAG, "builtin: mmap %d (%u B) failed", i, (unsigned)buf.length);
            close(fd);
            return false;
        }
        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "builtin: QBUF %d failed", i);
            close(fd);
            return false;
        }
    }

    /* Mirror in the SENSOR, not in PPA. PPA's mirror_x/mirror_y had no visible effect in either
     * pass (the driver does program the registers, so this is not an obvious software gap), and
     * the OV5647 can do it for free: its driver exposes ESP_CAM_SENSOR_HMIRROR, which esp_video
     * maps to V4L2_CID_HFLIP.
     *
     * The catch is that the sensor mirrors BEFORE our rotation, so the rotation has to change to
     * compensate. With a source W x H and Ms(x,y) = (W-1-x, y):
     *
     *     wanted:   Mh . R90  = (H-1-y, W-1-x)   <- an anti-transpose; no pure rotation does this
     *     R90 . Ms            = (y, x)           <- a plain transpose, NOT what we want
     *     R270 . Ms           = (H-1-y, W-1-x)   <- exactly the target
     *
     * Hence HFLIP here and ROTATION_ANGLE_270 below, rather than 90. */
    /* VIDIOC_S_EXT_CTRLS, not VIDIOC_S_CTRL: esp_video implements only the extended-controls
     * ioctl (see the case list in esp_video_ioctl.c), which is why the plain one was refused.
     *
     * Despite the name, this flips the OV5647 VERTICALLY - see the rotation comment below. The
     * pair (this flip, ROTATION_ANGLE_90) is what produces an upright picture. */
    struct v4l2_ext_control hflip_ctrl = { .id = V4L2_CID_HFLIP, .value = 1 };
    struct v4l2_ext_controls hflip = { .count = 1, .controls = &hflip_ctrl };
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &hflip) != 0) {
        ESP_LOGW(TAG, "builtin: V4L2_CID_HFLIP not accepted - image will be mirrored");
    } else {
        ESP_LOGI(TAG, "builtin: sensor flip enabled (V4L2_CID_HFLIP; acts vertically here)");
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "builtin: STREAMON failed");
        close(fd);
        return false;
    }

    /* The sensor is mounted rotated relative to the panel: the picture is only upright after a
     * 90 degree COUNTER-CLOCKWISE rotation followed by a horizontal mirror (confirmed by eye on
     * hardware). PPA does both in the scale pass - its rotation_angle is documented as
     * counter-clockwise, and mirror_x/mirror_y act on the TARGET block, i.e. after the rotation,
     * which is the order we need.
     *
     * Rotating swaps the axes, so the aspect fit is done on the ROTATED size: 640x800 into
     * 480x320 gives 240x300, not the 400x320 an unrotated 800x640 would give. */
    /* Crop the sensor frame to the panel's aspect BEFORE the rotation, so the result fills the
     * screen exactly instead of letterboxing. Rotating swaps the axes, so for a 3:2 panel the
     * pre-rotation crop needs height/width = 3/2, i.e. 640x960 out of 1280x960.
     *
     * 960x640 after rotation fits 480x320 at exactly 8/16 in both axes - on PPA's 1/16 grid, no
     * quantization loss - and it halves PPA's read: 614,400 pixels instead of 1,228,800.
     *
     * The cost is field of view: sensor x becomes screen y, so taking half the sensor's columns
     * costs half the VERTICAL view. Full frame would keep all of it but only fill 240x320 of the
     * 480x320 panel. Offsets kept even so chroma siting is unaffected. */
    s_bi_sh = BUILTIN_CAM_H;
    s_bi_sw = (int)((int64_t)BUILTIN_CAM_H * s_disp_h / s_disp_w) & ~1;   /* 960*320/480 = 640 */
    if (s_bi_sw > BUILTIN_CAM_W) {
        s_bi_sw = BUILTIN_CAM_W;   /* sensor is already narrower than the panel's aspect wants */
    }
    s_bi_sx = ((BUILTIN_CAM_W - s_bi_sw) / 2) & ~1;
    s_bi_sy = 0;
    fit_preserving_aspect(s_bi_sh, s_bi_sw, s_disp_w, s_disp_h, &s_bi_w, &s_bi_h);
    s_bi_x = (s_disp_w - s_bi_w) / 2;
    s_bi_y = (s_disp_h - s_bi_h) / 2;

    for (int i = 0; i < 2; i++) {
        if (!s_bi_tx[i]) {
            s_bi_tx[i] = heap_caps_aligned_alloc(P2_CACHE_LINE, ppa_out_bytes(s_disp_w, s_disp_h),
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
            if (!s_bi_tx[i]) {
                ESP_LOGE(TAG, "builtin: tx buffer %d alloc failed", i);
                close(fd);
                return false;
            }
        }
    }
    s_bi_tx_i = 0;
    s_bi_fd = fd;
    s_bi_frames = 0;
    ESP_LOGI(TAG, "builtin: OV5647 %s %dx%d crop %dx%d+%d+%d -> %dx%d at (%d,%d)",
             P2_BUILTIN_YUV420 ? "YUV420" : "RGB565",
             BUILTIN_CAM_W, BUILTIN_CAM_H, s_bi_sw, s_bi_sh, s_bi_sx, s_bi_sy,
             s_bi_w, s_bi_h, s_bi_x, s_bi_y);
    return true;
}

static void builtin_cam_stop(void)
{
    if (s_bi_fd < 0) {
        return;
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(s_bi_fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < BUILTIN_BUFS; i++) {
        if (s_bi_buf[i]) {
            munmap(s_bi_buf[i], s_bi_len[i]);
        }
        s_bi_buf[i] = NULL;
        s_bi_len[i] = 0;
    }
    /* Releasing the buffers is the point - a USB camera's MJPEG path wants this PSRAM back. */
    struct v4l2_requestbuffers req = {
        .count = 0, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP,
    };
    ioctl(s_bi_fd, VIDIOC_REQBUFS, &req);
    close(s_bi_fd);
    s_bi_fd = -1;
    /* Wait for the last blit before freeing. This was previously commented as already-complete,
     * which stopped being true the moment the blit was pipelined: render_task now issues
     * draw_bitmap and does NOT hold the completion semaphore afterwards, so without this the SPI
     * could still be reading s_bi_tx[i] as it is freed - a use-after-free that fires precisely
     * when a USB camera arrives and the built-in preview is torn down. */
    xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
    for (int i = 0; i < 2; i++) {
        free(s_bi_tx[i]);
        s_bi_tx[i] = NULL;
    }
    xSemaphoreGive(s_trans_done_sem);
    ESP_LOGI(TAG, "builtin: stopped (%u frames shown)", (unsigned)s_bi_frames);
}
#endif /* P2_BUILTIN_CAM */

#if P2_RENDER_STAGE_TRACE
static volatile const char *s_render_stage = "init";
static volatile uint32_t    s_render_laps;
#define RSTAGE(x) do { s_render_stage = (x); } while (0)
#define RLAP()    do { s_render_laps++; } while (0)

static void render_stage_task(void *arg)
{
    uint32_t last = 0;
    const char *last_stage = NULL;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        const uint32_t laps = s_render_laps;
        const char *st = (const char *)s_render_stage;
        if (laps == last && st == last_stage) {
            ESP_LOGE(TAG, "render STUCK in \"%s\" (laps=%u, unchanged 2s)",
                     st, (unsigned)laps);
        } else {
            ESP_LOGI(TAG, "render ok: +%u laps, now \"%s\"", (unsigned)(laps - last), st);
        }
        last = laps;
        last_stage = st;
    }
}
#else
#define RSTAGE(x) do { } while (0)
#define RLAP()    do { } while (0)
#endif

#if P2_WIFI_CAM
static bool s_wifi_inited;
/* Connection state, driven by events rather than polled. The disconnect REASON is the single
 * most useful thing here: without it a failure to connect is indistinguishable from a slow DHCP
 * server, and those need opposite fixes. */
static EventGroupHandle_t s_wifi_evt;
#define WIFI_BIT_CONNECTED  BIT0
#define WIFI_BIT_FAILED     BIT1
static volatile int s_wifi_last_reason;
static uint8_t  s_wifi_last_ch;       /* channel the camera was last seen on */
static bool     s_wifi_use_dhcp;      /* set if the static-IP assumption ever fails */
static uint32_t s_wifi_attempts, s_wifi_joins;

static const char *wifi_reason_str(int r)
{
    switch (r) {
    case WIFI_REASON_AUTH_EXPIRE:        return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE:         return "AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE:       return "ASSOC_EXPIRE";
    case WIFI_REASON_ASSOC_TOOMANY:      return "ASSOC_TOOMANY (AP full - one client only!)";
    case WIFI_REASON_NOT_AUTHED:         return "NOT_AUTHED";
    case WIFI_REASON_NOT_ASSOCED:        return "NOT_ASSOCED";
    case WIFI_REASON_ASSOC_LEAVE:        return "ASSOC_LEAVE";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:  return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND:        return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL:          return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL:         return "ASSOC_FAIL";
    case WIFI_REASON_BEACON_TIMEOUT:     return "BEACON_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL:    return "CONNECTION_FAIL";
    default:                             return "other";
    }
}

static void wifi_evt_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base != WIFI_EVENT) {
        return;
    }
    if (id == WIFI_EVENT_STA_CONNECTED) {
        xEventGroupSetBits(s_wifi_evt, WIFI_BIT_CONNECTED);
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = (const wifi_event_sta_disconnected_t *)data;
        s_wifi_last_reason = d ? d->reason : -1;
        xEventGroupSetBits(s_wifi_evt, WIFI_BIT_FAILED);
    }
}
/* Two slots so the task can assemble the next frame while render_task decodes the current one. */
static uint8_t *s_wifi_jpeg[2];
static size_t   s_wifi_jpeg_len[2];
static QueueHandle_t s_wifi_frame_q;      /* carries the ready slot index, depth 1 */
static volatile bool s_wifi_active;       /* a frame has arrived recently */
static uint32_t s_wifi_frames, s_wifi_dropped;
/* Split so a drop names its own cause: one lumped counter cannot distinguish a lossy link from
 * a renderer that is too slow, and those need opposite fixes. */
static uint32_t s_wd_reasm;    /* chunk gap / no SOI - the link lost a datagram */
static uint32_t s_wd_busy;     /* render_task had not taken the previous frame */
static uint32_t s_wd_info;     /* jpeg_decoder_get_info rejected the bitstream */
static uint32_t s_wd_decode;   /* the hardware decoder rejected it */
/* Orientation. s_wifi_rot_deg is what the renderer applies; the two candidates are kept
 * separately so a log line can show which one actually tracks reality. */
static volatile float s_wifi_rot_deg;
static volatile float s_imu_angle_deg;
static volatile float s_rot_xy, s_rot_yz, s_rot_xz;   /* all three candidates, for comparison */
static volatile int16_t s_imu_x, s_imu_y, s_imu_z;
static uint32_t s_imu_pkts;
static int64_t  s_wifi_asm_us;    /* first chunk -> last chunk of one frame */
static int64_t  s_wifi_gap_us;    /* interval between completed frames */
static int64_t  s_wifi_q_us;      /* frame complete -> render_task picks it up */
static int64_t  s_wifi_jpeg_t[2]; /* when each slot was filled */
static int64_t  s_wifi_wait_us;   /* stall waiting for the previous blit */
static int64_t  s_wifi_dec_us;
static int64_t  s_wifi_rot_us;   /* memcpy + JPEG decode + the rotate pass, measured together */
/* On-panel geometry, recomputed when the decoded frame size changes. The spec warns not to
 * hardcode 432x432: the NE3 variant of this camera family is 640x360, so the dimensions are read
 * from the JPEG header instead. */
static int s_wc_sw, s_wc_sh, s_wc_w, s_wc_h, s_wc_x, s_wc_y;
/* Two output buffers, as the built-in path uses. With one, the rotate pass cannot start until
 * the previous frame's blit has finished - measured at ~21 ms of pure stall per frame, on a
 * budget where our whole contribution was 60-110 ms. Alternating lets this frame's rotate
 * overlap the previous frame's transfer. */
static uint16_t *s_wc_tx[2];
static int       s_wc_tx_i;

/* Bring the radio up once. esp_wifi_remote forwards every call to the C6 over esp-hosted, so a
 * failure here is most likely the C6 not running the hosted slave firmware rather than anything
 * wrong with these calls. */
static bool wifi_cam_init(void)
{
    if (s_wifi_inited) {
        return true;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi: nvs_flash_init: %s", esp_err_to_name(err));
        return false;
    }
    if ((err = esp_netif_init()) != ESP_OK) {
        ESP_LOGE(TAG, "wifi: esp_netif_init: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "wifi: event loop: %s", esp_err_to_name(err));
        return false;
    }
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "wifi: esp_wifi_init: %s - is the C6 running esp-hosted slave firmware?",
                 esp_err_to_name(err));
        return false;
    }
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if ((err = esp_wifi_start()) != ESP_OK) {
        ESP_LOGE(TAG, "wifi: esp_wifi_start: %s", esp_err_to_name(err));
        return false;
    }
    s_wifi_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_evt_handler, NULL, NULL));
    s_wifi_inited = true;
    ESP_LOGI(TAG, "wifi: radio up via the onboard C6");
    return true;
}

/* Scan and report every AP. Returns true if one matches WIFI_CAM_PREFIX, copying its SSID out. */
static bool wifi_cam_scan(char *ssid_out, size_t ssid_len, uint8_t hint_ch, uint8_t *ch_out)
{
    /* A hinted single-channel scan is several times faster than sweeping all 13 - and once the
     * camera has been seen, its channel is known. Falls back to a full scan when the hint misses
     * (the camera does move channel: observed on 2, and once on 1). */
    wifi_scan_config_t scan = { .show_hidden = false, .channel = hint_ch };
    esp_err_t err = esp_wifi_scan_start(&scan, true);   /* blocking */
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "wifi: scan failed: %s", esp_err_to_name(err));
        return false;
    }
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n == 0) {
        ESP_LOGI(TAG, "wifi: no APs in range");
        return false;
    }
    wifi_ap_record_t *recs = calloc(n, sizeof(wifi_ap_record_t));
    if (!recs) {
        return false;
    }
    uint16_t got = n;
    esp_wifi_scan_get_ap_records(&got, recs);
    bool found = false;
    for (int i = 0; i < got; i++) {
        const char *ss = (const char *)recs[i].ssid;
        const bool match = (strncmp(ss, WIFI_CAM_PREFIX, strlen(WIFI_CAM_PREFIX)) == 0);
        ESP_LOGI(TAG, "wifi:   %-32s ch%-3d rssi %-5d auth %d%s",
                 ss, recs[i].primary, recs[i].rssi, (int)recs[i].authmode,
                 match ? "   <== earpick camera" : "");
        if (match && !found) {
            found = true;
            if (ssid_out) {
                strlcpy(ssid_out, ss, ssid_len);
            }
            if (ch_out) {
                *ch_out = recs[i].primary;
            }
        }
    }
    free(recs);
    return found;
}

/* Join the camera's open AP and wait for DHCP. Returns the assigned gateway, which for NE7 is
 * the camera itself - the spec says to select the protocol by gateway address, not by the brand
 * string, which misreports NE3. */
/* Join the camera's open AP.
 *
 * Static IP by default, DHCP only as a fallback. The camera's AP is always 192.168.1.0/24 with
 * the camera itself on .1 - that is how the NE7 protocol is identified in the first place - and
 * its DHCP server is a BK7231U with 250 mAh of battery: measured 3.6 s to hand out a lease, on
 * the critical path, with no recourse if it simply does not answer. Assigning ourselves .100
 * removes both the delay and the failure mode. s_wifi_use_dhcp flips on if the assumption ever
 * proves wrong, and then stays flipped.
 *
 * Waits on the CONNECTED event rather than polling for an address, so a failure reports its
 * reason instead of timing out anonymously. */
static bool wifi_cam_connect(const char *ssid, uint8_t channel, esp_ip4_addr_t *gw_out)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        return false;
    }
    s_wifi_attempts++;

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;   /* the camera AP has no password */
    if (channel) {
        wc.sta.channel = channel;   /* skip the driver's own scan for the AP */
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

    esp_netif_ip_info_t want = { 0 };
    if (!s_wifi_use_dhcp) {
        esp_netif_dhcpc_stop(netif);   /* ESP_ERR_..._ALREADY_STOPPED is fine */
        want.ip.addr = esp_ip4addr_aton("192.168.1.100");
        want.netmask.addr = esp_ip4addr_aton("255.255.255.0");
        want.gw.addr = esp_ip4addr_aton("192.168.1.1");
        esp_netif_set_ip_info(netif, &want);
    } else {
        esp_netif_dhcpc_start(netif);
    }

    xEventGroupClearBits(s_wifi_evt, WIFI_BIT_CONNECTED | WIFI_BIT_FAILED);
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "wifi: connect(\"%s\") call failed: %s", ssid, esp_err_to_name(err));
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_evt, WIFI_BIT_CONNECTED | WIFI_BIT_FAILED,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(8000));
    const int64_t assoc_ms = (esp_timer_get_time() - t0) / 1000;
    if (!(bits & WIFI_BIT_CONNECTED)) {
        if (bits & WIFI_BIT_FAILED) {
            ESP_LOGW(TAG, "wifi: join failed after %lldms - reason %d (%s)  [attempt %u]",
                     assoc_ms, s_wifi_last_reason, wifi_reason_str(s_wifi_last_reason),
                     (unsigned)s_wifi_attempts);
        } else {
            ESP_LOGW(TAG, "wifi: join timed out after %lldms, no event  [attempt %u]",
                     assoc_ms, (unsigned)s_wifi_attempts);
        }
        return false;
    }

    esp_netif_ip_info_t ip = { 0 };
    if (s_wifi_use_dhcp) {
        /* Only the fallback path has to wait for a lease. */
        for (int i = 0; i < 60; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0) {
                break;
            }
        }
        if (ip.ip.addr == 0) {
            ESP_LOGW(TAG, "wifi: associated in %lldms but no DHCP lease", assoc_ms);
            return false;
        }
    } else {
        ip = want;
    }

    s_wifi_joins++;
    ESP_LOGI(TAG, "wifi: joined \"%s\" in %lldms (%s)  ip " IPSTR "  gw " IPSTR
                  "  [%u/%u ok]",
             ssid, assoc_ms, s_wifi_use_dhcp ? "dhcp" : "static",
             IP2STR(&ip.ip), IP2STR(&ip.gw), (unsigned)s_wifi_joins,
             (unsigned)s_wifi_attempts);
    if (gw_out) {
        *gw_out = ip.gw;
    }
    return true;
}

/* The NE7 video/IMU session. Owns one UDP socket; returns when the stream dies so the caller can
 * rescan and start over. */
static void wifi_cam_session(esp_ip4_addr_t gw)
{
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        ESP_LOGE(TAG, "wifi: socket failed");
        return;
    }
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    struct timeval tv = { .tv_sec = 0, .tv_usec = 300000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_port = htons(WIFI_CAM_LOCAL_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(fd, (struct sockaddr *)&local, sizeof(local)) != 0) {
        ESP_LOGE(TAG, "wifi: bind :%d failed", WIFI_CAM_LOCAL_PORT);
        close(fd);
        return;
    }

    /* Discovery: a 4-byte probe to both documented ports. The reply's SOURCE address is the
     * camera, which is more trustworthy than hardcoding 192.168.1.1. */
    struct sockaddr_in cam = { .sin_family = AF_INET, .sin_addr.s_addr = gw.addr };
    /* 0x30 is what the vendor app transmits (T0/d.smali). 0x39 - previously recorded in the
     * spec - also works, so the camera is lenient on this byte, but match the vendor. */
    const uint8_t probe[4] = { 0x66, 0x30, 0x01, 0x01 };
    struct sockaddr_in bcast = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = (gw.addr & htonl(0xFFFFFF00)) | htonl(0xFF),
    };
    for (int port = 0; port < 2; port++) {
        bcast.sin_port = htons(port ? WIFI_CAM_DISC_PORT_B : WIFI_CAM_DISC_PORT_A);
        sendto(fd, probe, sizeof(probe), 0, (struct sockaddr *)&bcast, sizeof(bcast));
    }

    const uint8_t start_imu[3]   = { 0x86, 0x06, 0x01 };
    const uint8_t start_video[2] = { 0x20, 0x36 };
    cam.sin_port = htons(WIFI_CAM_IMU_PORT);
    sendto(fd, start_imu, sizeof(start_imu), 0, (struct sockaddr *)&cam, sizeof(cam));
    cam.sin_port = htons(WIFI_CAM_VIDEO_PORT);
    sendto(fd, start_video, sizeof(start_video), 0, (struct sockaddr *)&cam, sizeof(cam));
    ESP_LOGI(TAG, "wifi: handshake sent to " IPSTR, IP2STR(&gw));

    /* Reassembly. Chunks must arrive in order: we append at a running offset and drop the frame
     * on any gap. That is far simpler than holding per-chunk buffers, and dropping incomplete
     * frames is what the spec prescribes anyway - s_wifi_dropped makes the cost visible, so if
     * reordering turns out to matter on this link the counter will say so. */
    uint8_t *asm_buf = heap_caps_malloc(WIFI_CAM_JPEG_MAX, MALLOC_CAP_SPIRAM);
    if (!asm_buf) {
        ESP_LOGE(TAG, "wifi: assembly buffer alloc failed");
        close(fd);
        return;
    }
    int      cur_frame = -1;
    uint32_t asm_len = 0;
    uint8_t  next_chunk = 1;
    bool     frame_bad = false;
    int      slot = 0;

    int64_t last_video_us = esp_timer_get_time();
    int64_t first_chunk_us = last_video_us, prev_done_us = last_video_us;
    bool probe_sent = false;
    int probe_i = 0;
    int64_t last_probe_us = 0;
    int64_t last_imu_tx_us = 0;
    int stalls = 0;
    uint8_t pkt[1600];

    while (true) {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int n = recvfrom(fd, pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &flen);
        const int64_t now = esp_timer_get_time();

        /* Keepalive. The camera stops sending if it decides nobody is listening. */
        if (now - last_video_us > WIFI_CAM_STALL_MS * 1000) {
            cam.sin_port = htons(WIFI_CAM_VIDEO_PORT);
            sendto(fd, start_video, sizeof(start_video), 0, (struct sockaddr *)&cam, sizeof(cam));
            last_video_us = now;
            s_wifi_active = false;
            stalls++;
            ESP_LOGW(TAG, "wifi: video stalled (%d), re-sent start", stalls);
            /* Give up after a few, ALWAYS - not only when no frame was ever received.
             *
             * This previously tested the lifetime frame counter, which is non-zero forever after
             * the first frame, so the branch was unreachable and a session whose camera had been
             * switched off looped here indefinitely, re-sending the start command to nothing and
             * never returning to rescan. Switching the camera off and on then looked like "it
             * will not connect", when in fact we had never left the dead session. The counter was
             * also static, which would have made a LATER session abort almost at once. */
            if (stalls >= WIFI_CAM_MAX_STALLS) {
                ESP_LOGE(TAG, "wifi: %d stalls, abandoning the session to rescan "
                              "(camera off, out of range, or another client took it)", stalls);
                break;
            }
        } else if (n > 0) {
            stalls = 0;   /* any traffic at all means the camera is still there */
        }

        /* Faster than waiting for stalls: losing the AP raises STA_DISCONNECTED, which is exactly
         * what happens when the camera is powered off. Leave immediately so the outer loop can
         * rescan and be ready the moment it comes back. */
        if (xEventGroupGetBits(s_wifi_evt) & WIFI_BIT_FAILED) {
            ESP_LOGW(TAG, "wifi: association lost - reason %d (%s), leaving session",
                     s_wifi_last_reason, wifi_reason_str(s_wifi_last_reason));
            break;
        }
#if P2_WIFI_PROBE_SEQ
        /* The untried opcodes from the NE3 sibling, 5 s apart once streaming is established. */
        static const uint8_t seq[][2] = {
            { 0x20, 0x35 }, { 0x20, 0x38 }, { 0x30, 0x66 }, { 0x30, 0x67 },
        };
        if (probe_i < (int)(sizeof(seq) / sizeof(seq[0])) &&
            s_wifi_frames > 60 && now - last_probe_us > 5000000) {
            last_probe_us = now;
            cam.sin_port = htons(WIFI_CAM_VIDEO_PORT);
            sendto(fd, seq[probe_i], 2, 0, (struct sockaddr *)&cam, sizeof(cam));
            ESP_LOGW(TAG, "PROBE %d: sent %02X %02X to :%d (frames=%u, %dx%d)",
                     probe_i, seq[probe_i][0], seq[probe_i][1], WIFI_CAM_VIDEO_PORT,
                     (unsigned)s_wifi_frames, s_wc_sw, s_wc_sh);
            probe_i++;
        }
#elif P2_WIFI_PROBE_CMD
        if (!probe_sent && s_wifi_frames > 100) {
            probe_sent = true;
            const uint8_t pc[2] = { 0x20, (uint8_t)P2_WIFI_PROBE_CMD };
            cam.sin_port = htons(WIFI_CAM_VIDEO_PORT);
            sendto(fd, pc, sizeof(pc), 0, (struct sockaddr *)&cam, sizeof(cam));
            ESP_LOGW(TAG, "PROBE: sent 20 %02X after %u frames",
                     (unsigned)P2_WIFI_PROBE_CMD, (unsigned)s_wifi_frames);
        }
#endif
        if (now - last_imu_tx_us > 1000000) {
            cam.sin_port = htons(WIFI_CAM_IMU_PORT);
            sendto(fd, start_imu, sizeof(start_imu), 0, (struct sockaddr *)&cam, sizeof(cam));
            last_imu_tx_us = now;
        }
        if (n < 4) {
            continue;
        }
        if (ntohs(from.sin_port) == WIFI_CAM_IMU_PORT && n >= 24) {
            /* Both readings of the same 24 bytes - see the P2_WIFI_ROTATE comment. */
            const uint16_t raw_angle = ((uint16_t)pkt[18] << 8) | pkt[19];
            s_imu_angle_deg = (float)raw_angle;
            s_imu_x = (int16_t)(((uint16_t)pkt[0] << 8) | pkt[1]);
            s_imu_y = (int16_t)(((uint16_t)pkt[2] << 8) | pkt[3]);
            s_imu_z = (int16_t)(((uint16_t)pkt[4] << 8) | pkt[5]);
            const float r2d = 57.2957795f;
            s_rot_xy = atan2f((float)s_imu_x, (float)s_imu_y) * r2d;
            s_rot_yz = atan2f((float)s_imu_y, (float)s_imu_z) * r2d;
            s_rot_xz = atan2f((float)s_imu_x, (float)s_imu_z) * r2d;
            s_imu_pkts++;

#if P2_WIFI_ROTATE != WIFI_ROT_OFF
            float want;
            if (P2_WIFI_ROTATE == WIFI_ROT_ANGLE) {
                want = s_imu_angle_deg;
            } else if (P2_WIFI_ROT_AXES == WIFI_AXES_YZ) {
                want = s_rot_yz;
            } else if (P2_WIFI_ROT_AXES == WIFI_AXES_XZ) {
                want = s_rot_xz;
            } else {
                want = s_rot_xy;
            }
#if WIFI_ROT_GIMBAL_GUARD
            /* With the lens near-vertical, roll about the view axis is undefined and atan2
             * swings wildly. Hold the last stable angle. */
            const int32_t mag = (int32_t)s_imu_y * s_imu_y + (int32_t)s_imu_z * s_imu_z;
            if (P2_WIFI_ROTATE == WIFI_ROT_ACCEL && mag < (2000 * 2000)) {
                want = s_wifi_rot_deg;
            }
#endif
            /* Shortest-way-round low pass, so 359 -> 1 does not spin the long way. */
            float d = want - s_wifi_rot_deg;
            while (d > 180.0f)  { d -= 360.0f; }
            while (d < -180.0f) { d += 360.0f; }
            if (d > WIFI_ROT_DEADBAND || d < -WIFI_ROT_DEADBAND) {
                s_wifi_rot_deg += d * (WIFI_ROT_LPF_NUM / 100.0f);
            }
#endif
            continue;
        }
        if (ntohs(from.sin_port) == WIFI_CAM_DISC_PORT_B && n > 16) {
            /* The discovery descriptor, logged once. The spec lists "resolution change" under
             * known unknowns, so this is the only place the camera could advertise a mode or
             * frame rate we might ask for - worth seeing in full rather than assuming. */
            static bool shown = false;
            if (!shown) {
                shown = true;
                pkt[(n < (int)sizeof(pkt)) ? n : (int)sizeof(pkt) - 1] = 0;
                ESP_LOGW(TAG, "wifi descriptor (%d bytes):", n);
                /* Tab-indented with \n newlines; split so the log does not truncate it. */
                char *line = (char *)pkt, *nl;
                while (line && *line) {
                    nl = strchr(line, '\n');
                    if (nl) { *nl = 0; }
                    if (*line) { ESP_LOGW(TAG, "  %s", line); }
                    line = nl ? nl + 1 : NULL;
                }
            }
            continue;
        }
        if (ntohs(from.sin_port) != WIFI_CAM_VIDEO_PORT) {
            continue;   /* anything else */
        }

        const uint8_t fnum = pkt[0], flags = pkt[1], cnum = pkt[2];
        const bool last = (flags & 1) != 0;
        uint8_t *payload = pkt + 4;
        int plen = n - 4;
        if (last && plen >= WIFI_CAM_TRAILER) {
            plen -= WIFI_CAM_TRAILER;
        }

        if (fnum != cur_frame) {
            cur_frame = fnum;
            asm_len = 0;
            next_chunk = 1;
            frame_bad = false;
            first_chunk_us = now;   /* for the reassembly window */
        }
        if (cnum != next_chunk) {
            frame_bad = true;   /* gap or reorder: this frame is unusable */
        }
        if (!frame_bad && plen > 0 && asm_len + plen + 2 <= WIFI_CAM_JPEG_MAX) {
            memcpy(asm_buf + asm_len, payload, plen);
            asm_len += plen;
            next_chunk++;
        }

        if (!last) {
            continue;
        }
        last_video_us = now;
        if (frame_bad || asm_len < 4 || asm_buf[0] != 0xFF || asm_buf[1] != 0xD8) {
            s_wifi_dropped++; s_wd_reasm++;
            continue;
        }
        /* THE EOI TRAP: this camera omits FFD9 entirely. Without appending it the hardware
         * decoder has no end marker. file(1) and lenient decoders will accept the frame anyway,
         * so a "valid JPEG" is not evidence of correctness here. */
        asm_buf[asm_len++] = 0xFF;
        asm_buf[asm_len++] = 0xD9;

        if (s_wifi_jpeg[slot] && uxQueueMessagesWaiting(s_wifi_frame_q) == 0) {
            memcpy(s_wifi_jpeg[slot], asm_buf, asm_len);
            s_wifi_jpeg_len[slot] = asm_len;
            s_wifi_jpeg_t[slot] = now;
            s_wifi_asm_us = now - first_chunk_us;
            s_wifi_gap_us = now - prev_done_us;
            prev_done_us = now;
            xQueueOverwrite(s_wifi_frame_q, &slot);
            slot ^= 1;
            s_wifi_frames++;
            s_wifi_active = true;
        } else {
            s_wifi_dropped++; s_wd_busy++;   /* render_task has not consumed the previous one */
        }
    }

    /* Release the camera for the next client: it accepts exactly one, so leaving the stream
     * running makes a phone wait for a timeout. The vendor app sends this too (T0/d.smali). */
    const uint8_t stop_video[2] = { 0x20, 0x37 };
    cam.sin_port = htons(WIFI_CAM_VIDEO_PORT);
    sendto(fd, stop_video, sizeof(stop_video), 0, (struct sockaddr *)&cam, sizeof(cam));

    free(asm_buf);
    close(fd);
    s_wifi_active = false;
    ESP_LOGW(TAG, "wifi: session ended (%u frames, %u dropped)",
             (unsigned)s_wifi_frames, (unsigned)s_wifi_dropped);
}

static void wifi_cam_task(void *arg)
{
    /* Do NOT bring the radio up until it is actually wanted. esp-hosted's SDIO init fragments
     * internal RAM, which forces the LCD blit into strips (see blit_strips) and costs real
     * framerate: the built-in camera measured 32.0 fps / 23.7 ms with the radio down against
     * 20.7 fps / 49.5 ms with it up. A USB camera is the instrument and must not pay that, so
     * the radio stays off entirely while one is attached - which also matches the spec for this
     * feature: the WiFi camera is only for when nothing is on USB. */
    int idle_ticks = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
        idle_ticks = (s_cam_kind == CAM_NONE) ? idle_ticks + 1 : 0;
        if (idle_ticks >= 10) {   /* ~5 s with nothing on USB */
            break;
        }
    }
    if (!wifi_cam_init()) {
        ESP_LOGW(TAG, "wifi: unavailable, not retrying");
        vTaskDelete(NULL);
        return;
    }
    for (int i = 0; i < 2; i++) {
        s_wifi_jpeg[i] = heap_caps_malloc(WIFI_CAM_JPEG_MAX, MALLOC_CAP_SPIRAM);
    }
    for (int i = 0; i < 2; i++) {
        s_wc_tx[i] = heap_caps_aligned_alloc(P2_CACHE_LINE, ppa_out_bytes(s_disp_w, s_disp_h),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    }
    s_wifi_frame_q = xQueueCreate(1, sizeof(int));
    if (!s_wifi_jpeg[0] || !s_wifi_jpeg[1] || !s_wc_tx[0] || !s_wc_tx[1] || !s_wifi_frame_q) {
        ESP_LOGE(TAG, "wifi: buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        /* Only look while no USB camera is attached - that is the instrument and it wins. */
        if (s_cam_kind != CAM_NONE) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        char ssid[33] = { 0 };
        uint8_t ch = 0;
        /* Try the remembered channel first; only sweep everything if that misses. */
        bool seen = wifi_cam_scan(ssid, sizeof(ssid), s_wifi_last_ch, &ch);
        if (!seen && s_wifi_last_ch) {
            seen = wifi_cam_scan(ssid, sizeof(ssid), 0, &ch);
        }
        if (!seen) {
            s_wifi_last_ch = 0;
            vTaskDelay(pdMS_TO_TICKS(4000));
            continue;
        }
        s_wifi_last_ch = ch;
        ESP_LOGW(TAG, "wifi: camera \"%s\" on ch%d, joining", ssid, ch);

        /* Retry the join a few times before going back to scanning: a refused association is
         * often transient (the camera takes one client, and a previous session's socket can
         * linger), and a rescan costs far more than another attempt. */
        esp_ip4_addr_t gw = { 0 };
        bool joined = false;
        for (int attempt = 0; attempt < 3 && !joined; attempt++) {
            if (attempt) {
                esp_wifi_disconnect();
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            joined = wifi_cam_connect(ssid, ch, &gw) && gw.addr != 0;
        }
        if (joined) {
            const uint32_t before = s_wifi_frames;
            wifi_cam_session(gw);
            if (s_wifi_frames == before && !s_wifi_use_dhcp) {
                /* Associated and addressed, but the camera never sent a frame. The static-IP
                 * assumption is the thing most likely to be wrong, so fall back permanently. */
                s_wifi_use_dhcp = true;
                ESP_LOGW(TAG, "wifi: no frames with a static address - falling back to DHCP");
            }
        }
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(joined ? 2000 : 1000));
    }
}
#endif /* P2_WIFI_CAM */

/* ---------------- UVC callbacks ---------------- */

/* Frames are queued by the UVC driver task and consumed by render_task, so a disconnect can
 * free the driver's buffers while up to 3 pointers to them are still in the queue. The epoch
 * stamp is what makes a stale pointer recognisable: it is bumped on every open and every
 * disconnect, and render_task refuses to dereference an entry whose stamp does not match.
 * Without this, swapping cameras reads freed memory - and does it with the NEW camera's
 * geometry, which is why a reset was needed. */
typedef struct {
    uvc_host_frame_t *frame;
    int64_t arrival_us;
    uint32_t epoch;
} queued_frame_t;

static volatile uint32_t s_stream_epoch;

static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    queued_frame_t qf = {
        .frame = (uvc_host_frame_t *)frame,
        .arrival_us = esp_timer_get_time(),
        .epoch = s_stream_epoch,
    };
    if (xQueueSendToBack(s_frame_queue, &qf, 0) != pdPASS) {
        return true; /* queue full: drop this frame, return it immediately */
    }
    return false; /* ownership passed to render_task, which must call uvc_host_frame_return() */
}

static void stream_event_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGW(TAG, "Transfer error, err_no=%d", event->transfer_error.error);
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
#if P2_PORT_TRACE
        /* Before anything else: this callback runs on the UVC driver task, so the port may
         * still hold the state that caused the event. */
        {
            usb_dwc_hprt_reg_t h = { .val = USB_DWC_HS.hprt_reg.val };
            ESP_LOGE(TAG, "DISCONNECT: hprt=%08x connsts=%u ena=%u lnsts=%u <- connsts=1 means "
                          "the device never left",
                     (unsigned)h.val, (unsigned)h.prtconnsts,
                     (unsigned)h.prtena, (unsigned)h.prtlnsts);
            s_trace_run = false;
            if (h.prtconnsts) {
                s_port_disables++;   /* port error, not an unplug */
            }
            {
                unsigned mps = 0, ec = 0, epnum = 0;
                if (usb_isoc_chan_info(&mps, &ec, &epnum)) {
                    ESP_LOGE(TAG, "at failure, isoc chan: MPS=%u EC=%u EP=%u", mps, ec, epnum);
                }
            }
            port_trace_dump("disconnect");
        }
#endif
        ESP_LOGW(TAG, "camera disconnected");
        s_camera_connected = false;
        s_ffc_active = false; /* the sensor's pattern won't be the same after it re-powers */
        /* Forget what was attached. Otherwise the streaming task reopens with the OLD camera's
         * format - asking a webcam for 256x384 YUY2 - and burns a 5s stream_open timeout before
         * the retry picks up the newly detected values. It also lets render_task notice the
         * kind change and release the JPEG decoder. */
        s_cam_kind = CAM_NONE;
        /* Invalidate first, so anything still queued is recognisably stale, then hand the
         * queued frames back while the stream handle is alive. Closing with frames outstanding
         * frees the buffers under render_task's feet. */
        s_stream_epoch++;
        {
            queued_frame_t stale;
            int drained = 0;
            while (xQueueReceive(s_frame_queue, &stale, 0) == pdPASS) {
                uvc_host_frame_return(event->device_disconnected.stream_hdl, stale.frame);
                drained++;
            }
            if (drained) {
                ESP_LOGI(TAG, "drained %d queued frame(s) on disconnect", drained);
            }
        }
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        s_stream = NULL;
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "Frame buffer underflow");
        break;
    default:
        break;
    }
}


#if P2_USB_PROBE
/* Diagnostic USB host client, independent of the UVC driver. Answers two questions the UVC layer
 * cannot: what SPEED the device actually negotiated, and whether it ever fully enumerates at all.
 * Relevant because the ESP32-P4 host is reported to fail FULL-speed enumeration with exactly the
 * "HUB: Root port reset failed" we see, while high-speed devices work - and this camera is
 * high-speed on a PC. If its chirp degrades over its ~1m cable the P4 would fall back to full
 * speed and land in that failure mode. */
static usb_host_client_handle_t s_probe_client;

static const char *speed_str(usb_speed_t sp)
{
    switch (sp) {
    case USB_SPEED_LOW:  return "LOW (1.5 Mbit/s)";
    case USB_SPEED_FULL: return "FULL (12 Mbit/s)";
    case USB_SPEED_HIGH: return "HIGH (480 Mbit/s)";
    default:             return "unknown";
    }
}

/* ---- UVC Extension Unit discovery -----------------------------------------------------------
 * Decompiling the GOYOJO vendor app (THG Start) showed the GW192A is a rebadged HIKMICRO
 * MiniE/P09-class module, and that HIKMICRO drives these over a UVC EXTENSION UNIT: the SDK
 * locates the unit by GUID (ParseExtensionUnit) and then issues control transfers carrying a
 * command header. That channel, if this camera exposes it, is the route to a forced flat-field,
 * emissivity and distance correction, the device's own temperature scaling, and its model string.
 *
 * None of that is worth writing until the hardware says the unit is there, so this prints it:
 * the unit ID to put in wIndex, the GUID the SDK matches on, and the control bitmap that says
 * which selectors the device will answer. Class-specific VC descriptors are bDescriptorType 0x24
 * (CS_INTERFACE) with bDescriptorSubType 0x06 for VC_EXTENSION_UNIT. */
/* Discovered XU coordinates, consumed by gw_xu_enumerate(). 0 unit = nothing found. */
static volatile uint8_t s_xu_unit_id;
static volatile uint8_t s_xu_interface;

static void probe_dump_xu(usb_device_handle_t dev, uint8_t addr)
{
    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK || !cfg) {
        ESP_LOGW(TAG, "probe: addr %d has no readable config descriptor", addr);
        return;
    }
    int offset = 0;
    const usb_standard_desc_t *d = (const usb_standard_desc_t *)cfg;
    int n_xu = 0;
    uint8_t cur_if = 0;   /* wIndex needs the VideoControl interface the XU belongs to */
    while ((d = usb_parse_next_descriptor(d, cfg->wTotalLength, &offset)) != NULL) {
        if (d->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            cur_if = ((const usb_intf_desc_t *)d)->bInterfaceNumber;
            continue;
        }
        if (d->bDescriptorType != 0x24 || d->bLength < 22) {
            continue;                       /* not a class-specific VC descriptor */
        }
        /* Indexed as raw bytes: the usb component's usb_standard_desc_t is only the two-byte
         * header, with no trailing value array to walk. p[0] is bDescriptorSubType. */
        const uint8_t *p = ((const uint8_t *)d) + 2;
        if (p[0] != 0x06) {
            continue;
        }
        n_xu++;
        const uint8_t unit_id   = p[1];
        const uint8_t *guid     = &p[2];
        const uint8_t n_ctrl    = p[18];
        const uint8_t n_in_pins = p[19];
        ESP_LOGW(TAG, "probe: XU unit_id=%u on interface %u bNumControls=%u bNrInPins=%u",
                 unit_id, cur_if, n_ctrl, n_in_pins);
        s_xu_unit_id = unit_id;
        s_xu_interface = cur_if;
        ESP_LOGW(TAG, "probe: XU GUID %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-"
                      "%02x%02x%02x%02x%02x%02x",
                 guid[0], guid[1], guid[2], guid[3], guid[4], guid[5], guid[6], guid[7],
                 guid[8], guid[9], guid[10], guid[11], guid[12], guid[13], guid[14], guid[15]);
        /* bControlSize and bmControls sit after the variable-length baSourceID array. */
        const int ctrl_size_off = 20 + n_in_pins;
        if (ctrl_size_off < d->bLength - 2) {
            const uint8_t ctrl_size = p[ctrl_size_off];
            char bits[64];
            int c = 0;
            for (int i = 0; i < ctrl_size && ctrl_size_off + 1 + i < d->bLength - 2; i++) {
                c += snprintf(bits + c, sizeof(bits) - c, "%02x ", p[ctrl_size_off + 1 + i]);
            }
            ESP_LOGW(TAG, "probe: XU bControlSize=%u bmControls=%s", ctrl_size, bits);
        }
    }
    ESP_LOGW(TAG, "probe: addr %d has %d extension unit(s)%s", addr, n_xu,
             n_xu ? "" : " - no vendor control channel on this device");
}

static void usb_probe_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    (void)arg;
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        usb_device_handle_t dev = NULL;
        if (usb_host_device_open(s_probe_client, msg->new_dev.address, &dev) != ESP_OK) {
            ESP_LOGW(TAG, "probe: addr %d appeared but could not be opened",
                     msg->new_dev.address);
            return;
        }
        usb_device_info_t info;
        const usb_device_desc_t *desc = NULL;
        if (usb_host_device_info(dev, &info) == ESP_OK) {
            ESP_LOGW(TAG, "probe: addr %d speed=%s bMaxPacketSize0=%d cfg=%d",
                     info.dev_addr, speed_str(info.speed),
                     info.bMaxPacketSize0, info.bConfigurationValue);
        }
        if (usb_host_get_device_descriptor(dev, &desc) == ESP_OK && desc) {
            ESP_LOGW(TAG, "probe: addr %d %04x:%04x bcdUSB=%04x class=%d",
                     msg->new_dev.address, desc->idVendor, desc->idProduct,
                     desc->bcdUSB, desc->bDeviceClass);
        }
        probe_dump_xu(dev, msg->new_dev.address);
        usb_host_device_close(s_probe_client, dev);
    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        ESP_LOGW(TAG, "probe: device gone");
    }
}

static void usb_probe_task(void *arg)
{
    (void)arg;
    const usb_host_client_config_t cc = {
        .is_synchronous = false,
        .max_num_event_msg = 8,
        .async = { .client_event_callback = usb_probe_cb, .callback_arg = NULL },
    };
    if (usb_host_client_register(&cc, &s_probe_client) != ESP_OK) {
        ESP_LOGW(TAG, "probe: could not register diagnostic client");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGW(TAG, "probe: diagnostic USB client registered");
    while (true) {
        usb_host_client_handle_events(s_probe_client, portMAX_DELAY);
    }
}
#endif


#if P2_PORT_POWER_RECOVERY
/* If the port sits failing with nothing enumerated, cycle root port power. Two things this can
 * fix: a wedged host port state machine, and - if HPRT.prtpwr actually gates VBUS on this board -
 * a wedged device, by power-cycling it. The borescope appears to wedge after repeated failed
 * resets and then stay dead until it loses power, which is what makes its own baseline drift. */
static void usb_port_recovery_task(void *arg)
{
    (void)arg;
    const int idle_limit = 10;          /* ~10s with no camera before intervening */
    int idle = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (s_camera_connected) {
            idle = 0;
            continue;
        }
        if (++idle < idle_limit) {
            continue;
        }
        idle = 0;
        ESP_LOGW(TAG, "usb: no camera for %ds - cycling root port power", idle_limit);
        esp_err_t e1 = usb_host_lib_set_root_port_power(false);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_err_t e2 = usb_host_lib_set_root_port_power(true);
        ESP_LOGW(TAG, "usb: port power off=%s on=%s", esp_err_to_name(e1), esp_err_to_name(e2));
    }
}
#endif

static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        /* Instrumented: a device that enumerates and is then torn down repeatedly could be
         * self-inflicted - free_all() here would destroy a device the UVC driver has not managed
         * to claim yet. Log the flags so the ordering against the claim failure is visible. */
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            /* Instrumented once to check whether this was tearing devices down under the UVC
             * driver: it never fired during the scope's enumeration loop, so it is not. */
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
        }
    }
}

/* ---------------- Render task: raw -> palette -> SPI blit ---------------- */

/* ---- HIKMICRO extension-unit channel --------------------------------------------------------
 * The GW192A is a rebadged HIKMICRO MiniE/P09-class module (established by decompiling the
 * vendor app) and carries Hikvision's HCUSBSDK vendor channel on a UVC Extension Unit. The
 * exchange below was recovered by disassembling libHCUSBSDK.so; docs/GW192A_frame_layout.md has
 * the derivation and the full 415-command map.
 *
 * Addressing is ordinary UVC XU:
 *     wValue = control_selector << 8
 *     wIndex = (unit_id << 8) | interface          (0x0A00 on this camera)
 *     GET: bmRequestType 0xA1, bRequest 0x81 (GET_CUR)
 *     SET: bmRequestType 0x21, bRequest 0x01 (SET_CUR)
 *
 * Two selectors are infrastructure rather than data, and no command in the SDK targets either:
 *     CS 5  command switch - a 2-byte mailbox, { target_selector, sub_function }
 *     CS 6  status / last error - 1 byte: 1 busy, 0 success, anything else an error
 *
 * So every exchange is: switch on CS 5, transfer on the target selector, then poll CS 6 until it
 * stops reading busy. The switch is what makes the channel stateful - the SDK even caches it and
 * skips the write when the pair has not changed - and skipping it is why a naive read of these
 * selectors returns whatever command happened to be selected last.
 *
 * This implementation does the three steps every time and never retries a failed exchange. The
 * repo's generic-camera notes record what a vendor-request retry loop does to a stream: a
 * control-transfer storm that drowns it. One pass, then give up and say so. */
#define XU_CS_SWITCH   5
#define XU_CS_STATUS   6
/* Each selector on this camera declares a 512-byte length, so a full-length GET is the
 * spec-compliant request and comfortably covers the structures read here. It also needs the EP0
 * buffer patch in tools/patch_managed_components.sh: stock usb_host_uvc caps wLength at 56. */
#define XU_PAYLOAD_MAX 512

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static esp_err_t xu_ctrl(uint8_t bmreq, uint8_t breq, uint8_t cs, uint16_t len, uint8_t *data)
{
    if (!s_stream || !s_therm->xu_unit) {
        return ESP_ERR_INVALID_STATE;
    }
    return uvc_host_usb_ctrl(s_stream, bmreq, breq,
                             (uint16_t)cs << 8,
                             ((uint16_t)s_therm->xu_unit << 8) | s_therm->xu_interface,
                             len, data);
}

/* Latch which command the next transfer on `cs` refers to. */
static esp_err_t xu_select(uint8_t cs, uint8_t subfunc)
{
    uint8_t sel[2] = { cs, subfunc };
    return xu_ctrl(0x21, 0x01, XU_CS_SWITCH, sizeof(sel), sel);
}

/* Poll CS 6 the way CheckDeviceStatus() does: 10 ms between reads, bounded. */
static esp_err_t xu_wait_ready(int timeout_ms)
{
    for (int waited = 0;; waited += 10) {
        uint8_t st = 0xFF;
        esp_err_t e = xu_ctrl(0xA1, 0x81, XU_CS_STATUS, 1, &st);
        if (e != ESP_OK) {
            return e;
        }
        if (st == 0) {
            return ESP_OK;
        }
        if (st != 1) {
            ESP_LOGW(TAG, "XU: device reports error status 0x%02x", st);
            return ESP_FAIL;
        }
        if (waited >= timeout_ms) {
            ESP_LOGW(TAG, "XU: still busy after %d ms", timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* How many bytes the command just latched will return.
 *
 * This is NOT optional, and guessing is what made the first attempt fail: asking for the
 * selector's advertised 512 bytes made the camera STALL every GET_CUR. CUsbDeviceVideoBase::
 * GetDataLen() shows why - the SDK never picks the read length itself, it asks the device, and
 * ReadDataStrategy assigns the answer straight into the transfer (trans.length = GetDataLen()).
 *
 * The SDK reads FOUR bytes here where UVC defines GET_LEN as two, and asking for four fails on
 * this stack: usb_host_uvc requires actual_num_bytes == num_bytes, so a device that answers the
 * standard two bytes is reported as ESP_ERR_INVALID_RESPONSE rather than as a short read. libusb
 * hands the real count back instead, which is why the SDK never had to care. Two bytes it is -
 * the value has never been anywhere near 65535. */
static esp_err_t xu_data_len(uint8_t cs, uint32_t *out)
{
    uint8_t len2[2] = {0, 0};
    esp_err_t e = xu_ctrl(0xA1, 0x85, cs, sizeof(len2), len2);   /* GET_LEN */
    if (e != ESP_OK) {
        return e;
    }
    *out = (uint32_t)len2[0] | ((uint32_t)len2[1] << 8);
    return ESP_OK;
}

/* Switch, ask the length, read it, wait for the status register to go idle. */
static esp_err_t xu_read(uint8_t cs, uint8_t subfunc, uint8_t *buf, uint16_t cap, uint16_t *got)
{
    memset(buf, 0, cap);
    *got = 0;
    esp_err_t e = xu_select(cs, subfunc);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "XU: select CS %u sub 0x%02x failed: %s", cs, subfunc, esp_err_to_name(e));
        return e;
    }
    uint32_t want = 0;
    e = xu_data_len(cs, &want);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "XU: GET_LEN CS %u failed: %s", cs, esp_err_to_name(e));
        return e;
    }
    if (want == 0 || want > cap) {
        ESP_LOGW(TAG, "XU: CS %u sub 0x%02x wants %u bytes, buffer is %u - not reading",
                 cs, subfunc, (unsigned)want, cap);
        return ESP_ERR_INVALID_SIZE;
    }
    e = xu_ctrl(0xA1, 0x81, cs, (uint16_t)want, buf);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "XU: GET_CUR CS %u (%u bytes) failed: %s",
                 cs, (unsigned)want, esp_err_to_name(e));
        return e;
    }
    *got = (uint16_t)want;
    ESP_LOGW(TAG, "XU: CS %u sub 0x%02x returned %u bytes", cs, subfunc, (unsigned)want);
    return xu_wait_ready(500);
}

/* Copies a fixed-width, possibly unterminated vendor string out for logging. */
static void xu_str(char *dst, size_t dstsz, const uint8_t *src, size_t n)
{
    size_t i = 0;
    for (; i < n && i + 1 < dstsz && src[i] >= 0x20 && src[i] < 0x7F; i++) {
        dst[i] = (char)src[i];
    }
    dst[i] = '\0';
}

/* One pure read of each of the two structures worth having, logged and then done with.
 *
 * Both are GETs: the only write is the 2-byte selector latch, which is what the channel is for.
 * Field offsets come from the JNA bindings' getFieldOrder(), which is authoritative for the C
 * layout in a way the plain Java class is not - jadx lists those fields alphabetically, and
 * trusting that order would have put every value in the wrong place. */
static void xu_read_device_params(void)
{
    static uint8_t buf[XU_PAYLOAD_MAX] __attribute__((aligned(4)));

    /* USB_GET_SYSTEM_DEVICE_INFO - command 2011, CS 1 sub 0x01.
     * The full structure is 516 bytes and so spans two chunked transfers, but everything
     * identifying lives in the first 512: byDeviceType is at +196. */
    uint16_t got = 0;
    if (xu_read(1, 0x01, buf, sizeof(buf), &got) == ESP_OK && got >= 260) {
        const uint32_t dwsize = le32(buf);
        char model[65], fw[65], serial[65];
        xu_str(model,  sizeof(model),  buf + 196, 64);
        xu_str(fw,     sizeof(fw),     buf + 4,   64);
        xu_str(serial, sizeof(serial), buf + 264, 64);
        ESP_LOGW(TAG, "XU: device info dwSize=%u type='%s' fw='%s' sn='%s'",
                 (unsigned)dwsize, model, fw, serial);
    }

    /* USB_GET_THERMOMETRY_BASIC_PARAM - command 2030, CS 3 sub 0x01. The interesting one: it
     * carries byShiftLength and byFloatTransMode, HIKMICRO's convention for sending floats as
     * scaled integers, which would let the camera state its own value scaling instead of us
     * fitting counts_per_kelvin from two reference temperatures. */
    if (xu_read(3, 0x01, buf, sizeof(buf), &got) == ESP_OK && got >= 84) {
        const uint32_t dwsize = le32(buf);
        ESP_LOGW(TAG, "XU: thermometry dwSize=%u enabled=%u unit=%u range=%u",
                 (unsigned)dwsize, buf[5], buf[9], buf[10]);
        ESP_LOGW(TAG, "XU: thermometry emissivity=%u distance=%u reflective=%u(en %u) "
                      "envirotemp=%u(en %u) humidity=%u",
                 (unsigned)le32(buf + 20), (unsigned)le32(buf + 28),
                 (unsigned)le32(buf + 36), buf[32],
                 (unsigned)le32(buf + 80), buf[78], (unsigned)le32(buf + 72));
        ESP_LOGW(TAG, "XU: thermometry floatTransMode=%u shiftLength=%u thermalRawData=%u "
                      "sse=%u autoDrift=%u calibCoef=%u(en %u)",
                 buf[76], buf[77], buf[35], buf[34], buf[33],
                 (unsigned)le32(buf + 12), buf[11]);
        ESP_LOGW(TAG, "XU: thermometry range limits %d..%d, excursion %d..%d",
                 (int)le32(buf + 60), (int)le32(buf + 56),
                 (int)le32(buf + 68), (int)le32(buf + 64));
        /* The whole point of the read: if the camera reports a shift length, its values are
         * scaled by a power of ten and our two-point fit can be checked against it. */
        if (buf[77]) {
            float scale = 1.0f;
            for (int i = 0; i < buf[77] && i < 9; i++) {
                scale *= 10.0f;
            }
            ESP_LOGW(TAG, "XU: shiftLength %u => vendor floats are scaled by %.0f "
                          "(emissivity %.3f, distance %.3f)",
                     buf[77], scale, le32(buf + 20) / scale, le32(buf + 28) / scale);
        }
    }

    /* Several commands in one pass, to separate "our framing is wrong" from "this module does
     * not implement that command". If every one answers with nothing but a dwSize then the
     * framing is incomplete; if one comes back populated, the framing is right and the GW192A
     * simply does not implement the industrial thermometry API its SDK describes. */
    static const struct { uint8_t cs, sub; const char *name; } probe[] = {
        { 1, 0x01, "SYSTEM_DEVICE_INFO" },
        { 8, 0x11, "DEVICE_INFO_CAPABILITIES" },
        { 3, 0x01, "THERMOMETRY_BASIC" },
        { 3, 0x02, "THERMOMETRY_MODE" },
        { 3, 0x14, "THERMAL_STREAM_CTRL" },
        { 3, 0x1f, "THERMAL_STREAM_YUV_RES" },
        { 8, 0x3c, "YUV_RES_CAPABILITIES" },
        { 10, 0x01, "VIDEO_CODE" },
        { 2, 0x05, "IMAGE_ENHANCEMENT" },
    };
    for (size_t i = 0; i < sizeof(probe) / sizeof(probe[0]); i++) {
        uint16_t n = 0;
        if (xu_read(probe[i].cs, probe[i].sub, buf, sizeof(buf), &n) != ESP_OK) {
            continue;
        }
        unsigned nonzero = 0, last = 0;
        for (uint16_t k = 0; k < n; k++) {
            if (buf[k]) {
                nonzero++;
                last = k;
            }
        }
        char hex[3 * 16 + 1];
        int c = 0;
        for (int k = 0; k < 16 && k < n; k++) {
            c += snprintf(hex + c, sizeof(hex) - c, "%02x ", buf[k]);
        }
        ESP_LOGW(TAG, "XU: CS %2u sub 0x%02x %-26s %u nonzero (last +%u): %s",
                 probe[i].cs, probe[i].sub, probe[i].name, nonzero, last, hex);
    }

    /* Raw head of the last response, so a layout that does not match can still be diagnosed
     * from the console rather than needing another build. */
    char hex[3 * 24 + 1];
    int c = 0;
    for (int i = 0; i < 24; i++) {
        c += snprintf(hex + c, sizeof(hex) - c, "%02x ", buf[i]);
    }
    ESP_LOGW(TAG, "XU: last response +00 %s", hex);
}

/* ---- GW192A extension-unit enumeration ------------------------------------------------------
 * Read-only reconnaissance of the vendor channel the probe found. For each control selector the
 * device claims in bmControls, ask:
 *   GET_INFO (0x86) - 1 byte of capability bits: bit0 GET supported, bit1 SET supported
 *   GET_LEN  (0x85) - 2 bytes, the control's payload length
 *   GET_CUR  (0x81) - the payload itself
 * and print them. That is the map needed before writing anything: a selector that reports no SET
 * support, or a length that does not match a structure we know, is one to leave alone.
 *
 * Deliberately GET-only, and deliberately once. The generic-camera notes in this repo record what
 * happens when vendor requests are fired at a device that does not implement them - EP0 stalls on
 * every request and a control-transfer storm that drowns the stream - so this touches only
 * selectors the descriptor advertises, makes one pass, and never retries.
 *
 * UVC addressing: wValue = CS << 8, wIndex = (unit_id << 8) | interface. */
/* Read the vendor channel once per boot and log what it says. GETs only, plus the 2-byte selector
 * latch each, no retries, no writes to any setting.
 *
 * DEFAULT 0, because as it stands the pass learns nothing: the transport works - selector latch,
 * GET_LEN, GET_CUR and the status register all succeed, and the status reads 0 for success - but
 * the camera answers every command with the same two bytes, 00 02, and no structure behind them.
 * Measured across nine different commands on five selectors. The request framing is therefore
 * still incomplete (see docs/GW192A_frame_layout.md), and until it is there is no reason to poke
 * the extension unit on every boot. Set to 1 to re-run the pass while working on it. */
#define P2_GW_XU_READ 0

#define P2_GW_XU_PROBE 0
#if P2_GW_XU_PROBE
static void gw_xu_enumerate(void)
{
    const uint8_t unit = s_xu_unit_id;
    if (!unit || !s_stream) {
        ESP_LOGW(TAG, "XU: no extension unit discovered (is P2_USB_PROBE enabled?)");
        return;
    }
    const uint16_t windex = ((uint16_t)unit << 8) | s_xu_interface;
    ESP_LOGW(TAG, "XU: enumerating unit %u on interface %u (wIndex=0x%04x)",
             unit, s_xu_interface, windex);

    for (uint8_t cs = 1; cs <= 10; cs++) {
        const uint16_t wvalue = (uint16_t)cs << 8;
        uint8_t info = 0;
        esp_err_t e_info = uvc_host_usb_ctrl(s_stream, 0xA1, 0x86, wvalue, windex, 1, &info);

        uint8_t lenbuf[2] = {0, 0};
        esp_err_t e_len = uvc_host_usb_ctrl(s_stream, 0xA1, 0x85, wvalue, windex,
                                            sizeof(lenbuf), lenbuf);
        const uint16_t len = (uint16_t)lenbuf[0] | ((uint16_t)lenbuf[1] << 8);

        if (e_info != ESP_OK && e_len != ESP_OK) {
            ESP_LOGW(TAG, "XU: CS %2u - no answer (info %s, len %s)", cs,
                     esp_err_to_name(e_info), esp_err_to_name(e_len));
            continue;
        }
        ESP_LOGW(TAG, "XU: CS %2u info=0x%02x (%s%s) len=%u", cs, info,
                 (info & 1) ? "GET " : "", (info & 2) ? "SET" : "", len);

        /* Every selector on this device reports 512 bytes, which is the SDK's command-mailbox
         * shape rather than a per-control structure: the header goes in, the response comes back
         * in the same buffer. Read the whole thing but print only the head, where a header
         * would live - 512 bytes of mostly padding down a 115200 baud console is not worth it. */
        if (e_len != ESP_OK || len == 0 || len > 512) {
            continue;
        }
        static uint8_t cur[512];
        memset(cur, 0, sizeof(cur));
        if (uvc_host_usb_ctrl(s_stream, 0xA1, 0x81, wvalue, windex, len, cur) != ESP_OK) {
            ESP_LOGW(TAG, "XU: CS %2u GET_CUR failed", cs);
            continue;
        }
        size_t last = 0;
        for (size_t i = 0; i < len; i++) {
            if (cur[i]) {
                last = i;
            }
        }
        char hex[3 * 32 + 1];
        for (size_t off = 0; off < 64 && off < len; off += 32) {
            int c = 0;
            for (size_t i = off; i < off + 32 && i < len; i++) {
                c += snprintf(hex + c, sizeof(hex) - c, "%02x ", cur[i]);
            }
            ESP_LOGW(TAG, "XU: CS %2u +%02u %s", cs, (unsigned)off, hex);
        }
        ESP_LOGW(TAG, "XU: CS %2u last non-zero byte at %u of %u", cs, (unsigned)last, len);
    }
    ESP_LOGW(TAG, "XU: enumeration done");
}
#endif

/* ---- GW192A calibration instrumentation ------------------------------------------------------
 * Temporary, and separate from P2_GW_PROBE: that one owns the camera and dumps whole modes, this
 * one rides along on the live render path so the picture stays on screen while it reports. It
 * answers two questions that the first probe left open:
 *   1. where the marker pixels actually are, which decides whether masking them is enough or the
 *      plane needs reinterpreting;
 *   2. what the centre of the image reads in raw counts, so pointing the camera at two surfaces
 *      of known temperature pins down counts_per_kelvin instead of assuming it.
 * Set to 0 once the scale is settled. */
#define P2_GW_CALIB 1
#if P2_GW_CALIB
static void gw_calib_report(const uint16_t *raw16, size_t npix, uint16_t centre_v,
                            uint16_t min_v, uint16_t max_v)
{
    static bool mapped = false;
    if (!mapped) {
        mapped = true;
        const int w = s_therm->raw_w;
        char line[160];
        int p = 0, shown = 0;
        size_t n = 0;
        for (size_t i = 0; i < npix; i++) {
            if (raw16[i] < 32768u) {
                continue;
            }
            n++;
            if (shown < 24) {
                shown++;
                p += snprintf(line + p, sizeof(line) - p, " r%u,c%u=%04x",
                              (unsigned)(i / w), (unsigned)(i % w), raw16[i]);
            }
        }
        ESP_LOGW(TAG, "GWCAL markers: %u of %u pixels >=0x8000;%s",
                 (unsigned)n, (unsigned)npix, shown ? line : " none");
    }

    static int64_t last_us = 0;
    const int64_t now = esp_timer_get_time();
    if (now - last_us < 2000000) {
        return;
    }
    last_us = now;
    /* Scanned here rather than reusing the render pass's min_v/max_v. Those come from a cache
     * refreshed every MINMAX_PERIOD frames, which is right for driving a palette and wrong for
     * calibration: against a 23 C wall it reported a range of 5068..5491 while the centre read
     * 5053, i.e. a centre BELOW the frame minimum, which is impossible within one frame and made
     * the numbers impossible to trust. A calibration readout has to describe one frame. */
    uint16_t lo = 0xFFFF, hi = 0;
    uint64_t sum = 0;
    for (size_t i = 0; i < npix; i++) {
        const uint16_t v = raw16[i];
        if (raw_invalid(v)) {
            continue;
        }
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        sum += v;
    }
    const unsigned mean = npix ? (unsigned)(sum / npix) : 0;
    /* Printed as raw counts AND as the temperature the current scale implies, so a wrong scale
     * shows up as an implausible number instead of hiding behind a plausible-looking one. */
    ESP_LOGW(TAG, "GWCAL centre=%u (%.1f C)  frame %u..%u mean %u (%.1f..%.1f C)  "
                  "@%.3f counts/K bias %.0f  [cached %u..%u]",
             (unsigned)centre_v, raw_to_celsius(centre_v),
             (unsigned)lo, (unsigned)hi, mean,
             raw_to_celsius(lo), raw_to_celsius(hi),
             s_therm->counts_per_kelvin, s_therm->counts_bias,
             (unsigned)min_v, (unsigned)max_v);
}
#endif

static void render_task(void *arg)
{
    static int frames_converted = 0;
    static int64_t convert_time_sum_us = 0;
    bool disconnect_shown = false;
    bool no_temp_shown = false;
    int no_frame_ticks = 0;
    uint16_t last_min_v = 0, last_max_v = 0;

    cam_kind_t last_kind = CAM_NONE;
    /* NULL, not s_therm: the per-camera setup below (layout, flat-field re-key) has to run for
     * the FIRST thermal camera too, including a P2 Pro that matches the boot-time default. The
     * stored flat-field table used to be loaded in app_main(); seeding this with s_therm would
     * have meant it was then never loaded at all. */
    const thermal_model_info_t *last_therm = NULL;
    queued_frame_t qf;
#if P2_BUILTIN_CAM
    bool bi_shown = false;
#endif
#if P2_WIFI_CAM
    bool wc_shown = false;   /* has the panel been cleared for this WiFi session? */
#endif
#if P2_BUILTIN_CAM
    int64_t bi_sem_sum = 0, bi_ppa1_sum = 0, bi_ppa2_sum = 0, bi_wait_sum = 0, bi_dq_sum = 0;
    int bi_n = 0;
#endif
    while (true) {
        RLAP();
        RSTAGE("loop top");
#if P2_BUILTIN_CAM
        /* No USB camera: fall back to the board's own OV5647. Driven from render_task rather
         * than its own task on purpose - this is the only task allowed to touch s_rgb_buf,
         * s_rgb_tx and the panel, and a second drawer would race the SPI completion semaphore. */
        if (s_cam_kind == CAM_NONE && s_wifi_active) {
            /* WiFi earpick camera: preferred over the built-in one. Only reachable with no USB
             * camera attached, and the built-in preview is torn down first so the two never
             * compete for PSRAM bandwidth and PPA time. */
            if (bi_shown || s_bi_fd >= 0) {
                bi_shown = false;
                builtin_cam_stop();
            }
            /* Clear the whole panel once on entry. The WiFi image is a 297x297 circle centred in
             * a 480x320 panel, so without this the margins keep showing whatever the built-in
             * camera last drew full-screen. Stripped, because WiFi being up means internal RAM
             * is fragmented and a single full-frame blit would fail. */
            if (!wc_shown) {
                wc_shown = true;
                last_kind = CAM_NONE;
                disconnect_shown = false;
                xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
                memset(s_rgb_tx, 0, (size_t)s_disp_w * s_disp_h * sizeof(uint16_t));
                s_pending_arrival_us = esp_timer_get_time();
                blit_strips(0, 0, s_disp_w, s_disp_h, s_rgb_tx);
                continue;
            }
            int slot = 0;
            RSTAGE("wifi frame wait");
            if (xQueueReceive(s_wifi_frame_q, &slot, pdMS_TO_TICKS(100)) != pdPASS) {
                continue;
            }
            s_wifi_q_us = esp_timer_get_time() - s_wifi_jpeg_t[slot];
            const uint8_t *jb = s_wifi_jpeg[slot];
            const size_t   jl = s_wifi_jpeg_len[slot];

            jpeg_decode_picture_info_t pi = { 0 };
            if (jpeg_decoder_get_info(jb, jl, &pi) != ESP_OK || !pi.width || !pi.height) {
                s_wifi_dropped++; s_wd_info++;
                continue;
            }
            if ((int)pi.width != s_wc_sw || (int)pi.height != s_wc_sh) {
                s_wc_sw = pi.width;
                s_wc_sh = pi.height;
                fit_preserving_aspect(s_wc_sw, s_wc_sh, s_disp_w, s_disp_h, &s_wc_w, &s_wc_h);
                s_wc_x = (s_disp_w - s_wc_w) / 2;
                s_wc_y = (s_disp_h - s_wc_h) / 2;
                ESP_LOGI(TAG, "wifi: %dx%d JPEG -> %dx%d at (%d,%d)",
                         s_wc_sw, s_wc_sh, s_wc_w, s_wc_h, s_wc_x, s_wc_y);
                /* New geometry: wipe the panel so nothing of the previous source survives. */
                xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
                memset(s_rgb_tx, 0, (size_t)s_disp_w * s_disp_h * sizeof(uint16_t));
                s_pending_arrival_us = esp_timer_get_time();
                blit_strips(0, 0, s_disp_w, s_disp_h, s_rgb_tx);
                last_kind = CAM_NONE;
                continue;
            }
            if (!jpeg_decoder_ensure(s_wc_sw, s_wc_sh) || jl > s_jpeg_in_cap) {
                s_wifi_dropped++; s_wd_decode++;
                continue;
            }
            const int64_t t0 = esp_timer_get_time();
            memcpy(s_jpeg_in, jb, jl);
            uint32_t out_len = 0;
            const jpeg_decode_cfg_t dcfg = {
                .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
                .rgb_order     = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
                .conv_std      = JPEG_YUV_RGB_CONV_STD_BT601,
            };
            if (jpeg_decoder_process(s_jpeg_dec, &dcfg, s_jpeg_in, jl,
                                     s_jpeg_out, s_jpeg_out_cap, &out_len) != ESP_OK) {
                s_wifi_dropped++; s_wd_decode++;
                continue;
            }
            const int64_t t_dec = esp_timer_get_time();
            /* One CPU pass does rotation, scaling, the circular mask and the byte swap, replacing
             * the two PPA transfers this path used before - PPA cannot rotate by an arbitrary
             * angle. Runs BEFORE the wait, into the buffer the display is not reading. */
            wifi_rotate_blit((const uint16_t *)s_jpeg_out, s_wc_sw, s_wc_sh,
                             s_wc_tx[s_wc_tx_i], s_wc_w, s_wc_h, s_wifi_rot_deg);
            const int64_t t_rot = esp_timer_get_time();
            xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
            s_wifi_wait_us = esp_timer_get_time() - t_rot;
            s_wifi_dec_us = t_dec - t0;        /* memcpy + JPEG decode */
            s_wifi_rot_us = t_rot - t_dec;     /* the rotate/mask pass, now measured alone */

            s_pending_arrival_us = t0;
            blit_strips(s_wc_x, s_wc_y,
                        s_wc_x + s_wc_w, s_wc_y + s_wc_h, s_wc_tx[s_wc_tx_i]);
            s_wc_tx_i ^= 1;
            int64_t wnow = esp_timer_get_time();
            if (wnow - s_last_report_us >= 2000000) {
#if P2_WIFI_LATENCY_TRACE
                ESP_LOGI(TAG, "wifi jpeg %dx%d  last frame %u bytes",
                         s_wc_sw, s_wc_sh, (unsigned)s_wifi_jpeg_len[s_wc_tx_i ^ 1]);
#endif
                ESP_LOGI(TAG, "wifi displayed=%d avg_e2e=%" PRId64 "us | frames=%u "
                              "drops: reasm=%u busy=%u info=%u decode=%u",
                         s_frames_rendered,
                         s_frames_rendered ? s_e2e_time_sum_us / s_frames_rendered : 0,
                         (unsigned)s_wifi_frames, (unsigned)s_wd_reasm, (unsigned)s_wd_busy,
                         (unsigned)s_wd_info, (unsigned)s_wd_decode);
#if P2_WIFI_LATENCY_TRACE
                {
                    const int64_t blit = (s_frames_rendered
                                          ? s_e2e_time_sum_us / s_frames_rendered : 0)
                                         - s_wifi_dec_us - s_wifi_rot_us - s_wifi_wait_us;
                    ESP_LOGI(TAG, "wifi latency us: reasm=%" PRId64 " queue=%" PRId64
                                  " decode=%" PRId64 " rotate=%" PRId64 " wait=%" PRId64
                                  " blit=~%" PRId64 " | OURS=%" PRId64 "  gap=%" PRId64,
                             s_wifi_asm_us, s_wifi_q_us, s_wifi_dec_us, s_wifi_rot_us,
                             s_wifi_wait_us, blit,
                             s_wifi_asm_us + s_wifi_q_us + s_wifi_dec_us + s_wifi_rot_us
                                 + s_wifi_wait_us + blit,
                             s_wifi_gap_us);
                }
#endif
                ESP_LOGW(TAG, "imu xyz=%6d %6d %6d | XY=%4.0f YZ=%4.0f XZ=%4.0f | ANGLE=%4.0f "
                              "| applied=%4.0f",
                         s_imu_x, s_imu_y, s_imu_z, s_rot_xy, s_rot_yz, s_rot_xz,
                         s_imu_angle_deg, s_wifi_rot_deg);
                s_frames_rendered = 0;
                s_e2e_time_sum_us = 0;
                s_last_report_us = wnow;
            }
            continue;
        }
#if P2_WIFI_CAM
        wc_shown = false;   /* not showing WiFi any more: next entry clears the panel again */
#endif
        if (s_cam_kind == CAM_NONE) {
            /* Debounced. The built-in camera is a fallback and must never compete with the
             * instrument: starting it eagerly meant it ran all through USB enumeration and after
             * every brief disconnect, stealing PSRAM bandwidth and PPA time from the thermal
             * path - which showed up as extra latency on the P2. A device that is about to
             * enumerate clears CAM_NONE well inside this window. */
            if (s_bi_fd < 0 && !s_bi_idle_since) {
                s_bi_idle_since = esp_timer_get_time();
            }
            if (s_bi_fd < 0 &&
                esp_timer_get_time() - s_bi_idle_since < BUILTIN_IDLE_WAIT_US) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            if (builtin_cam_start()) {
                if (!bi_shown) {
                    /* Wipe the USB view's side bar and letterbox margins once on entry. */
                    bi_shown = true;
                    last_kind = CAM_NONE;
                    disconnect_shown = false;
                    xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
                    memset(s_rgb_tx, 0, (size_t)s_disp_w * s_disp_h * sizeof(uint16_t));
                    s_pending_arrival_us = esp_timer_get_time();
                    blit_strips(0, 0, s_disp_w, s_disp_h, s_rgb_tx);
                    continue;
                }
                struct v4l2_buffer buf = {
                    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP,
                };
                const int64_t t_dq0 = esp_timer_get_time();
                RSTAGE("builtin DQBUF");
                if (ioctl(s_bi_fd, VIDIOC_DQBUF, &buf) != 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                bi_dq_sum += esp_timer_get_time() - t_dq0;
#if P2_BI_RATE_TEST
                /* Camera-only rate: straight back on the queue, nothing rendered. */
                ioctl(s_bi_fd, VIDIOC_QBUF, &buf);
                bi_n++;
                {
                    int64_t rnow = esp_timer_get_time();
                    if (rnow - s_last_report_us >= 2000000) {
                        ESP_LOGW(TAG, "RATE TEST (no render): %d frames in %" PRId64 "ms -> %d fps"
                                      "  avg dqbuf=%" PRId64 "us",
                                 bi_n, (rnow - s_last_report_us) / 1000,
                                 (int)((int64_t)bi_n * 1000000 / (rnow - s_last_report_us)),
                                 bi_dq_sum / bi_n);
                        bi_n = 0; bi_dq_sum = 0;
                        s_last_report_us = rnow;
                    }
                }
                continue;
#endif
                const int64_t t0 = esp_timer_get_time();
                const int64_t t_sem = t0;   /* PPA now runs before the wait, see below */

                const ppa_srm_oper_config_t scale = {
                    .in = {
                        .buffer = s_bi_buf[buf.index],
                        .pic_w = BUILTIN_CAM_W, .pic_h = BUILTIN_CAM_H,
                        .block_w = s_bi_sw, .block_h = s_bi_sh,
                        .block_offset_x = s_bi_sx, .block_offset_y = s_bi_sy,
#if P2_BUILTIN_YUV420
                        .srm_cm = PPA_SRM_COLOR_MODE_YUV420,
                        .yuv_range = PPA_COLOR_RANGE_LIMIT,
                        .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
#else
                        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
#endif
                    },
                    .out = {
                        .buffer = s_rgb_buf,
                        .buffer_size = ppa_out_bytes(s_bi_w, s_bi_h),
                        .pic_w = s_bi_w, .pic_h = s_bi_h,
                        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
                    },
                    /* Scale is applied before the rotation, so x maps to the source's x: the
                     * output's WIDTH comes from the source's height and vice versa.
                     *
                     * 90, with the sensor flip before it. Derivation, with source W x H and
                     * the rotation counter-clockwise (which both earlier symptoms confirmed -
                     * R90 alone left the horizontal wrong, R270 alone left the vertical wrong):
                     *
                     *     wanted:            Mh . R90 = (H-1-y, W-1-x)   <- an anti-transpose
                     *     R90 . Mv_source             = (H-1-y, W-1-x)   <- this
                     *
                     * Mv_source, not Mh_source, because the control esp_video calls
                     * V4L2_CID_HFLIP flips this sensor VERTICALLY: the OV5647 driver's register
                     * tables comment 0x3820 as horizontal and 0x3821 as vertical, transposed
                     * from the datasheet. Measured, not assumed - with ROTATION_ANGLE_270 the
                     * result was a plain transpose, i.e. exactly 180 degrees off.
                     *
                     * PPA's own mirror_x/mirror_y were tried in both passes and both axes and
                     * had no visible effect at all, which is why the mirror lives in the sensor. */
                    .rotation_angle = PPA_SRM_ROTATION_ANGLE_90,
                    .scale_x = (float)s_bi_h / s_bi_sw,
                    .scale_y = (float)s_bi_w / s_bi_sh,
                    .mode = PPA_TRANS_MODE_BLOCKING,
                };
                ppa_do_scale_rotate_mirror(s_ppa_client, &scale);
                const int64_t t_ppa1 = esp_timer_get_time();

                /* Second pass for the ST7796's byte order, exactly as the USB preview does:
                 * PPA's byte_swap affects how it READS, so it cannot fold into the scale. */
                const ppa_srm_oper_config_t swap = {
                    .in = {
                        .buffer = s_rgb_buf,
                        .pic_w = s_bi_w, .pic_h = s_bi_h,
                        .block_w = s_bi_w, .block_h = s_bi_h,
                        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
                    },
                    .out = {
                        .buffer = s_bi_tx[s_bi_tx_i],
                        .buffer_size = ppa_out_bytes(s_bi_w, s_bi_h),
                        .pic_w = s_bi_w, .pic_h = s_bi_h,
                        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
                    },
                    .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                    .scale_x = 1.0f, .scale_y = 1.0f,
                    .byte_swap = true,
                    .mode = PPA_TRANS_MODE_BLOCKING,
                };
                ppa_do_scale_rotate_mirror(s_ppa_client, &swap);
                const int64_t t_ppa2 = esp_timer_get_time();

                ioctl(s_bi_fd, VIDIOC_QBUF, &buf);

                /* Wait for the PREVIOUS blit only now - the conversion above has already
                 * overlapped it. This is the whole point of the second tx buffer. */
                const int64_t t_wait0 = esp_timer_get_time();
                RSTAGE("builtin blit wait");
                xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
                bi_wait_sum += esp_timer_get_time() - t_wait0;

                s_pending_arrival_us = t0;
                /* The return value matters: on failure NO completion callback fires, so the
                 * semaphore is never given and the next iteration blocks forever. That is
                 * exactly how this froze when WiFi came up - measured as
                 * "render STUCK in builtin blit wait". Hand the token back ourselves. */
                esp_err_t de = blit_strips(s_bi_x, s_bi_y, s_bi_x + s_bi_w, s_bi_y + s_bi_h,
                                           s_bi_tx[s_bi_tx_i]);
                if (de != ESP_OK) {
                    static int warned = 0;
                    if (warned < 3) {
                        warned++;
                        ESP_LOGE(TAG, "builtin: draw_bitmap failed: %s | INTERNAL free %u "
                                      "largest %u | DMA free %u largest %u | blit %d bytes",
                                 esp_err_to_name(de),
                                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
                                 s_bi_w * s_bi_h * 2);
                    }
                }
                s_bi_tx_i ^= 1;
                s_bi_frames++;
                int64_t now = esp_timer_get_time();
                bi_sem_sum  += t_sem  - t0;   /* now always 0; kept so the format is stable */
                bi_ppa1_sum += t_ppa1 - t_sem;
                bi_ppa2_sum += t_ppa2 - t_ppa1;
                bi_n++;
                if (now - s_last_report_us >= 2000000) {
                    ESP_LOGI(TAG, "builtin displayed=%d avg_e2e=%" PRId64 "us (%dx%d -> %dx%d)",
                             s_frames_rendered,
                             s_frames_rendered ? s_e2e_time_sum_us / s_frames_rendered : 0,
                             BUILTIN_CAM_W, BUILTIN_CAM_H, s_bi_w, s_bi_h);
#if P2_BI_STAGE_TIMING
                    if (bi_n) {
                        ESP_LOGI(TAG, "builtin stages: ppa_scale=%" PRId64 "us ppa_swap=%" PRId64
                                      "us blit_wait=%" PRId64 "us (n=%d)",
                                 bi_ppa1_sum / bi_n, bi_ppa2_sum / bi_n, bi_wait_sum / bi_n, bi_n);
                    }
#endif
#if P2_BI_AE_TRACE
                    /* Is AGC actually writing the sensor? All the IPA algorithms (AGC for gain
                     * and exposure, ATC for the target level) default to enabled and the ISP was
                     * demonstrably computing luma statistics, so the open question is whether the
                     * result reaches the sensor. Constant values under changing light means it
                     * does not; moving values mean AE works and the target is simply wrong. */
                    {
                        /* One control per call: a single unsupported id fails the whole
                         * request, which is what hid this the first time. And it is
                         * V4L2_CID_EXPOSURE, not _ABSOLUTE - esp_video 0.9.0 maps
                         * EXPOSURE_ABSOLUTE to ESP_CAM_SENSOR_EXPOSURE_US, which the OV5647
                         * driver does not implement; EXPOSURE_VAL is the one it has. */
                        int exp = -1, gain = -1;
                        struct v4l2_ext_control ce = { .id = V4L2_CID_EXPOSURE };
                        struct v4l2_ext_controls ge = { .count = 1, .controls = &ce };
                        if (ioctl(s_bi_fd, VIDIOC_G_EXT_CTRLS, &ge) == 0) { exp = ce.value; }
                        struct v4l2_ext_control cg = { .id = V4L2_CID_GAIN };
                        struct v4l2_ext_controls gg = { .count = 1, .controls = &cg };
                        if (ioctl(s_bi_fd, VIDIOC_G_EXT_CTRLS, &gg) == 0) { gain = cg.value; }
                        ESP_LOGI(TAG, "builtin AE: exposure=%d gain=%d (-1 = not readable)",
                                 exp, gain);
                    }
#endif
                    bi_sem_sum = bi_ppa1_sum = bi_ppa2_sum = bi_wait_sum = 0;
                    bi_n = 0;
                    s_frames_rendered = 0;
                    s_e2e_time_sum_us = 0;
                    s_last_report_us = now;
                }
                continue;
            }
        } else {
            s_bi_idle_since = 0;   /* a USB camera is present: re-arm the debounce */
            if (bi_shown || s_bi_fd >= 0) {
                /* Hand the panel and the PSRAM back. */
                bi_shown = false;
                builtin_cam_stop();
            }
        }
#endif
        /* Timeout rather than block forever: with no camera there are no frames, and this is
         * the only task allowed to draw, so without waking up periodically the "no camera"
         * state could never reach the screen. */
        RSTAGE("usb frame wait");
        if (xQueueReceive(s_frame_queue, &qf, pdMS_TO_TICKS(500)) != pdPASS) {
            /* Enumerated but no frames: a brief gap is just the FFC shutter (~0.5-1s), so only
             * treat a long silence as a stalled stream worth renegotiating. */
            if (s_camera_connected && ++no_frame_ticks >= 6) { /* ~3s */
                ESP_LOGW(TAG, "No frames for ~3s while connected - requesting stream restart");
                s_stream_restart_req = true;
                no_frame_ticks = 0;
            }
            if (!s_camera_connected && !disconnect_shown) {
                /* Blank the image area too - otherwise the last frame sits there looking live. */
                xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
                draw_status_bar("NO\nCAM"); /* queued first, so its completion is the
                                                       * one the ISR swallows (SPI queue is FIFO) */
                memset(s_rgb_tx, 0, (size_t)s_img_w * s_fb_h * sizeof(uint16_t));
                s_pending_arrival_us = esp_timer_get_time();
                blit_strips(s_img_off_x, s_img_off_y,
                                           s_img_off_x + s_img_w, s_img_off_y + s_fb_h, s_rgb_tx);
                disconnect_shown = true;
            }
            continue;
        }
        no_frame_ticks = 0;
        /* A frame queued before the last open/disconnect points at buffers the driver has
         * already freed. Drop it WITHOUT dereferencing it, and without returning it - whatever
         * owned it is gone. This is the camera hot-swap fix. */
        if (qf.epoch != s_stream_epoch) {
            continue;
        }
        if (disconnect_shown) {
            disconnect_shown = false;
            s_osd_dirty = true; /* bring the temperature readout back */
        }

        /* Wait until the PREVIOUS SPI transfer out of s_rgb_buf has actually finished before
         * we start overwriting it with this frame's pixels - this is the fix for the tearing:
         * draw_bitmap() is async, so without this we could rewrite the buffer mid-transfer. */
        xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);

        /* Swapping between a thermal camera and a webcam changes the whole layout, so wipe the
         * panel once rather than leaving the old picture, side bar or letterbox margins behind.
         * Done as ONE full-screen draw: the completion callback gives a binary semaphore and
         * counts a frame, so a burst of strip draws would unbalance it and skew the e2e stats.
         * Skipping this frame lets the next iteration re-take the semaphore normally. */
        if (s_cam_kind != last_kind || s_therm != last_therm) {
            ESP_LOGI(TAG, "camera kind %d -> %d (%s), clearing panel",
                     (int)last_kind, (int)s_cam_kind, s_therm->name);
            last_kind = s_cam_kind;
            /* A different thermal sensor means a different image size, a different bar width and
             * a flat-field table that no longer applies. Safe to do here and nowhere else: this
             * runs on the task that owns the layout, and the SPI transfer semaphore above has
             * already confirmed the panel is idle. */
            if (s_therm != last_therm) {
                last_therm = s_therm;
                thermal_layout();
                ffc_reset_for_camera();
                s_scale_manual = false;   /* endpoints latched in the old camera's counts */
            }
            /* Hand back the JPEG decoder's buffers when the new camera does not need them -
             * 640x480 costs ~1.2 MB of PSRAM across input and output. Done here rather than in
             * the connect callback because this runs on the task that does the decoding, so it
             * cannot free the buffers out from under a decode in progress. */
            if (s_cam_kind != CAM_GENERIC && s_jpeg_dec) {
                ESP_LOGI(TAG, "jpeg: releasing decoder (%dx%d) - camera no longer needs it",
                         s_jpeg_w, s_jpeg_h);
                jpeg_del_decoder_engine(s_jpeg_dec);
                s_jpeg_dec = NULL;
                free(s_jpeg_in);  s_jpeg_in  = NULL;
                free(s_jpeg_out); s_jpeg_out = NULL;
                s_jpeg_in_cap = s_jpeg_out_cap = 0;
                s_jpeg_w = s_jpeg_h = 0;
                s_jpeg_failed = false;   /* a fresh camera deserves a fresh attempt */
            }
            memset(s_rgb_tx, 0, (size_t)s_disp_w * s_disp_h * sizeof(uint16_t));
            s_pending_arrival_us = esp_timer_get_time();
            blit_strips(0, 0, s_disp_w, s_disp_h, s_rgb_tx);
            s_osd_dirty = true;            /* thermal mode has to repaint its side bar */
            if (s_stream) {
                uvc_host_frame_return(s_stream, qf.frame);
            }
            continue;
        }

        int64_t convert_start_us = esp_timer_get_time();

        /* ---- generic camera: just show the picture ----------------------------------------
         * Two PPA passes and no per-pixel CPU work: the first converts YUY2 to RGB565 while
         * scaling to the letterboxed box, the second does the byte swap the ST7796 needs (its
         * byte_swap flag affects how PPA READS, so it cannot be folded into the first pass). */
        if (s_cam_kind == CAM_GENERIC) {
            /* What PPA will read, and in which colour space. YUY2 arrives ready to scale;
             * MJPEG has to go through the hardware decoder first. */
            void *ppa_src = qf.frame->data;
            ppa_srm_color_mode_t ppa_src_cm = PPA_SRM_COLOR_MODE_YUV422_YUYV;

            if (s_cam_fmt == UVC_VS_FORMAT_MJPEG) {
                /* A frame that lost packets mid-stream still gets delivered (the component only
                 * validates the SOI, and we have the payload error check disabled), so it turns
                 * up here as a JPEG with no EOI. Counting those separately from decoder errors
                 * is what tells us the link's real per-frame completion rate. */
                const uint8_t *jd = qf.frame->data;
                const size_t   jl = qf.frame->data_len;
                const bool complete = (jl >= 4 && jd[jl - 2] == 0xFF && jd[jl - 1] == 0xD9);

                if (!complete || !jpeg_decoder_ensure(s_cam_w, s_cam_h) || jl > s_jpeg_in_cap) {
                    if (!complete) {
                        s_mj_truncated++;
                    } else {
                        s_mj_decode_err++;
                    }
                    xSemaphoreGive(s_trans_done_sem);
                    if (s_stream) {
                        uvc_host_frame_return(s_stream, qf.frame);
                    }
                    continue;
                }

                /* Copy into the decoder's own buffer rather than handing it the UVC frame: the
                 * JPEG peripheral has its own alignment requirements and the UVC driver's
                 * PSRAM frame buffers carry no such guarantee. ~23 KB, so it does not show up
                 * against a 40 ms frame budget. */
                memcpy(s_jpeg_in, jd, jl);

                /* BGR, not RGB: the decoder's "RGB" order writes the components in the
                 * opposite byte order to the RGB565 that PPA produces from YUY2, and the
                 * downstream byte-swap pass is tuned for PPA's convention. With RGB here the
                 * picture arrives with red and blue traded. */
                const jpeg_decode_cfg_t dcfg = {
                    .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
                    .rgb_order     = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
                    .conv_std      = JPEG_YUV_RGB_CONV_STD_BT601,
                };
                uint32_t out_len = 0;
                esp_err_t je = jpeg_decoder_process(s_jpeg_dec, &dcfg, s_jpeg_in, jl,
                                                    s_jpeg_out, s_jpeg_out_cap, &out_len);
                if (je != ESP_OK) {
                    s_mj_decode_err++;
                    static bool warned_dec = false;
                    if (!warned_dec) {
                        ESP_LOGW(TAG, "jpeg: decode failed: %s (%u B in)",
                                 esp_err_to_name(je), (unsigned)jl);
                        warned_dec = true;
                    }
                    xSemaphoreGive(s_trans_done_sem);
                    if (s_stream) {
                        uvc_host_frame_return(s_stream, qf.frame);
                    }
                    continue;
                }
                s_mj_ok++;
                ppa_src    = s_jpeg_out;
                ppa_src_cm = PPA_SRM_COLOR_MODE_RGB565;
            } else {
                const size_t need_px = (size_t)s_cam_w * s_cam_h * 2;
                if (qf.frame->data_len < need_px) {
                    static bool warned_short = false;
                    if (!warned_short) {
                        ESP_LOGW(TAG, "generic frame is %d bytes, expected %d - skipping",
                                 (int)qf.frame->data_len, (int)need_px);
                        warned_short = true;
                    }
                    xSemaphoreGive(s_trans_done_sem);
                    if (s_stream) {
                        uvc_host_frame_return(s_stream, qf.frame);
                    }
                    continue;
                }
            }

            const ppa_srm_oper_config_t yuv_cfg = {
                .in = {
                    .buffer = ppa_src,
                    .pic_w = s_cam_w, .pic_h = s_cam_h,
                    .block_w = s_src_w, .block_h = s_src_h,
                    .block_offset_x = s_src_x, .block_offset_y = s_src_y,
                    .srm_cm = ppa_src_cm,
                    .yuv_range = PPA_COLOR_RANGE_LIMIT,   /* UVC YUY2 is studio-range */
                    .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
                },
                .out = {
                    .buffer = s_rgb_buf,
                    .buffer_size = ppa_out_bytes(s_gen_w, s_gen_h),
                    .pic_w = s_gen_w, .pic_h = s_gen_h,
                    .block_offset_x = 0, .block_offset_y = 0,
                    .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
                },
                .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                .scale_x = (float)s_gen_w / s_src_w,
                .scale_y = (float)s_gen_h / s_src_h,
                .mirror_x = false,
                .mirror_y = false,
                .mode = PPA_TRANS_MODE_BLOCKING,
            };
            esp_err_t ge = ppa_do_scale_rotate_mirror(s_ppa_client, &yuv_cfg);
            if (ge != ESP_OK) {
                static bool warned_ppa = false;
                if (!warned_ppa) {
                    /* Most likely cause is alignment: PPA wants its buffers on a cache line and
                     * the UVC driver's frame buffers carry no such guarantee. */
                    ESP_LOGE(TAG, "generic: PPA YUV pass failed: %s (frame data %p)",
                             esp_err_to_name(ge), qf.frame->data);
                    warned_ppa = true;
                }
            }

#if P2_USB_OSD
            usb_osd_draw(s_rgb_buf, s_gen_w, s_gen_h);
#endif
            const ppa_srm_oper_config_t gswap = {
                .in = {
                    .buffer = s_rgb_buf,
                    .pic_w = s_gen_w, .pic_h = s_gen_h,
                    .block_w = s_gen_w, .block_h = s_gen_h,
                    .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
                },
                .out = {
                    .buffer = s_rgb_tx,
                    .buffer_size = ppa_out_bytes(s_gen_w, s_gen_h),
                    .pic_w = s_gen_w, .pic_h = s_gen_h,
                    .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
                },
                .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                .scale_x = 1.0f, .scale_y = 1.0f,
                .byte_swap = true,
                .mode = PPA_TRANS_MODE_BLOCKING,
            };
            ppa_do_scale_rotate_mirror(s_ppa_client, &gswap);

            convert_time_sum_us += (esp_timer_get_time() - convert_start_us);
            frames_converted++;
            if (s_stream) {
                uvc_host_frame_return(s_stream, qf.frame);
            }

            s_pending_arrival_us = qf.arrival_us;
            blit_strips(s_gen_x, s_gen_y,
                                       s_gen_x + s_gen_w, s_gen_y + s_gen_h, s_rgb_tx);

            int64_t gnow = esp_timer_get_time();
            if (gnow - s_last_report_us >= 2000000) {
                int n_e2e = s_frames_rendered, n_conv = frames_converted;
                if (n_e2e > 0 && n_conv > 0) {
                    /* Snapshot for the on-screen readout: the serial adapter and the camera's
                     * adapter cannot always share the board's two sockets, so the panel has to
                     * be able to report this on its own. */
                    s_stat_fps_x10    = (uint32_t)((int64_t)n_e2e * 10 * 1000000 /
                                                   (gnow - s_last_report_us));
                    s_stat_convert_us = (uint32_t)(convert_time_sum_us / n_conv);
                    s_stat_e2e_us     = (uint32_t)(s_e2e_time_sum_us / n_e2e);
                    ESP_LOGI(TAG, "generic displayed=%d avg_convert=%" PRId64
                                  "us avg_e2e=%" PRId64 "us (%ux%u %s -> %dx%d)",
                             n_e2e, convert_time_sum_us / n_conv, s_e2e_time_sum_us / n_e2e,
                             s_cam_w, s_cam_h,
                             (s_cam_fmt == UVC_VS_FORMAT_MJPEG) ? "MJPEG" : "YUY2",
                             s_gen_w, s_gen_h);
                }
                /* Printed even when nothing was displayed: "delivered but all truncated" is
                 * the single most informative state this link can be in, and a silent log
                 * would look identical to no frames arriving at all. */
                if (s_cam_fmt == UVC_VS_FORMAT_MJPEG) {
                    const uint32_t tot = s_mj_ok + s_mj_truncated + s_mj_decode_err;
                    if (tot) {
                        ESP_LOGI(TAG, "mjpeg ok=%u truncated=%u decode_err=%u (%u%% complete)",
                                 (unsigned)s_mj_ok, (unsigned)s_mj_truncated,
                                 (unsigned)s_mj_decode_err, (unsigned)(s_mj_ok * 100 / tot));
                    }
                    s_mj_ok = s_mj_truncated = s_mj_decode_err = 0;
                }
                s_frames_rendered = 0;
                s_e2e_time_sum_us = 0;
                frames_converted = 0;
                convert_time_sum_us = 0;
                s_last_report_us = gnow;
            }
            continue;
        }

        /* Where the radiometric plane sits depends on the model: the bottom half of the frame on
         * the P2 Pro, the very start of it on the GW192A. Either way, a frame too short to
         * contain it must not be parsed - reading past the end walks into whatever PSRAM
         * follows and renders as garbage rather than failing loudly. Seen for real on a P2 that
         * the vendor phone app had left in image-only mode (256x192, 98304 bytes). */
        const size_t needed = s_therm->raw_offset +
                              raw_plane_words((size_t)s_therm->raw_w * s_therm->raw_h) * 2;
        if (qf.frame->data_len < needed) {
            static bool warned = false;
            if (!warned) {
                ESP_LOGE(TAG, "frame is %d bytes, need %d - camera has no radiometric half "
                              "(image-only mode?). Not rendering.", (int)qf.frame->data_len, (int)needed);
                warned = true;
            }
            if (!no_temp_shown) {
                xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);
                draw_status_bar("NO\nTEMP");
                memset(s_rgb_tx, 0, (size_t)s_img_w * s_fb_h * sizeof(uint16_t));
                s_pending_arrival_us = esp_timer_get_time();
                blit_strips(s_img_off_x, s_img_off_y,
                                           s_img_off_x + s_img_w, s_img_off_y + s_fb_h, s_rgb_tx);
                no_temp_shown = true;
            }
            if (s_stream) {
                uvc_host_frame_return(s_stream, qf.frame);
            }
            s_stream_restart_req = true; /* renegotiate - don't just sit here showing an error */
            continue;
        }
        no_temp_shown = false;

        const uint8_t *raw_half = qf.frame->data + s_therm->raw_offset;
        const uint16_t *raw16 = (const uint16_t *)raw_half;
        size_t npix = (size_t)s_therm->raw_w * s_therm->raw_h;
        if (s_therm->marker_stride) {
            raw16 = destuff_plane(raw16, npix);
        }
        /* Both of these read P2-specific signatures - the 0x8000 "not calibrated yet" sentinel
         * and the spatial roughness that marks a pre-flat-field frame - and they drive the
         * STARTING / CALIBRATING overlays. The GW192A shows neither, so running them on it would
         * only invent phases it does not have. */
        if (s_therm->vendor_cmds) {
            p2_frame_diag(raw16, npix, qf.frame->data_len, s_stream_open_us);
            p2_log_first_clean(raw16, npix, s_stream_open_us);
        }

        /* The camera re-runs its flat-field every MAX_INTERVAL (60s), closing the shutter and
         * freezing the image each time. PROP_SWITCH turns that off. Only ever applied AFTER the
         * first calibration: the startup calibration is itself an auto-shutter event, so doing
         * this any earlier leaves the image permanently uncorrected. */
        if (s_autoshutter_req) {
            s_autoshutter_req = false;
            const uint16_t want = s_autoshutter_on ? 1 : 0;
            esp_err_t we = p2_long_cmd_write(P2_CMD_SET_AUTO_SHUTTER, P2_ASP_PROP_SWITCH, want);
            uint8_t rb[2] = {0xFF, 0xFF};
            esp_err_t re = p2_long_cmd_read(P2_CMD_GET_AUTO_SHUTTER, P2_ASP_PROP_SWITCH,
                                            rb, sizeof(rb));
            const unsigned got = (re == ESP_OK) ? (unsigned)((rb[0] << 8) | rb[1]) : 0xFFFFu;
            ESP_LOGW(TAG, "auto-shutter: PROP_SWITCH := %u -> write %s, reads back %u%s",
                     want, (we == ESP_OK) ? "OK" : p2_err(we), got,
                     (got == want) ? "" : "  <== REFUSED");
            s_osd_dirty = true;
        }

        /* --- flat-field correction --- */
        if (s_ffc_req) { /* start a capture; accumulate UNcorrected frames */
            s_ffc_req = false;
            s_ffc_active = false;
            s_ffc_remaining = FFC_FRAMES;
            memset(s_ffc_accum, 0, npix * sizeof(uint32_t));
            s_osd_dirty = true;
        }
        if (s_ffc_remaining > 0) {
            for (size_t i = 0; i < npix; i++) {
                s_ffc_accum[i] += raw16[i];
            }
            if (--s_ffc_remaining == 0) {
                /* Each pixel's deviation from the frame mean IS its offset error - valid only
                 * because the operator pointed the camera at a uniform surface. */
                uint64_t sum = 0;
                for (size_t i = 0; i < npix; i++) {
                    sum += s_ffc_accum[i];
                }
                int32_t mean = (int32_t)(sum / npix / FFC_FRAMES);
                int sat = 0;
                for (size_t i = 0; i < npix; i++) {
                    int32_t d = (int32_t)(s_ffc_accum[i] / FFC_FRAMES) - mean;
                    if (d >  127) { d =  127; sat++; }
                    if (d < -128) { d = -128; sat++; }
                    s_ffc_offset[i] = (int8_t)d;
                }
                if (sat) {
                    ESP_LOGW(TAG, "FFC: %d pixels saturated the int8 offset range", sat);
                }
                s_ffc_active = true;
                ESP_LOGI(TAG, "FFC captured over %d frames (mean %d)", FFC_FRAMES, (int)mean);
                ffc_store_save(npix);
            }
            s_osd_dirty = true;
        }

        /* Recompute the min/max normalization range every MINMAX_PERIOD frames by default, OR
         * immediately if the previous frame actually needed one (a pixel fell outside the
         * cached range) - demand-triggered, not a blind schedule. A fixed period alone means a
         * sudden hot/cold pixel stays clamped (saturated white/black) for up to MINMAX_PERIOD
         * frames even though it's now clearly out of range; reacting to that directly cuts the
         * worst-case lag to one frame while keeping the common-case cost low, same principle as
         * SimpleHandheldThermalImager/docs/PERFORMANCE.md's "throttle on tolerance, not a blind
         * rebuild schedule" lesson. */
#define MINMAX_PERIOD 8
        static int minmax_counter = 0;
        static bool need_recompute = true; /* force a real scan on the very first frame */
        static uint16_t cached_min_v = 0, cached_max_v = 1, cached_range = 1;
        bool out_of_range_this_frame = false;
        if (s_scale_latch_req) { /* freeze whatever the auto scale had settled on */
            s_manual_min = cached_min_v;
            s_manual_max = cached_max_v;
            s_manual_range = cached_range;
            s_scale_latch_req = false;
        }
        const bool manual = s_scale_manual;
        if (manual && (s_adj_min_steps || s_adj_max_steps)) {
            int dmin = s_adj_min_steps, dmax = s_adj_max_steps;
            s_adj_min_steps = 0;
            s_adj_max_steps = 0;
            int32_t nmin = (int32_t)s_manual_min + dmin * adj_step_raw();
            int32_t nmax = (int32_t)s_manual_max + dmax * adj_step_raw();
            if (nmin < 0) nmin = 0;
            if (nmax > 65535) nmax = 65535;
            /* keep the endpoints ordered - ignore a nudge that would cross them over */
            if (nmin < nmax) {
                s_manual_min = (uint16_t)nmin;
                s_manual_max = (uint16_t)nmax;
                s_manual_range = s_manual_max - s_manual_min;
                s_osd_dirty = true; /* show the new endpoints straight away */
            }
        }
        /* In manual mode the full-frame min/max scan is skipped entirely - a locked scale
         * doesn't need it, and it hands back a few ms of the per-frame budget. */
        if (!manual && (minmax_counter == 0 || need_recompute || s_osd_dirty)) {
            uint16_t min_v = 0xFFFF, max_v = 0;
            for (size_t i = 0; i < npix; i++) {
                uint16_t v = ffc_px(raw16, i);
                if (raw_invalid(v)) {
                    continue;
                }
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
            }
            if (min_v > max_v) {     /* every pixel was a marker - keep the previous range */
                min_v = cached_min_v;
                max_v = cached_max_v;
            }
            cached_min_v = min_v;
            cached_max_v = max_v;
            cached_range = (max_v > min_v) ? (max_v - min_v) : 1;
            minmax_counter = 0;
            need_recompute = false;
        }
        if (!manual) {
            minmax_counter = (minmax_counter + 1) % MINMAX_PERIOD;
        }
        uint16_t min_v = manual ? s_manual_min   : cached_min_v;
        uint16_t max_v = manual ? s_manual_max   : cached_max_v;
        uint16_t range = manual ? s_manual_range : cached_range;
        uint16_t centre_v = centre_raw(raw16);
        /* Snapshot the palette once per frame rather than reading the volatile per pixel. */
        const uint16_t *lut = s_palettes[s_palette];

        for (size_t i = 0; i < npix; i++) {
            uint16_t v = ffc_px(raw16, i);
            if (raw_invalid(v)) {
                /* Paint a marker as the coldest colour instead of letting it saturate white:
                 * it is a handful of scattered pixels, and a dark speck reads as noise while a
                 * white one reads as a hotspot that is not there. It must NOT set
                 * out_of_range_this_frame either, or the min/max scan would re-run every frame
                 * forever chasing a pixel that is never in range. */
                s_small_rgb[i] = lut[0];
                continue;
            }
            /* min_v/max_v can be stale (see MINMAX_PERIOD above), so a pixel can legitimately
             * fall outside [min_v, max_v] between recalculations. Clamp instead of letting the
             * uint8_t cast wrap: a hot pixel above the stale max must saturate to 255 (white),
             * not wrap around to near 0 (black) - which is exactly the "goes black until the
             * scale readjusts" artifact seen on hardware. */
            int32_t diff = (int32_t)v - (int32_t)min_v;
            if (diff < 0) {
                diff = 0;
                out_of_range_this_frame = true;
            }
            uint32_t scaled = ((uint32_t)diff * 255) / range;
            if (scaled > 255) {
                scaled = 255;
                out_of_range_this_frame = true;
            }
            uint8_t norm = (uint8_t)scaled;
            s_small_rgb[i] = lut[norm];
        }
        if (out_of_range_this_frame && !manual) {
            need_recompute = true; /* scene range actually changed - don't wait out the period */
        }

        /* Upscale from the sensor's native size to s_img_w x s_img_h (a slight
         * inset from the full panel, see IMG_COVERAGE) using the PPA hardware scaler, instead
         * of a CPU gather loop - a software nearest-neighbor version of this (indexed PSRAM
         * reads) cost ~16ms of CPU time and dragged the whole pipeline behind real time. */
        const ppa_srm_oper_config_t srm_config = {
            .in = {
                .buffer = s_small_rgb,
                .pic_w = s_therm->raw_w, .pic_h = s_therm->raw_h,
                .block_w = s_therm->raw_w, .block_h = s_therm->raw_h,
                .block_offset_x = 0, .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .out = {
                .buffer = s_rgb_buf,
                .buffer_size = ppa_out_bytes(s_img_w, s_fb_h),
                .pic_w = s_img_w, .pic_h = s_fb_h,
                .block_offset_x = 0, .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0, /* rotation is handled by the panel's own swap_xy/mirror */
            .scale_x = (float)s_img_w / s_therm->raw_w,
            .scale_y = (float)s_img_h / s_therm->raw_h,
            .mirror_x = false,
            .mirror_y = false,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };
        esp_err_t ppa_err = ppa_do_scale_rotate_mirror(s_ppa_client, &srm_config);
        if (ppa_err != ESP_OK) {
            ESP_LOGE(TAG, "ppa_do_scale_rotate_mirror failed: %s", esp_err_to_name(ppa_err));
        }

        /* PPA only fills the top s_img_h rows; blank the remainder so nothing stale shows
         * through behind the readouts. */
        if (s_fb_h > s_img_h) {
            memset(&s_rgb_buf[(size_t)s_img_h * s_img_w], 0,
                   (size_t)(s_fb_h - s_img_h) * s_img_w * sizeof(uint16_t));
        }

        /* A cold camera goes through two distinct unusable phases, and they need different
         * labels - the earlier version called both "CALIBRATING", which was wrong for the first
         * and absent for the second.
         *
         *   STARTING    every radiometric pixel is the constant 0x8000 sentinel. The camera is
         *               booting: its command channel does not even answer yet, and the shutter
         *               has not fired. Nothing is being calibrated.
         *   CALIBRATING real but uncorrected data (~53x spatially rougher than a finished image).
         *               The camera is waiting on its scheduled shutter, and the flat-field lands
         *               at the END of this window. This is the phase that is genuinely calibration,
         *               and it is the one that used to show through as a garbled picture.
         *
         * The roughness test is bounded to the startup window and stops once a clean frame has
         * been seen, so a high-contrast scene can never cause a spurious overlay later on. */
        const bool flat = (min_v == P2_RAW_UNCAL && max_v == P2_RAW_UNCAL);
        const bool in_startup_window =
            !s_clean_logged && (esp_timer_get_time() - s_stream_open_us) < 10000000;
        const bool uncorrected = in_startup_window && !flat && s_startup_roughness >= 100;
        const char *notice = flat ? "STARTING" : (uncorrected ? "CALIBRATING" : NULL);

        const bool uncalibrated = (notice != NULL);
        if (uncalibrated) {
            memset(s_rgb_buf, 0, (size_t)s_img_w * s_fb_h * sizeof(uint16_t));
            const int cal_scale = 3;
            int tw = (int)strlen(notice) * (FONT_W + 1) * cal_scale;
            draw_text_to(s_rgb_buf, s_img_w, s_fb_h,
                         (s_img_w - tw) / 2, (s_fb_h - FONT_H * cal_scale) / 2,
                         cal_scale, 0xFFFF, notice);
        }

        if (s_osd_enabled && !uncalibrated) { /* OSD toggle covers the overlays too, not just the bar */
            draw_crosshair();
            draw_readouts(min_v, max_v, centre_v);
        }

        /* Byte-swap for the panel's big-endian transfer requirement - done by the PPA rather
         * than the CPU. A 1:1 SRM with byte_swap reads each pixel's bytes swapped, passes the
         * value through unscaled, and writes it back normally, so memory [b0,b1] comes out
         * [b1,b0]. That is exactly the swap, and it replaces a ~5.5ms CPU pass over the whole
         * frame. PPA can't safely read and write one buffer, hence the separate s_rgb_tx. */
        const ppa_srm_oper_config_t swap_config = {
            .in = {
                .buffer = s_rgb_buf,
                .pic_w = s_img_w, .pic_h = s_fb_h,
                .block_w = s_img_w, .block_h = s_fb_h,
                .block_offset_x = 0, .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .out = {
                .buffer = s_rgb_tx,
                .buffer_size = ppa_out_bytes(s_img_w, s_fb_h),
                .pic_w = s_img_w, .pic_h = s_fb_h,
                .block_offset_x = 0, .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
            .scale_x = 1.0f,
            .scale_y = 1.0f,
            .mirror_x = false,
            .mirror_y = false,
            .byte_swap = true,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };
        esp_err_t swap_err = ppa_do_scale_rotate_mirror(s_ppa_client, &swap_config);
        if (swap_err != ESP_OK) {
            ESP_LOGE(TAG, "PPA byte-swap pass failed: %s", esp_err_to_name(swap_err));
        }

#if P2_GW_CALIB
        if (!s_therm->vendor_cmds) {
            gw_calib_report(raw16, npix, centre_v, min_v, max_v);
        }
#endif
        last_min_v = min_v;
        last_max_v = max_v;
        s_dbg_min_v = min_v;
        s_dbg_max_v = max_v;
        convert_time_sum_us += (esp_timer_get_time() - convert_start_us);
        frames_converted++;

        /* Frame data is fully consumed into s_rgb_buf now - release it back to the UVC driver
         * before kicking off the (possibly slower) SPI transfer, not after. */
        if (s_stream) { /* can be NULL if the camera vanished while this frame was queued */
            uvc_host_frame_return(s_stream, qf.frame);
        }

        /* The centre reading changes continuously as the camera is aimed, but the status-bar
         * transfer costs ~3ms of SPI against a ~39ms-of-40ms budget - redrawing it every frame
         * would put us straight back into frame-buffer underflow. Rate-limit it, and still skip
         * the transfer when the rendered text wouldn't change. */
#define BAR_MIN_INTERVAL_US 250000
        static int64_t last_bar_us = 0;
        static char last_bar_text[48] = "";
        int64_t bar_now = esp_timer_get_time();
        if (bar_now - last_bar_us >= BAR_MIN_INTERVAL_US || s_osd_dirty) {
            char key[48];
            /* The readouts live on the image now, so the bar only changes when a control
             * changes - which means it almost never needs re-transmitting. */
            snprintf(key, sizeof(key), "%d%d%d%d",
                     s_osd_enabled ? 1 : 0, s_palette, s_scale_manual ? 1 : 0,
                     s_ffc_remaining ? 2 : (s_ffc_active ? 1 : 0));
            if (s_osd_dirty || strcmp(key, last_bar_text) != 0) {
                update_status_bar(min_v, max_v, centre_v);
                snprintf(last_bar_text, sizeof(last_bar_text), "%s", key);
                s_osd_dirty = false;
            }
            last_bar_us = bar_now;
        }

        s_pending_arrival_us = qf.arrival_us; /* read by on_color_trans_done once this transfer completes */
        blit_strips(s_img_off_x, s_img_off_y,
                                   s_img_off_x + s_img_w, s_img_off_y + s_fb_h, s_rgb_tx);

        int64_t now = esp_timer_get_time();
        if (now - s_last_report_us >= 2000000) { /* every ~2s */
            int n_e2e = s_frames_rendered;
            int n_conv = frames_converted;
            if (n_e2e > 0 && n_conv > 0) {
                ESP_LOGI(TAG, "displayed=%d avg_convert=%" PRId64 "us avg_e2e(arrival->on screen)=%" PRId64 "us raw[min=%u max=%u]",
                         n_e2e, convert_time_sum_us / n_conv, s_e2e_time_sum_us / n_e2e, last_min_v, last_max_v);
            }
            s_frames_rendered = 0;
            s_e2e_time_sum_us = 0;
            frames_converted = 0;
            convert_time_sum_us = 0;
            s_last_report_us = now;
        }
    }
}

#if P2_GEN_BENCH
/* Advance to the next preset and renegotiate. Re-selection reuses the cached advertised list, so
 * this does not need the camera to be replugged. */
static void gen_bench_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(P2_GEN_BENCH_MS));
        if (s_cam_kind != CAM_GENERIC || !s_camera_connected || s_adv_n == 0) {
            continue;
        }
        const int next = (s_gen_preset + 1) %
                         (int)(sizeof(s_gen_presets) / sizeof(s_gen_presets[0]));
        const gen_preset_t *p = &s_gen_presets[next];

        /* Only accept the preset if the camera actually advertises its mode. */
        bool have = false;
        for (size_t i = 0; i < s_adv_n; i++) {
            if (s_adv[i].format == s_cam_fmt &&
                s_adv[i].h_res == p->cam_w && s_adv[i].v_res == p->cam_h) {
                have = true;
                break;
            }
        }
        if (!have) {
            ESP_LOGW(TAG, "bench: camera does not advertise %ux%u, skipping preset %s",
                     p->cam_w, p->cam_h, p->name);
            s_gen_preset = next;
            continue;
        }

        s_gen_preset = next;
        const bool res_changed = (s_cam_w != p->cam_w || s_cam_h != p->cam_h);
        s_cam_w = p->cam_w;
        s_cam_h = p->cam_h;

        s_src_w = p->cam_w; s_src_h = p->cam_h; s_src_x = 0; s_src_y = 0;
        s_gen_crop = p->crop && p->cam_w >= s_disp_w && p->cam_h >= s_disp_h;
        if (s_gen_crop) {
            s_gen_w = s_disp_w; s_gen_h = s_disp_h;
            s_src_w = s_disp_w; s_src_h = s_disp_h;
            s_src_x = ((int)p->cam_w - s_disp_w) / 2 & ~1;
            s_src_y = ((int)p->cam_h - s_disp_h) / 2 & ~1;
        } else {
            fit_preserving_aspect(p->cam_w, p->cam_h, s_disp_w, s_disp_h, &s_gen_w, &s_gen_h);
        }
        s_gen_x = (s_disp_w - s_gen_w) / 2;
        s_gen_y = (s_disp_h - s_gen_h) / 2;

        /* Stats belong to the preset that produced them. */
        s_stat_fps_x10 = s_stat_convert_us = s_stat_e2e_us = 0;
        s_mj_ok = s_mj_truncated = s_mj_decode_err = 0;

        ESP_LOGW(TAG, "bench: preset %s (%ux%u -> %dx%d%s)", p->name,
                 p->cam_w, p->cam_h, s_gen_w, s_gen_h, s_gen_crop ? " crop" : " scale");
        if (res_changed) {
            s_stream_restart_req = true;   /* a new resolution needs renegotiation */
        } else {
            /* scale <-> crop is a PPA change only; wipe the panel so the old letterbox
             * margins do not linger around a full-screen crop. */
            s_osd_dirty = true;
        }
    }
}
#endif

/* ---------------- UVC stream open ---------------- */

/* ---- GW192A frame-layout probe ----------------------------------------------------------------
 * One-shot reverse engineering aid, not part of normal operation. The GOYOJO GW192A advertises
 * six modes whose geometry makes no sense as pictures (96x100, 96x176, 96x194, 96x216, 96x322
 * YUY2 plus 96x96 NV12), so the only way to learn which one carries radiometric data is to
 * stream each in turn and look at the bytes.
 *
 * What the dump reports per mode, for one settled frame:
 *  - data_len against w*h*2, so a mode that delivers short frames is visible immediately.
 *  - per-row mean of the bytes: an 8-bit picture row sits near the scene grey level, a row of
 *    16-bit counts does not, and padding or telemetry rows stand out as constant.
 *  - per-row mean of the uint16 LE words. This is the discriminator that matters: the P2 Pro
 *    radiometric half is Kelvin x 64, so a room-temperature row reads ~19200. Eight-bit image
 *    data read as uint16 lands near 33000 instead, and either is far from the other.
 *  - a hexdump of the first row and the last two rows, where a header or telemetry block would be.
 *
 * Set P2_GW_PROBE to 0 to get the normal firmware back. */
#define P2_GW_PROBE 0
#if P2_GW_PROBE

static uint8_t *s_gw_copy;          /* PSRAM snapshot of one frame, dumped after the stream stops */
static size_t   s_gw_copy_len;
static volatile bool s_gw_have_copy;
static volatile int  s_gw_frames;

#define GW_COPY_CAP (128 * 1024)

static bool gw_probe_frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    /* Frame 8 rather than frame 0: the first few frames of any UVC stream can be partial, and on
     * a thermal core they also predate the first flat-field correction. */
    if (++s_gw_frames == 8 && !s_gw_have_copy && s_gw_copy) {
        size_t n = frame->data_len < GW_COPY_CAP ? frame->data_len : GW_COPY_CAP;
        memcpy(s_gw_copy, frame->data, n);
        s_gw_copy_len = n;
        s_gw_have_copy = true;   /* written last: the dumping task reads it as the ready flag */
    }
    return true;   /* nothing is queued, so the driver can have the buffer straight back */
}

static void gw_hexrow(const char *what, const uint8_t *p, int n)
{
    char line[3 * 32 + 1];
    for (int off = 0; off < n; off += 32) {
        int k = (n - off < 32) ? (n - off) : 32;
        int c = 0;
        for (int i = 0; i < k; i++) {
            c += snprintf(line + c, sizeof(line) - c, "%02x ", p[off + i]);
        }
        ESP_LOGW(TAG, "GW   %s +%3d: %s", what, off, line);
    }
}

static void gw_dump(int idx, const uvc_host_frame_info_t *fi)
{
    const uint8_t *d = s_gw_copy;
    const size_t len = s_gw_copy_len;
    /* How the HOST laid the frame out, which is what we have to index with. YUY2 is 2 bytes per
     * pixel; the NV12 luma plane is 1, with the chroma plane following at half height. */
    const int stride = (fi->format == UVC_VS_FORMAT_NV12) ? (int)fi->h_res : (int)fi->h_res * 2;
    const size_t rows = len / stride;

    ESP_LOGW(TAG, "GW[%d] fmt=%d %ux%u: data_len=%u (expected %u), stride=%d -> %u rows",
             idx, (int)fi->format, fi->h_res, fi->v_res, (unsigned)len,
             (unsigned)((size_t)stride * fi->v_res), stride, (unsigned)rows);

    char l8[80], l16[112];
    int p8 = 0, p16 = 0;
    for (size_t r = 0; r < rows; r++) {
        const uint8_t *row = d + r * stride;
        uint32_t s8 = 0, s16 = 0;
        for (int c = 0; c < stride; c++) {
            s8 += row[c];
        }
        for (int c = 0; c + 1 < stride; c += 2) {
            s16 += (uint32_t)row[c] | ((uint32_t)row[c + 1] << 8);
        }
        p8  += snprintf(l8  + p8,  sizeof(l8)  - p8,  "%4u", (unsigned)(s8 / stride));
        p16 += snprintf(l16 + p16, sizeof(l16) - p16, "%6u", (unsigned)(s16 / (stride / 2)));
        if ((r % 16) == 15 || r == rows - 1) {
            ESP_LOGW(TAG, "GW[%d] row %3u u8 :%s", idx, (unsigned)(r - r % 16), l8);
            ESP_LOGW(TAG, "GW[%d] row %3u u16:%s", idx, (unsigned)(r - r % 16), l16);
            p8 = p16 = 0;
        }
    }

    gw_hexrow("row0  ", d, stride < 64 ? stride : 64);
    if (rows >= 2) {
        gw_hexrow("row-2 ", d + (rows - 2) * stride, stride < 64 ? stride : 64);
        gw_hexrow("row-1 ", d + (rows - 1) * stride, stride < 64 ? stride : 64);
    }
}

static void gw_probe_task(void *arg)
{
    s_gw_copy = heap_caps_malloc(GW_COPY_CAP, MALLOC_CAP_SPIRAM);
    if (!s_gw_copy) {
        ESP_LOGE(TAG, "GW probe: no PSRAM for the frame snapshot");
        vTaskDelete(NULL);
    }
    while (s_adv_n == 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGW(TAG, "GW probe: %u advertised modes", (unsigned)s_adv_n);
    for (size_t i = 0; i < s_adv_n; i++) {
        /* Intervals first: 25 fps is hardcoded everywhere else in this firmware, and a mode that
         * does not offer it is exactly how the normal path fails on this camera. */
        ESP_LOGW(TAG, "GW[%d] fmt=%d %ux%u default_interval=%u type=%u",
                 (int)i, (int)s_adv[i].format, s_adv[i].h_res, s_adv[i].v_res,
                 (unsigned)s_adv[i].default_interval, (unsigned)s_adv[i].interval_type);
        if (s_adv[i].interval_type == 0) {
            ESP_LOGW(TAG, "GW[%d]   continuous %u..%u step %u", (int)i,
                     (unsigned)s_adv[i].interval_min, (unsigned)s_adv[i].interval_max,
                     (unsigned)s_adv[i].interval_step);
        } else {
            for (int k = 0; k < s_adv[i].interval_type && k < CONFIG_UVC_INTERVAL_ARRAY_SIZE; k++) {
                ESP_LOGW(TAG, "GW[%d]   interval[%d]=%u (%.1f fps)", (int)i, k,
                         (unsigned)s_adv[i].interval[k],
                         s_adv[i].interval[k] ? 1e7f / s_adv[i].interval[k] : 0.0f);
            }
        }

        s_gw_have_copy = false;
        s_gw_frames = 0;

        uvc_host_stream_config_t cfg = {
            .event_cb = NULL,
            .frame_cb = gw_probe_frame_cb,
            .user_ctx = NULL,
            .usb = { .vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID, .uvc_stream_index = 0 },
            /* fps 0 asks for the device default interval. Naming a rate is what makes
             * negotiation fail on the modes this camera only offers at one speed. */
            .vs_format = { .h_res = s_adv[i].h_res, .v_res = s_adv[i].v_res,
                           .fps = 0, .format = s_adv[i].format },
            .advanced = { .number_of_frame_buffers = 3, .frame_size = 0,
                          .frame_heap_caps = MALLOC_CAP_SPIRAM,
                          .number_of_urbs = 4, .urb_size = 0 },
        };
        uvc_host_stream_hdl_t st = NULL;
        esp_err_t e = uvc_host_stream_open(&cfg, pdMS_TO_TICKS(3000), &st);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "GW[%d] open failed: %s", (int)i, esp_err_to_name(e));
            continue;
        }
        e = uvc_host_stream_start(st);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "GW[%d] start failed: %s", (int)i, esp_err_to_name(e));
            uvc_host_stream_close(st);
            continue;
        }
        for (int t = 0; t < 50 && !s_gw_have_copy; t++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        uvc_host_stream_stop(st);
        uvc_host_stream_close(st);
        if (s_gw_have_copy) {
            gw_dump((int)i, &s_adv[i]);
        } else {
            ESP_LOGE(TAG, "GW[%d] no frames arrived in 5 s (%d callbacks)", (int)i, s_gw_frames);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    ESP_LOGW(TAG, "GW probe: done");
    free(s_gw_copy);
    s_gw_copy = NULL;
    vTaskDelete(NULL);
}
#endif /* P2_GW_PROBE */

static void streaming_task(void *arg)
{
    /* app_main allocates the frame buffers concurrently with enumeration - don't touch them until
     * it confirms they exist. Re-given immediately so a reconnect never blocks here. */
    xSemaphoreTake(s_buffers_ready, portMAX_DELAY);
    xSemaphoreGive(s_buffers_ready);
    BOOT_MARK("uvc: streaming task released");

    uvc_host_stream_config_t stream_config = {
        .event_cb = stream_event_callback,
        .frame_cb = frame_callback,
        .user_ctx = NULL,
        .usb = {
            .vid = UVC_HOST_ANY_VID,
            .pid = UVC_HOST_ANY_PID,
            .uvc_stream_index = 0,
        },
        .vs_format = {
            .h_res = s_cam_w,     /* set by uvc_event_cb from what the camera advertises */
            .v_res = s_cam_h,
            .fps = STREAM_FPS,
            .format = UVC_VS_FORMAT_YUY2,   /* re-read per iteration from s_cam_fmt */
        },
        .advanced = {
            .number_of_frame_buffers = 3,
            .frame_size = 0,
            .frame_heap_caps = MALLOC_CAP_SPIRAM,
            .number_of_urbs = 4,
            .urb_size = 0,
        },
    };

    while (true) {
        /* Re-read per iteration: the struct is built once, but swapping cameras changes the
         * format we should ask for, and a stale value here would request 256x384 from a webcam. */
        /* Wait for the connect callback to say what is actually attached. Opening before that
         * means opening with the previous camera's format. */
        if (s_cam_kind == CAM_NONE) {
            static int waited = 0;
            if (++waited % 100 == 0) {   /* ~10s */
                ESP_LOGW(TAG, "no usable camera detected yet (waiting)");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        stream_config.vs_format.h_res = s_cam_w;
        stream_config.vs_format.v_res = s_cam_h;
        stream_config.vs_format.format = s_cam_fmt;

#if P2_NO_OPEN_GENERIC
        /* Experiment: for a generic camera, do NOT open a stream - just let the device sit
         * enumerated. Isolates "this device is unstable on our port" from "our stream_open is
         * what destabilises it". The probe client keeps reporting whether it stays attached. */
        if (s_cam_kind == CAM_GENERIC) {
            static bool said = false;
            if (!said) {
                ESP_LOGW(TAG, "P2_NO_OPEN_GENERIC: leaving the camera enumerated, not opening");
                said = true;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
#endif
#if P2_TX2RX_SWEEP
        /* One value per stream attempt. Written live rather than only at init: the PHY picks it
         * up for subsequent traffic, which is what lets a single session cover the whole range
         * instead of costing eight replugs. */
        static const uint8_t tx2rx_vals[] = { 4, 5, 6, 7, 3, 2, 1, 0 };
        static int tx2rx_i = 0;
        USB_UTMI.fc_06.hs_tx2rx_dly_cnt_sel = tx2rx_vals[tx2rx_i];
        ESP_LOGW(TAG, "sweep: hs_tx2rx_dly_cnt_sel = %u (fc06=0x%08x)",
                 (unsigned)tx2rx_vals[tx2rx_i], (unsigned)USB_UTMI.fc_06.val);
        s_mj_ok = s_mj_truncated = s_mj_decode_err = 0;
#endif
        BOOT_MARK("uvc: calling stream_open");
        ESP_LOGI(TAG, "Opening %ux%u %s @%.1ffps (%s)...", s_cam_w, s_cam_h,
                 (s_cam_fmt == UVC_VS_FORMAT_MJPEG) ? "MJPEG" : "YUY2", STREAM_FPS,
                 (s_cam_kind == CAM_THERMAL) ? s_therm->name : "generic");
        esp_err_t err = uvc_host_stream_open(&stream_config, pdMS_TO_TICKS(5000), &s_stream);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "uvc_host_stream_open failed: %s - retrying", esp_err_to_name(err));
            s_stream = NULL;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        BOOT_MARK("uvc: stream_open returned");
#if P2_EARLY_CONFIG
        /* The shutter delays are counted from preview start, so configuring after stream_start
         * may be too late to move the FIRST shutter - by then the camera has already scheduled
         * it. The device handle exists from stream_open, so try the channel here. On a cold
         * camera it may well not answer yet; that is why the late path still exists. */
        s_shutter_cfg_done = false;
        s_p2_wait_ms = 60;
        for (int i = 0; i < 12 && !s_shutter_cfg_done; i++) {
            uint8_t probe[2];
            if (p2_long_cmd_read(P2_CMD_GET_AUTO_SHUTTER, P2_ASP_MIN_INTERVAL,
                                 probe, sizeof(probe)) == ESP_OK) {
                s_p2_wait_ms = 1000;
                ESP_LOGW(TAG, "p2: channel answered BEFORE preview start (attempt %d)", i + 1);
                p2_write_shutter_schedule("early");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        s_p2_wait_ms = 1000;
        if (!s_shutter_cfg_done) {
            ESP_LOGW(TAG, "p2: channel not up before preview start - will configure after");
        }
#endif
        err = uvc_host_stream_start(s_stream);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "uvc_host_stream_start failed: %s - retrying", esp_err_to_name(err));
            uvc_host_stream_close(s_stream);
            s_stream = NULL;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        BOOT_MARK("uvc: stream_start returned (first image)");
        s_stream_open_us = esp_timer_get_time();
        s_clean_logged = false;   /* re-arm for this stream */
        /* The camera forgets the shutter schedule whenever it loses power, so this must be
         * re-armed per stream, NOT left latched from the first successful write after boot.
         * Getting that wrong silently restored the stock 5s+4s on every replug. */
        s_shutter_cfg_done = false;
        ESP_LOGI(TAG, "Stream opened, starting continuous render");
#if P2_PORT_TRACE
        s_trace_head = 0;
        s_trace_run = true;
        /* The comparison that matters. Measured on the WORKING plain-USB-2.0-hub path:
         * MPS 1020, EC 3, EP 1, ~25-30 fps, TR 0, PDIS 0. So the host copes with
         * high-bandwidth isoc (3 transactions per microframe) just fine, and if the failing
         * path shows the same three numbers then nothing about the host configuration differs
         * and the cause is not in software at all. */
        {
            unsigned mps = 0, ec = 0, epnum = 0;
            /* Give the stack a moment to program the channel before reading it back. */
            vTaskDelay(pdMS_TO_TICKS(5));
            if (usb_isoc_chan_info(&mps, &ec, &epnum)) {
                ESP_LOGW(TAG, "isoc chan: MPS=%u EC=%u EP=%u  (hub case measured 1020/3/1)",
                         mps, ec, epnum);
            } else {
                ESP_LOGW(TAG, "isoc chan: none found");
            }
        }
#endif
        /* P2-only. These are InfiRay vendor control requests; firing them at an ordinary webcam
         * makes it STALL EP0 on every one, and the channel-availability probe retries for 15s at
         * 25ms intervals - a control-transfer storm that drowns the stream it is supposed to
         * precede. Seen for real against an iMac webcam. */
        if (s_cam_kind == CAM_THERMAL && s_therm->vendor_cmds) {
            p2_camera_init();
        }
#if P2_GW_XU_PROBE
        if (s_cam_kind == CAM_THERMAL && !s_therm->vendor_cmds) {
            static bool xu_enum_done = false;
            if (!xu_enum_done) {
                xu_enum_done = true;   /* one pass per boot, not per stream */
                gw_xu_enumerate();
            }
        }
#endif
#if P2_GW_XU_READ
        /* One pass per boot. Deliberately not per stream: a camera that answers once has
         * nothing more to say, and a camera that does not should not be asked repeatedly. */
        if (s_cam_kind == CAM_THERMAL && s_therm->xu_unit) {
            static bool xu_read_done = false;
            if (!xu_read_done) {
                xu_read_done = true;
                xu_read_device_params();
            }
        }
#endif
        s_stream_restart_req = false;
        s_stream_epoch++;
        s_camera_connected = true;

        /* Hold here until something asks for a renegotiation, or the device vanishes (in which
         * case stream_event_callback has already closed it and cleared s_stream). */
        while (s_stream && !s_stream_restart_req) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        s_camera_connected = false;
        if (s_stream && s_stream_restart_req) {
            ESP_LOGW(TAG, "Tearing down stream to renegotiate");
            uvc_host_stream_stop(s_stream);
            uvc_host_stream_close(s_stream);
            s_stream = NULL;
        }
        s_stream_restart_req = false;
#if P2_TX2RX_SWEEP
        /* Scored on the way out, while the counters still describe THIS value. */
        ESP_LOGW(TAG, "sweep: tx2rx=%u ended -> ok=%u truncated=%u decode_err=%u",
                 (unsigned)tx2rx_vals[tx2rx_i], (unsigned)s_mj_ok,
                 (unsigned)s_mj_truncated, (unsigned)s_mj_decode_err);
        tx2rx_i = (tx2rx_i + 1) % (int)(sizeof(tx2rx_vals) / sizeof(tx2rx_vals[0]));
#endif
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void uvc_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        return;
    }
    s_connect_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Device connected, addr=%d, starting streaming task", event->device_connected.dev_addr);

    /* Dump what the camera actually advertises - this is how we tell "image-only mode" apart
     * from a streaming fault: the 256x384 entry simply disappears from the list. */
    size_t n = event->device_connected.frame_info_num;
    uvc_host_frame_info_t *list = calloc(n, sizeof(uvc_host_frame_info_t));
    if (list) {
        size_t actual = n;
        if (uvc_host_get_frame_list(event->device_connected.dev_addr,
                                    event->device_connected.uvc_stream_index,
                                    (uvc_host_frame_info_t (*)[])list, &actual) == ESP_OK) {
            for (size_t i = 0; i < actual; i++) {
                ESP_LOGI(TAG, "  advertised[%d]: fmt=%d %ux%u", (int)i, (int)list[i].format,
                         list[i].h_res, list[i].v_res);
            }

            s_adv_n = (actual < P2_MAX_ADVERTISED) ? actual : P2_MAX_ADVERTISED;
            memcpy(s_adv, list, s_adv_n * sizeof(uvc_host_frame_info_t));

            /* Thermal if one of the known stacked radiometric formats is on offer, otherwise a
             * plain preview. Table order is the priority order, so a device advertising two of
             * them (none does today) resolves deterministically. */
            const thermal_model_info_t *therm = NULL;
            for (size_t m = 0; m < THERMAL_MODEL_COUNT && !therm; m++) {
                for (size_t i = 0; i < actual; i++) {
                    if (list[i].format == UVC_VS_FORMAT_YUY2 &&
                        list[i].h_res == s_thermal_models[m].stream_w &&
                        list[i].v_res == s_thermal_models[m].stream_h) {
                        therm = &s_thermal_models[m];
                        break;
                    }
                }
            }
            if (therm) {
                s_therm = therm;
                s_cam_kind = CAM_THERMAL;
                s_cam_w = therm->stream_w;
                s_cam_h = therm->stream_h;
                s_cam_fmt = UVC_VS_FORMAT_YUY2;   /* the raw plane must not be compressed */
                ESP_LOGI(TAG, "camera: %s thermal (stream %ux%u, %ux%u radiometric at +%u, "
                              "%.0f counts/K, vendor channel %s)",
                         therm->name, s_cam_w, s_cam_h, therm->raw_w, therm->raw_h,
                         (unsigned)therm->raw_offset, therm->counts_per_kelvin,
                         therm->vendor_cmds ? "yes" : "no");
            } else {
                /* Pick the largest mode up to P2_GEN_OVERSAMPLE x the panel in each axis.
                 *
                 * The previous rule took the largest mode that FIT inside the panel, which meant
                 * 320x240 upscaled to 420x315 - throwing away detail the camera was willing to
                 * give. Downscaling from a larger capture is sharper for the same framerate, and
                 * measured on the borescope the cost is nil: 320x240 and 640x480 showed no
                 * discernible difference in fps or end-to-end latency, confirmed on hardware.
                 *
                 * The cap matters though. Past ~2x the panel the extra pixels cannot be seen and
                 * are not free: this camera's 1280x720 MJPEG wants 46 MB/s and 4x the JPEG decode
                 * of 640x480 for no visible gain. With the cap at 2 the rule lands on 640x480
                 * here (640 <= 960, 480 <= 640) and rejects 1280x720 (1280 > 960). */
                /* Prefer MJPEG when the camera offers it: ~7x fewer isochronous packets per
                 * frame, which on a marginal link is the only lever that matters (see
                 * P2_GENERIC_PREFER_MJPEG). YUY2 is the fallback. */
                enum uvc_host_stream_format want_fmt = UVC_VS_FORMAT_YUY2;
#if P2_GENERIC_PREFER_MJPEG
                for (size_t i = 0; i < actual; i++) {
                    if (list[i].format == UVC_VS_FORMAT_MJPEG) {
                        want_fmt = UVC_VS_FORMAT_MJPEG;
                        break;
                    }
                }
#endif
                uint16_t bw = 0, bh = 0;
#if P2_GEN_BENCH_UNUSED
                for (size_t i = 0; i < actual; i++) {
                    if (list[i].format == want_fmt &&
                        list[i].h_res == s_gen_presets[s_gen_preset].cam_w &&
                        list[i].v_res == s_gen_presets[s_gen_preset].cam_h) {
                        bw = list[i].h_res;
                        bh = list[i].v_res;
                        break;
                    }
                }
                if (!bw)
#endif
#if P2_GENERIC_FORCE_W && P2_GENERIC_FORCE_H
                for (size_t i = 0; i < actual; i++) {
                    if (list[i].format == want_fmt &&
                        list[i].h_res == P2_GENERIC_FORCE_W &&
                        list[i].v_res == P2_GENERIC_FORCE_H) {
                        bw = list[i].h_res;
                        bh = list[i].v_res;
                        ESP_LOGW(TAG, "generic: forced mode %ux%u", bw, bh);
                        break;
                    }
                }
                if (!bw)
#endif
                for (size_t i = 0; i < actual; i++) {
                    if (list[i].format != want_fmt) {
                        continue;
                    }
                    const bool within = list[i].h_res <= s_disp_w * P2_GEN_OVERSAMPLE &&
                                        list[i].v_res <= s_disp_h * P2_GEN_OVERSAMPLE;
                    if (within && (uint32_t)list[i].h_res * list[i].v_res >
                                  (uint32_t)bw * bh) {
                        bw = list[i].h_res;
                        bh = list[i].v_res;
                    }
                }
                if (!bw) {
                    /* Every advertised mode is over the cap. Take the smallest rather than
                     * nothing - a picture at some cost beats no picture. */
                    for (size_t i = 0; i < actual; i++) {
                        if (list[i].format != want_fmt) {
                            continue;
                        }
                        if (!bw || (uint32_t)list[i].h_res * list[i].v_res <
                                   (uint32_t)bw * bh) {
                            bw = list[i].h_res;
                            bh = list[i].v_res;
                        }
                    }
                }
                if (bw && bh) {
                    s_cam_kind = CAM_GENERIC;
                    s_cam_w = bw;
                    s_cam_h = bh;
                    s_cam_fmt = want_fmt;
                    /* Default: read the whole frame. A cropping preset narrows this instead
                     * of scaling, so PPA runs at 1:1 and the pixels are native. */
                    s_src_w = bw; s_src_h = bh; s_src_x = 0; s_src_y = 0;
                    s_gen_crop = false;
                    if (s_gen_crop && bw >= s_disp_w && bh >= s_disp_h) {
                        /* 1:1 centre crop. Offsets are kept even: PPA reads YUV422 in pairs and
                         * an odd x offset would swap the chroma siting. */
                        s_gen_w = s_disp_w;
                        s_gen_h = s_disp_h;
                        s_src_w = s_disp_w;
                        s_src_h = s_disp_h;
                        s_src_x = ((bw - s_disp_w) / 2) & ~1;
                        s_src_y = ((bh - s_disp_h) / 2) & ~1;
                    } else {
                        s_gen_crop = false;
                        fit_preserving_aspect(bw, bh, s_disp_w, s_disp_h, &s_gen_w, &s_gen_h);
                    }
                    s_gen_x = (s_disp_w - s_gen_w) / 2;
                    s_gen_y = (s_disp_h - s_gen_h) / 2;
                    ESP_LOGI(TAG, "camera: generic %s %ux%u -> %dx%d at (%d,%d)",
                             (want_fmt == UVC_VS_FORMAT_MJPEG) ? "MJPEG" : "YUY2",
                             bw, bh, s_gen_w, s_gen_h, s_gen_x, s_gen_y);
                } else {
                    ESP_LOGW(TAG, "camera: no usable %s format advertised",
                             (want_fmt == UVC_VS_FORMAT_MJPEG) ? "MJPEG" : "YUY2");
                }
            }
        }
        free(list);
    }
#if P2_GEN_BENCH
    static bool bench_started = false;
    if (!bench_started) {
        bench_started = true;
        xTaskCreate(gen_bench_task, "genbench", 3072, NULL, 2, NULL);
    }
#endif
    static bool streaming_task_started = false; /* the task now loops forever and handles
                                                 * reconnects itself - only ever spawn one */
    if (!streaming_task_started) {
        streaming_task_started = true;
#if P2_GW_PROBE
        /* The probe owns the camera for the whole session: it opens and closes every advertised
         * mode in turn, so a second client negotiating alongside it would corrupt the results. */
        xTaskCreatePinnedToCore(gw_probe_task, "gwprobe", 6144, NULL, USB_HOST_PRIORITY - 2, NULL, tskNO_AFFINITY);
#else
        xTaskCreatePinnedToCore(streaming_task, "streaming", 6144, NULL, USB_HOST_PRIORITY - 2, NULL, tskNO_AFFINITY);
#endif
    }
}

void app_main(void)
{
    build_palettes();

    s_frame_queue = xQueueCreate(3, sizeof(queued_frame_t));
    assert(s_frame_queue);
    s_buffers_ready = xSemaphoreCreateBinary();
    assert(s_buffers_ready);

    /* USB first. Enumeration costs ~550ms and proceeds on usb_lib_task, so start it now and do
     * display init and buffer allocation inside that window instead of serialising after it.
     * Nothing here touches the display, and the streaming task waits on s_buffers_ready. */
    /* Temporary: let the UVC component's own DEBUG lines through so the 650ms inside
     * uvc_host_stream_start can be attributed. Needs CONFIG_LOG_MAXIMUM_LEVEL_DEBUG. */
#if P2_DIAG
    esp_log_level_set("uvc", ESP_LOG_DEBUG);
    esp_log_level_set("uvc-control", ESP_LOG_DEBUG);
    esp_log_level_set("uvc-stream", ESP_LOG_DEBUG);
#endif

#if P2_USB_ENUM_TRACE
    /* The enumerator names each stage it fails at; raise its log level so the failing stage is
     * visible rather than inferred from which descriptor read errored. */
    esp_log_level_set("ENUM", ESP_LOG_DEBUG);
    esp_log_level_set("HUB", ESP_LOG_DEBUG);
    esp_log_level_set("USBH", ESP_LOG_DEBUG);
#endif
    BOOT_MARK("usb: installing host");
    ESP_LOGI(TAG, "Installing USB Host (native HS OTG port)");
    /* The P4 has two OTG controllers - USB_DWC_LL_GET_HW(num) gives USB_DWC_FS for num==1 and
     * USB_DWC_HS otherwise - so peripheral_map = BIT1 puts the host on the FULL-SPEED controller.
     * At 12 Mbit/s the signalling is far more tolerant and high-speed disconnect detection (which
     * is what drops the borescope) does not apply. The cost is bandwidth: ~1.1 MB/s realistic,
     * so 320x240 YUY2 manages ~7 fps rather than 25. Whether the FS controller's pins even reach
     * this board's OTG connector is a schematic question - if they do not, nothing enumerates. */
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
#if P2_MANUAL_PORT_POWER
        /* Install with the root port UNPOWERED so the UTMI trims can be written after PHY
         * init (which does usb_utmi_ll_reset_register(), wiping anything written earlier) but
         * BEFORE anything is attached. Writing them after the port is live glitches the link -
         * measured: writing even the hardware defaults post-install dropped enumerations from
         * 5-10 per 22s to 0. This also makes set_root_port_power(true) valid; it returns
         * ESP_ERR_INVALID_STATE ("already powered") otherwise. */
        .root_port_unpowered = true,
#endif
#if P2_FORCE_FULL_SPEED
        .peripheral_map = BIT1,
#endif
    };
#if P2_FORCE_FULL_SPEED
    ESP_LOGW(TAG, "USB: forcing the FULL-SPEED controller (peripheral_map=BIT1)");
#endif
    ESP_ERROR_CHECK(usb_host_install(&host_config));

#if P2_MASK_HS_DISCONNECT
    /* EXPERIMENT. The borescope completes every enumeration stage and is then reported
     * "device gone" - a FALSE high-speed disconnect. HS disconnect is detected purely
     * electrically (the host samples the differential amplitude during each SOF's EOP; an
     * unloaded bus roughly doubles it past ~625mV), so reflections on a marginal cable can trip
     * it while the device is still physically present and fine. The P4's USB register space
     * exposes no PHY tuning - no squelch, no threshold, no termination trim - only this mask.
     *
     * Masking it reaches behind the host library, which owns this register. If it works, genuine
     * removal is no longer detected by interrupt; the existing "no frames for ~3s -> restart"
     * supervisor in render_task is the fallback. */
    USB_DWC_HS.gintmsk_reg.disconnintmsk = 0;
    ESP_LOGW(TAG, "USB: HS disconnect interrupt MASKED (experiment)");
#endif
#if P2_UTMI_TRIM
    /* Written here: after usb_host_install() has brought the PHY up (and done its register
     * reset), but before the root port is powered and anything can attach. */
    ESP_LOGW(TAG, "UTMI before: fc00=0x%08x fc01=0x%08x fc02=0x%08x",
             (unsigned)USB_UTMI.fc_00.val, (unsigned)USB_UTMI.fc_01.val,
             (unsigned)USB_UTMI.fc_02.val);
    /* Written individually: a bitfield assignment is a read-modify-write of the whole 32-bit
     * register, so if any register has bits that do not read back what must be written, touching
     * it corrupts them. Testing one at a time shows which. */
#if P2_UTMI_W_FC00
    USB_UTMI.fc_00.adj_res_hs  = P2_UTMI_RES_HS;
#endif
#if P2_UTMI_W_FC01
    USB_UTMI.fc_01.adj_vref_sq = P2_UTMI_VREF_SQ;
#endif
#if P2_UTMI_W_FC02
    USB_UTMI.fc_02.adj_vsw_hs  = P2_UTMI_VSW_HS;
#endif
    ESP_LOGW(TAG, "UTMI after : fc00=0x%08x fc01=0x%08x fc02=0x%08x",
             (unsigned)USB_UTMI.fc_00.val, (unsigned)USB_UTMI.fc_01.val,
             (unsigned)USB_UTMI.fc_02.val);
#endif
#if P2_TX2RX_SWEEP
    ESP_LOGW(TAG, "UTMI fc06=0x%08x (hs_tx2rx_dly_cnt_sel=%u)",
             (unsigned)USB_UTMI.fc_06.val, (unsigned)USB_UTMI.fc_06.hs_tx2rx_dly_cnt_sel);
#endif
#if P2_MANUAL_PORT_POWER
    {
        esp_err_t pwr = usb_host_lib_set_root_port_power(true);
        ESP_LOGW(TAG, "usb: root port powered manually -> %s", esp_err_to_name(pwr));
    }
#endif
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, USB_HOST_PRIORITY, NULL, 0);
#if P2_UFRAME_CHECK
    xTaskCreatePinnedToCore(uframe_check_task, "uframechk", 3072, NULL, 2, NULL, 1);
#endif
#if P2_PORT_TRACE
    /* Core 1, below the USB stack: sampling must not delay the transfers it is observing. */
    xTaskCreatePinnedToCore(port_trace_task, "porttrace", 3072, NULL, 2, NULL, 1);
#endif
#if P2_PORT_POWER_RECOVERY
    xTaskCreate(usb_port_recovery_task, "usbrecov", 3072, NULL, 3, NULL);
#endif
#if P2_USB_PROBE
    xTaskCreate(usb_probe_task, "usbprobe", 4096, NULL, USB_HOST_PRIORITY - 3, NULL);
#endif

    BOOT_MARK("usb: host installed, installing uvc driver");
    ESP_LOGI(TAG, "Installing UVC host driver");
    const uvc_host_driver_config_t uvc_driver_config = {
        .driver_task_stack_size = 4 * 1024,
        .driver_task_priority = USB_HOST_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_event_cb,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_driver_config));

    ESP_LOGI(TAG, "Waiting for P2 Pro to enumerate...");


    /* display_init() sets s_disp_w/s_disp_h for the configured orientation - buffers below
     * depend on that, so this must run first. */
    BOOT_MARK("app_main entered");
    ESP_ERROR_CHECK(display_init());
    BOOT_MARK("display_init returned");

    /* PPA requires its input/output buffers aligned to the cache line size (64B covers both
     * L1 and L2 on P4); plain heap_caps_malloc doesn't guarantee that. */
    s_small_rgb = heap_caps_aligned_alloc(P2_CACHE_LINE, RAW_H_RES * RAW_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    assert(s_small_rgb);

    /* CPU-only (every pass that reads it is a plain loop), so this needs no DMA capability -
     * but it is sized for the biggest sensor in the table rather than the attached one, so that
     * swapping cameras re-points the parse without reallocating on the render task. */
    s_raw_destuff = heap_caps_malloc(RAW_H_RES * RAW_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    assert(s_raw_destuff);

    /* Sized for the LARGER of the thermal image and a full-screen generic preview: a webcam's
     * picture is letterboxed to the panel, which can exceed the thermal image's 416x312. Only
     * the used region is ever transferred, so the extra PSRAM costs nothing per frame. */
    const size_t fb_px = ((size_t)s_img_w * s_fb_h > (size_t)s_disp_w * s_disp_h)
                         ? (size_t)s_img_w * s_fb_h : (size_t)s_disp_w * s_disp_h;
    /* PPA validates that out.buffer_size is cache-line aligned, not just the address. Declaring
     * the USED extent fails whenever w*h*2 is not a multiple of 64 - 374x306 gives 228888, which
     * is 24 over. The thermal path only passed because 416x312x2 = 259584 happens to divide by 64.
     * The field means the size OF THE BUFFER, so publish the real allocation and round it up. */
    s_fb_bytes = (fb_px * sizeof(uint16_t) + (P2_CACHE_LINE - 1)) & ~(size_t)(P2_CACHE_LINE - 1);  /* covers both paths */
    s_rgb_buf = heap_caps_aligned_alloc(P2_CACHE_LINE, s_fb_bytes,
                                         MALLOC_CAP_SPIRAM);
    s_rgb_tx = heap_caps_aligned_alloc(P2_CACHE_LINE, s_fb_bytes,
                                        MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (!s_rgb_tx) {
        s_rgb_tx = heap_caps_aligned_alloc(P2_CACHE_LINE, s_fb_bytes, MALLOC_CAP_DMA);
    }
    assert(s_rgb_buf && s_rgb_tx);

    /* Kept in PSRAM. Moving these two (96KB each) to internal SRAM made the apply pass cheaper
     * but starved the USB stack, which needs internal DMA memory for its ISOC transfers: the
     * camera dropped out with a continuous "Frame buffer underflow". Internal RAM here is not
     * free real estate. If the ~4.8ms/frame apply cost needs fixing, fold the offset into the
     * existing palette/min-max passes instead of buying speed with internal memory. */
    s_ffc_offset    = heap_caps_malloc(RAW_H_RES * RAW_V_RES * sizeof(int8_t),   MALLOC_CAP_SPIRAM);
    s_ffc_accum     = heap_caps_malloc(RAW_H_RES * RAW_V_RES * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    assert(s_ffc_offset && s_ffc_accum);
    /* Not loaded here any more: which table is valid depends on the sensor that turns up, and
     * at this point nothing has enumerated. render_task calls ffc_reset_for_camera() as soon as
     * it knows, which costs one 48KB partition read against a camera that takes ~1.1 s to
     * appear - invisible in the boot budget. */

    const ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 2, /* scale pass + byte-swap pass */
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_client_config, &s_ppa_client));
    BOOT_MARK("buffers + PPA client ready");

    /* Sized for the full panel, not the current bar: the bar is 64px wide for a 256x192 sensor
     * and 192px for a 96x96 one, and a camera swap re-lays-out the panel without reallocating.
     * 300KB of PSRAM against a 32MB part is the cheap way to make that resize unable to overrun. */
    const size_t bar_bytes = (size_t)s_disp_w * s_disp_h * sizeof(uint16_t);
    s_status_buf = heap_caps_malloc(bar_bytes, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (!s_status_buf) {
        s_status_buf = heap_caps_malloc(bar_bytes, MALLOC_CAP_DMA);
    }
    assert(s_status_buf);

    esp_err_t touch_err = touch_init();
    if (touch_err != ESP_OK) {
        ESP_LOGW(TAG, "Touch init failed (%s) - OSD toggle won't be available", esp_err_to_name(touch_err));
    } else {
        xTaskCreatePinnedToCore(touch_task, "touch", 3072, NULL, 5, NULL, 0);
    }

    /* Bumped from 4096: update_status_bar()/draw_text()/snprintf (float formatting) added real
     * stack depth to this task's call chain, and a silent stack overflow here could manifest as
     * unrelated-looking memory corruption - cheap to rule out. */
    xTaskCreatePinnedToCore(render_task, "render", 6144, NULL, USB_HOST_PRIORITY - 1, NULL, 1);

    /* Everything the streaming/render path touches now exists; let the streaming task proceed. */
#if P2_RENDER_STAGE_TRACE
    xTaskCreate(render_stage_task, "rstage", 3072, NULL, 1, NULL);
#endif
#if P2_WIFI_CAM
    xTaskCreate(wifi_cam_task, "wificam", 4096, NULL, 3, NULL);
#endif
    BOOT_MARK("app_main done, releasing streaming task");
    xSemaphoreGive(s_buffers_ready);
}
