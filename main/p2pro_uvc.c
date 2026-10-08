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
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ppa.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7796.h"

#include "usb/usb_host.h"
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
#define BAR_SLOTS 7    /* top to bottom */
#define SLOT_MAXPLUS 0
#define SLOT_MAXMINUS 1
#define SLOT_FFC 2
#define SLOT_PAL 3
#define SLOT_SCL 4
#define SLOT_MINPLUS 5
#define SLOT_MINMINUS 6

/* Software flat-field correction. The camera's own shutter FFC is strictly better (it has a
 * real uniform reference); this corrects residual drift between those, and needs the user to
 * aim at something thermally uniform before pressing. Never persisted - RAM only. */
#define FFC_FRAMES 32          /* ~1.3s at 25fps - averages the temporal noise down */
static int16_t  *s_ffc_offset;    /* per-pixel deviation from the frame mean */
static uint16_t *s_ffc_corrected; /* raw minus offset, so downstream code is unchanged */
static uint32_t *s_ffc_accum;
static volatile bool s_ffc_active = false;
static volatile bool s_ffc_req = false;
static int s_ffc_remaining = 0;

/* One press = 2 C. Raw counts are Kelvin*64, so a 2 K step is 2*64. */
#define ADJ_STEP_RAW (2 * 64)
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
        if ((i % RAW_H_RES) != 0) {
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
 * p2pro-rs and verified sane against indoor-scene readings on this hardware). */
static float raw_to_celsius(uint16_t raw)
{
    return raw / 64.0f - 273.15f;
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
static uint16_t centre_raw(const uint16_t *raw16)
{
    uint32_t sum = 0;
    const int cx = RAW_H_RES / 2, cy = RAW_V_RES / 2;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            sum += raw16[(cy + dy) * RAW_H_RES + (cx + dx)];
        }
    }
    return (uint16_t)(sum / 9);
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
static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)io; (void)edata; (void)user_ctx;
    if (s_expect_status_bar_completion) {
        s_expect_status_bar_completion = false;
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
static int ppa_quantized_out(int in_dim, int desired_out)
{
    int sixteenths = (int)(((float)desired_out / in_dim) * 16.0f); /* floor, as the driver casts */
    return (sixteenths / 16) * in_dim + ((sixteenths % 16) * in_dim) / 16;
}

static esp_err_t display_init(void)
{
    display_compute_logical_size(&s_orientation, &s_disp_w, &s_disp_h);
    /* Round width down to a multiple of 32 so each row is exactly 64 bytes (one cache line)
     * of RGB565 - PPA's output buffer size must be cache-line aligned, and since each row is
     * then already a whole number of cache lines, any height works without further rounding. */
    s_img_w = ppa_quantized_out(RAW_H_RES, ((s_disp_w - SIDE_PANEL_W) / 32) * 32);
    s_img_h = ppa_quantized_out(RAW_V_RES, s_disp_h); /* largest PPA-exact height that fits */
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
    s_img_off_y = 0; /* flush top, status bar occupies the bottom s_bar_h rows */

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
        int pal_y = s_bar_y + slot_btn_y(SLOT_PAL);
        int scl_y = s_bar_y + slot_btn_y(SLOT_SCL);
        bool in_ffc = in_bar && s_osd_enabled &&
                      x >= (uint16_t)bx && x < (uint16_t)(bx + BTN_W) &&
                      y >= (uint16_t)ffc_y && y < (uint16_t)(ffc_y + BTN_H);
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
                    ESP_LOGI(TAG, "FFC cleared");
                } else {
                    s_ffc_req = true;
                    ESP_LOGI(TAG, "FFC capture requested - aim at a uniform surface");
                }
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

/* ---------------- UVC callbacks ---------------- */

typedef struct {
    uvc_host_frame_t *frame;
    int64_t arrival_us;
} queued_frame_t;

static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    queued_frame_t qf = {
        .frame = (uvc_host_frame_t *)frame,
        .arrival_us = esp_timer_get_time(),
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
        ESP_LOGW(TAG, "P2 Pro disconnected");
        s_camera_connected = false;
        s_ffc_active = false; /* the sensor's pattern won't be the same after it re-powers */
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

static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
        }
    }
}

/* ---------------- Render task: raw -> palette -> SPI blit ---------------- */

static void render_task(void *arg)
{
    static int frames_converted = 0;
    static int64_t convert_time_sum_us = 0;
    bool disconnect_shown = false;
    bool no_temp_shown = false;
    int no_frame_ticks = 0;
    uint16_t last_min_v = 0, last_max_v = 0;

    queued_frame_t qf;
    while (true) {
        /* Timeout rather than block forever: with no camera there are no frames, and this is
         * the only task allowed to draw, so without waking up periodically the "no camera"
         * state could never reach the screen. */
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
                esp_lcd_panel_draw_bitmap(s_panel, s_img_off_x, s_img_off_y,
                                           s_img_off_x + s_img_w, s_img_off_y + s_fb_h, s_rgb_tx);
                disconnect_shown = true;
            }
            continue;
        }
        no_frame_ticks = 0;
        if (disconnect_shown) {
            disconnect_shown = false;
            s_osd_dirty = true; /* bring the temperature readout back */
        }

        /* Wait until the PREVIOUS SPI transfer out of s_rgb_buf has actually finished before
         * we start overwriting it with this frame's pixels - this is the fix for the tearing:
         * draw_bitmap() is async, so without this we could rewrite the buffer mid-transfer. */
        xSemaphoreTake(s_trans_done_sem, portMAX_DELAY);

        int64_t convert_start_us = esp_timer_get_time();

        /* The 256x384 format is preview on top + radiometric below. If the camera has been put
         * into image-only mode (256x192, 98304 bytes) the second half simply isn't there, and
         * reading it walks off the end of the frame buffer into whatever PSRAM follows - which
         * renders as garbage rather than failing loudly. Seen for real after the camera was
         * reconfigured by the vendor phone app. */
        const size_t needed = (size_t)RAW_H_RES * RAW_V_RES * 4;
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
                esp_lcd_panel_draw_bitmap(s_panel, s_img_off_x, s_img_off_y,
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

        const uint8_t *raw_half = qf.frame->data + (RAW_H_RES * RAW_V_RES * 2); /* skip preview half */
        const uint16_t *raw16 = (const uint16_t *)raw_half;
        size_t npix = RAW_H_RES * RAW_V_RES;
        p2_frame_diag(raw16, npix, qf.frame->data_len, s_stream_open_us);
        p2_log_first_clean(raw16, npix, s_stream_open_us);

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
                for (size_t i = 0; i < npix; i++) {
                    s_ffc_offset[i] = (int16_t)((int32_t)(s_ffc_accum[i] / FFC_FRAMES) - mean);
                }
                s_ffc_active = true;
                ESP_LOGI(TAG, "FFC captured over %d frames (mean %d)", FFC_FRAMES, (int)mean);
            }
            s_osd_dirty = true;
        }
        if (s_ffc_active) {
            for (size_t i = 0; i < npix; i++) {
                int32_t v = (int32_t)raw16[i] - s_ffc_offset[i];
                if (v < 0) v = 0;
                if (v > 65535) v = 65535;
                s_ffc_corrected[i] = (uint16_t)v;
            }
            raw16 = s_ffc_corrected; /* everything downstream uses corrected data */
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
            int32_t nmin = (int32_t)s_manual_min + dmin * ADJ_STEP_RAW;
            int32_t nmax = (int32_t)s_manual_max + dmax * ADJ_STEP_RAW;
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
                uint16_t v = raw16[i];
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
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
            uint16_t v = raw16[i];
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

        /* Upscale from the native RAW_H_RES x RAW_V_RES image to s_img_w x s_img_h (a slight
         * inset from the full panel, see IMG_COVERAGE) using the PPA hardware scaler, instead
         * of a CPU gather loop - a software nearest-neighbor version of this (indexed PSRAM
         * reads) cost ~16ms of CPU time and dragged the whole pipeline behind real time. */
        const ppa_srm_oper_config_t srm_config = {
            .in = {
                .buffer = s_small_rgb,
                .pic_w = RAW_H_RES, .pic_h = RAW_V_RES,
                .block_w = RAW_H_RES, .block_h = RAW_V_RES,
                .block_offset_x = 0, .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .out = {
                .buffer = s_rgb_buf,
                .buffer_size = (size_t)s_img_w * s_fb_h * sizeof(uint16_t),
                .pic_w = s_img_w, .pic_h = s_fb_h,
                .block_offset_x = 0, .block_offset_y = 0,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0, /* rotation is handled by the panel's own swap_xy/mirror */
            .scale_x = (float)s_img_w / RAW_H_RES,
            .scale_y = (float)s_img_h / RAW_V_RES,
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
                .buffer_size = (size_t)s_img_w * s_fb_h * sizeof(uint16_t),
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
        esp_lcd_panel_draw_bitmap(s_panel, s_img_off_x, s_img_off_y,
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

/* ---------------- UVC stream open ---------------- */

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
            .h_res = STREAM_H_RES,
            .v_res = STREAM_V_RES,
            .fps = STREAM_FPS,
            .format = UVC_VS_FORMAT_YUY2,
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
        BOOT_MARK("uvc: calling stream_open");
        ESP_LOGI(TAG, "Opening %dx%d YUY2 @%.1ffps...", STREAM_H_RES, STREAM_V_RES, STREAM_FPS);
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
        p2_camera_init();
        s_stream_restart_req = false;
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
        }
        free(list);
    }
    static bool streaming_task_started = false; /* the task now loops forever and handles
                                                 * reconnects itself - only ever spawn one */
    if (!streaming_task_started) {
        streaming_task_started = true;
        xTaskCreatePinnedToCore(streaming_task, "streaming", 6144, NULL, USB_HOST_PRIORITY - 2, NULL, tskNO_AFFINITY);
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

    BOOT_MARK("usb: installing host");
    ESP_LOGI(TAG, "Installing USB Host (native HS OTG port)");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, USB_HOST_PRIORITY, NULL, 0);

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
    s_small_rgb = heap_caps_aligned_alloc(64, RAW_H_RES * RAW_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    assert(s_small_rgb);

    s_rgb_buf = heap_caps_aligned_alloc(64, (size_t)s_img_w * s_fb_h * sizeof(uint16_t),
                                         MALLOC_CAP_SPIRAM);
    s_rgb_tx = heap_caps_aligned_alloc(64, (size_t)s_img_w * s_fb_h * sizeof(uint16_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (!s_rgb_tx) {
        s_rgb_tx = heap_caps_aligned_alloc(64, (size_t)s_img_w * s_fb_h * sizeof(uint16_t), MALLOC_CAP_DMA);
    }
    assert(s_rgb_buf && s_rgb_tx);

    s_ffc_offset    = heap_caps_malloc(RAW_H_RES * RAW_V_RES * sizeof(int16_t),  MALLOC_CAP_SPIRAM);
    s_ffc_corrected = heap_caps_malloc(RAW_H_RES * RAW_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    s_ffc_accum     = heap_caps_malloc(RAW_H_RES * RAW_V_RES * sizeof(uint32_t), MALLOC_CAP_SPIRAM);
    assert(s_ffc_offset && s_ffc_corrected && s_ffc_accum);

    const ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 2, /* scale pass + byte-swap pass */
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_client_config, &s_ppa_client));
    BOOT_MARK("buffers + PPA client ready");

    s_status_buf = heap_caps_malloc((size_t)s_bar_w * s_bar_h * sizeof(uint16_t),
                                     MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (!s_status_buf) {
        s_status_buf = heap_caps_malloc((size_t)s_bar_w * s_bar_h * sizeof(uint16_t), MALLOC_CAP_DMA);
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
    BOOT_MARK("app_main done, releasing streaming task");
    xSemaphoreGive(s_buffers_ready);
}
