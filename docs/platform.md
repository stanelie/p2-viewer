# Board, platform and display notes

The Waveshare ESP32-P4-WIFI6-Touch-LCD-3.5 itself: boot time, flash layout, the power button,
the display bug that bit every camera path, and the limits that are not going away.

## Boot time

Measured breakdown to first image, 3049 ms originally, **1709 ms** now:

| phase | cost | notes |
|---|---|---|
| ROM + bootloader + PSRAM + system init | ~410 ms | `CONFIG_SPIRAM_MEMTEST=n` removed ~890 ms of this |
| USB host install | 40 ms | moved to the *front* of `app_main` |
| display init | 310 ms | now runs **inside** the enumeration wait, so it is free |
| camera USB enumeration | ~550 ms | the camera's own; overlapped with display init |
| `uvc_host_stream_start` | ~650 ms | the camera processing SET_INTERFACE; not reducible from the host |

The structural win was ordering: USB enumeration takes ~550 ms and proceeds on its own task, so
installing the host first and doing display init and buffer allocation inside that window hides
~310 ms. A semaphore gates the streaming task so it cannot touch buffers before they exist.

Things tried that did **not** help: `CONFIG_ESPTOOLPY_FLASHMODE_QIO` (the esp32p4 target forces
`dio` regardless), silencing the bootloader log (~3 ms once it was already at WARN), and
`CONFIG_ESP_CONSOLE_UART_BAUDRATE` (only settable with `ESP_CONSOLE_UART_CUSTOM`, otherwise pinned
at 115200).

Set `P2_DIAG 1` in `main/p2pro_uvc.c` to re-enable the investigation instrumentation (auto-shutter
dump, per-frame content diagnostic, UVC component DEBUG logs). The one-line
`camera calibrated: clean image N ms after stream open` report is always on.

## Flash layout (16MB)

The board carries 16MB but the image header originally declared 2MB, so the bootloader logged
`Detected size(16384k) larger than the size in the binary image header(2048k)` and ignored the
rest. With `CONFIG_ESPTOOLPY_FLASHSIZE_16MB` the whole chip is addressable.

| partition | type | offset | size | |
|---|---|---|---|---|
| `nvs` | data/nvs | 0x9000 | 24K | |
| `phy_init` | data/phy | 0xf000 | 4K | |
| `factory` | app | 0x10000 | **3M** | app is ~1.08MB, 35% used |
| `ffc` | data/0x40 | 0x310000 | 64K | flat-field table, 48KB + header |
| `storage` | data/fat | 0x320000 | 12.875M | declared, not mounted |

The app partition **grew from 1MB to 3MB in v1.3.0** to fit the WiFi stack - `esp_wifi_remote`
plus `esp_hosted` plus lwIP do not come close to fitting in what was left of 1MB. That shifts
`ffc` and `storage`, so **a flat-field table stored by an earlier release is discarded once on
update** and has to be recaptured. Nothing else depends on those offsets.

The FFC table gets a raw partition rather than NVS because 256*192 int8 is 48KB plus a 16-byte
header, and NVS at 24KB cannot hold it; a raw partition also avoids blob overhead and can be
written in one erase+write.

`storage` is declared but **not mounted** - nothing consumes it yet, and mounting a filesystem
at boot would cost time against a ~1.7s startup. Mount it on demand when something needs it.

## The LCD blit was doing a 307 KB internal malloc on every frame

Bringing WiFi up exposed a latent bug in **every** display path, including the thermal one.

`esp_lcd` never sets `SPI_TRANS_DMA_USE_PSRAM`. The SPI driver therefore treats a PSRAM source
buffer as needing a bounce (`spi_master.c`: `need_malloc` is true for an external pointer without
that flag, allocated `MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL`), so a full-screen 480x320 blit asked for
**307,200 bytes of contiguous internal RAM every frame**. That worked purely because internal RAM
was near-empty. esp-hosted's SDIO init fragments it to a **32 KB** largest block, after which every
blit returned `ESP_ERR_NO_MEM`.

And a failed draw produces **no completion callback**, so render_task blocked on the semaphore
forever - diagnosed with a stage trace plus a watcher task:

```
render STUCK in "builtin blit wait" (laps=63, unchanged 2s)
builtin: draw_bitmap failed: ESP_ERR_NO_MEM | INTERNAL free 181983 largest 32768
```

Fixes: the draw return value is checked and the semaphore handed back on failure (no more hang),
and `blit_strips()` splits a frame into `BLIT_STRIP_BYTES` strips so the bounce is bounded
regardless of panel size. Measured:

| regime | fps | e2e |
|---|---|---|
| single full-frame draw | **30-32** | 23.7 ms |
| 16 strips | 20.7 | 49.5 ms |
| retry the full draw every frame, then strip | 16 | 64 ms |

So stripping is the **fallback, not the default**, and the fallback is **sticky** - retrying the
307 KB allocation every frame cost more than it saved. It re-probes every ~600 frames in case the
heap recovers.

`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` was tried and changed nothing: the fragmentation comes
from esp-hosted's SDIO init, not from WiFi/lwIP buffers. The failure is contiguity, not shortage.

**WiFi init is deferred** until no USB camera has been present for ~5 s, so with the P2 attached
the radio never starts, internal RAM is never fragmented, and the instrument keeps its full
framerate.


## The display bus runs at 80 MHz, not 120

`LCD_PIXEL_CLOCK_HZ` is 80 MHz, which is Waveshare's validated default. **120 MHz reproduced boot
hangs** and was reverted. This matters because the SPI blit is the bottleneck in every camera
path on this board - 21 ms of wire time for a 176 KB frame, 14.5 ms for a full-screen 307 KB one -
so the clock is the first thing that looks like free performance. It is not available.

## Power button

Holding the power button for **1 s** powers the device off. The AXP2101's own `PWROFF` long-press
(REG `0x27` bits 3:2) only offers 4/6/8/10 s and shipped set to 6 s, so 2 s is not reachable by
configuration. Instead the chip's long-press *interrupt* (bits 5:4, options 1/1.5/2/2.5 s) is set to
1 s and the firmware commands the shutdown itself via `0x10` bit 0. The IRQLEVEL mapping was
confirmed from two measured points: `01` produced a 1.48 s gap between the press and long-press
interrupts, and `10` produced a verified 2 s power-off, so `00` is 1 s.

`OFFLEVEL` is also dropped to its 4 s minimum. That is deliberately longer than the 1 s software
path, so it acts as a backstop: if the firmware wedges, holding the button still cuts power in
hardware.

Observed on hardware, status register `0x49`: **bit 1 = press, bit 2 = long press at IRQLEVEL,
bit 0 = release**. Those bits latch even with the interrupt masked in `0x41`, so polling is
sufficient and the interrupt-enable registers are left untouched. The task requires a *fresh press*
before honouring a long press - otherwise holding the button to switch the device **on** leaves a
long-press bit set and it would shut straight back down.

The PMIC shares the touch I2C bus (port 1, SDA 7, SCL 8). A scan of that bus finds `0x18`,
`0x34` (AXP2101), `0x36` and `0x38` (FT5x06 touch).


## Known limitations

- **Screen tearing is not fixable on this board.** Confirmed from the schematic: there is no TE net
  (so no vsync sync) and only MOSI (so no quad-mode speedup). 258 KB cannot be pushed below the
  panel's ~16.7 ms refresh, so scan-out always crosses the write. Host-side double buffering does
  not help — the tear forms in the panel's own GRAM. The tear-free path would be the board's
  unpopulated MIPI-DSI pads.
- **The camera cannot be powered from the internal battery unmodified.** Both Type-C VBUS pins share
  one net feeding the AXP2101's charger *input*; the PMIC is an NVDC charger with no boost at all.
  Powering the camera on battery needs an added 3.7→5 V boost module.
- The image is ~8% vertically stretched at 416x312 (PPA scale factors quantize to 1/16 steps).

