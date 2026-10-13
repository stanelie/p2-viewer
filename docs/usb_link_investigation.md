# The borescope USB link investigation

A long hunt for why one borescope would not work on the OTG port. It ended at a connector rather
than at the SoC, and the resolution is the first section below. Everything after it is the trail,
kept so none of it gets retried, and because some of the measurements along the way (the P4's
undocumented UTMI PHY trims, a squelch sweep, an enumerator stage trace) are worth having on
their own.

The one lesson worth carrying forward: when a fault is physical, get an exact description of the
cable chain before theorising about the controller.

## RESOLVED: it was the camera's micro-USB plug half and its micro-USB -> USB-C adapter

**The borescope works directly on the ESP32. No hub, no hub IC, no firmware change.**

This camera has an unusual dual plug: a USB-A shell whose cover flips aside to reveal a micro-USB
connector. Every "direct" test in this investigation went through the **micro-USB** half plus a
micro-USB -> USB-C adapter. Every working hub test used the **USB-A** half. The variable was never
"direct vs through a hub" at all:

```
failing:  camera -[1 m cable]- flip-cover plug as MICRO-USB -[micro->C adapter]- ESP32
working:  camera -[1 m cable]- flip-cover plug as USB-A     -[A->C adapter]-     ESP32
```

Plugging the USB-A half into the same A->C adapter already used for the hub gives video immediately.
The fault is in the micro-USB path - the flip-cover's micro-USB wiring, its connector, or that
cheap micro->C adapter. It is not the P4's PHY, not the hub's presence, and not anything in software.

**Why that path works on a phone but not here.** It is a marginal link, not a broken one, and the
two hosts have different analogue margin. Evidence from our own logs: on the failing path the P4
completed *every* control transfer - device descriptor, full config descriptor, UVC probe/commit,
`SET_INTERFACE` - repeatedly and correctly. It only failed once continuous isochronous traffic
started. A link that carries sparse traffic perfectly and dies under sustained load is marginal by
definition. A phone's USB PHY is discrete silicon with impedance-matched routing, common-mode
chokes and proper ESD parts; the P4's is integrated, on a dev board, at the end of an adapter
stack. "Works on the phone" means clean *enough* for a better receiver, not clean.

Most likely mechanism, though unmeasured (no scope on this bus): USB-C carries two D+/D- pin pairs
(A6/A7 and B6/B7), and cheap legacy->C adapters sometimes bridge the data lines to *both* so they
work in either orientation without CC logic. The unused pair then hangs off the signal as a
**stub**, and stub reflections add spurious edges at packet end - which is precisely what gets
flagged as babble, and precisely why the failure is traffic-dependent.

**Not isolated: the adapter vs the flip-cover contacts.** The micro-USB path contains both a
dual-plug switching mechanism and a cheap adapter, and only the pair was tested. If it ever
matters, trying the micro-USB half through a *different* micro->C adapter separates them: still
failing means the camera's flip-cover contacts, working means that adapter.

**What the instrumentation got right.** The port-error diagnosis held up: the core really was
clearing `prtena` while `prtconnsts` stayed 1, with `DISCONNINT` never firing - a port error, not a
disconnect. And the decisive measurement pointed the right way: `MPS=1020 EC=3 EP=1` on the working
AND failing paths, 5/5 each, proving the host programmed the controller identically and the
difference had to be physical. It was physical. I just had the wrong physical component, because I
did not know the micro-USB adapter was in the chain.

**What it cost.** Six host-side hypotheses built and measured away (listed under "Hypotheses killed by measurement" below), plus a long detour into
hub behaviour. The lesson is cheap and worth keeping: **when a fault is physical, get an exact
description of the cable chain before theorising about the controller.** "Connected directly" was
hiding two connectors and an adapter.


## That camera does not enumerate on this board

Measured, and it appears to be physical rather than a software problem:

- It enumerates and streams fine on a Linux host.
- The **P2 Pro works reliably on the same port with the same firmware**, so the port and host stack
  are fine.
- It is plugged **directly** into the OTG connector, no adapter.
- Failures land at varying enumeration stages - `CHECK_SHORT_DEV_DESC`, `CHECK_SHORT_LANGID_TABLE`,
  `CHECK_FULL_CONFIG_DESC` - with `Dev N EP 0 Error` and ~120 `HUB: Root port reset failed` per
  minute, and the device address incrementing as it re-enumerates. A size or parsing fault would
  fail at the *same* stage every time; `CHECK_SHORT_DEV_DESC` is an 8-byte control read.
- `stream_open` then fails `ESP_ERR_INVALID_STATE`, which in `usbh_ep_alloc` means
  `dev_obj->constant.config_desc` is NULL - i.e. it is racing a device instance that already died,
  not a fault in the streaming code.
- It is **not** power: `bMaxPower` requests 500 mA, but the camera runs cooler than the P2, its
  illumination LED stays steady and it never visibly drops out.

What it actually does, measured with a second diagnostic USB host client (`P2_USB_PROBE`):

```
probe: addr 12 speed=HIGH (480 Mbit/s) bMaxPacketSize0=64 cfg=1
Device connected, addr=12
probe: device gone                                  <- ~10ms later
```

It negotiates **high speed** and enumerates **completely** - device descriptor and configuration
both readable - and then detaches within ~10 ms. The address climbs on every cycle. On other
rounds it dies earlier, at `Dev 0 EP 0 Error` (address 0, i.e. the first 8-byte descriptor read).

Eight hypotheses eliminated by measurement, so none of them get retried:

| ruled out | how |
|---|---|
| power / brownout | 150 mA measured on an inline meter; LED steady, runs cooler than the P2 |
| full-speed fallback hitting the known P4 FS enumeration bug | probe reports **HIGH** every time |
| config descriptor too large | `wTotalLength` 702 bytes vs our 4096 limit |
| IDF 5.5.5 NULL `enum_filter_cb` regression | the block is inside `#if ENABLE_ENUM_FILTER_CALLBACK`, and that option is unset here |
| our `usb_host_device_free_all()` on `NO_CLIENTS` | instrumented - it never fires during the loop |
| our `stream_open` destabilising it | with `P2_NO_OPEN_GENERIC` it still cycled 13 times in 40 s |
| raising the USB enumeration timings | `DEBOUNCE 500 / HOLD 50 / RECOVERY 200 / SET_ADDR 50` made it **worse** - the root port reset then failed outright and enumeration never started |
| a powered hub regenerating the signal | tested through a powered dock: no change |

What remains is an electrical / PHY-level interaction specific to this device on this port: EP0
control transfers error at random stages and the device detaches, while the **P2 Pro works on the
same port** and this camera works on a PC. The untested variables are the OTG connector's modified
wiring, the scope's ~1 m of thin cable, and the P4 OTG PHY's tolerance of the two combined.

**Answered by testing an iMac webcam on the same port: it enumerates once, cleanly, with zero HUB
errors and streams fine.** So the port, the wiring and the host stack are all sound, and this
scope is specifically the device that does not work.

## What it actually does (enum stage trace)

With `P2_USB_ENUM_TRACE` the enumerator names every stage, and the answer is not what the error
messages suggest:

```
ENUM: GET_SHORT_DEV_DESC OK
ENUM: CHECK_SHORT_DEV_DESC OK
ENUM: SECOND_RESET OK
ENUM: SECOND_RESET_COMPLETE OK
ENUM: SET_ADDR OK          (dev_addr=1)
ENUM: CHECK_ADDR OK
ENUM: SET_ADDR_RECOVERY OK
ENUM: GET_FULL_DEV_DESC OK
HUB:  Device tree node (port 0, uid=1): device gone     <-- a DISCONNECT
ENUM: CANCEL OK
```

**Every stage passes, and then the hub reports a disconnect** - 18 cycles in 22 s. So the
`CHECK_SHORT_DEV_DESC FAILED` / `CHECK_FULL_CONFIG_DESC FAILED` messages are collateral: the device
vanishes mid-sequence and whichever transfer is in flight errors. The enumerator is working
correctly, and `ENUM_STAGE_SECOND_RESET` (the "old devices get confused" workaround) passes fine,
so that is not it either.

A host seeing a device vanish when the device is demonstrably fine points at **false high-speed
disconnect detection**: HS disconnect is sensed electrically off the differential pair, and a
marginal link makes the host conclude the device unplugged. That fits the asymmetry - the webcam's
short lead works, the scope's ~1 m of thin cable does not, and a PC's PHY tolerates it.

**No USB timing knob helps.** Tested one at a time (the earlier four-at-once test was bad
experimental design and its conclusion was worthless), 25 s windows:

| config | HUB errors | ENUM failures | enumerations | stream opened |
|---|---|---|---|---|
| baseline | 51 | 3 | 6 | no |
| `RESET_RECOVERY_MS=100` | 59 | 1 | 0 | no |
| `SET_ADDR_RECOVERY_MS=50` | 46 | 3 | 3 | no |
| `DEBOUNCE_DELAY_MS=500` | 27 | 3 | 3 | no |

**A powered USB-C dock does not help either, but inconclusively:** with the dock between board and
camera the board sees *nothing at all* - no `HUB: Root port reset`, no device tree node, silence
after boot. Enumeration never starts, so this is at connection-detect level, not a descriptor or
protocol problem.

Why the dock does not attach is **unknown**, and two plausible-sounding explanations are already
ruled out:

- Not the VBUS *source*. A hub sees 5 V on the pin and does not care whether an external boost or
  the host cable put it there.
- Not VBUS *collapsing* under the dock's own load. The dock is powered from its PD input by a
  USB-C supply, so its hub controller does not draw from the board at all.

What is established is only that enumeration never starts, so it is at connection-detect level
rather than anything protocol-shaped. No mechanism beyond that is worth writing down until
something is measured.

Either way this cannot say whether a hub would fix the signalling - a **plain bus-powered USB 2.0
hub**, which needs no CC negotiation and no PD, is the cheaper way to get a signal repeater into
the path.

## The workaround: a plain USB 2.0 hub

**Confirmed working.** A plain bus-powered USB 2.0 hub between the board and the borescope gives a
clean, stable image at the full 25 fps. A hub terminates and re-drives the camera's link on its own
downstream port, so the marginal cable stops being the ESP's problem - exactly what false
high-speed disconnect detection predicts. This is the recommended fix: no firmware change, no
framerate cost.

**Full speed is NOT available as an alternative on this board.** Speed is settled by the chirp
handshake during bus reset so it cannot be forced from the host for a given port, but the P4 has
two OTG controllers - `USB_DWC_LL_GET_HW(num)` gives `USB_DWC_FS` for `num == 1` - so
`usb_host_config_t.peripheral_map = BIT1` installs the host on the full-speed one, where 12 Mbit/s
signalling would sidestep HS disconnect detection entirely.

Tested (`P2_FORCE_FULL_SPEED`): `usb_host_install()` succeeds with no error, and then **nothing
enumerates at all** - not one root port reset in 28 s. The FS controller's pins are not routed to
this board's OTG connector. Do not retry on this hardware.

It would have cost framerate anyway: full speed gives ~1.1 MB/s realistically, so 320x240 YUY2
manages ~7 fps against 25 through the hub. Recovering 25 fps at full speed would mean MJPEG plus
the P4's hardware JPEG decoder.

## Why it works on a phone but not here

High-speed disconnect detection is **purely electrical and has no handshake**: the host samples the
differential amplitude during the EOP of each SOF, and an unloaded bus roughly doubles it past
~625 mV. On a long thin cable, attenuation and impedance discontinuities mean reflections in that
EOP window can cross the threshold while the device is still present and perfectly healthy - which
is exactly the observed "every stage OK, then device gone".

Phone and PC SoCs tolerate the same cable because their PHYs have trimmed terminations and
*adjustable* disconnect thresholds and squelch, plus impedance-controlled routing to the connector.

**The P4 DOES expose analogue PHY trims**, and no IDF change is needed to reach them. They are not
in the DWC core register map (which is where I first looked, and wrongly concluded there was
nothing) but in a separate UTMI PHY block - `soc/usb_utmi_struct.h` under
`components/soc/esp32p4/register/hw_ver3/`, already on the include path because soc's CMakeLists
appends the `hw_ver` directory for the chip revision. `usb_phy.c` never writes them (its
`utmi_hal_context` is "unused for now"), so they sit at hardware defaults and can be set straight
from application code via `USB_UTMI`:

| field | default | options | relevance |
|---|---|---|---|
| `adj_res_hs` | `0x4` = 45 Ω | `0x0`=40 Ω, `0x6`=50 Ω | HS termination trim - changes the very amplitude disconnect is judged on |
| `adj_vref_sq` | `0x2` = 124 mV | `0x0`=92 mV, `0x3`=152 mV | squelch detection threshold |
| `adj_vsw_hs` | `0x4` = 400 mV | `0x0`=320 mV, `0x7`=460 mV | TX eye / output swing |
| `adj_pw_hs` | `0xF` = 400 mV | `0x1`=100 mV … | reduced-swing power saving |
| `adj_iref_res`, `adj_pll`, `adj_txclk_phase` | | | other analogue trims |

`P2_UTMI_TRIM` wires these up. **Write them between `usb_host_install()` and powering the root
port** - use `root_port_unpowered = true` and call `usb_host_lib_set_root_port_power(true)`
afterwards. That sequencing is verified harmless on its own (HUB 25-27, enum 4-6, same as
baseline), and it is also what makes the power call valid; it returns `ESP_ERR_INVALID_STATE`
("already powered") otherwise.

**The header's documented defaults are not all correct.** Measured on hardware:

```
fc_00 = 0x00000080  ->  adj_res_hs  = 0x4 (45 ohm)      as documented
fc_01 = 0x000000f8  ->  adj_vref_sq = 0x8               header says 0x2 - IT IS 0x8
                        adj_pw_hs   = 0xF (400 mV)      as documented
fc_02 = 0x00000047  ->  adj_iref_res = 0x7, adj_vsw_hs = 0x4 (400 mV)
```

This matters: writing `adj_vref_sq = 0x2` because the header calls it the default actually *cuts*
the squelch threshold hard and kills enumeration - 42-43 HUB errors and 0 enumerations, against
20-31 and 2-10 at the real `0x8`. An earlier claim here that "the write itself breaks enumeration"
was wrong; it was the value. Writing a register with its true value is harmless (verified
per-register).

Squelch sweep result: `0x9`, `0xA`, `0xC`, `0xF` all make the device **invisible** - no hub
activity at all - so `0x8` already sits at the edge of detection with no headroom upward. **One
run at `0xA` did produce `enum=3` and `streamopen=3`, the only stream opens ever observed on this
camera**, but it was not reproducible and the camera went electrically absent shortly afterwards.
Unconfirmed, and worth retrying on a known-good rig.

## Squelch 0xA is a real improvement, but not a fix

With the camera freshly power-cycled and `adj_vref_sq = 0xA` as the **only** change:

| | default `0x8` | `0xA` |
|---|---|---|
| HUB errors | 20-43 | **2-7** |
| enumerations | 3-10 | 11 |
| **stream opens** | **0, always** | **7-11** |
| frames | 0 | 0 |

Stream opens had *never* happened on this camera before. So raising the squelch threshold genuinely
fixes the link-level instability - the device stops being dropped mid-enumeration. The failure then
moves downstream: the stream opens and isochronous data never completes a frame. Disabling the
remaining payload check (`CONFIG_UVC_CHECK_PAYLOAD_HEADER_ERR=n`) does not change that.

`0xA` is **not** the default, because it is untested against the P2 Pro and the webcam: higher
squelch makes a device harder to see (`0x9` and above made the borescope invisible in other runs),
so it could plausibly break the cameras that currently work. Test those before adopting it.

**Measurement cost warning.** The borescope *wedges* after repeated failed attempts - it keeps its
LED lit and keeps drawing current while presenting nothing on the bus - so every data point needs a
physical replug, and run-to-run variance is large (identical firmware gives 2-10 enumerations, and
one run gave `ESP_ERR_NOT_FOUND` and no stream at all). That variance eventually exceeded the
effect sizes being measured, which is where this investigation stopped.

## A bandwidth table in an earlier version of these notes was wrong

The isoc figures I first derived for this camera were too low by 3x. `lsusb -v` prints
`wMaxPacketSize 0x03fc  3x 1020 bytes`, and my parser took the byte count while discarding the
`3x` - the transactions-per-microframe multiplier for high-bandwidth isochronous. The component
gets it right: `max_packet_size *= (USB_EP_DESC_GET_MULT(ep_desc) + 1)`, giving 3060 bytes per
microframe, i.e. **~24.5 MB/s, not 8.16 MB/s**. So 640x480 YUY2 at 25 fps (15.4 MB/s) was always
within budget, and "320x240 is the only mode that fits" was a parsing bug, not a finding.


## Hypotheses killed by measurement

All mine, all discarded on evidence. Recorded so none is tried again:

| hypothesis | how it died |
|---|---|
| packets per frame (lossy link) | 8/8 turn-around values, 162 attempts: `ok=0 truncated=0 decode_err=0`. Zero *truncated* means no partial data ever arrives |
| burst size / high-bandwidth isoc | forcing single-transaction 944 MPS failed identically to 3060 |
| HS bus turn-around time | swept `fc_06.hs_tx2rx_dly_cnt_sel` 0-7 in firmware, one value per stream attempt; all zero |
| squelch threshold `adj_vref_sq` | reduced a symptom count, not the cause; reverted to the 0x8 hardware default |
| payload / endpoint mismatch | clamped committed `dwMaxPayloadTransferSize` to 944; camera **re-raised it to 3060 on 64 of 64 attempts** |
| ESP-IDF cannot do high-bandwidth isoc (`MULTCNT` unwritten) | the working path reads **EC=3**, correctly programmed. My grep searched the databook name `multcnt`; this struct calls the field `ec` |

Also abandoned unmeasured, once the real cause was found: `HFIR.frint` is hardcoded to 6000 for high
speed in `usb_dwc_ll_hfir_set_frame_interval()`, implying a 48 MHz PHY clock, while the same
function sets `phyif = 1` (16-bit UTMI, normally 30 MHz). That file is copy-pasted across
esp32s2/s3/p4 and S2/S3 are full-speed-only parts. Possibly an IDF oddity, possibly correct for P4
silicon; `P2_UFRAME_CHECK` will count `HFNUM.frnum` against `esp_timer` (8000/s is correct) if it
ever matters. It is not our bug.

## Diagnostics kept, all default off

| flag | what it does |
|---|---|
| `P2_USB_OSD` | draws `FMT/MPS/EC/EP` and ok/truncated/port-disable counters on the panel, read straight from HCCHAR. The only way to instrument a configuration the serial adapter cannot physically coexist with |
| `P2_PORT_TRACE` | ring-buffers HPRT/GINTSTS at ~100 us and dumps the transitions on a port event |
| `P2_UFRAME_CHECK` | measures the real microframe period from `HFNUM.frnum` |

## Historical note: the isoc-no-retries reasoning


With squelch `0xA` the link-level problem is solved - enumeration, format negotiation and
`stream_open` all succeed. Frames still never complete, and two separate attempts to reduce the
isochronous demand changed nothing:

| attempt | result |
|---|---|
| clamp the host-proposed `dwMaxPayloadTransferSize` to 1020 in `SET_CUR(PROBE)` | transmitted (`clamping 3060 -> 1020`) but the camera ignores it and still reports 3060 |
| `MAX_MPS_IN` 4096 -> 1020, which is what actually picks the alt setting | worked - endpoint went from **3060 MPS** (3 transactions/microframe) to **944 MPS** (single) - and frames were still 0 |

944 x 8000 = 7.55 MB/s against the 3.84 MB/s that 320x240 YUY2 at 25 fps needs, so **bandwidth was
never the constraint**. The real asymmetry is that **control transfers are retried and isochronous
transfers are not**. That is exactly why enumeration and probe/commit succeed on this link while
video does not: a frame needs ~160 consecutive packets to arrive intact, and on a link with this
error rate none ever does.

No host-side setting can make an unretried transfer survive a lossy link. Both component patches
were reverted; `managed_components` is back to stock.

## For a portable build: a hub IC inside the enclosure

If carrying a hub is the objection rather than the hub itself, the answer is to stop carrying one -
a single USB 2.0 hub IC (FE1.1s and similar are a few mm square and cost very little) soldered
inside the case, between the OTG connector and the camera socket, is permanent and invisible. It
puts a properly terminated PHY at the camera's end of the cable, which is the thing that actually
fixes this, and it needs no firmware change at all.

The alternative is a borescope with a shorter or better-shielded cable; this camera's ~1 m of thin
cable is the root cause, and it is not detachable.

## Validity rule for any future sweep

A run with **`HUB == 0` and `enum == 0` means nothing was attached** - discard it, do not score it
as "this setting failed". Several of my sweep results were exactly that, and I initially scored
them as negative results for the trims. Check for device presence before attributing an outcome to
a setting. Use *"does a stream open"* as the metric; enumeration counts are noisy (identical runs
give 2-10).

Masking the disconnect interrupt does **not** work (5 enumerations, 0 stream opens, unchanged):
`disconnint` is the OTG-level disconnect, whereas the host port state machine detects removal via
`HPRT.prtconndet` and the port interrupt, and masking *that* would mask connect events too.

A hub remains the reliable answer - it is what the USB specification provides repeaters for, and it
re-drives the marginal segment with a properly terminated PHY at the camera's end.

Anything further needs a scope on D+/D-, not more software.

