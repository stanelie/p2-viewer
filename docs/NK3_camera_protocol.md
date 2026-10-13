# NK3 / HND Wireless Otoscope — Camera Integration Spec

Handoff document. Everything needed to pull live video from this camera into an
application, with no vendor app and no cloud service.

**Status:** working. Verified end-to-end on real hardware 2026-10-11, 13.2 fps sustained.
Also running on an ESP32-P4 viewer (2026-10-12) at 13 fps with zero frames dropped at any
stage, which independently confirms ~13 fps is the camera's rate and not a host limitation.
Protocol cross-checked against the decompiled vendor app on 2026-10-12; see **[apk]** below.

Facts below are tagged:
- **[measured]** — observed on this physical unit.
- **[source]** — read from the reference implementation (haxko/NE3-Scope), not
  independently re-verified here.
- **[apk]** — read from the decompiled vendor app (`com.xiaozhen.beauty.hnd`,
  `T0/d.smali`). This is the authority on what the vendor actually sends; several
  **[source]** facts below turned out to be wrong and are corrected inline.

---

## 1. The device

A WiFi "earpick"/otoscope camera sold as **NK3**, paired with the Android/iOS app
**HND**. It is an access point, not a client: it creates its own WiFi network and
serves video over UDP to whoever joins.

Its own discovery response **[measured]**:

```json
{
  "model":       "EAR_CAMERA",
  "brand":       "HND-NE3",
  "soc":         "BK7231UQFN40",
  "hardware":    "1.0.1",
  "firmware":    "1.0.0",
  "imu_axis":    "3",
  "wifi_mode":   "AP",
  "bat_capacity":"250mAh",
  "designed_by": "C-Chip Technology Co., Ltd"
}
```

### CRITICAL: the brand string lies about the protocol

Two incompatible protocols exist in this device family, known upstream as **NE3** and
**NE7**. They differ in IP address, ports, packet framing — everything.

This unit reports `"brand": "HND-NE3"` but **speaks NE7** **[measured]**. Do not
select a protocol from the brand string. Select it from the **gateway address**:

| Gateway you are assigned | Protocol | Video transport |
|---|---|---|
| `192.168.169.1` | NE3 | UDP port 8800, 56-byte headers, ACK/REQ driven |
| `192.168.1.1`   | **NE7 — this unit** | UDP 44506, 4-byte headers, broadcast discovery |

**This document specifies NE7 only.**

---

## 2. Preconditions

1. **Join the camera's AP.** SSID looks like `HNDEC_55-XXXXXX` or `HNDEC-XXXXXX`
   (this unit: `HNDEC_55-00561`). Power on by holding the button ~2s; the blue LED
   blinks when ready and goes solid once a client is connected. No password.
2. **It accepts exactly one client at a time** **[measured/source]**. If the phone is
   on the AP, your app gets nothing. Disconnect the phone first.
3. **2.4 GHz, channel 1 only** **[source]**. Nearby APs on channels 1–2 will degrade
   the stream.
4. **The camera AP uses `192.168.1.0/24`** — the single most common home-router subnet.
   On a multi-homed machine this can collide. Bind your socket to the specific
   interface (`SO_BINDTODEVICE` on Linux, or bind to the local IP the camera's DHCP
   gave you) rather than relying on the routing table. On Android the equivalent is
   `Network.bindSocket()`, which also lets mobile data stay up.

---

## 3. Protocol

All traffic is **UDP**, unencrypted, no authentication. Camera is `192.168.1.1`.

### 3.1 Ports

| Port | Direction | Purpose |
|---|---|---|
| 36123 | local bind | Source port for everything. Camera replies here. |
| ~~58090~~ | — | **Not a real port.** The vendor app never sends to it **[apk]**, which is why it is silent. Drop it. |
| 46526 | → broadcast | Discovery probe. The only one the vendor uses **[apk]**, and the only one that answers **[measured]** |
| 44506 | → camera, then ← | Video channel |
| 52219 | → camera, then ← | IMU / telemetry channel |

Bind **one** local UDP socket to `0.0.0.0:36123` and use it for all of the above.
Replies are demultiplexed by the **source port** of each incoming datagram.

### 3.2 Handshake

**Step 1 — discovery.** Broadcast a 4-byte probe to `192.168.1.255`:

```
66 30 01 01   ->  192.168.1.255:46526      # what the vendor app sends [apk]
```

The vendor sends this **twice**, to 46526 only **[apk]**.

Note the third nibble: the vendor byte is **`0x30`**, not the `0x39` previously recorded
here. `66 39 01 01` also works **[measured]** — the camera is evidently lenient on that
byte — but `0x30` is what the real app transmits. Earlier revisions of this document had
`0x39` and sent to a second port, 58090, that the vendor never uses.

Requires `SO_BROADCAST`. The camera replies from `192.168.1.1:46526` with the
~504-byte JSON descriptor in §1 (tab-indented, `\n` newlines). Take the responder's
IP from the datagram source address rather than hardcoding it.

**Step 2 — start the streams.** Two magic payloads, to the camera's unicast IP:

```
86 06 01   ->  192.168.1.1:52219    # starts IMU
20 36      ->  192.168.1.1:44506    # starts video
```

Video datagrams begin arriving immediately.

**Step 3 — stopping.** `20 37` to 44506 stops the video stream **[apk]** — the vendor app
logs `"send stop rev img cmd."` immediately before sending it. Not required, but it is the
polite way to release the camera for the next client.

### 3.2.1 THE COMPLETE COMMAND SET — there are only four

The vendor app contains **no native library at all**; the entire protocol is Java in
`T0/d.smali`, so nothing is hidden. It sends exactly four payloads **[apk]**:

| payload | port | purpose |
|---|---|---|
| `66 30 01 01` | 46526 (broadcast, x2) | discovery |
| `86 06 01` | 52219 | start IMU |
| `20 36` | 44506 | start video |
| `20 37` | 44506 | stop video |

Searching that class for `resolution`, `width`, `height`, `fps`, `quality`, `bitrate` or
`mode` returns **nothing**. **There is no way to change the video mode** — not in this
protocol and not in the vendor app. 432x432 at ~13 fps is what the hardware does, and the
app cannot do better either. Budget accordingly and stop looking.

Other constants confirmed from the app **[apk]**: video receive buffer 1472 bytes, IMU
receive buffer 24 bytes, discovery socket timeout 2000 ms, stream socket timeout 3000 ms.

### 3.3 Video framing

Each datagram on port 44506 is one chunk of one JPEG frame.

```
offset  size  field
0       1     frame_number     wraps at 256; changes = new frame
1       1     flags            bit 0 set => this is the LAST chunk of the frame
2       1     chunk_number     1-based
3       1     flags2           purpose unknown
4       ...   payload          JPEG bytes
```

Reassembly:

1. Store payload at index `chunk_number - 1`.
2. **Do NOT discard any bytes.** Earlier revisions said to drop the last 5 bytes of the
   final chunk as "a trailer, not image data" **[source]**. That is **wrong on this unit**:
   those 5 bytes are real scan data **[measured]**. Stripping them truncates every frame,
   and a strict decoder then reports *"the number of data units obtained after decoding a
   frame of image is different from the number of data units calculated based on the image
   resolution"*.

   The trap is that roughly **30% of frames still decode** when truncated, because JPEG can
   complete if the shortfall lands in padding - so the symptom is a half-working stream
   rather than an obvious failure. If you are seeing ~70% decode failures, this is why.
3. If `flags & 1` **and** you hold `chunk_number` chunks, the frame is complete.
   Concatenate indices `0 .. chunk_number-1` in order.
4. When `frame_number` changes, drop any partial frame and start fresh. UDP means
   loss is normal; dropping an incomplete frame is correct.

Observed: 4–10 chunks per frame, ~4.5–14 KB per frame, **432×432 RGB baseline JPEG**
**[measured]**. (Note: the NE3 variant is 640×360. Do not hardcode either — read the
dimensions from the decoded JPEG.)

### 3.4 THE EOI TRAP — read this before debugging a decoder

**The camera omits the JPEG end-of-image marker.** Assembled frames begin with `FFD8`
and contain **no `FFD9` anywhere** — searching for one returns "not found"
**[measured]**.

```
jpeg = concat(chunks) + b"\xFF\xD9"      # you MUST append this
```

Two ways this wastes your time:

- Upstream notes describe NE7 frames as "complete JPEGs the scope pads to standard
  sizes," which suggests *trimming* trailing padding. That is the wrong mental model
  for this unit. There is nothing to trim past the scan data; there is a marker
  **missing**. A `rfind(FFD9)`-based trim finds nothing and silently does nothing.
- **`file(1)` will call the broken data a valid JPEG**, reporting
  `JPEG image data, baseline, precision 8, 432x432, components 3`, because it only
  sniffs the header. A libjpeg-based decoder may also return an image while emitting a
  "premature end of data" *warning* you are not watching for.

So `file` agreeing is **not** evidence your frames are right. Verify with a decoder in
strict mode — see §6.

### 3.5 IMU / telemetry — image rotation

Datagrams on port **52219**, **24 bytes** **[measured]**. This channel is the *only*
thing needed for auto-rotation: the camera tells you its orientation, and you rotate
the image to match. No extra protocol work is required to obtain it.

**To receive it at all you must keep it alive.** It is sparse — one 6s sample yielded
2 IMU datagrams against 3 video frames **[measured]**. Re-send `86 06 01` to port 52219
roughly every 100 ms, and keep video flowing too. **Never gate frame display on an IMU
packet arriving** — it will stall your video.

#### Byte layout — DISPUTED, verify before relying on it

The upstream implementation parses it as **[source]**:

```
struct.unpack(">BxBxBxxxxxxxxxxxxxHxxxx", d)   # -> (x, y, z, angle)
#               ^  ^  ^              ^
#        bytes  0  2  4           18-19
```

That takes bytes 0, 2 and 4 as **uint8** axes and skips bytes 1, 3, 5. **On this unit
those skipped bytes are non-zero**, so the parse is discarding real data **[measured]**.

Single at-rest sample **[measured]**:

```
fd 73 3c 2e e4 1c 00 00 00 00 00 00 00 00 00 00 00 00 00 9b 00 00 00 00
 \___/ \___/ \___/                                      \___/
   x?    y?    z?                                        angle
```

**Hypothesis: x/y/z are int16 big-endian at offsets 0, 2, 4.** Under that reading
`(x, y, z) = (-653, 15406, -7140)`, whose magnitude is **16993 — i.e. 1.037 g if
2^14 counts = 1 g**. That is exactly what a stationary 3-axis accelerometer should
report, and `atan2(x, y) = -2.4 deg` matches the camera lying flat, which is how it was
sitting. The uint8 reading gives `(253, 60, 228)`, which has no such interpretation.

This rests on **one sample** and is not confirmed. Run `imu_calibrate.py` (ships
alongside this document) to settle it on real hardware.

#### The `angle` field

`angle` is **uint16 big-endian at bytes 18-19**. The vendor app uses it to keep the
image upright; apply it as a rotation of **`-angle` degrees about the image centre**
**[source]**.

**Caveat:** in the one at-rest sample `angle = 155` while `atan2(x, y) = -2.4 deg`.
Those disagree, so one of the following is true and **you must determine which**:
- `angle` is degrees but referenced to a different zero than the accelerometer axes;
- `angle` is not degrees (fixed-point, or 0-65535 mapped onto a turn);
- `angle` is not image roll at all, in which case derive roll yourself via
  `atan2(x, y)` from the int16 axes.

`imu_calibrate.py` discriminates these: rotate the camera through four flat 90-degree
poses and check whether `angle` steps by ~90 each time.

#### Where to apply the rotation

**Client-side.** Rotating server-side forces a JPEG decode plus re-encode on every
frame and will cost most of the 13 fps budget. Options, cheapest first:

- **Browser:** `<img>`/`<canvas>` with `transform: rotate(-Ndeg)`. Free — the compositor
  does it. The camera's **square 432x432** frames mean rotation never exposes empty
  corners inside a square viewport, which is almost certainly why that aspect was chosen.
- **GPU / render layer:** rotate the texture quad or pass the angle into a shader.
- **Native UI:** a view/layer transform rather than touching pixels.

Smooth the angle before applying it — raw accelerometer-derived roll jitters, and a
visibly twitching image is worse than a static one. A low-pass filter
(`a = 0.85*a + 0.15*new`) or a deadband of a few degrees is enough. Also decide what
happens when the camera points straight up or down, where roll about the view axis is
mathematically undefined (gimbal condition) and will swing wildly: typically hold the
last stable angle.

### 3.6 Keepalive / recovery

The camera stops sending if it concludes nobody is listening **[measured]**.

- On a video-read timeout (~2s), re-send `20 36` to port 44506. The stream resumes.
- Re-sending `86 06 01` to 52219 roughly once per second keeps telemetry flowing.
- IMU datagrams are **much** rarer than video — 2 IMU vs 3 frames in one 6s sample.
  Never gate frame delivery on an IMU packet arriving.

---

## 4. Minimal reference implementation

Python 3, standard library only. Prints a complete JPEG to stdout.

```python
import socket, struct

CAM = "192.168.1.1"
IFACE = b"wlp3s0\0"          # interface joined to the camera AP; adjust or drop

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, IFACE)   # Linux only
s.bind(("0.0.0.0", 36123))
s.settimeout(3.0)

# 1. discover
for p in (58090, 46526):
    s.sendto(bytes([0x66, 0x39, 0x01, 0x01]), ("192.168.1.255", p))
info, addr = s.recvfrom(1500)
cam = addr[0]
print(info.decode())

# 2. start streams
s.sendto(bytes([0x86, 0x06, 0x01]), (cam, 52219))
s.sendto(bytes([0x20, 0x36]),       (cam, 44506))

# 3. reassemble one frame
cur, chunks = None, {}
while True:
    data, addr = s.recvfrom(2048)
    if addr[1] != 44506 or len(data) < 4:
        continue
    fnum, flags, cnum, _ = struct.unpack("BBBB", data[:4])
    payload = data[4:]
    last = bool(flags & 1)
    # NOTE: do NOT strip a trailer here. There isn't one - those bytes are scan data.
    if fnum != cur:
        cur, chunks = fnum, {}
    chunks[cnum - 1] = payload
    if last and len(chunks) == cnum:
        jpeg = b"".join(chunks[i] for i in range(cnum)) + b"\xFF\xD9"
        open("frame.jpg", "wb").write(jpeg)
        break
```

Two companion scripts ship with this document:

- **`nk3_mjpeg.py`** — MJPEG HTTP server with keepalive; serves `/`, `/snapshot.jpg`
  and `/stream.mjpg` on port 8080.
- **`imu_calibrate.py`** — resolves the open IMU questions in §3.5 against real
  hardware. `--live` streams decoded values while you rotate the camera by hand. Run it and point any
browser, VLC, or OBS at `http://localhost:8080/stream.mjpg` to confirm the hardware
works before integrating.

---

## 5. Measured performance

On the reference setup **[measured]**:

| Metric | Value |
|---|---|
| Frame rate | 13.2 fps (66 frames / 5s) |
| Bandwidth | ~168 KB/s |
| Resolution | 432 × 432 |
| Frame size | 4.5 – 14 KB |
| Chunks/frame | 4 – 10 |
| Latency to first frame | < 1s after handshake |
| Ping RTT to camera | ~2.9 ms |

Budget accordingly: this is a ~13 fps source, not 30. Do not build a pipeline that
assumes a fixed frame interval — frame size and rate vary with scene complexity.

---

## 6. Verification checklist

Work through this in order. Each step isolates one failure.

1. **Associated?** Confirm the interface is on the camera's SSID, by name — not by
   assuming which interface is which. (`nmcli -t -f DEVICE,CONNECTION dev`)
2. **Reachable?** `ping 192.168.1.1` should answer in single-digit ms.
3. **Discovery?** Broadcast the probe; expect ~504 bytes of JSON from
   `192.168.1.1:46526`. No reply almost always means a second client (the phone) holds
   the camera.
4. **Chunks?** After the start packets, confirm datagrams arriving from port 44506
   with a plausible 4-byte header.
5. **Frames assemble?** First 2 bytes of an assembled frame must be `FF D8`.
6. **Frames decode — strictly.** Do not trust `file`. Force warnings to errors and
   read a pixel:

```python
from PIL import Image
import io, warnings
warnings.simplefilter("error")          # truncation warnings become exceptions
im = Image.open(io.BytesIO(jpeg)); im.load()
print(im.size, im.mode, im.getpixel((216, 216)))
```

   Expect `(432, 432) RGB` and a real pixel value. If this throws but `file` was
   happy, you forgot the `FFD9` in §3.4.

---

## 7. Known unknowns

Not reverse-engineered. Do not guess at these against real hardware.

- ~~**Write/control commands.**~~ **RESOLVED, and the earlier claim here was wrong.** This
  section used to say the descriptor's `cam_brightness` (50) and `calibrated` (0) are
  values "the vendor app clearly sets". It does not set them. The app's complete command
  set is the four payloads in §3.2.1 **[apk]** and none of them writes anything -
  `cam_brightness` and `calibrated` are **reported** by the camera, not configured. There
  is also no native library to go looking in; the whole protocol is Java.
- **LED control, resolution change, battery level polling** - not merely unexplored but
  **absent from the vendor app**. If the camera implements them, the official client never
  uses them, so there is no reference to crib from.
- ~~**Untried candidates from the NE3 sibling.**~~ **TRIED. All dead - do not re-probe.**
  The same APK carries the NE3 implementation (`T0/f.smali`, port 7080) whose `0x20 0x3X`
  family is larger - `20 35`, `20 36`, `20 37`, `20 38`, plus `30 66` and `30 67`
  **[apk]**. All four opcodes NE7 never uses were sent to a live stream on port 44506,
  5 s apart, with the decoded JPEG dimensions logged throughout **[measured]**:

  | probe | frames in | result |
  |---|---|---|
  | `20 35` | 61 | 432x432, ~13 fps, no drops, no stall |
  | `20 38` | 120 | unchanged |
  | `30 66` | 181 | unchanged |
  | `30 67` | 247 | unchanged |

  The camera **silently ignores all of them and keeps streaming** - which also means
  unknown `0x20 0x3X` opcodes are safe to probe here, unlike some devices. Together with
  the vendor app having only four commands and no mode control, **432x432 at ~13 fps is
  this hardware's ceiling.** There is nothing left to find on this axis.
- **IMU byte layout** (see §3.5): uint8-at-even-offsets vs int16-big-endian is
  unresolved; the int16 reading is strongly favoured by a 1.037 g magnitude but rests
  on a single sample. Run `imu_calibrate.py`.
- **Units and reference of the `angle` field**, and whether it is image roll at all.
  In the one at-rest sample it disagreed with accelerometer-derived roll by ~157 deg.
- `flags2` in the video header (byte 3). Still unknown. The "5-byte trailer" that used to
  be listed alongside it does not exist - see §3.3.

---

## 8. Provenance

Protocol originally reverse-engineered by the **haxko e.V.** hackerspace (WTFPL) for
the natfire NE3/NE7 earpick scopes: <https://github.com/haxko/NE3-Scope>.
Downstream viewer with protocol auto-detection:
<https://github.com/The-Dorkknight/earscope-app>.

Same-chipset (BK7231U) work on related "Suear" cameras, if you need to go further:
<https://github.com/SeanPesce/Suear-Web-Viewer>.

This document adds the NE7 confirmation for the NK3/`HNDEC_55` unit, the
brand-string-vs-protocol mismatch, the missing-`FFD9` finding, and the measured
performance figures above.

Revised 2026-10-12 against the decompiled vendor app (`com.xiaozhen.beauty.hnd`, no native
library, protocol entirely in `T0/d.smali`). That pass corrected four things previously
taken from the reference implementation: the discovery byte is `0x30` not `0x39`, port
58090 does not exist, `20 37` stops the video stream, and the vendor app does **not** write
`cam_brightness` or `calibrated`. It also established that the complete command set is four
payloads with no mode or resolution control of any kind.

One further correction from the ESP32 port: **there is no 5-byte trailer on the final
chunk** of a frame on this unit - see the note in §3.3. Stripping it truncates every frame.
