# Notes

Working notes for [p2-viewer](../README.md). These are mostly measurements rather than
documentation: a fair amount of this hardware was reverse-engineered, and the point of writing it
down is that nobody repeats a dead end. Where something was ruled out, the method that ruled it
out is recorded with it.

Everything here was observed on real hardware unless it says otherwise.

## Cameras

- **[thermal_cameras.md](thermal_cameras.md)** - how two different thermal cameras share one
  pipeline, the per-model table, flat-field correction and the auto-shutter toggle.
- **[p2pro_protocol.md](p2pro_protocol.md)** - the InfiRay P2 Pro: stream format, the three
  cold-start phases, how the 9 s startup became 5 s, and the vendor command channel. Dangerous
  opcodes are flagged.
- **[GW192A_frame_layout.md](GW192A_frame_layout.md)** - the GOYOJO GW192A, which is a rebadged
  HIKMICRO module: frame map, marker words, temperature calibration, and the HIKMICRO extension
  unit whose protocol is recovered and whose transport works but which still returns no data.
- **[generic_uvc_cameras.md](generic_uvc_cameras.md)** - non-thermal USB cameras: mode selection,
  MJPEG through the hardware JPEG decoder, measured steady state, hot-swap.
- **[wifi_camera.md](wifi_camera.md)** - the NK3 WiFi otoscope: joining, UDP reassembly,
  auto-rotation from its accelerometer, and the latency work.
- **[NK3_camera_protocol.md](NK3_camera_protocol.md)** - that camera's protocol as a standalone
  spec, with each fact tagged by how it is known (measured here, read from the reference
  implementation, or read from the decompiled vendor app).
- **[builtin_camera.md](builtin_camera.md)** - the board's own OV5647 through the P4's ISP. The
  single biggest finding: the sensor mode matters more than anything host-side, and the binning
  mode is both more pixels and 2.7x faster.

## Platform

- **[platform.md](platform.md)** - boot time accounting, flash layout, the power button, the
  display clock, and the `esp_lcd` bug that was doing a 307 KB internal allocation on every
  frame in every camera path.
- **[usb_link_investigation.md](usb_link_investigation.md)** - the long hunt for why one borescope
  would not work on the OTG port. The answer was a connector. Kept for the PHY measurements it
  produced on the way (the P4's undocumented UTMI trims, a squelch sweep, an enumerator trace)
  and for the list of hypotheses that are now closed.

## Reference

- **[P2Pro_cmd_reference.py](P2Pro_cmd_reference.py)** - a copy of the vendor command transport
  from [LeoDJ/P2Pro-Viewer](https://github.com/LeoDJ/P2Pro-Viewer), kept so the framing stays
  readable independently of this firmware.
- **[../tools/hcusbsdk/](../tools/hcusbsdk/)** - the scripts used to pull the HIKMICRO command map
  out of `libHCUSBSDK.so`, plus the resulting 415-command map.
