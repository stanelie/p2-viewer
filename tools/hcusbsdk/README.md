# HIKMICRO / HCUSBSDK analysis tools

Throwaway-grade reverse engineering scripts, kept because they produced
`docs/GW192A_frame_layout.md` and will be needed again to finish driving the GW192A's extension
unit. Nothing here is built or run by the firmware.

They take the GOYOJO vendor app (`com.guardExpert.thgStartGoogle`, "THG Start") as input. The APK
is not redistributed here — supply your own copy and unpack it into its own empty directory:

```bash
unzip -o app.apk 'classes*.dex' 'lib/arm64-v8a/lib*.so' -d apk_x
```

Requires `python -m pip install capstone pyelftools`, plus a JDK and
[jadx](https://github.com/skylot/jadx) for the Java side.

| script | what it does |
|---|---|
| `dexstr.py` | dumps a DEX string table, no Java needed — how `classes2.dex` was identified as the one holding the SDK |
| `disas.py` | disassembles named AArch64 functions out of a `.so`; `--list [filter]` to find them |
| `xref.py` | finds `BL`/`B` call sites targeting an address, naming the enclosing function |
| `cmdmap.py` | walks every `CUsbCommandConfig<ID>::Excute()` and recovers the `(control selector, sub-function)` pair it encodes |

`command_map.txt` is `cmdmap.py`'s output for `libHCUSBSDK.so`: 415 commands. Two cautions —
the SDK serves card readers and CDC devices as well as cameras, so much of it is irrelevant to a
thermal module; and a few extracted selectors come out above 10 (11, 23, 24, 25, and one absurd
415), which are either other device families or mis-parses of the heuristic. Check those against
the function by hand before believing them.

Run Python with `-I` on anything that reads the unpacked APK, and keep these scripts in a
different directory from the extracted files.

Worked example — recovering the command switch:

```bash
python -I disas.py apk_x/lib/arm64-v8a/libHCUSBSDK.so --list SwitchCommand
python -I disas.py apk_x/lib/arm64-v8a/libHCUSBSDK.so \
    _ZN8HCUSBSDK17AccessDeviceModel19CUsbDeviceVideoBase13SwitchCommandERKNS_18tagXU_COMMAND_HEADEhj
python -I xref.py apk_x/lib/arm64-v8a/libHCUSBSDK.so 0x23c3f4
python -I cmdmap.py apk_x/lib/arm64-v8a/libHCUSBSDK.so | head
```
