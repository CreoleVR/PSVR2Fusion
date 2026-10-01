# PSVR2Fusion

A SteamVR add-on that turns the PlayStation VR2 into a lighthouse-tracked
headset. Mount a Vive tracker (or any SteamVR-tracked device) on the PSVR2 and
the headset takes that tracker's pose, with its orientation fused with the
PSVR2's own 2 kHz IMU. Index controllers, trackers and the PSVR2 then share one
lighthouse universe, with no SpaceCalibrator and no inside-out tracking drift.

Sony's driver keeps doing everything else: display, Sense controllers, eye
tracking, haptics.

## Requirements

- Windows, SteamVR, the PSVR2 on the PC adapter with Sony's **PlayStation VR2
  App**. [PSVR2 Toolkit](https://github.com/BnuuySolutions/PSVR2Toolkit) is
  supported but not needed.
- Lighthouse base stations and a tracker fixed rigidly to the headset. Its
  pose is used as the head pose as is, with no mounting offset, so mount it
  with its orientation matching the headset's.

## Install

1. Download `PSVR2Fusion-<version>.zip` from
   [Releases](https://github.com/CreoleVR/PSVR2Fusion/releases) and extract the
   `psvr2fusion` folder somewhere permanent.
2. Register it with SteamVR (adjust the paths):

   ```
   "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe" adddriver "C:\path\to\psvr2fusion"
   ```

3. Put your tracker's serial (e.g. `LHR-1A2B3C4D`) on its own line in
   `psvr2fusion\bin\win64\tracker.txt`. SteamVR shows it under
   **Settings > Controllers > Manage Trackers**. Edits apply within a second,
   without restarting SteamVR.
4. Start SteamVR with the PSVR2 connected.

To remove it: `vrpathreg removedriver "C:\path\to\psvr2fusion"`, or turn it
off under **Settings > Startup / Shutdown > Manage Add-ons**.

## How it works

- **Virtual tracker.** The driver adds a device, `psvr2fusion0`, and sets the
  SteamVR tracking override `/devices/psvr2fusion/psvr2fusion0` →
  `/devices/playstation_vr2/playstation_vr2_headset`, so its pose becomes the
  headset's. It only does this while the PSVR2 is SteamVR's HMD; with another
  headset it stays out of the way.
- **Raw IMU.** Sony's driver reads the headset's status interface (USB
  interface 7, endpoint 0x88) on its own thread: a 32-byte header plus 24-byte
  IMU records at 2 kHz. PSVR2Fusion hooks that thread's `ReadPipe` inside
  Sony's DLL with MinHook (chaining in front of PSVR2 Toolkit's hook when the
  toolkit is installed), integrates the gyro, and never writes to the
  headset.
- **Fusion.** A complementary filter: the rotation from the gyro's frame to
  the lighthouse frame follows the tracker with time constant `tauSeconds`,
  so the gyro carries fast motion and the tracker removes drift. Position and
  velocity come from the tracker. The pose is sent at `updateHz`.
- **No fallback.** If the tracker isn't seen, the headset has no pose; Sony's
  inside-out tracking isn't mixed in. If the IMU stops, orientation comes
  straight from the tracker.
- **Room setup.** The headset is pointed at its own room setup file,
  `config\psvr2fusion\chaperone_info.vrchap` next to Sony's, and its universe
  id is pinned, so the lighthouse room setup isn't shared with or overwritten
  by another headset. SteamVR reads that file but cannot save into it: Room
  Setup changes last for one session, so edit the file to keep them.

## Settings

Optional, in `steamvr.vrsettings` under `"driver_psvr2fusion"`:

| Key | Default | |
|---|---|---|
| `trackerSerial` | | Tracker serial, used when `tracker.txt` names none |
| `fusion` | `true` | `false`: tracker orientation only, no IMU |
| `tauSeconds` | `1.0` | How fast the tracker corrects the gyro |
| `updateHz` | `1000` | Pose updates per second (30–1000) |
| `isolateRoomSetup` | `true` | Use the room setup file above |
| `roomUniverseId` | `"1788272693"` | Universe id to pin (`"0"`: don't pin) |

## Logs

Everything is in SteamVR's `vrserver.txt` with a `psvr2fusion:` prefix. Every
10 seconds it logs the IMU state, packet and sample counts, the sample rate
(about 2000 Hz when live), and the angle between the fused and tracker
orientation. When the tracker isn't found, it lists every device it can see
with its serial.

## Compatibility

The hook targets the Sony driver build that PSVR2 Toolkit supports. The code
bytes at both addresses are checked first. On any other build, the IMU is left
alone and the log says `Sony driver build not recognised`. The headset then
tracks from the tracker alone.

## Building

Visual Studio 2022 (C++ workload) and CMake 3.19 or newer:

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
build\Release\psvr2fusion_tests.exe
```

The driver is built into `driver/psvr2fusion/bin/win64/`, so that folder is the
add-on itself.

## License

MIT, see [LICENSE](LICENSE). Third-party code and references are listed in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

PSVR2Fusion is not affiliated with Sony Interactive Entertainment or Valve.
PlayStation and PS VR2 are trademarks of Sony Interactive Entertainment Inc.
