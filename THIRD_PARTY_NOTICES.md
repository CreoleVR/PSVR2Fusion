# Third-party notices

## OpenVR SDK 2.15.6

`external/openvr/headers/openvr_driver.h` is from ValveSoftware/openvr tag
`v2.15.6`.

Copyright (c) 2015, Valve Corporation. Licensed under the 3-clause BSD license;
the complete text is in `external/openvr/LICENSE`.

Source: https://github.com/ValveSoftware/openvr

## MinHook

MinHook under `external/minhook` is Copyright (C) 2009-2017 Tsuda Kageyu, with
Hacker Disassembler Engine portions Copyright (c) 2008-2009 Vyacheslav Patkov.
It is distributed under a 2-clause BSD license; see
`external/minhook/LICENSE.txt`.

Source: https://github.com/TsudaKageyu/minhook

## References (no code copied)

- **Monado**'s psvr2 driver (BSL-1.0): the PSVR2 status/IMU record layout,
  sensor scales and axis mapping used in `src/imu.h`.
  https://gitlab.freedesktop.org/monado/monado
- **PSVR2 Toolkit** (MIT): the addresses of `CaesarManager`'s getter and
  `CaesarUsbThread::ReadPipe` in Sony's driver, used in `src/imu_source.cpp`.
  https://github.com/BnuuySolutions/PSVR2Toolkit
