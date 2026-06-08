# Sound Inference BLE Sketch

This folder adds BLE result transmission for the Edge Impulse sound classifier.
The ZIP currently in `test` is a firmware-only deployment with `.bin` files, so
it cannot be edited directly. To change behavior, export the model from Edge
Impulse as an Arduino library and compile this source sketch.

## What BLE Sends

The sketch advertises as `EI-Sound-BLE` and exposes this custom BLE service:

- Service: `19B10000-E8F2-537E-4F6C-D104768A1214`
- Top result notify/read: `19B10001-E8F2-537E-4F6C-D104768A1214`
- Full score string notify/read: `19B10002-E8F2-537E-4F6C-D104768A1214`
- Inference count notify/read: `19B10003-E8F2-537E-4F6C-D104768A1214`

Example top-result payload:

```text
12,crackling_fire,0.988
```

Example full-score payload:

```text
12:clock_alarm=0.000,crackling_fire=0.988,crying_baby=0.000,dog=0.008,glass_breaking=0.004,siren=0.000,train=0.000
```

## Setup

1. In Edge Impulse Studio, use `Deployment -> Arduino library`, not the
   precompiled firmware deployment.
2. Install the downloaded Edge Impulse Arduino library ZIP. This workspace
   already has `ei-sound-detection-arduino-1.0.16-impulse-#1.zip` installed:

```powershell
.\arduino-cli\arduino-cli.exe lib install --zip-path "path\to\your-edge-impulse-arduino-library.zip" --config-file arduino-cli.yaml
```

3. Confirm the generated inference header:

```powershell
Get-ChildItem arduino-user\libraries -Recurse -Filter "*inferencing.h"
```

   The current sketch uses:

```cpp
#include <Sound_Detection_inferencing.h>
```

4. Compile for Arduino Nano 33 BLE:

```powershell
.\arduino-cli\arduino-cli.exe compile --fqbn arduino:mbed_nano:nano33ble test\sound_ble_inference --config-file arduino-cli.yaml
```

5. Upload after connecting the board:

```powershell
.\arduino-cli\arduino-cli.exe upload -p COMx --fqbn arduino:mbed_nano:nano33ble test\sound_ble_inference --config-file arduino-cli.yaml
```

Replace `COMx` with the board port from:

```powershell
.\arduino-cli\arduino-cli.exe board list --config-file arduino-cli.yaml
```

## Testing BLE

Use a BLE scanner such as `nRF Connect` on a phone:

1. Scan for `EI-Sound-BLE`.
2. Connect to it.
3. Open service `19B10000-E8F2-537E-4F6C-D104768A1214`.
4. Enable notifications on characteristic
   `19B10001-E8F2-537E-4F6C-D104768A1214`.
5. Run sounds near the board and watch the label/confidence update after each
   inference window.

## Viewing Results in PowerShell

Open the Arduino CLI serial monitor at `115200` baud:

```powershell
.\arduino-cli\arduino-cli.exe monitor -p COM3 -c baudrate=115200 --config-file arduino-cli.yaml
```

If the port is not `COM3`, check it with:

```powershell
.\arduino-cli\arduino-cli.exe board list --config-file arduino-cli.yaml
```

The Nano 33 BLE PDM library does not accept the model's `12000 Hz` sample rate
directly, so the sketch records PDM audio at `16000 Hz` and resamples incoming
samples to `12000 Hz` before inference.
