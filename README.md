# ssafy-race

SSAFY Race bot workspace.

## Structure

- `java/MyCar.java`: main driving bot logic.
- `java/MyCarLogicTest.java`: small logic checks for steering and speed control.
- `java/DrivingInterface/DrivingInterface.java`: Java interface model used by the bot.
- `java/variants/`: map-specific `MyCar.java` versions kept for copy-based switching.
- `python/my_car.py`: Python speed-mode driving bot logic.
- `python/test_my_car_logic.py`: Python logic checks for steering, speed control, obstacle avoidance, and recovery.
- `tools/run_speed_candidates.py`: runs Python speed candidates in parallel in isolated temporary folders.
- `docs/HANDOFF_2026-06-23.md`: continuation notes for another Codex/computer.
- `docs/benchmark_summary.md`: compact benchmark history, without raw generated logs.
- `settings/settings-basic.json`: AirSim settings for the basic map (`Map: "10"`).
- `settings/settings-speed.json`: AirSim settings for the speed map (`Map: "31"`).

Simulator binaries, DLLs, build outputs, and downloaded ZIP files are intentionally excluded from Git.

## Variant Switching

Each file under `java/variants/` keeps the internal class name as `MyCar`.
To run a variant, copy it over both active files:

```powershell
Copy-Item java\variants\MyCarSpeed.java java\MyCar.java -Force
Copy-Item java\variants\MyCarSpeed.java C:\SSAFY_RACE\MyCar_Java\Bot_Java\MyCar.java -Force
```

## Local Paths

- Simulator: `C:\SSAFY_RACE\Simulator`
- Runnable Java template: `C:\SSAFY_RACE\MyCar_Java\Bot_Java`
- Runnable Python template: `C:\SSAFY_RACE\MyCar_Python\Bot_Python`
- AirSim settings: `C:\Users\myway\Documents\AirSim\settings.json`

## Current Speed-Mode Handoff

For continuation work, start with `docs/HANDOFF_2026-06-23.md`.
The current active Python candidate is `python/my_car.py`.

Sync it into the simulator runtime:

```powershell
Copy-Item python\my_car.py C:\SSAFY_RACE\MyCar_Python\Bot_Python\my_car.py -Force
```

## Test

### Python speed logic

Run local logic tests:

```powershell
& 'C:\Users\myway\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m unittest discover -s python -v
```

Run Python speed candidates in parallel. The default candidate is `python/my_car.py`, plus any files under `python/variants/*.py`:

```powershell
& 'C:\Users\myway\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\run_speed_candidates.py --jobs 6 --python 'C:\Users\myway\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
```

To run in the SSAFY Python runtime, copy `python/my_car.py` over the template runtime `my_car.py` and use `settings/settings-speed.json` as AirSim `settings.json`.

### Java logic

Run from the runnable Java template folder where `DrivingInterface.dll` exists:

```powershell
cd C:\SSAFY_RACE\MyCar_Java\Bot_Java
& 'C:\Program Files\Eclipse Adoptium\jdk-17.0.19.10-hotspot\bin\javac.exe' -encoding UTF-8 'MyCarLogicTest.java' 'MyCar.java' 'DrivingInterface\DrivingInterface.java'
& 'C:\Program Files\Eclipse Adoptium\jdk-17.0.19.10-hotspot\bin\java.exe' -cp '.' 'MyCarLogicTest'
```
