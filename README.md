# ssafy-race

SSAFY Race Java bot workspace.

## Structure

- `java/MyCar.java`: main driving bot logic.
- `java/MyCarLogicTest.java`: small logic checks for steering and speed control.
- `java/DrivingInterface/DrivingInterface.java`: Java interface model used by the bot.
- `settings/settings-basic.json`: AirSim settings for the basic map (`Map: "10"`).

Simulator binaries, DLLs, build outputs, and downloaded ZIP files are intentionally excluded from Git.

## Local Paths

- Simulator: `C:\SSAFY_RACE\Simulator`
- Runnable Java template: `C:\SSAFY_RACE\MyCar_Java\Bot_Java`
- AirSim settings: `C:\Users\myway\Documents\AirSim\settings.json`

## Test

Run from the runnable Java template folder where `DrivingInterface.dll` exists:

```powershell
cd C:\SSAFY_RACE\MyCar_Java\Bot_Java
& 'C:\Program Files\Eclipse Adoptium\jdk-17.0.19.10-hotspot\bin\javac.exe' -encoding UTF-8 'MyCarLogicTest.java' 'MyCar.java' 'DrivingInterface\DrivingInterface.java'
& 'C:\Program Files\Eclipse Adoptium\jdk-17.0.19.10-hotspot\bin\java.exe' -cp '.' 'MyCarLogicTest'
```
