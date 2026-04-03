# GPS Guided Navigation Boat

Arduino Nano based autonomous boat project using:
- NEO-6M GPS
- HMC5883L compass
- Two brushed motors with ESCs
- Differential thrust steering
- FlySky receiver with CH6 AUX switch for Manual / Autopilot mode

## Features
- Manual mode from RC transmitter
- Autopilot mode from AUX switch
- Waypoint navigation
- Differential thrust control
- Modular code structure inspired by classic RoboBoat autopilot organization

## Project Structure
- `GPS_Guided_Boat_Nano.ino` - main entry point
- `Init.ino` - hardware initialization
- `
cat > README.md <<'EOF'
# GPS Guided Navigation Boat

Arduino Nano based autonomous boat project using:
- NEO-6M GPS
- HMC5883L compass
- Two brushed motors with ESCs
- Differential thrust steering
- FlySky receiver with CH6 AUX switch for Manual / Autopilot mode

## Features
- Manual mode from RC transmitter
- Autopilot mode from AUX switch
- Waypoint navigation
- Differential thrust control
- Modular code structure inspired by classic RoboBoat autopilot organization

## Project Structure
- `GPS_Guided_Boat_Nano.ino` - main entry point
- `Init.ino` - hardware initialization
- `Navigation.ino` - GPS, compass, waypoint navigation
- `PID_Control.ino` - heading PID control
- `Motor_Control.ino` - motor mixing and control
- `Debug.ino` - serial debug output
- `defines.h` - configuration constants
- `waypoints.h` - mission waypoints

## Hardware
- Arduino Nano
- NEO-6M GPS
- HMC5883L compass
- FlySky receiver
- 2 brushed ESCs
- 2 DC motors

## Status
Code verified with Arduino CLI for Arduino Nano.

## Notes
This repository is prepared and compile-checked before hardware integration.
Future work includes sensor calibration, field tuning, and water testing.

## Build
Compile from WSL using Arduino CLI:

```bash
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 .

git add README.md
git commit -m "Update project README"
git push
mkdir -p code/GPS_Guided_Boat_Nano
