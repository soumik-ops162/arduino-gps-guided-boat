# Project Summary

This project is a GPS guided navigation boat built around an Arduino Nano.

## Main Components
- NEO-6M GPS
- HMC5883L compass
- Two brushed motors with ESCs
- FlySky receiver using CH6 AUX for mode switching

## Modes
- Manual mode from RC transmitter
- Autopilot mode for waypoint navigation

## Control Method
The boat uses differential thrust:
- left motor speed increases while right decreases to turn one way
- right motor speed increases while left decreases to turn the other way

## Current Status
- Code structure completed
- Code compile-verified with Arduino CLI in WSL
- Uploaded to GitHub
- Hardware integration pending
