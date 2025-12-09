# EMG-Controlled Prosthetic Arm
A functional low-cost EMG-controlled prosthetic hand. Reads forearm muscle EMG signals, classifies intent, and controls servos/motors to replicate natural grasp patterns.

## 🎯 Project Goal
Create an open, affordable prosthetic arm platform that is:
- Real-time
- Modular
- Expandable (3D printed)
- Designed for meaningful assistive use

## 🚀 Features
- Custom made sensor for EMG acquisition
- Hardware band-pass filtering (20–450 Hz)
- Envelope extraction + smoothing
- Microcontroller digital signal classifier
- 3 grip modes:
  - Power grasp
  - Pinch
  - Precision grip
- Safety controller to avoid overheating motors

## 🧠 System Architecture
Skin → EMG sensor → BPF → ADC → Signal classification → Motor controller → Prosthetic finger actuation
```
prosthetic-arm/
├── firmware
│   ├── gesture_classifier.cpp
│   ├── emg_filter.cpp
│   ├── motor_controller.cpp
├── hardware
│   ├── schematics/
│   ├── pcb/
│   ├── wiring/
│   ├── mechanical/
│   ├── stl_files/
├── test
│   ├── emg_plots
│   ├── recorded_sessions
```

## 🛠️ Tech Stack
- Custom EMG sensor
- ESP32 or Arduino Nano
- L293D + servos
- 3D-printed hand (Fusion 360 files included)
- Python visualization utilities

## ▶️ Demo Suggestions
- Live EMG contraction controlling finger flexion
- Grasp a cup, grab pen, hold objects
- Noise demonstration: before vs. after filtering

## 📌 Roadmap
- [ ] Add adaptive machine learning classifier
- [ ] Add force sensor feedback
- [ ] Add Bluetooth app for grip switching


