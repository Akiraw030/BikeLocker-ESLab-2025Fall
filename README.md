# 🚲 BikeLocker - Smart IoT Bicycle Lock
**ESLab 2025 Fall Term Project**:
A multi-functional smart bicycle lock based on STM32 and Flutter.

## 📖 Overview
BikeLocker is a smart bicycle lock system designed to solve campus theft issues. Unlike traditional passive locks, BikeLocker provides active monitoring, anti-theft alarms, and riding data analysis with user-friendly design.
The system consists of two main parts:
- The Lock (Hardware): An STM32-based device attached to the bike, featuring motion detection, alarm (buzzer), monitor display, and BLE communication.
- The Controller (App): An Android mobile app for remote control, status monitoring, and viewing riding statistics.

## ✨ Key Features
### 🔒 Locking & Unlocking
1. App Control: Users can lock/unlock via the "BikeLocker" mobile app over Bluetooth.
1. Emergency Unlock: In case of no phone, users can use the User Button on the STM32 board with a specific pattern (Short-Short-Long) to lock/unlock.
1. Status Indicator: LED indicators on the board show the current state (LED2 Green = Locked, LED8 Red = Unlocked).
### 🛡️ Anti-Theft (Locked State)
1. Find My Bike: Users can trigger the buzzer remotely from the App to locate their bike.
1. Significant Motion Detection: If the bike is moved illegally, the accelerometer detects significant motion, triggers the alarm, and records the Timestamp.
1. Incident Logging: Upon the next unlock, the App retrieves and displays the history of abnormal vibration events with calculated timestamps.
### 🚴 Riding Monitor (Unlocked State)
1. Auto-Lock: Automatically locks if no motion is detected for a certain period to prevent forgetting to lock.
1. Riding Dashboard: Calculates real-time Speed (via Hall Sensor) and Calories burned, displaying them on both the STM32 screen and the Mobile App.

## 🛠️ System Architecture
1. STM32 Firmware (Hardware)
   - Platform: STM32L475VGT6 (B-L475E-IOT01A1)
   - Peripherals: LSM6DSL (Accelerometer), BlueNRG-MS (BLE), GPIO (Buzzer/LED), Flash - - Memory (Log storage).
   - Logic: Manages state machine, sensor data processing, and BLE broadcasting.
2. Mobile App (Software)
   - Framework: Android Studio + Flutter.
   - Library: flutter_reactive_ble for robust Bluetooth communication.
   - Function: Acts as the controller and display.
   - Time Sync: Uses a relative time calculation method (Unlock Time - Uptime Difference) to determine exact event times without requiring the STM32 to maintain a real-time clock.

## 🚀 Installation & Usage

**STM32 Setup**
1. Import the project into STM32CubeIDE.
1. Connect the B-L475E-IOT01A1 board.
1. Build and Run the firmware.
1. Ensure the BlueNRG expansion board is properly mounted (if used) or onboard BLE is initialized.

**App Setup**
1. Ensure Flutter SDK is installed.
1. Navigate to the Android app directory:
```
cd "Android app"
flutter pub get
```
1. Connect an Android device and run:
```
flutter run --release
```

## 📸 Demo video
- Youtube video: https://youtu.be/uPS2nr-1a1U

## 👥 Team Members

- 陳貫今 b12901081 - STM32 Part
- 陳政年 b12901109 - Android App Part
