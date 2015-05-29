**SQUAD** is a homebrew flight controller I've been working on.

It is a flight controller based on an Arduino Uno, a MPU 6050 and a HC-05 Bluetooth module.

Included in the project is a simple helper to read registers of the IMU.

Hard copy versioning before I decided to use git. All upcoming versions will be done in SQuad.

V3 - Polls with "Req Data" for throttle speed(2 Bytes), pitch(2 Bytes), roll(2 Bytes), (yaw trim right (bit 3), yaw trim left (bit 2) and emergency stop (bit 1))(1 Byte).