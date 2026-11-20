#!/usr/bin/env bash
# Is the BNO055 IMU answering on I2C bus 1? Run with the robot stack STOPPED.
if i2cdetect -y 1 | grep -qE '^20:.* (28|29)( |$)'; then
  echo "IMU OK (BNO055 at 0x28/0x29)"
else
  echo "IMU MISSING on i2c-1: power-cycle the robot (battery off ~10 s)"; exit 1
fi
