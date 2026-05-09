# RCBRO - RC Based Self Balancing Robot

---

Based on this project: https://github.com/jjrobots/B-ROBOT_EVO2

---

# Introduction
RCBRO is a Radio Control Based Self Balancing Robot. It is a two-wheeled robot that can balance itself using a 
gyroscope and accelerometer. The robot can be controlled using a remote control, allowing it to move forward, backward, and turn.

This project is designed around the Teensy 2.0 board, which feature a ATMega32U4 microcontroller. The Teensy 2.0 is a powerful 
and versatile microcontroller that is well-suited for robotics projects. It has a built-in USB interface, which allows for 
easy programming and communication with the robot.

## Pinout
The pinout for the Teensy 2.0 board is as follows:

![Teensy 2.0 pinout](Img/Teensy_pinout.jpg)

# Transmitter

The RC transmitter used in this project is a 3-channel FlySky FS-GT3B transmitter. It operates on the 2.4GHz 
frequency and has a range of up to 500 meters. The transmitter has a built-in LCD screen that displays the current 
channel and battery status.

The transmitter is flashed with [Custom Firmare 0.6.1](https://github.com/semerad/gt3b/tree/master), which allows for better control and customization of the transmitter. 
The custom firmware also allows for the use of additional channels, which can be used for controlling additional features of the robot.

Newer versions of the transmitter has to be tweaked a little to be able to bind with the receiver. Check the RC folder for more details.