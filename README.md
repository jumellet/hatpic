# HATPIC: An Open-Source Single Axis Haptic Joystick for Robotic Development

The only thing that is difficult with this device, is to pronounce its name.

![alt text](https://github.com/jumellet/hatpic/blob/dev/assets/images/hatpic-device.jpg)

The Hatpic device is an open source 1 Degree of Freedom (DoF) haptic joystick.

# How to use

Ensure motor ID set to 1.

After connection of the devic with USB, check the address of the USB port.

In a first terminal,

```
roscore
```

in a second terminal,

```
python3 hatpic-ros1.py
```
