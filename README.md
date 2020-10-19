## Description
Make something useful out of your old home computer. Together with a Raspberry Pi and some optional electronic parts, you get a cool computer with vintage look and feel which you can use for almost all sort of things that can be done with a modern one. And that´s almost everything that you can imagine. This Repo consists of:
- A modified matrix keyboard driver and its device tree overlay file. This allows you to connect the keyboard directly to the GPIO pins of the Raspberry Pi. The overlay supplied here was created for Commodore VIC-20 and C64 keyboards but can be easily adapted to other matrix keyboards.
- An XKB symbol file for the X11 subsystem defining individual keys so that the character sets of modern keyboards and special characters like \ _ ä etc. can be used. 
- Schematics and systemd scripts that automatically shut down the Raspberry Pi and then turn it off after the power switch is turned off. With this, it can be switched off just as easily as a vintage home computer, without having to shut down the Pi.

## Installation
- Recommended: Raspberry Pi 4B with Raspbian Buster, but it can work with others too. However I haven't tested it.
- Commodore VIC-20 or C64 matrix keyboard. For others, the mappings in the device tree overlay file */driver/c2020.dts* must be changed accordingly

In a command shell type:
```
$ sudo apt update
$ sudo apt upgrade
```

When used without autoshutdown circuit:
```
$ cd driver
$ sudo ./install
```
When used with autoshutdown circuit:
```
$ sudo ./install
```
Install Vice (optional):
- Download source code: https://sourceforge.net/projects/vice-emu/files/releases/vice-3.4.tar.gz/download
- Unpack the Tarball:
```
$ tar -xvf vice-3.4.tar.gz
```
- Install required packages:
```
$ sudo apt --yes install  flex bison xa65 libasound2-dev libsdl2-dev texinfo
```
- Build, Compile and install Vice:
```
$ ./configure --without-pulse --with-alsa 
$ make -j4
$ sudo make install
```
- Install configuration files to adapt Vice to the C2020:
```
$ ./viceinstall
```
- Start C64 emulation:
```
$ ./c64.sh
```

## Autoshutdown Circuit
The Autoshutdown circuit can be built with generic components. It should be noted that the bridge rectifier, the relay and the diode D1 should be able to withstand currents up to 1A, and the step-down converter should be able to deliver output currents up to two 2A. You can use relays with coil voltages other than 5V, if you connect PIN 4 to the power input of the step down converter instead and make sure that its maximal coil voltage is not exceeded.
Autoshutdown requires the keyboard driver module to be installed and loaded to function.

## Joystick
One legacy Atari or Commodore joystick is supported. A second joystick can be connected via USB. Some games like "Winter games" switch over to the (non existant) second joystick and cannot be played any more. Press (CTRL)+(f3) to toggle back to the existing joystick in this case.

## More
Key Mappings for X11 and Vice:

[X11 (xkb)]:./driver/keymap.md
[Vice]: ./vice/hotkeys.md
[X11 (xkb)]

[Vice]

