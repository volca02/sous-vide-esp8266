# sous-vide-esp8266

This project implements a simple timer and PID controller for a dumb slow cooker to become both a timer enabled slow cooker and a simple sous-vide cooker.

## About

The goal is to have a solid state relay control a simple crock-pot (or other compatible simple slow cooker that comes without any electronics). The usage of such contraption is both to add a timer to that slow cooker, and optionally hold a set temperature (when PID control is enabled in setup screen).

## Materials

You will need a couple of components:
 * [Wemos D1 Mini - or a clone](https://www.aliexpress.com/item/32681374223.html?spm=a2g0s.9042311.0.0.36504c4dLBnw4e)
 * [DS18B20 Waterproof Temp sensor](https://www.aliexpress.com/item/32305869288.html?spm=a2g0s.9042311.0.0.36504c4dLBnw4e)
 * [Solid state relay (DC control, AC output)](https://www.aliexpress.com/item/4000045425145.html?spm=a2g0s.9042311.0.0.36504c4dLBnw4e)
 * [SH1106Spi display](https://www.aliexpress.com/item/32844104782.html?spm=a2g0s.9042311.0.0.27424c4d58Iwcy)
 * [Rotary encoder](https://www.aliexpress.com/item/32915420023.html?spm=a2g0s.9042311.0.0.27424c4d58Iwcy)
 * Power cable
 * 5V power adaptor circuit (i.e. something that converts wall socket power to 5V for the Wemos D1)
 * Your local wall mounted power socket (or a cable power socket)
 * An enclosure - a 3d printable model is in model/ directory (written in openSCAD) - I used universal pcb mounted into the frame, and an a gutted old adapter for the power supply (so the power supply mount will have to be modified for your situation). Sorry the code is a bit messy.

## Wiring

- `GND` is the D1's ground pin
- `3.3V` is the D1's 3V pin

### Temperature sensor

The Black wire connects to `GND`, Red wire to `3.3V`, Yellow wire to `D4`

### Solid state relay

The DC part connects `-` to `GND`,  `+` to `D8`
The AC part connects one power wire through the relay (as appropriate to your country). Ground wire connects directly through to the output socket.

### Display

These are the display's connections:

 * `D7` - `SDA`
 * `D5` - `SCK`
 * `D3` - `RST`
 * `D1` - `DC`
 * `GND` - `CS`
 * `3.3V` - `VDD`

### Rotary encoder

Due to the limited number of pins, A0 is used as a button detection pin.

 * `3.3V` - pin marked `5V`
 * `GND` - `GND`
 * `A0` - `KEY`
 * `D2` - `S1`
 * `D0` - `S2`


 ### Power adaptor

 The input power gets connected to the power adaptor.
 Power adaptor output connects to `5V` and `GND` leads of the Wemos D1.
