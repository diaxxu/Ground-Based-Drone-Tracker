# Wiring Diagram

This document describes every electrical connection in the system at the connector level. Follow in order. Do not power the system until all connections are verified.

---

## Power Architecture

```
Mains AC
    |
    v
Mean Well S-240-24 (24V 10A)
    |
    +--[XT60]--> ODrive S1 DC input (terminals VBus + GND)
    |
    +--[D24V50F5 buck]--> 5V rail --> Jetson Orin Nano USB-C PD (if no dedicated charger)
    |
    +--[D24V22F12 buck]--> 12V rail --> Camera (USB draws from Jetson, no separate power)
    |
    +--[Fuse 3A]--> BMI088 breakout 3.3V pin via Jetson 3.3V header
```

Important: ODrive S1 accepts 12-56V DC. Connect 24V directly with 14 AWG wire. Max continuous current draw with two GBM5208 motors at full torque is approximately 16A. The 10A PSU is sufficient for tracking operation (average ~4A).

---

## Connector Map

### ODrive S1

| ODrive terminal | Connects to | Wire gauge |
|---|---|---|
| VBus (red) | PSU 24V positive via XT60 | 14 AWG |
| GND (black) | PSU 24V negative via XT60 | 14 AWG |
| M0 Phase A | GBM5208 pan motor phase A (any color) | 16 AWG |
| M0 Phase B | GBM5208 pan motor phase B | 16 AWG |
| M0 Phase C | GBM5208 pan motor phase C | 16 AWG |
| M1 Phase A | GBM5208 tilt motor phase A | 16 AWG |
| M1 Phase B | GBM5208 tilt motor phase B | 16 AWG |
| M1 Phase C | GBM5208 tilt motor phase C | 16 AWG |
| ENC0 (SPI) | AMT21 pan encoder SPI header | 24 AWG x4 |
| ENC1 (SPI) | AMT21 tilt encoder SPI header | 24 AWG x4 |
| USB-C | Jetson Orin Nano USB-A port via USB-C cable | USB cable |

Motor phase connections do not have a polarity — if the motor spins the wrong direction during setup, swap any two of the three phase wires.

### AMT21 Encoder (both units identical wiring)

| AMT21 pin | Connects to | Notes |
|---|---|---|
| 1 - GND | ODrive ENC GND | |
| 2 - 5V | ODrive ENC 5V | |
| 3 - MOSI | ODrive ENC MOSI | |
| 4 - MISO | ODrive ENC MISO | |
| 5 - CLK | ODrive ENC CLK | |
| 6 - CS | ODrive ENC CS | |

The AMT21 magnet (6mm diameter, included with encoder) mounts on the motor shaft rear face. Gap between magnet and encoder face must be 0.5-1.5 mm.

### BMI088 IMU

| BMI088 pin | Connects to | Notes |
|---|---|---|
| VIN | Jetson 3.3V header pin 1 | Do not use 5V — BMI088 is 3.3V |
| GND | Jetson GND header pin 6 | |
| SDA | Jetson I2C SDA (pin 3) | 4.7k pullup resistor to 3.3V |
| SCL | Jetson I2C SCL (pin 5) | 4.7k pullup resistor to 3.3V |

I2C address: 0x68 (SDO pin to GND) or 0x69 (SDO to 3.3V). Default in firmware config is 0x68.

### AWR1843BOOST Radar

| AWR1843 connector | Connects to | Notes |
|---|---|---|
| 5V DC input | Jetson USB-A port via USB | Powers radar from Jetson USB |
| UART1 (60-pin header pins 9,10,11) | Jetson USB-UART adapter (CH340/CP2102) | Baud 921600 — data port |
| UART2 (60-pin header pins 5,6,7) | Jetson USB-UART adapter (second) | Baud 115200 — config port |

The AWR1843BOOST can also be powered from its USB Micro-B connector directly if a dedicated 5V supply is available. The radar draws approximately 800 mA at peak.

### ELP OV2710 Camera

| Connection | Details |
|---|---|
| USB 3.0 Type-A | Plug directly into Jetson USB 3.0 port (blue port) |
| Lens mount | CS-mount — thread the varifocal lens onto the camera body |

No additional power wiring required. USB provides power and data.

---

## Ground Loops

All GND references must be connected at a single star point — the PSU negative terminal. Do not create ground loops between the ODrive GND and the Jetson GND through multiple paths. Run one dedicated GND wire from each device back to the PSU negative bus bar.

---

## Pre-Power Checklist

Before connecting to mains:

- [ ] All XT60 connectors fully seated and heat-shrink applied
- [ ] No bare wire strands touching chassis or other conductors
- [ ] ODrive VBus voltage verified at PSU terminals with multimeter (should read 24.0V +/- 0.5V)
- [ ] 3.3V rail to BMI088 verified (should read 3.28-3.35V)
- [ ] Motor phase wires not touching each other (short circuit before calibration is destructive)
- [ ] AMT21 magnets seated and gap verified at 0.5-1.5mm
- [ ] Jetson boots to login prompt before connecting radar and camera
