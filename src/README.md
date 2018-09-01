These are my modifications to the [hoverboard-firmware-hack by NiklasFauth](https://github.com/NiklasFauth/hoverboard-firmware-hack)

## the Hardware

I run this Firmware on Board with the GD32 MCU (no good image available).
The Firmware is built and flashed with PlatformIO on VisualStudio Code.

The Sensor boards where removed.
I attached a HC06 style Bluetooth Module to USART2 PBA2/PA3 (left sensor board connector).
The Input is attached via I2C to PB10/PB11 (right sensor board connector)

Steering is done by speed difference of the wheels.

---

### Telemetry
Telemtetry is sent over Bluetooth and interpreted by the Android App "Bluetooth electronics" [google play link](https://play.google.com/store/apps/details?id=com.keuwl.arduinobluetooth)

### Mode selection
The operation mode can be selected while power-on.
To select a mode press and hold the power button.
While holding the power button, the nunchuck buttons can be used to increase/decrease the mode number.
The current mode is indicated by beeps.

#### Mode 1
Default mode, max speed about half of possible speed. Input Filter is slow, so only slow changes to speed and steering are possible.
  
#### Mode 2
Full speed with Turbo. Input Filter like mode 1.

#### Mode 3
Full speed with Turbo. Input Filter is faster. I should be possible to slip the wheels.

### Control
Speed is increased and decreased by pushing the Joystick on the nunchuck forwards/backwards. When the speed is at ~80%, the turbo can be activated with button-Z. The Turbo works by weakening the magnetic field while increasing the commutation speed. Be careful with that function, the speed is dangerous. Protective wear is advised.

When Button-C is pressed, the steering should be stronger. Be careful at high speeds!