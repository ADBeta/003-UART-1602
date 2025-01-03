# 003-UART-1602
UART/RS232 Enabled controller for a basic `1602 Monochrome LCD`, in 4-bit mode
using the `CH32V003 - TSSOP-20` MCU  
Features:
* Selectable Baudrate
* Backlight Control using UART or Hardware override
* 4-bit LCD Control protocol support
* Automatic Line Scroll
* Operation down to 2.7v
* LCD Contrast Charge Pump
* Advance Display Control over UART

<div style="display: flex; align-items: center; gap: 10px;">
  <img src="/images/Schematic.png" alt="Schematic" width="800"><br>
  <img src="/images/Front.png" alt="PCB_Front" width="400">
  <img src="/images/Back.png" alt="PCB_Back" width="400">
</div>

## Display Operation
The display will read any incoming UART or RS232 Data and print it to the 
screen in realtime - If the edge of the screen is exceeded, the display will
drop incoming data.  
When `Line Feed` is received, and the Display is already on `Line 2`, the display
will Scroll Down, so the line being printed to remains as `Line 2`  

See [Control Characters](#control-characters) for a list of control commands
that can be used for the display

## Startup Conditions
When the Display is powered on, and before it receives any UART data, the
device is in the following state:
* LCB Backlight OFF
* Display Configured and blanked

## Selectable Baudrate
Receiver Baudrate is selected via the DIP-Switches onboard.  
The setting is checked once a second so the setting can be 
changed on the fly.  
```
B1 B0  Baud
0  0   4800
0  1   9600
1  0   115200
1  1   460800
```

## Low Voltage Operation
There is a negative charge pump onboard that provides a near-rail negative
voltage, that is fed to one end of the contrast selection Potentiometer.  
These LCDs are only officially rated at 5v, so use lower voltages 
**AT  YOUR OWN RISK**.  
To disable the charge pump, do not populate the components, and short
the `JMP_V0` Jumper.  
The `Minimum Voltage` for the unit is 2.7v

## Control Characters
`BELL    0x07` - Flashes the Backlight quickly for 3 Seconds  
`LF      0x0A` - Feeds line down, keeping Column Position  
`CR      0x0D` - Returns to the First Column on the current line  
`DC1     0x11` - Device Control 1, Turns Backlight On  
`DC2     0x12` - Device Control 2, Turns Backlight Off  
`DC3     0x13` -   
`DC4     0x14` -  
`ESCAPE  0x1B` - Clears the Screen and returns to position 0,0  

Any Control Characters not mentioned above will be ignored.

## Pinout
```
PA1  Baud Select 1
PA2  Baud Select 2

PC0  ---
PC1  LCD_RS  Register Select
PC2  LCD_RW  Read/Write
PC3  LCD_EN  Enable
PC4  LCD_D4  Display Data Pin 4
PC5  LCD_D5  Display Data Pin 5
PC6  LCD_D6  Display Data Pin 6
PC7  LCD_D7  Display Data Pin 7

PD0  Backlight Control
PD1  !!Reserved for SWIO!!
PD2  --
PD3  --
PD4  Charge Pump PWM
PD5  TX  UART Transmit (Not Connected)
PD6  RX  UART Receive
PD7  Reserved for NRST
```

## MkII PCB Partslist
```
C1    0603 1uF 10V
C2    0603 1uF 10V
C3    0603 1uF 10V
R1    0603 4K7
R2    100K 5mm x 5mm Potentiometer
D1    BAT54S
Q1    AO3401 P-Ch
Q2    A03400 N-Ch
U1    CH32V003
SW1   1.27mm 2P DIP Switch
```

## TODO
* Device Contol 3 and 4 Behaviour

----
ADBeta (c) 2024 - 2025
