# UART_LCD
UART/RS232 Enabled controller for a basic `1602 Monochrome LCD`, in 4-bit mode
using the `CH32V003 - TSSOP-20` MCU  
Features:
* UART and RS232 Input
* Selectable Baudrate
* Backlight Control using UART or Hardware override
* 4-bit LCD Control protocol support
* Automatic Line Scroll
* Operation down to 2.7v
* LCD Contrast Charge Pump
* Advance Display Control over UART


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
The setting is checked once every 1 Second so the setting can be 
changed in realtime, allowing for live monitoring for correct setting.  
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
the `JMP_V0_GND` Jumper.

## Control Characters
`BELL    0x07` - Flashes the Backlight quickly for 3 Seconds  
`LF      0x0A` - Feeds line down, keeping Column Position  
`CR      0x0D` - Returns to the First Column on the current line  
`DC1     0x11` - Device Control 1, Turns Backlight On  
`DC2     0x12` - Device Control 2, Turns Backlight Off  
`DC3     0x13`   
`DC4     0x14`  
`CANCEL  0x18`  
`ESCAPE  0x1B` - Clears the Screen and returns to position 0,0  

`TAB` will be replaced with a `SPACE`. `DEL`, `BACKSPACE`, and any other 
Control Characters not mentioned above will be ignored.

## Pinout
```
PA1  Baud Select 1
PA2  Baud Select 2

PC0  LCD_RS  Register Select
PC1  LCD_RW  Read/Write
PC2  LCD_EN  Enable
PC3  LCD_D4  Display Data Pin 4
PC4  LCD_D5  Display Data Pin 5
PC5  LCD_D6  Display Data Pin 6
PC6  LCD_D7  Display Data Pin 7
PC7  --

PD0  Backlight Control
PD1  !!Reserved for SWIO!!
PD2  --
PD3  --
PD4  Charge Pump PWM
PD5  TX  UART Transmit (Not Connected)
PD6  RX  UART Receive
PD7  Reserved for NRST
```

## Partslist

## TODO
* UART Handling to LCD Buffer
* Control Chars handling
* Line Shift mechanism


----
ADBeta (c) 2024
