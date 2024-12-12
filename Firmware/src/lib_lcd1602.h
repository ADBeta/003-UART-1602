/******************************************************************************
* 1602 LCD Control Library
* 
* Uses lib_gpioctrl for general IO Control:
* 	https://github.com/ADBeta/CH32V003_lib_gpioctrl
*
* Work In Progress
* ADBeta (c) 2024
******************************************************************************/
#ifndef LIB_LCD1602_H
#define LIB_LCD1602_H

#include "ch32v003fun.h"
#include "lib_gpioctrl.h"

/*** Strcuts and Enums *******************************************************/
/// @brief LCD Data Struct for Pins and Device Info
/// TODO: Add 8-bit support
typedef struct {
	GPIO_PIN LCD_RS;      // Register Select
	GPIO_PIN LCD_RW;      // Read / Write
	GPIO_PIN LCD_EN;      // Enable
	GPIO_PIN LCD_DA[4];   // Data
} lcd_device_t;

/// @brief LCD Position struct
typedef struct {
	uint8_t x;
	uint8_t y;
} lcd_position_t;


/*** Low Level API Functions *************************************************/
/// @brief Initialises the LCD
/// @param device, const lcd_device_t pointer
/// @return None
/// TODO: Allow for 8-bit
/// TODO: Return values
void lcd_init(const lcd_device_t *device);

/// @brief Low level Transmits a byte to the LCD
/// @param device, LCD device pointer
/// @param uint8_t byte, input data. In 4-bit mode, only uses the 4 LSBs
/// @return None
void lcd_transmit_byte(const lcd_device_t *device, const uint8_t byte);

/// @brief Low Level Receives a byte from the LCD
/// @param device, LCD device pointer
/// @return uint8_t byte read from the LCD
uint8_t lcd_receive_byte(const lcd_device_t *device);

/*** High Level API Functions ************************************************/
/// @biref Sends a byte to the LCD in CMD mode
/// @param device, LCD device pointer
/// @param data, 8bit data to be sent
void lcd_send_cmd(const lcd_device_t *device, const uint8_t data);

/// @biref Sends a byte to the LCD in CHAR mode
/// @param device, LCD device pointer
/// @param data, 8bit data to be sent
/// @return None
void lcd_send_char(const lcd_device_t *device, const char data);

/// @brief reads the char data at the current Address, then incriments address
/// @param device, LCD Device pointer
/// @return uint8_t char data (> ASCII)
uint8_t lcd_read_char(const lcd_device_t *device);

/// @brief Sets the position of the cursor on the display
/// @param device, LCD device pointer
/// @param pos, lcd position pointer
/// @return None
void lcd_set_pos(const lcd_device_t *device, const lcd_position_t pos);

/// @brief Gets the current position of the cursor on the display
/// @param device, LCD pointer
/// @return lcd_position_t 
lcd_position_t lcd_get_pos(const lcd_device_t *device);

/// @brief Sends a null-terminated string to the LCD
/// @param device, LCD device pointer
/// @param str, string to send
/// @return None
/// NOTE: Make this only allow up to 16 Chars??
void lcd_send_string(const lcd_device_t *device, const char *str);

#endif
