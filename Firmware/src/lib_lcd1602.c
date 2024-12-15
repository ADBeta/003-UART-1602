/******************************************************************************
* 1602 LCD Control Library
* 
* Uses lib_gpioctrl for general IO Control:
* 	https://github.com/ADBeta/CH32V003_lib_gpioctrl
*
* Work In Progress
* ADBeta (c) 2024
******************************************************************************/

#include "lib_lcd1602.h"
#include "ch32v003fun.h"
#include "lib_gpioctrl.h"

#include <string.h>

#define NOP_DELAY 20

/*** Static Functions ********************************************************/
__attribute__((always_inline))
static inline void delay_nop(uint32_t count) {
	while(count--) __asm__ volatile("nop");
}

/*** Low Level API Functions *************************************************/
/// TODO: Allow for 8-bit
/// TODO: Return values
void lcd_init(const lcd_device_t *device)
{
	gpio_set_mode(device->LCD_RS, OUTPUT_10MHZ_PP);
	gpio_set_mode(device->LCD_RW, OUTPUT_10MHZ_PP);
	gpio_set_mode(device->LCD_EN, OUTPUT_10MHZ_PP);
	for(uint8_t pin = 0; pin < 4; pin++)
	{
		gpio_set_mode(device->LCD_DA[pin], OUTPUT_10MHZ_PP);
		gpio_digital_write(device->LCD_DA[pin], GPIO_LOW);
	}
	gpio_digital_write(device->LCD_EN, GPIO_HIGH);

	// Initialise the displays deviceuration
	lcd_send_cmd(device, 0x03);  // Spam 8-bit mode 3 times to force an init
	lcd_send_cmd(device, 0x03);
	lcd_send_cmd(device, 0x03);
	lcd_send_cmd(device, 0x02);  // 4-bit Mode
	lcd_send_cmd(device, 0x28);  // Init 4-bit mode
	lcd_send_cmd(device, 0x0C);  // Dispay ON, Cursor OFF
	lcd_send_cmd(device, 0x06);  // Auto-Incriment Cursor
	lcd_send_cmd(device, 0x01);  // Clear
	lcd_send_cmd(device, 0x80);  // Return to Home Position
}


uint8_t lcd_receive_byte(const lcd_device_t *device)
{
	// Set READ Mode
	gpio_digital_write(device->LCD_RW, GPIO_HIGH);

	// Set the Data Pins to INPUT
	for(uint8_t pin = 0; pin < 4; pin++)
		gpio_set_mode(device->LCD_DA[pin], INPUT_FLOATING);

	uint8_t byte = 0;

	// 4-bit mode:
	// Read the Upper Nibble
	gpio_digital_write(device->LCD_EN, GPIO_HIGH);
	delay_nop(NOP_DELAY);
	for(int8_t bit = 3; bit >= 0; bit--)
		byte |= (gpio_digital_read(device->LCD_DA[bit]) << bit);
	gpio_digital_write(device->LCD_EN, GPIO_LOW);
	byte = byte << 4;

	// Read Lower Nibble
	gpio_digital_write(device->LCD_EN, GPIO_HIGH);
	delay_nop(NOP_DELAY);
	for(int8_t bit = 3; bit >= 0; bit--)
		byte |= (gpio_digital_read(device->LCD_DA[bit]) << bit);
	gpio_digital_write(device->LCD_EN, GPIO_LOW);

	return byte;
}


void lcd_transmit_byte(const lcd_device_t *device, const uint8_t byte)
{
	// Set WRITE Mode
	gpio_digital_write(device->LCD_RW, GPIO_LOW);
	
	// Set the Data Pins to OUTPUT
	for(uint8_t pin = 0; pin < 4; pin++)
		gpio_set_mode(device->LCD_DA[pin], OUTPUT_10MHZ_PP);


	// 4-bit mode:
	// Send the Upper Nibble
	uint8_t nibble = byte >> 4;
	gpio_digital_write(device->LCD_EN, GPIO_HIGH);
	for(int8_t bit = 3; bit >= 0; bit--)
		gpio_digital_write(device->LCD_DA[bit], (nibble >> bit) & 0x01);
	delay_nop(NOP_DELAY);
	gpio_digital_write(device->LCD_EN, GPIO_LOW);

	// Send Lower Nibble
	nibble = byte & 0x0F;
	gpio_digital_write(device->LCD_EN, GPIO_HIGH);
	for(int8_t bit = 3; bit >= 0; bit--)
		gpio_digital_write(device->LCD_DA[bit], (nibble >> bit) & 0x01);
	delay_nop(NOP_DELAY);
	gpio_digital_write(device->LCD_EN, GPIO_LOW);

	// 8-bit mode TODO:

}


/*** High Level API Functions ************************************************/
void lcd_send_cmd(const lcd_device_t *device, const uint8_t data)
{
	// Set Instruction Mode
	gpio_digital_write(device->LCD_RS, GPIO_LOW);
	delay_nop(750);
	lcd_transmit_byte(device, data);

	// Read Instruction mode until BUSY_FLAG is not set
	gpio_digital_write(device->LCD_RS, GPIO_LOW);
	while((lcd_receive_byte(device) & 0x80));
}


void lcd_send_char(const lcd_device_t *device, const char data)
{
	// Set Character Mode
	gpio_digital_write(device->LCD_RS, GPIO_HIGH);
	delay_nop(750);
	lcd_transmit_byte(device, data);

	// Read Instruction mode until BUSY_FLAG is not set
	gpio_digital_write(device->LCD_RS, GPIO_LOW);
	while((lcd_receive_byte(device) & 0x80));
}


uint8_t lcd_read_char(const lcd_device_t *device)
{
	// Set Character Mode
	gpio_digital_write(device->LCD_RS, GPIO_HIGH);
	delay_nop(750);
	return lcd_receive_byte(device);
}


void lcd_set_pos(const lcd_device_t *device, lcd_position_t pos)
{
	if(pos.x > 39 || pos.y > 1) return;
	// TODO: make this neater?
	// Set DDRAM instruction is 0x80, Plus x_pos, y_pos is incriments of 0x40
	lcd_send_cmd(device, 0x80 + pos.x + (pos.y * 0x40));
}


lcd_position_t lcd_get_pos(const lcd_device_t *device)
{
	// Read the Busy_flag and address, discard busy flag
	gpio_digital_write(device->LCD_RS, GPIO_LOW);
	uint8_t addr = lcd_receive_byte(device) & 0x7F;

	// x_pos is the addr excluding 0x40 bit, y_pos is 0x00 below 0x40, 0x01 above 0x40 
	return (lcd_position_t){ .x = addr & 0x3F, .y  = addr >> 6 };
}


void lcd_send_string(const lcd_device_t *device, const char *str)
{
	size_t len = strlen(str);
	for(size_t chr = 0; chr < len; chr++)
		lcd_send_char(device, str[chr]);
}


void lcd_read_string(const lcd_device_t *device, char *str, const size_t chars)
{
	for(size_t cchar = 0; cchar < chars; cchar++)
	{
		*str = lcd_read_char(device);
		str++;
	}
	*str = '\n';
}

