/******************************************************************************
* UART_LCD
* A UART Enabled Controller for a basic 1602 Monochrome LCD
* For more information see the GitHub: 
* https://github.com/ADBeta/UART_LCD
*
* Ver 0.3 07 Dec 2024 
* ADBeta (c) 2024
******************************************************************************/
#include "ch32v003fun.h"
#include "lib_gpioctrl.h"
#include "lib_uart.h"
#include "lib_lcd1602.h"

#include <stdio.h>

/*** Pin Definitions *********************************************************/
#define LCD_BL       GPIO_PD0
#define BAUD_SEL_1   GPIO_PA1
#define BAUD_SEL_2   GPIO_PA2
#define PUMP_PWM     GPIO_PD4

/*** Timing Variables ********************************************************/
#define BAUD_CHECK_MS   500
#define DISP_UPDATE_MS   50

/*** Macros ******************************************************************/
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)
#define SYSTICK_ONE_MICROSECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000000)
#define millis() (g_systick_millis)
#define micros() (SysTick->CNT / SYSTICK_ONE_MICROSECOND)

/*** Globals *****************************************************************/
// UART Ring Buffer and Receive buffer
#define UART_BUFFER_SIZE 128
static uint8_t uart_buffer[UART_BUFFER_SIZE] = {0x00};
static uint8_t recv_buffer[UART_BUFFER_SIZE] = {0x00};	


// Create a UART Configuration 
static uart_config_t uart_conf = {
	.baudrate    = UART_BAUD_4800,
	.wordlength  = UART_WORDLENGTH_8,
	.parity      = UART_PARITY_NONE,
	.stopbits    = UART_STOPBITS_ONE,
	.flowctrl    = UART_FLOWCTRL_NONE,
};

// Create LCD Device
static lcd_device_t lcd_dev = {
	.LCD_RS = GPIO_PC0,
	.LCD_RW = GPIO_PC1,
	.LCD_EN = GPIO_PC2,
	.LCD_DA = {GPIO_PC3, GPIO_PC4, GPIO_PC5, GPIO_PC6},
};

// Incremented in the SysTick IRQ once per millisecond
volatile uint32_t g_systick_millis;

static uint32_t last_baud_update_millis = 0;
static uint32_t last_disp_update_millis = 0;

/*** Forward Declaration *****************************************************/
/// @brief Initialise the SysTick interrupt to incriment every 1 millisecond
/// @param None
/// @return None
static void systick_init(void);

/// @brief SysTick IRQ
/// @param none
/// @return none
__attribute__((interrupt))
void SysTick_Handler(void);

/// @brief Initialises PWM on Timer2,         NOTE: PD4 in this case
/// @param None
/// @return None
void pwm_init(void);

/// @brief Sets the Duty Cycle PWM output
/// @param duty, 0 - 255
/// @return None
__attribute__((always_inline))
inline void pwm_set_duty(const uint32_t duty);



/*** Main ********************************************************************/
int main(void)
{
	SystemInit();

	// Backlight is output, turn off by default
	gpio_set_mode(LCD_BL, OUTPUT_10MHZ_PP);
	gpio_digital_write(LCD_BL, GPIO_LOW);
	// Selection Switches are input
	gpio_set_mode(BAUD_SEL_1, INPUT_FLOATING);
	gpio_set_mode(BAUD_SEL_2, INPUT_FLOATING);
	// Charge Pump is output in alternate function mode for PWM
	gpio_set_mode(PUMP_PWM, OUTPUT_10MHZ_PP | OUTPUT_PP_AF);

	// Wait for power to settle then init LCD
	Delay_Ms(250);
	lcd_init(&lcd_dev);

	// Start the PWM on PD4: 50% Duty, 9.9KHz
	pwm_init();
	pwm_set_duty(127);

	// Enable the systick for timing update and read of user settings
	systick_init();
	
	// Initialise the UART driver, will start populating ring buffer
	uart_init(uart_buffer, UART_BUFFER_SIZE, &uart_conf);


	lcd_send_string(&lcd_dev, "Testing");




	// TODO: Move this
	USART1->BRR = UART_BAUD_9600;

	/*** Main Loop ***/
	while(true)
	{
		/*** Baudrate Selection Switch ***/
		if(g_systick_millis - last_baud_update_millis > BAUD_CHECK_MS)
		{


			last_baud_update_millis = g_systick_millis;
		}


		/*** UART Data Parsing ***/
		if(g_systick_millis - last_disp_update_millis > DISP_UPDATE_MS)
		{
			// Read any bytes availabe in the buffer, then parse it 
			size_t recv_bytes = uart_read(recv_buffer, UART_BUFFER_SIZE);
			for(uint8_t byte = 0; byte < recv_bytes; byte++)
			{
				// Handle Control Chars








				lcd_send_char(&lcd_dev, recv_buffer[byte]);

			}



			last_disp_update_millis = g_systick_millis;
		}

	} // End of loop
} // End of main()




























/*** Functions ***************************************************************/
static void systick_init(void)
{
	// Reset any pre-existing configuration
	SysTick->CTLR = 0x0000;
	
	// Set the compare register to trigger once per millisecond
	SysTick->CMP = SYSTICK_ONE_MILLISECOND - 1;

	// Reset the Count Register, and the global millis counter to 0
	SysTick->CNT = 0x00000000;
	g_systick_millis = 0x00000000;
	
	// Set the SysTick Configuration
	// NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
	// busywait delay funtions used by ch32v003_fun.
	SysTick->CTLR |= SYSTICK_CTLR_STE   |  // Enable Counter
	                 SYSTICK_CTLR_STIE  |  // Enable Interrupts
	                 SYSTICK_CTLR_STCLK ;  // Set Clock Source to HCLK/1
	
	// Enable the SysTick IRQ
	NVIC_EnableIRQ(SysTicK_IRQn);
}


__attribute__((interrupt))
void SysTick_Handler(void)
{
	// Increment the Compare Register for the next trigger
	SysTick->CMP += SYSTICK_ONE_MILLISECOND;

	// Clear the trigger state for the next IRQ
	SysTick->SR = 0x00000000;

	// Increment the milliseconds count
	g_systick_millis++;
}


void pwm_init(void)
{
	// NOTE: Uses TIM2 Channel 1 (PD0) as the PWM Output pin
	// Change CHCTLR1 to CHCTLR2 for CH3 or CH4

	// Enable TIM2 Clock
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	
	// Reset TIM2, Inits all registers
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

	// Set Prescaler (9.9kHz)
	TIM2->PSC = 0x0012;
	// Set PWM Max Value (Autoreload Value)
	TIM2->ATRLR = 254;

	// Set the Compare Capture Register for Channel 1
	// TIM2_OC1M = 0b111 - PWM Mode 2 - Enable Preload
	TIM2->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1M_0 | TIM_OC1PE;

	// Enable auto-reload
	TIM2->CTLR1 |= TIM_ARPE;

	// Enable channel output, set polarity ACTIVE_LOW
	TIM2->CCER |= TIM_CC1E | TIM_CC1P;

	// Initialise Counter
	TIM2->SWEVGR |= TIM_UG;

	// Enable TIM2
	TIM2->CTLR1 |= TIM_CEN;
}


__attribute__((always_inline))
inline void pwm_set_duty(const uint32_t duty)
{
	// NOTE: Channel 1, change CHxCVR for channel selection
	TIM2->CH1CVR = duty;
}
