/**
 * Proyek Pengganti Integrated Energy Management oleh Embedder
 * Anggota kelompok:
 * Daril Muhammad Rafan Syah - 2206083640
 * Khansa Mahira - 2206819413
 * Yasyfa Azkaa Wiwaha - 1906398465
 */

#include <asf.h>
#include <stdio.h>
#include <inttypes.h>
#include <board.h>
#include <string.h>
#include <avr/io.h>
#include "FreeRTOS\include\FreeRTOS.h"
#include "FreeRTOS\include\queue.h"
#include "FreeRTOS\include\task.h"
#include "FreeRTOS\include\timers.h"
#include "FreeRTOS\include\semphr.h"


/* USART */
#define USART_SERIAL_EXAMPLE             &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE    9600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false

// Define PWM parameters
#define PWM_PERIOD 20            // PWM period for controlling brightness
#define FULL_BRIGHTNESS 20       // Full brightness (100% duty cycle)
#define HALF_BRIGHTNESS 5        // Half brightness (25% duty cycle)

// Define ADC and current sensor parameters
#define MY_ADC ADCA              // Use ADC A
#define MY_ADC_CH ADC_CH0        // Use channel 0 for ADC
#define VCC 5.0                  // Sensor operating voltage (5V)
#define SENSITIVITY 0.185        // Sensitivity for ACS712 5A (185 mV/A)
#define V_OFFSET (VCC / 2.0)     // Zero-current voltage (2.5V for 5V supply)

/* Buzzer */
#define BUZZER_PIN PIN4_bm
#define BUZZER_TIMER TCC0


/* Define a task */
static portTASK_FUNCTION_PROTO(control_led, p_);

static void setUpSerial(void);
static void sendChar(char c);
static void sendString(char *text);
static char receiveChar();
static void receiveString(void);

static void adc_init(void);
static uint16_t adc_read(void);
static void pwm_init(void);
static void buttons_init(void);

static void sensors_init();
static float read_current(void);
static void update_brightness(uint16_t compare_value);
static void adjust_brightness(void);

static void relay_init(void);
static void relay_control(bool state);

static void buzzer_init(void);
static void tone(uint16_t frequency);

/* Define semaphore */
SemaphoreHandle_t xSemaphore;
uint16_t counter = 0;
static volatile int8_t bendera = 0;

/* USART */
static char strbuf[201];
static char reads[100];
int result = 0;
char in = 'x';

/* ADC */
static char strbufadc[128];

/* USART */
char *str1 = "atas ";
char *str2 = "bawah ";

/* USART */
static void setUpSerial(void) {
	// Baud rate selection
	// BSEL = (2000000 / (2^0 * 16*9600) -1 = 12.0208... ~ 12 -> BSCALE = 0
	// FBAUD = ( (2000000)/(2^0*16(12+1)) = 9615.384 -> mendekati lah ya
	
	USARTC0_BAUDCTRLB = 0; //memastikan BSCALE = 0
	USARTC0_BAUDCTRLA = 0x0C; // 12
	
	//USARTC0_BAUDCTRLB = 0; //Just to be sure that BSCALE is 0
	//USARTC0_BAUDCTRLA = 0xCF; // 207
	
	//Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	//8 data bits, no parity and 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	
	//Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

static void sendChar(char c) {
	
	while( !(USARTC0_STATUS & USART_DREIF_bm) ); //Wait until DATA buffer is empty
	
	USARTC0_DATA = c;
	
}

static void sendString(char *text) {
	while(*text)
	{
		sendChar(*text++);
		//usart_putchar(USART_SERIAL_EXAMPLE, *text++);
	}
}

static char receiveChar() {
	while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Wait until receive finish
	return USARTC0_DATA;
}

static void receiveString(void) {
	int i = 0;
	while(1){
		//char inp = receiveChar();
		char inp = usart_getchar(USART_SERIAL_EXAMPLE);
		if(inp=='\n') break;
		else reads[i++] = inp;
	}
	if(strcmp(str1,reads) == 0){
		gpio_set_pin_high(J2_PIN0);
		}else if(strcmp(str2,reads) == 0){
		gpio_set_pin_high(J2_PIN0);
		}else{
		gpio_set_pin_low(J2_PIN0);
	}
}

/* ADC */
static void adc_init(void) {
    struct adc_config adc_conf;
    struct adc_channel_config adcch_conf;

    // Read and modify ADC configuration
    adc_read_configuration(&MY_ADC, &adc_conf);
    adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);

    // Configure ADC settings
    adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC); // 12-bit resolution, VCC as reference
    adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);                    // Manual trigger
    adc_set_clock_rate(&adc_conf, 200000UL);                                         // Set clock rate

    // Configure channel input
    adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1); // Use PB0 (ADC0)

    // Write configuration back to ADC
    adc_write_configuration(&MY_ADC, &adc_conf);
    adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}

static uint16_t adc_read(void) {
	uint16_t result;
	adc_enable(&MY_ADC);                          // Enable ADC
	adc_start_conversion(&MY_ADC, MY_ADC_CH);     // Start conversion
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH); // Wait for result
	result = adc_get_result(&MY_ADC, MY_ADC_CH);  // Get result
	adc_disable(&MY_ADC);                         // Disable ADC to save power
	return result;
}

// Function to initialize PWM
static void pwm_init(void) {
    // Enable TCC0 for PWM
    tc_enable(&TCC0);

    // Set PC0 as output for PWM signal
    PORTC.DIRSET = PIN0_bm;

    // Set Timer/Counter Clock Prescaler to 1024
    TCC0.CTRLA = TC_CLKSEL_DIV1024_gc;

    // Configure Single Slope PWM mode and enable Compare Channel A
    TCC0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;

    // Set initial Period (PER) and Compare Value (CCA) for full brightness
    TCC0.PER = PWM_PERIOD;       // Sets the PWM frequency
    TCC0.CCA = FULL_BRIGHTNESS;  // Start with 100% brightness
}

// Function to initialize buttons (SW1 and SW2)
static void buttons_init(void) {
	// Configure PF1 (SW1) and PF2 (SW2) as input pins
	PORTF.DIRCLR = PIN1_bm | PIN2_bm;

	// Enable pull-up resistors for SW1 and SW2
	PORTF.PIN1CTRL = PORT_OPC_PULLUP_gc; // Pull-up for SW1
	PORTF.PIN2CTRL = PORT_OPC_PULLUP_gc; // Pull-up for SW2
}

/* Sensor, LED, buzzer, relay */
static void sensors_init() {
	/*
	 * Set PC0, PC1, and PC4 as input
	 * PC0 to control relay
	 * PC4 to control buzzer
	 * PC6 to control ACS712
	 */
	PORTC.DIRCLR = PIN0_bm | PIN4_bm | PIN6_bm;
}

// Function to calculate current from ADC value
static float read_current(void) {
	uint16_t adc_value = adc_read();          // Read ADC value
	float v_out = (adc_value * VCC) / 4095.0; // Convert ADC value to voltage (12-bit resolution)
	return (v_out - V_OFFSET) / SENSITIVITY;  // Convert voltage to current
}

// Function to update brightness (PWM duty cycle)
static void update_brightness(uint16_t compare_value) {
	TCC0.CCA = compare_value; // Update Compare Value for brightness
}

// Function to adjust brightness based on button input
static void adjust_brightness(void) {
	static uint16_t current_brightness = FULL_BRIGHTNESS; // Track current brightness

	if (!(PORTF.IN & PIN1_bm)) { // SW1 pressed (PF1 is LOW)
		if (current_brightness != FULL_BRIGHTNESS) { // Avoid redundant updates
			current_brightness = FULL_BRIGHTNESS;
			update_brightness(FULL_BRIGHTNESS); // Set to full brightness
		}
	}
	else if (!(PORTF.IN & PIN2_bm)) { // SW2 pressed (PF2 is LOW)
		if (current_brightness != HALF_BRIGHTNESS) { // Avoid redundant updates
			current_brightness = HALF_BRIGHTNESS;
			update_brightness(HALF_BRIGHTNESS); // Set to half brightness
		}
	}
}

/* Relay */
static void relay_init(void) {
	ioport_set_pin_dir(PIN0_bm, IOPORT_DIR_OUTPUT);
	gpio_set_pin_low(PIN0_bm); // Start with the relay deactivated.
}

static void relay_control(bool state) {
	if (state) {
		gpio_set_pin_high(PIN0_bm); // Activate relay
		} else {
		gpio_set_pin_low(PIN0_bm); // Deactivate relay
	}
}

/* Buzzer */
static void buzzer_init(void) {
	// Set the buzzer pin as output
	PORTC.DIRSET = BUZZER_PIN;

	// Configure the timer for PWM
	BUZZER_TIMER.CTRLA = TC_CLKSEL_DIV64_gc; // Clock prescaler
	BUZZER_TIMER.CTRLB = TC_WGMODE_FRQ_gc | TC0_CCBEN_bm; // Frequency mode, enable output
	BUZZER_TIMER.PER = 0; // Default period (will be set in tone function)
	BUZZER_TIMER.CCB = 0; // Default duty cycle
}

// Function to generate a tone at the specified frequency
static void tone(uint16_t frequency) {
	if (frequency == 0) {
		BUZZER_TIMER.CTRLA = 0; // Stop the timer
		PORTC.OUTCLR = BUZZER_PIN; // Turn off the buzzer
		return;
	}


	// Calculate the period for the desired frequency
	uint16_t period = F_CPU / (64 * frequency); // Adjust based on prescaler

	// Set the timer period and duty cycle
	BUZZER_TIMER.PER = period;
	BUZZER_TIMER.CCB = period / 2; // 50% duty cycle

	// Start the timer
	PORTC.OUTSET = BUZZER_PIN; // Ensure the buzzer pin is high
}


static portTASK_FUNCTION_PROTO(control_led, p_) {
	
	while (1) {
		adjust_brightness();          // Check buttons and adjust brightness
		float current = read_current(); // Read current from sensor
		printf("Current: %.2f A\n", current); // Print current reading
		snprintf(strbufadc, sizeof(strbufadc), "Current: %.2f A\n", current);
		gfx_mono_draw_string(strbuf, 0, 8, &sysfont);
		delay_ms(500);                // Delay for smoother output
	}
}


int main (void)
{
	gfx_mono_init();
	sysclk_init();
	board_init();
	pwm_init();
    buttons_init();
	adc_init();
	
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	
	/* USART */
	PORTC_OUTSET = PIN3_bm; //PC3 as TX
	PORTC_DIRSET = PIN3_bm; //TX pin as output
	
	PORTC_OUTCLR = PIN2_bm; //PC2 as RX
	PORTC_DIRCLR = PIN2_bm; //RX pin as input
	
	setUpSerial();
	
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);
	
	ioport_set_pin_dir(J2_PIN0, IOPORT_DIR_OUTPUT);
	
	/* Create the task */
	xTaskCreate(control_led, "", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);	// higher priority
	/* Semaphore */
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
	
	/* Start the task */
	gfx_mono_draw_string("Embedder", 0, 0, &sysfont);
	vTaskStartScheduler();

}
