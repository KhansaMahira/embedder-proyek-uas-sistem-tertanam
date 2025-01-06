/**
 * Proyek Pengganti Integrated Energy Management oleh Embedder
 * Anggota kelompok:
 * Daril Muhammad Rafan Syah - 2206083640
 * Khansa Mahira - 2206819413
 * Yasyfa Azkaa Wiwaha - 1906398465
 */

#include <asf.h>
#include <stdio.h>
#include <ioport.h>
#include <board.h>
#include <inttypes.h>
#include <string.h>
#include "FreeRTOS\include\FreeRTOS.h"
#include "FreeRTOS\include\queue.h"
#include "FreeRTOS\include\task.h"
#include "FreeRTOS\include\timers.h"
#include "FreeRTOS\include\semphr.h"

// Define period values for PWM
#define PWM_ON 200       // PWM active duty cycle
#define PWM_OFF 0        // PWM off duty cycle
#define MY_ADC ADCA
#define MY_ADC_CH ADC_CH0
#define SENSITIVITY 0.185       // Sensitivity for ACS712 5A (185 mV/A)
#define V_OFFSET (VCC / 2.0)    // Zero-current voltage (2.5V for 5V supply)
#define PIN_BUZZER PIN0_bm
#define PIN_RELAY PIN1_bm          // Buzzer connected to PC1
#define FILTER_SIZE 10          // Number of samples for the moving average
#define VCC 3.3                 // Voltage supply in volts
#define ENERGY_THRESHOLD_BUZZER 1.0  // Threshold energy for buzzer (in Joules)
#define ENERGY_THRESHOLD_RELAY 6.0   // Threshold energy for relay (in Joules)
#define ENERGY_UPDATE_INTERVAL_MS 1000 // Update energy every 1 second

#define USART_SERIAL_EXAMPLE             &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE    9600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false

static char strbuf[128];
static double total_energy = 0;   // Total accumulated energy in Joules

/* Define semaphore */
static SemaphoreHandle_t xSemaphore;
static uint16_t counter = 0;
static volatile int8_t bendera = 0;

/* USART */
static char strbufusart[201];
static char reads[100];
static char in = 'x';
static char *str1 = "atas ";
static char *str2 = "bawah ";

/* Define tasks */
static portTASK_FUNCTION_PROTO(control_led, p_);

static void setUpSerial(void);
static void sendChar(char c);
static void sendString(char *text);
static char receiveChar();
static void receiveString(void);

static void relay_putus(void);
static void pwm_init(void);
static void adc_init(void);
static uint16_t adc_read(void);
static int convert_adc_to_milliamp(uint16_t adc_value);
static void calculate_energy_and_check(int current_mA);


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

static void buzzer_init(void){
	PORTC.DIRSET = PIN_BUZZER;
}

// Function to disconnect the relay
static void relay_putus(void) {
	PORTC.DIRSET = PIN_RELAY; // Configure relay control pin as output
}

// Function to initialize PWM
static void pwm_init(void) {

	// Use a different timer for PWM (e.g., TCC1)
	TCC1.CTRLA = TC_CLKSEL_DIV1024_gc; // Set prescaler
	TCC1.CTRLB = TC_WGMODE_SS_gc | TC1_CCAEN_bm; // Single Slope PWM, Enable Compare A
	TCC1.PER = 1000; // Set period for desired frequency
	TCC1.CCA = PWM_OFF; // Initially set duty cycle to 0
	PORTC.DIRSET = PIN0_bm; // Set PC0 as output
}

// Function to initialize ADC
static void adc_init(void) {
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_AREFA);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);

	adcch_set_input(&adcch_conf, J2_PIN0, ADCCH_NEG_NONE, 1);

	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
}

// Function to read ADC value
static uint16_t adc_read(void) {
	uint16_t result;

	adc_enable(&MY_ADC);                          // Enable ADC
	adc_start_conversion(&MY_ADC, MY_ADC_CH);     // Start ADC conversion
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH); // Wait for ADC to finish
	result = adc_get_result(&MY_ADC, MY_ADC_CH);  // Get ADC result
	adc_disable(&MY_ADC);                         // Disable ADC (optional for power saving)

	return result;
}

// Function to convert ADC reading to current in milliamps (mA)
static int convert_adc_to_milliamp(uint16_t adc_value) {
	int16_t adjusted_value = adc_value - 2568; // Subtract baseline (calibration value)
	int current_mA = adjusted_value * 10;      // Each 1 ADC unit corresponds to 10mA
	if (current_mA < 0) current_mA = 0;        // Ensure no negative currents
	return current_mA;
}


// Function to calculate energy and check thresholds
static void calculate_energy_and_check(int current_mA) {
	// Calculate power in Watts (current in mA to Amps)
	double power = (VCC * current_mA) / 1000.0;

	// Add energy (Power × Time, where Time is 1 second)
	total_energy += power;
	
	// Convert total_energy to string with two decimal places
	snprintf(strbufusart, sizeof(strbufusart), "%.2f", total_energy);
	sendString(strbufusart);  // Send the formatted string over USART

	// Check thresholds
	if (total_energy > ENERGY_THRESHOLD_RELAY) {
		relay_putus(); // Disconnect the load
		sendChar('0');
		TCC0.CCA = PWM_OFF; // Disable PWM
		PORTC.OUTCLR = PIN_BUZZER;
	}
	else if (total_energy > ENERGY_THRESHOLD_BUZZER) {
		sendChar('1');
		TCC0.CCA = PWM_ON; // Activate buzzer
		PORTC.OUTSET = PIN_BUZZER;
	}
}

static portTASK_FUNCTION_PROTO(control_led, p_) {
	
	while (1) {
		// Read the raw ADC value
		uint16_t adc_value = adc_read();

		// Convert ADC value to current in milliamps
		int current_mA = convert_adc_to_milliamp(adc_value);

		// Calculate energy and check thresholds
		calculate_energy_and_check(current_mA);

		// Display the current value in milliamps
		snprintf(strbuf, sizeof(strbuf), "Current: %4d mA", current_mA);
		gfx_mono_draw_string(strbuf, 0, 8, &sysfont);
		
		// Display energy in Joules on the LCD
		int display_energy = (int)(total_energy * 100); // Convert to "cents" of Joules
		snprintf(strbuf, sizeof(strbuf), "Energy: %d.%02d J", display_energy / 100, display_energy % 100);
		gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

		delay_ms(1000);  // Delay for 1 second
	}
}


int main(void) {
	// Initialize system clock, board, and peripherals
	sysclk_init();
	board_init();
	pwm_init();
	adc_init();
	gfx_mono_init();

	// Turn LCD backlight on
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	
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