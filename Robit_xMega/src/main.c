/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
 /**
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

#include "pwm16.h"

#include "RF24.h"
#include "RF24_config.h"
#include "nRF24L01.h"

//#include "motor.h"
//#include "motor_config.h"

//-----------------------------------------------------------------------------
// USB

static bool flag_autorize_cdc_transfer = false;

bool callback_cdc_enable(void) {
	flag_autorize_cdc_transfer = true;
	return true;
}

void callback_cdc_disable(void) {
	flag_autorize_cdc_transfer = false;
}

void usb_init(void) {
	udc_start();
}

void udi_cdc_puts (const char *send) {
	// Cycle through each character individually
	while (*send) {
		udi_cdc_putc(*send++);
	}
}

void print_value (uint16_t value) {
	char buffer[9];
	//itoa(value, buffer, 16);
	//udi_cdc_puts(buffer);
	//udi_cdc_puts("\t");
	itoa((long)value, buffer, 10);
	udi_cdc_puts(buffer);
	//udi_cdc_puts("\t");
	//itoa(value, buffer, 2);
	//udi_cdc_puts(buffer);
	//udi_cdc_puts("\n\r");
}


//-----------------------------------------------------------------------------
// rf24

uint8_t addresses[][6] = {"robot"};
uint16_t joystick[6];  // 6 element array holding Joystick readings

//-----------------------------------------------------------------------------

//#define		MOTOR_COUNT			4		/**< Number of motors attached */

//MOTOR_OBJECT_t	motors[MOTOR_COUNT];
//SERVO_OBJECT_t	servos[SERVO_COUNT];

//-----------------------------------------------------------------------------



int main (void) {
	// Insert system clock initialization code here (sysclk_init()).

	// -------------------------------------------------------------------
	// USB
	
	sysclk_init();
	
	irq_initialize_vectors();
	cpu_irq_enable();
	// -------------------------------------------------------------------
	
	board_init();
	
	// -------------------------------------------------------------------
	// USB
	
	udc_start();
	
	// -------------------------------------------------------------------
		
	// -------------------------------------------------------------------
	// Motors
	/*
	motor_init(&motors[0], PWM_TCF0, PWM_CH_A, 4000, GPIO_F0, GPIO_A0);
	motor_init(&motors[1], PWM_TCF0, PWM_CH_B, 4000, GPIO_F1, GPIO_A1);
	motor_init(&motors[2], PWM_TCF0, PWM_CH_C, 4000, GPIO_F2, GPIO_A2);
	motor_init(&motors[3], PWM_TCF0, PWM_CH_D, 4000, GPIO_F3, GPIO_A3);
		
	motor_set(&motors[0], 5000, FORWARD);
	motor_set(&motors[1], 5000, FORWARD);
	motor_set(&motors[2], 5000, FORWARD);
	motor_set(&motors[3], 5000, FORWARD);

	// Servos
	
	servo_init(&servos[0], PWM_TCD0, PWM_CH_A, 50, GPIO_D0);
	servo_init(&servos[1], PWM_TCD0, PWM_CH_B, 50, GPIO_D1);
	servo_init(&servos[2], PWM_TCD0, PWM_CH_C, 50, GPIO_D2);
	servo_init(&servos[3], PWM_TCD0, PWM_CH_D, 50, GPIO_D3);

	// Steppers

gpio_set_pin_high(STEPPER_0_DIR);
gpio_set_pin_high(STEPPER_0_EN);
gpio_set_pin_high(STEPPER_0_RST);
gpio_set_pin_high(STEPPER_0_SLP);

//var microstep = {1:[0,0,0], 2:[1,0,0], 4:[0,1,0], 8:[1,1,0], 16:[1,1,1]};
//var step_resolution = 16; // may be 1, 2, 4, 8, 16
//wpi.digitalWrite(ms1_pin,microstep[step_resolution][0]);
//wpi.digitalWrite(ms2_pin,microstep[step_resolution][1]);
//wpi.digitalWrite(ms3_pin,microstep[step_resolution][2]);

gpio_set_pin_low(STEPPER_0_MS1);
gpio_set_pin_low(STEPPER_0_MS2);
gpio_set_pin_low(STEPPER_0_MS3);

*/
	// -------------------------------------------------------------------
	
	// -------------------------------------------------------------------
	// RF24
	rf24_begin();
	
	rf24_setPayloadSize(32);
	rf24_setPALevel(RF24_PA_MAX);
	dynamic_payloads_enabled=false;

	gpio_toggle_pin(LED_D5);
	rf24_setAddressWidth(5);
	rf24_openReadingPipe(1,addresses[0]);
	rf24_startListening();
	
	// -------------------------------------------------------------------
	
	
	while (1) {
		// data comes from remote control about every 100ms
		delay_ms(25);
		
		if ( rf24_available(NULL) ) {
			gpio_toggle_pin(LED_D5);
			
			// Fetch the data payload
			rf24_read( joystick, 12 ); //ech should figure this out better

			udi_cdc_puts("Rjoy ");
			print_value(joystick[0]);
			udi_cdc_puts(" ");
			print_value(joystick[1]);
			udi_cdc_puts(" Ljoy ");
			print_value(joystick[2]);
			udi_cdc_puts(" ");
			print_value(joystick[3]);
			udi_cdc_puts(" R ");
			print_value(joystick[4] & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[4]>>1 & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[4]>>2 & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[4]>>3 & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[4]>>4 & 0b00000001);
			udi_cdc_puts(" L ");
			print_value(joystick[5] & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[5]>>1 & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[5]>>2 & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[5]>>3 & 0b00000001);
			udi_cdc_puts(" ");
			print_value(joystick[5]>>4 & 0b00000001);
			udi_cdc_puts("\r\n");
		}
		
	/*
		for (uint16_t angle_loop = 0; angle_loop < 18000; angle_loop=angle_loop + 500) {
			servo_set(&servos[0], angle_loop);
			servo_set(&servos[1], angle_loop);
			servo_set(&servos[2], angle_loop);
			servo_set(&servos[3], angle_loop);
		}
	*/	
	
	}
	
}