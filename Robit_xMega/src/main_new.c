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
/*
MOTOR_OBJECT_t	motors[MOTOR_COUNT];
SERVO_OBJECT_t	servos[SERVO_COUNT];
STEPPER_OBJECT_t steppers[STEPPER_COUNT];

//-----------------------------------------------------------------------------

#define JOYSTICK_HIGH		650
#define JOYSTICK_LOW		450
#define SERVO_DELTA_ANGLE	100
*/
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
	motor_init(&motors[0], PWM_TCF0, PWM_CH_A, 4000, MOTOR_0_PWM, MOTOR_0_DIR);
	motor_init(&motors[1], PWM_TCF0, PWM_CH_B, 4000, MOTOR_1_PWM, MOTOR_1_DIR);
	motor_init(&motors[2], PWM_TCF0, PWM_CH_C, 4000, MOTOR_2_PWM, MOTOR_2_DIR);
	motor_init(&motors[3], PWM_TCF0, PWM_CH_D, 4000, MOTOR_3_PWM, MOTOR_3_DIR);
		
	motor_set(&motors[0], 0, FORWARD);
	motor_set(&motors[1], 0, FORWARD);
	motor_set(&motors[2], 0, FORWARD);
	motor_set(&motors[3], 0, FORWARD);

	// Servos
	
	servo_init(&servos[0], PWM_TCD0, PWM_CH_A, 50, SERVO_0_PWM);
	servo_init(&servos[1], PWM_TCD0, PWM_CH_B, 50, SERVO_1_PWM);
	servo_init(&servos[2], PWM_TCD0, PWM_CH_C, 50, SERVO_2_PWM);
	servo_init(&servos[3], PWM_TCD0, PWM_CH_D, 50, SERVO_3_PWM);

	// Steppers
	stepper_init(&steppers[0], PWM_TCE0, PWM_CH_A, 500, STEPPER_0_PWM, STEPPER_0_DIR, STEPPER_0_EN, STEPPER_0_RST, STEPPER_0_SLP, STEPPER_0_MS1, STEPPER_0_MS2, STEPPER_0_MS3);
	stepper_step_size(&steppers[0], 4);
	stepper_set(&steppers[0], STOP);
*/
	// -------------------------------------------------------------------
	
	// -------------------------------------------------------------------
	// RF24
	rf24_begin();
	
	rf24_setPayloadSize(32);
	rf24_setPALevel(RF24_PA_MAX);
	dynamic_payloads_enabled=false;

	rf24_openReadingPipe(1,addresses[0]);
	rf24_startListening();
	// -------------------------------------------------------------------
	
	
	while (1) {
		
		delay_us(50);

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
			
			
			// Motor control needs to drive forward/reverse and turn at the same time...somewhat complicated logic needed
			/*
			if (joystick[1] > JOYSTICK_HIGH) { // Right Y
				motor_set(&motors[0], 5000, FORWARD);
				motor_set(&motors[1], 5000, FORWARD);
				motor_set(&motors[2], 5000, FORWARD);
				motor_set(&motors[3], 5000, FORWARD);
			} else if (joystick[1] < JOYSTICK_LOW) {
				motor_set(&motors[0], 5000, BACKWARD);
				motor_set(&motors[1], 5000, BACKWARD);
				motor_set(&motors[2], 5000, BACKWARD);
				motor_set(&motors[3], 5000, BACKWARD);
			} else {
				motor_set(&motors[0], 0, FORWARD);
				motor_set(&motors[1], 0, FORWARD);
				motor_set(&motors[2], 0, FORWARD);
				motor_set(&motors[3], 0, FORWARD);
			}
			
			
			if (joystick[2] > JOYSTICK_HIGH) { // Left X
				motor_set(&motors[0], 5000, FORWARD);
				motor_set(&motors[1], 5000, FORWARD);
				motor_set(&motors[2], 5000, BACKWARD);
				motor_set(&motors[3], 5000, BACKWARD);
			} else if (joystick[2] < JOYSTICK_LOW) {
				motor_set(&motors[0], 5000, FORWARD);
				motor_set(&motors[1], 5000, FORWARD);
				motor_set(&motors[2], 5000, BACKWARD);
				motor_set(&motors[3], 5000, BACKWARD);
			} else {
				motor_set(&motors[0], 0, FORWARD);
				motor_set(&motors[1], 0, FORWARD);
				motor_set(&motors[2], 0, FORWARD);
				motor_set(&motors[3], 0, FORWARD);
			}	

			
			// Arm Rotate
			if ((joystick[4] & 0b00000001) == 0) {
				stepper_set(&steppers[0], FORWARD);
			}
			if ((joystick[4]>>1 & 0b00000001) == 0) {
				stepper_set(&steppers[0], BACKWARD);
			}
			
			// Arm Forward/Back
			if ((joystick[4]>>3 & 0b00000001) == 0) {
				servo_set(&servos[0], servos[0].angle+SERVO_DELTA_ANGLE);
			}
			if ((joystick[4]>>2 & 0b00000001) == 0) {
				servo_set(&servos[0], servos[0].angle-SERVO_DELTA_ANGLE);
			}
			
			// Arm Up/Down
			if ((joystick[5]>>3 & 0b00000001) == 0) {
				servo_set(&servos[1], servos[1].angle+SERVO_DELTA_ANGLE);
			}
			if ((joystick[5]>>2 & 0b00000001) == 0) {
				servo_set(&servos[1], servos[1].angle-SERVO_DELTA_ANGLE);
			}
			
			// Gripper Open/Close
			if ((joystick[5] & 0b00000001) == 0) {
				servo_set(&servos[2], servos[2].angle+SERVO_DELTA_ANGLE);
			}
			if ((joystick[5]>>1 & 0b00000001) == 0) {
				servo_set(&servos[2], servos[2].angle-SERVO_DELTA_ANGLE);
			}
			*/
			
		}

		
	}
	
}