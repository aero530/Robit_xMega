/*
 * motor.c
 *
 * Created: 3/17/2015 12:19:37 PM
 *  Author: Phillip
 */ 
#include "asf.h"
#include "motor.h"
#include "motor_config.h"
#include "pwm16.h"

void motor_init(MOTOR_OBJECT_t *motor, enum pwm_tc_t tc, enum pwm_channel_t channel, uint16_t freq_hz, port_pin_t pwm_pin, port_pin_t dir_pin) {

	ioport_configure_pin(dir_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH );
	ioport_configure_pin(pwm_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );

	// Set up PWM
	// motor is a pointer to a MOTOR_OBJECT_t structure.
	// motor->pwm returns the actual pwm value (referenced by the pointer motor)
	// &(motor->pwm) makes a pointer to the pwm value (itself a structre) which is referenced by the pointer called motor
	pwm_init(&(motor->pwm), tc, channel, freq_hz);

	motor->direction_pin = dir_pin;

	// Initialize the motor speed to zero (in case the pwm get started by some means other than motor_set)
	motor->speed = 0;
	
	// Set motor direction to high (forward)
	gpio_set_pin_high(motor->direction_pin);
	
	// Start PWM
	pwm_start16(&(motor->pwm), motor->speed);
	
};

void motor_set(MOTOR_OBJECT_t *motor, uint16_t speed, uint8_t direction) {

	// Set motor direction
	if (direction == FORWARD) {
		gpio_set_pin_high(motor->direction_pin);
	} else {
		gpio_set_pin_low(motor->direction_pin);
	}
	
	// Update stored value (for reference)
	if (speed<=0) {
		motor->speed=0;
	} else if (speed>10000) {
		motor->speed=10000;
	} else {
		motor->speed = speed;	
	}
	
	// Set PWM
	pwm_set_duty_cycle16(&(motor->pwm), motor->speed);
};




void servo_init(SERVO_OBJECT_t *servo, enum pwm_tc_t tc, enum pwm_channel_t channel, uint16_t freq_hz, port_pin_t pwm_pin) {

	ioport_configure_pin(pwm_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );

	// Set up PWM
	// motor is a pointer to a MOTOR_OBJECT_t structure.
	// motor->pwm returns the actual pwm value (referenced by the pointer motor)
	// &(motor->pwm) makes a pointer to the pwm value (itself a structre) which is referenced by the pointer called motor
	pwm_init(&(servo->pwm), tc, channel, freq_hz);
	
	// Initialize the motor speed to zero (in case the pwm get started by some means other than motor_set)
	servo->angle = 9000;
	
	// Start PWM
	pwm_start16(&(servo->pwm), servo_duty_cycle(servo->angle));
	
};

void servo_set(SERVO_OBJECT_t *servo, int16_t angle) {
	
	// Update stored value (for reference)
	if (angle<=0) {
		servo->angle=0;
	} else if (angle>18000) {
		servo->angle=18000;
	} else {
		servo->angle=angle;
	}
	
	/* 
	0deg = 1ms
	90deg = 1.5ms
	180deg = 2ms
	*/
	
	// Servo
	// Period = 20ms => 50Hz
	// Min 1ms pulse width = 0deg   => 1ms/20ms =>  5% => 500
	// Max 2ms pulse width = 180deg => 2ms/20ms => 10% => 1000
	
	// 3% = -90deg
	// 15% = 90deg
	
	// Set PWM
	pwm_set_duty_cycle16(&(servo->pwm), servo_duty_cycle(servo->angle));
};

uint16_t servo_duty_cycle(uint16_t angle) {
	uint16_t duty_cycle=0;
	
	//duty_cycle = 500 / 18000 * angle_millideg + 500
	
	duty_cycle = (uint16_t)(angle / 36) + 500;
	return duty_cycle;
};






void stepper_init(STEPPER_OBJECT_t *stepper, enum pwm_tc_t tc, enum pwm_channel_t channel, uint16_t freq_hz, port_pin_t pwm_pin, port_pin_t dir_pin, port_pin_t en_pin, port_pin_t rst_pin, port_pin_t slp_pin, port_pin_t ms1_pin, port_pin_t ms2_pin, port_pin_t ms3_pin) {
	
	ioport_configure_pin(dir_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH );
	ioport_configure_pin(pwm_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );
	ioport_configure_pin(en_pin,  IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH ); // high disables output
	ioport_configure_pin(rst_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH ); // high enables device, low puts everything in known state
	ioport_configure_pin(slp_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH ); // low puts device into sleep mode disabling outputs and some internal circuits
	ioport_configure_pin(ms1_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW ); // ms1,2,3 low sets device to full step
	ioport_configure_pin(ms2_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );
	ioport_configure_pin(ms3_pin, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW );


	// Set up PWM
	// motor is a pointer to a MOTOR_OBJECT_t structure.
	// motor->pwm returns the actual pwm value (referenced by the pointer motor)
	// &(motor->pwm) makes a pointer to the pwm value (itself a structre) which is referenced by the pointer called motor
	pwm_init(&(stepper->pwm), tc, channel, freq_hz);

	stepper->direction_pin = dir_pin;
	stepper->enable_pin = en_pin;
	stepper->reset_pin = rst_pin;
	stepper->sleep_pin = slp_pin;
	stepper->ms1_pin = ms1_pin;
	stepper->ms2_pin = ms2_pin;
	stepper->ms3_pin = ms3_pin;
	
	// Start PWM with 50% duty cycle
	pwm_start(&(stepper->pwm), 50);
	
};

void stepper_step_size(STEPPER_OBJECT_t *stepper, uint8_t size) {
	
	// Size can be 1, 2, 4, 8, 16
	
	//MS1	MS2		MS3		Microstep Resolution
	//Low	Low		Low		Full step
	//High	Low		Low		Half step
	//Low	High	Low		Quarter step
	//High	High	Low		Eighth step
	//High	High	High	Sixteenth step
	
	switch (size) {
		case 1 :
		gpio_set_pin_low(stepper->ms1_pin);
		gpio_set_pin_low(stepper->ms2_pin);
		gpio_set_pin_low(stepper->ms3_pin);
		break;
		case 2 :
		gpio_set_pin_high(stepper->ms1_pin);
		gpio_set_pin_low(stepper->ms2_pin);
		gpio_set_pin_low(stepper->ms3_pin);
		break;
		case 4 :
		gpio_set_pin_low(stepper->ms1_pin);
		gpio_set_pin_high(stepper->ms2_pin);
		gpio_set_pin_low(stepper->ms3_pin);
		break;
		case 8 :
		gpio_set_pin_high(stepper->ms1_pin);
		gpio_set_pin_high(stepper->ms2_pin);
		gpio_set_pin_low(stepper->ms3_pin);
		break;
		case 16 :
		gpio_set_pin_high(stepper->ms1_pin);
		gpio_set_pin_high(stepper->ms2_pin);
		gpio_set_pin_high(stepper->ms3_pin);
		break;
		default : // default to full step
		gpio_set_pin_low(stepper->ms1_pin);
		gpio_set_pin_low(stepper->ms2_pin);
		gpio_set_pin_low(stepper->ms3_pin);
		break;
	}
};

void stepper_set(STEPPER_OBJECT_t *stepper, uint8_t direction) {


	if (direction == STOP) {
		gpio_set_pin_high(stepper->enable_pin);	// high disables output
		stepper_sleep(stepper);
	} else {
		// Make sure the device is awake
		stepper_wake(stepper);

		if (direction == FORWARD) {
			gpio_set_pin_high(stepper->direction_pin);
		} else {
			gpio_set_pin_low(stepper->direction_pin);
		};
		gpio_set_pin_low(stepper->enable_pin); // low enables output
	}

};



void stepper_sleep(STEPPER_OBJECT_t *stepper) {
	gpio_set_pin_low(stepper->sleep_pin);
};
	

void stepper_wake(STEPPER_OBJECT_t *stepper) {
	gpio_set_pin_high(stepper->sleep_pin);
};
	