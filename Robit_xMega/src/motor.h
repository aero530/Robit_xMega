/*
 * motor.h
 *
 * Created: 3/17/2015 12:21:38 PM
 *  Author: Phillip
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#define		MOTOR_COUNT			4		/**< Number of motors attached */
#define		SERVO_COUNT			4		/**< Number of motors attached */
#define		STEPPER_COUNT		1		/**< Number of motors attached */

#define		FORWARD				1
#define		BACKWARD			0
#define		REVERSE				0
#define		STOP				2

/**
  @enum MOTOR_OBJECT_t motors[MOTOR_COUNT];
  @brief Contains data about motor PWM settings
 */
typedef struct {
	struct pwm_config pwm;
	port_pin_t direction_pin;
	double speed;	/**< set pwm duty cycle in the range 0-10000 */
} MOTOR_OBJECT_t;

extern	MOTOR_OBJECT_t	motors[MOTOR_COUNT];

void motor_init(MOTOR_OBJECT_t *motor, enum pwm_tc_t tc, enum pwm_channel_t channel, uint16_t freq_hz, port_pin_t pwm_pin, port_pin_t dir_pin);

void motor_set(MOTOR_OBJECT_t *motor, uint16_t speed, uint8_t direction);





typedef struct {
	struct pwm_config pwm;
	double pwm_value;	/**< set pwm duty cycle in the range 0-10000 */
	int16_t  angle;  /**< angle of servo in millidegrees range 0-18000 (deg) */
} SERVO_OBJECT_t;

extern	SERVO_OBJECT_t	servos[SERVO_COUNT];

void servo_init(SERVO_OBJECT_t *servo, enum pwm_tc_t tc, enum pwm_channel_t channel, uint16_t freq_hz, port_pin_t pwm_pin);

void servo_set(SERVO_OBJECT_t *servo, int16_t angle);

uint16_t servo_duty_cycle(uint16_t angle);




typedef struct {
	struct pwm_config pwm;
	double pwm_value;
	port_pin_t direction_pin;
	port_pin_t enable_pin;
	port_pin_t reset_pin;
	port_pin_t sleep_pin;
	port_pin_t ms1_pin;
	port_pin_t ms2_pin;
	port_pin_t ms3_pin;
} STEPPER_OBJECT_t;

extern	STEPPER_OBJECT_t	steppers[STEPPER_COUNT];

void stepper_init(STEPPER_OBJECT_t *stepper, enum pwm_tc_t tc, enum pwm_channel_t channel, uint16_t freq_hz, port_pin_t pwm_pin, port_pin_t dir_pin, port_pin_t en_pin, port_pin_t rst_pin, port_pin_t slp_pin, port_pin_t ms1_pin, port_pin_t ms2_pin, port_pin_t ms3_pin);

void stepper_step_size(STEPPER_OBJECT_t *stepper, uint8_t size);

void stepper_set(STEPPER_OBJECT_t *stepper, uint8_t direction);

void stepper_sleep(STEPPER_OBJECT_t *stepper);

void stepper_wake(STEPPER_OBJECT_t *stepper);

#endif /* MOTOR_H_ */