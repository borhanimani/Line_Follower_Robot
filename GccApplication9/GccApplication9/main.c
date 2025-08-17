/*
 * GccApplication9.c
 *
 * Created: 8/17/2025 3:43:25 PM
 * Author : Fares Darab , Borhan Imani
 */ 

#include <stdbool.h>
#include <avr/io.h>

#define OC2_RIGHT_DDR DDRB
#define OC0_LEFT_DDR DDRD

#define OC2_RIGHT_PORT PORTB
#define OC0_LEFT_PORT PORTD

#define OC2_RIGHT_PORT_NUM 3
#define OC2_LEFT_PORT_NUM 6

#define RIGHT_MOTOR_ENABLE_DDR DDRB
#define LEFT_MOTOR_ENABLE_DDR DDRD

#define RIGHT_MOTOR_ROTATION_DDR DDRB
#define LEFT_MOTOR_ROTATION_DDR DDRB

#define RIGHT_MOTOR_ROTATION_PORT PORTB
#define LEFT_MOTOR_ROTATION_PORT PORTB

#define RIGHT_MOTOR_ROTATION_PORT_NUM_CLOCK 0
#define RIGHT_MOTOR_ROTATION_PORT_NUM_CCLOCK 1
#define LEFT_MOTOR_ROTATION_PORT_NUM_CLOCK 2
#define LEFT_MOTOR_ROTATION_PORT_NUM_CCLOCK 4

#define HIGH_SPEED 200 
#define MID_SPEED 125
#define LOW_SPEED 50 
#define STOP 0 

#define SENSOR_DDR DDRC
#define SENSOR_PORT PORTC

void init_sensor(void){
	SENSOR_DDR = 0;
}

void init_motor(void){
	OC0_LEFT_DDR |= (1 << OC2_LEFT_PORT_NUM);
	OC2_RIGHT_DDR |= (1 << OC2_RIGHT_PORT_NUM);
	RIGHT_MOTOR_ROTATION_DDR |= (1 << RIGHT_MOTOR_ROTATION_PORT_NUM_CCLOCK) | (1 << RIGHT_MOTOR_ROTATION_PORT_NUM_CLOCK);
	LEFT_MOTOR_ROTATION_DDR |= (1 << LEFT_MOTOR_ROTATION_PORT_NUM_CCLOCK) | (1 << LEFT_MOTOR_ROTATION_PORT_NUM_CLOCK);
}

void init(void){
	init_motor();
	init_motor_pulse();
	init_sensor();
}

void init_motor_pulse(){
	TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1);
	TCCR0B = (1 << CS00);
	TCCR2A = (1<<WGM21) | (1<<WGM20) | (1 << COM2A1);
	TCCR2B = (1 << CS20);
}

void set_right_motor_speed(int speed_mode){
	OCR0A = speed_mode;
}

void set_left_motor_speed(int speed_mode){
	OCR2A = speed_mode;
}

void set_right_motor_rotation(void){
	RIGHT_MOTOR_ROTATION_PORT |= (1 << RIGHT_MOTOR_ROTATION_PORT_NUM_CLOCK);
}

void set_left_motor_rotation(void){
	LEFT_MOTOR_ROTATION_PORT |= (1 << LEFT_MOTOR_ROTATION_PORT_NUM_CCLOCK);
}

void move_forward(void){
	set_left_motor_speed(HIGH_SPEED);
	set_right_motor_speed(HIGH_SPEED);
}

void turn_left_soft(void){
	set_left_motor_speed(MID_SPEED);
	set_right_motor_speed(HIGH_SPEED);
}

void turn_right_soft(void){
	set_left_motor_speed(HIGH_SPEED);
	set_right_motor_speed(MID_SPEED);
}

void turn_left_hard(void){
	set_left_motor_speed(LOW_SPEED);
	set_right_motor_speed(HIGH_SPEED);
}

void turn_right_hard(void){
	set_left_motor_speed(HIGH_SPEED);
	set_right_motor_speed(LOW_SPEED);
}

void stop(void){
	set_left_motor_speed(STOP);
	set_right_motor_speed(STOP);
}


int read_and_report_sensors(void){
	int report = SENSOR_PORT;
	return report;
}


// void decide(void){
// 
// }

// void wait(void){
// 
// }


int main(void)
{
	init();
	set_left_motor_rotation();
	set_right_motor_rotation();


    while (1) 
    {
		
    }
}

