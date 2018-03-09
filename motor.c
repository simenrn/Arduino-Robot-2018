/************************************************************************/
// File:			motor.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Register ticks from optical encoders mounted to
//                  cogwheels
//
// Note: Motor is active low
// ISR routines in main.c
/************************************************************************/

/* AVR INCLUDES    */
#include <avr/io.h>
#include <avr/interrupt.h>

/*  Custom includes    */
#include "defines.h"
#include "motor.h"
#include "LED.h"

/* Function vMotorEncoderTickFromISR() uses global variables */
//extern volatile int16_t gLeftWheelTicks;
//extern volatile int16_t gRightWheelTicks;
/* Functions vMotorMove...() uses global variables */
//extern uint8_t gLeftWheelDirection;
//extern uint8_t gRightWheelDirection;

/************************************************************************/
// Initialize the motor encoder and control pins
// Trigger on rising edge
// Input: pins PD2 and PD3 (INT0 and INT1)
// Note: ISR Routines are in main.c
/************************************************************************/
void vMotor_init(){
    /* Initialize motor pins as output */
    DDRB |= (1<<motorLeftOn);
    DDRB |= (1<<motorLeftBackward) | (1<<motorLeftForward);
    DDRB |= (1<<motorRightOn);
    DDRH |= (1<<motorRightForward) | (1<<motorRightBackward); 
    
    /* Set motor high and direction low to ensure zero movement */
    PORTB &= ~(1<<motorLeftOn);
    PORTB &= ~(1<<motorRightOn);
    PORTH &= ~(1<<motorRightBackward) & ~(1<<motorRightForward);
    PORTB &= ~(1<<motorLeftForward) & ~(1<<motorLeftBackward);
    
    /* Initialize motor encoder pins as input */
    DDRD &= ~((1<<encoderPinRight) & (1<<encoderPinLeft));
    /* Enable pull up for encoder inputs */
    PORTD |= (1<<encoderPinLeft) | (1<<encoderPinRight);
    
    /* nRF dongle pin 19 as input pin */    
    DDRE &= ~(1<<nRF19); // dongle pin input    
    
    /* Clear interrupt enable bits to ensure no interrupts occur */
    EIMSK &= ~((1<<INT2) & (1<<INT3) & (1<<INT4));

    /* Set interrupt to trigger on rising edge for motor and any change for dongle */
    /* Datasheet p110-11 table 15-1,3 */
    EICRA |= (1<<ISC21) | (1<<ISC20) | (1<<ISC31) | (1<<ISC30);                             //MÅ KANKJSE SJEKKE UT DENNE NØYERE
    EICRB |= (1<<ISC40);
    
    /* Clear interrupt flag for INT2, 3 and 4 */
    EIFR = (1<<INTF2) | (1<<INTF3) | (1<<INTF4);

    /* Enable interrupts for INT2, 3 and 4 */
    EIMSK |= (1<<INT2) | (1<<INT3) | (1<<INT4);
    
    /* Set up PWM for left motor connected to OC0A (8bit PWM) and right motor */
	/* connected to OC1B (16bit PWM)
    /* Clear OCnA/OCnB on Compare Match, set */
    /* OCnA/OCnB at BOTTOM (non-inverting mode) */
    /* Datasheet p.132 Table 14-3 */    
    TCCR0A |= (1<<COM0A1) | (0<<COM0A0);
    TCCR1A |= (1<<COM1B1) | (0<<COM1B1);
    /* Waveform generation mode 3: Fast 8bit PWM */
    /* Top 0x00FF, Update bottom, flag set on top */
    /* Datasheet p.133 Table 14-5 */
	TCCR0B |= (0<<WGM02);
    TCCR0A |= (1<<WGM01) | (1<<WGM00);
	
	TCCR1B |= (0<<WGM13) | (1<<WGM12);
	TCCR1A |= (0<<WGM11) | (1<<WGM10);
    
    /* Clock select bit description: */
    /* clkI/O/8 (From prescaler) - Datasheet p.157 Table 17-6*/
    TCCR0B |= (0<<CS02) | (1<<CS01) | (0<<CS00);
	TCCR1B |= (0<<CS12) | (1<<CS11) | (0<<CS10); 
	
	/* Set other motorpins to normal operation mode (connected to PWM ports)	*/
	/* Datasheet p.155 Table 17-3												*/
	
	TCCR1A |= (0<<COM1A1) | (0<<COM1A0); // MC_IN1 left backwards connected to OC1A
	TCCR2A |= (0<<COM2A1) | (0<<COM2A0); // MC_IN1 left backwards connected to OC2A
	TCCR2A |= (0<<COM2B1) | (0<<COM2B0); // MC_IN1 left backwards connected to OC2B
	TCCR4A |= (0<<COM4C1) | (0<<COM4C0); // MC_IN1 left backwards connected to OC4C
	
}

void vMotorMoveLeftForward(uint8_t actuation, uint8_t *leftWheelDirection){
    motorLeftPWM = actuation;
    PORTB |= (1<<motorLeftForward);
    PORTB &= ~(1<<motorLeftBackward);
    *leftWheelDirection = motorLeftForward;
}
void vMotorMoveRightForward(uint8_t actuation, uint8_t *rightWheelDirection){
    motorRightPWM = actuation;
    PORTH |= (1<<motorRightForward);
    PORTH &= ~(1<<motorRightBackward);
    *rightWheelDirection = motorRightForward;
}

void vMotorMoveLeftBackward(uint8_t actuation, uint8_t *leftWheelDirection){
    motorLeftPWM = actuation;
    PORTB &= ~(1<<motorLeftForward);
    PORTB |= (1<<motorLeftBackward);
    *leftWheelDirection = motorLeftBackward;
}
void vMotorMoveRightBackward(uint8_t actuation, uint8_t *rightWheelDirection){
    motorRightPWM = actuation;
    PORTH &= ~(1<<motorRightForward);
    PORTH |= (1<<motorRightBackward);
    *rightWheelDirection = motorRightBackward;
}

void vMotorBrakeLeft(){
	motorLeftPWM = 0;
    //motorLeftPWM = 255;											MIDLERTIDIG ENDRING: MÅ KANSKJE FIKSE DENNE IGJEN NÅR ROBOTEN FUNGERER. NÅ BREMSER DEN IKKE
    //PORTE |= (1<<motorLeftOn);
    PORTB &= ~(1<<motorLeftForward);
    PORTB &= ~(1<<motorLeftBackward);
    /* Do not set any direction to motor here, it can overshoot */
}

void vMotorGlideLeft(){
    motorLeftPWM = 0;
    //PORTE &= ~(1<<motorLeftOn);									
    PORTB &= ~(1<<motorLeftForward);
    PORTB &= ~(1<<motorLeftBackward);
}

void vMotorBrakeRight(){
	motorRightPWM = 0;
    //motorRightPWM = 255;											MIDLERTIDIG ENDRING:MÅ KANSKJE FIKSE DENNE IGJEN NÅR ROBOTEN FUNGERER. NÅ BREMSER DEN IKKE
    //PORTH |= (1<<motorRightOn);
    PORTH &= ~(1<<motorRightForward);
    PORTH &= ~(1<<motorRightBackward);
    /* Do not set any direction to motor here, it can overshoot */
}

void vMotorGlideRight(){
    motorRightPWM = 0;
    //PORTH &= ~(1<<motorRightOn);
    PORTH &= ~(1<<motorRightForward);
    PORTH &= ~(1<<motorRightBackward);
}

/* Switch for robot movement to abstract the logic away from main */

void vMotorMovementSwitch(int16_t leftSpeed, int16_t rightSpeed, uint8_t *leftWheelDirection, uint8_t *rightWheelDirection){
    if (leftSpeed > 0){
		vMotorMoveLeftForward(leftSpeed, leftWheelDirection);
    }else if(leftSpeed < 0){
		vMotorMoveLeftBackward(-leftSpeed,leftWheelDirection);
    }else{
		vMotorBrakeLeft();
	}
	
	if (rightSpeed > 0) {
		vMotorMoveRightForward(rightSpeed,rightWheelDirection);
	}else if (rightSpeed < 0) {
		vMotorMoveRightBackward(-rightSpeed,rightWheelDirection);
	}else {
		vMotorBrakeRight();
	}
	
	
	
	
	
	/*
	switch (movement){
        case moveForward:{
            if (tmp_leftWheelTicks > tmp_rightWheelTicks){
                vMotorBrakeLeft();
                vMotorMoveRightForward(rightOutput, rightWheelDirection);
            }
            else if (tmp_rightWheelTicks > tmp_leftWheelTicks){
                vMotorBrakeRight();
                vMotorMoveLeftForward(leftOutput, leftWheelDirection);
            }
            else{
                vMotorMoveLeftForward(leftOutput, leftWheelDirection);
                vMotorMoveRightForward(rightOutput, rightWheelDirection);
            }                
            break;
        }
        case moveBackward:{
                if (tmp_leftWheelTicks < tmp_rightWheelTicks){
                    vMotorBrakeLeft();
                    vMotorMoveRightBackward(rightOutput, rightWheelDirection);
                }
                else if (tmp_rightWheelTicks < tmp_leftWheelTicks){
                    vMotorBrakeRight();
                    vMotorMoveLeftBackward(leftOutput, leftWheelDirection);
                }
                else{
                vMotorMoveLeftBackward(leftOutput, leftWheelDirection);
                vMotorMoveRightBackward(rightOutput, rightWheelDirection);
                }                
            break;
        }
        case moveClockwise:{
            if(-tmp_rightWheelTicks > tmp_leftWheelTicks){
                vMotorBrakeRight();
                vMotorMoveLeftForward(leftOutput, leftWheelDirection);
            }
            else if (tmp_leftWheelTicks > -tmp_rightWheelTicks){
                vMotorBrakeLeft();
                vMotorMoveRightBackward(rightOutput, rightWheelDirection);
            }
            else{
                vMotorMoveLeftForward(leftOutput, leftWheelDirection);
                vMotorMoveRightBackward(rightOutput, rightWheelDirection);
            }                
            break;
        }
        case moveCounterClockwise:{
            if(-tmp_leftWheelTicks > tmp_rightWheelTicks){
                vMotorBrakeLeft();
                vMotorMoveRightForward(rightOutput, rightWheelDirection);
            }
            else if (tmp_rightWheelTicks > -tmp_leftWheelTicks){
                vMotorBrakeRight();
                vMotorMoveLeftBackward(leftOutput, leftWheelDirection);
            }
            else {
                vMotorMoveLeftBackward(leftOutput, leftWheelDirection);
                vMotorMoveRightForward(rightOutput, rightWheelDirection);
            }                
            break;
        }
        case moveRightForward:{
            vMotorMoveRightForward(rightOutput, rightWheelDirection);
            vMotorBrakeLeft();
            break;
        }
        case moveLeftForward:{
            vMotorMoveLeftForward(leftOutput, leftWheelDirection);
            vMotorBrakeRight();
            break;
        }
        case moveRightBackward:{
            vMotorMoveRightBackward(rightOutput, rightWheelDirection);
            vMotorBrakeLeft();
            break;
        }
        case moveLeftBackward:{
            vMotorMoveLeftBackward(leftOutput, leftWheelDirection);
            vMotorBrakeRight();
            break;
        }
        default:{
            vMotorBrakeLeft();
            vMotorBrakeRight();
            break;
        }
        
    }
	*/
	
	
	
	

}

/* Handle ISR ticks from encoder, Please note that we are losing accuracy here due to division */
void vMotorEncoderLeftTickFromISR(uint8_t wheelDirection, int16_t *leftWheelTicks, uint8_t leftEncoderTicks){
    switch (wheelDirection){
        case motorLeftForward:{
            *leftWheelTicks += leftEncoderTicks / 2;
            break;
        }
        case  motorLeftBackward:{
            *leftWheelTicks -= leftEncoderTicks / 2;
            break;
        }
        default:
        // We have a count when the robot is supposedly not moving.
        break;
    }
}
void vMotorEncoderRightTickFromISR(uint8_t wheelDirection, int16_t *rightWheelTicks, uint8_t rightEncoderTicks){
    switch (wheelDirection){
        case motorRightForward:{
            *rightWheelTicks += rightEncoderTicks / 2;
            break;
        }
        case  motorRightBackward:{
            *rightWheelTicks -= rightEncoderTicks / 2;
            break;
        }
        default:
        // We have a count when the robot is supposedly not moving.
        break;
    }
}

