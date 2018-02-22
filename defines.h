/************************************************************************/
// File:			defines.h
// Author:			Erlend Ese, NTNU Spring 2016
// Defines located in one file
//
// Note:
//  Defines regarding FreeRTOS are not here, check
//      - FreeRTOSConfig.h
//      - port.c in portable/GCC/ATmega1284
//  Not all registers are redefined 
//      - Timer/Counter Control registers
//      - Interrupt control registers
//      - ADCs
//
/************************************************************************/

#ifndef DEFINES_H_
#define DEFINES_H_

#define ROBOT_NAME "Arduino"
#define ROBOT_NAME_LENGTH 7

/************************************************************************/
/* PHYSICAL CONSTANTS - If the robot is changed these need to be changed 
 Some of these will be sent to server during the start-up-handshake   
 */

#define WHEELBASE_MM             147.0 /* Length between wheel centers  */
#define ROBOT_TOTAL_WIDTH_MM     180 /* From outer rim to outer rim   */
#define ROBOT_TOTAL_LENGTH_MM    245 /* From front to aft, total      */
#define ROBOT_AXEL_OFFSET_MM     56  /* From center of square         */
#define SENSOR_TOWER_OFFSET_X_MM 56  /* From center of square         */ 
#define SENSOR_TOWER_OFFSET_Y_MM 0   /* From center of square         */
#define SENSOR_OFFSET_RADIUS_MM  21  /* From center of tower          */
#define ROBOT_DEADLINE_MS        200 /* Interval between measurements */
#define SENSOR1_HEADING_DEG      0   /* Sensor angle relative to body */
#define SENSOR2_HEADING_DEG      90
#define SENSOR3_HEADING_DEG      180
#define SENSOR4_HEADING_DEG      270

#define WHEEL_FACTOR_MM 0.638          /* Calculated, see below */ 

/*
Wheel factor is the circumference of the wheel divided by ticks per 
rotation. Motors have a gear ratio of 48:1, and the encoder is attached
to not geared down shaft with a 8 pole magnet. This means for every full
rotation of the wheel, the encoder magnet will have 48 full rotations
with 8 tick per rotation.

-> (2*pi*39mm)/(48/1*8) = 0.6380 mm/tick - Length the robot travels per
										   divided tick

(Ticks are divided in motor.c)              						   */

/************************************************************************/
/* Program settings                                                     */
#define PERIOD_MOTOR_MS         20
#define PERIOD_ESTIMATOR_MS     40
#define PERIOD_SENSORS_MS       200
#define moveStop                0
#define moveForward             1
#define moveBackward            2
#define moveClockwise           3
#define moveCounterClockwise    4
#define moveLeftForward         5
#define moveRightForward        6
#define moveLeftBackward        7
#define moveRightBackward       8
#define F_CPU                   16000000UL

/************************************************************************/
/* MICROCONTROLLER PIN CONSTANTS                                        */
/* Distance sensor                                                      */
#define distSensPort    PORTF   /* Sensors connected to port A          */
#define distSensReg     DDRF    /* Port A register used                 */
#define distSensLeft    PINF1   /* Left sensor connected to PF1      */
#define distSensRear    PINF3   /* Backward sensor connected to PF3     */
#define distSensRight   PINF2   /* Right sensor connected to PF2        */
#define distSensFwd     PINF0   /* Forward sensor connected to PF0         */

/* Motor control                                                        */
#define motorRightOn        PINB6
#define motorRightPWM       OCR1B
#define motorRightForward   PINH6   /* Right motor, pin 1               */
#define motorRightBackward  PINH5   /* Right motor, pin 2               */

#define motorLeftOn         PINB7
#define motorLeftPWM        OCR0A
#define motorLeftForward    PINB4   /* Left motor, pin 2                */
#define motorLeftBackward   PINB5   /* Left motor, pin 1                */

/* Motor encoder                                                        */
#define encoderPinLeft  PIND3       /*   Left encoder connected to PD3  */
#define encoderPinRight PIND2       /*   Right encoder connected to PD2 */
#define leftWheelCount INT3_vect    /* Interrupt vector for PD3         */
#define rightWheelCount INT2_vect   /* Interrupt vector for PD2         */


/* Servo                                                                */
#define servoReg        DDRH        /* Port H register used             */
#define servoPin        PINH4       /* Servo connected to pin H4         */
#define servoOCR        OCR4B       /* Output compare register for PH4  */

/* LED                                                                  */
#define ledPORT		    PORTL   /*  LEDs connected to port D            */
#define ledReg          DDRL    /*  Port D register used                */
#define ledGREEN	    PINL2   /*  Green LED connected                 */
#define ledYELLOW	    PINL1   /*  Yellow LED connected                */
#define ledRED		    PINL0   /*  Green LED connected                 */

/* Communication                                                        */
#define USART_BAUDRATE 38400UL  /* If changed, also change in nRF51 code*/       
#define nRF19 PINE4             
#define nRF51_status INT4_vect  /* PinE4 has interrupt available        */
#define nRF20 PINE5				/* Unused at this time 					*/             

#define DDR_SPI DDRB            /* Data direction register for SPI      */
#define DD_MOSI PINB2           /* MOSI pin                             */
#define DD_SCK  PINB1           /* SCK pin                              */
#define DD_MISO PINB3           /* MISO pin                             */
#define IMU_SS  PINB0           /* Slave select pin for IMU             */


/************************************************************************/
/* COMMANDS                                                             */
#define TRUE 1
#define FALSE 0

/************************************************************************/
/* Macros                                                               */
#define nRFconnected !(PINE & (1<<nRF19))
#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI


#endif /* DEFINES_H_ */