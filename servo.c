/************************************************************************/
// File:			servo.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Servo driver for sensor tower
/************************************************************************/

/*  AVR includes    */
#include <avr/io.h>

/*  Custom includes    */
#include "servo.h"
#include "defines.h"
#include "server_communication.h"

/************************************************************************/
//     Values found by testing:
//     0 degrees = 1345, 90 degrees = 2990
//     Yields:
//     Step = (2990-1345)/90degrees = 18.3
// Array index contains corresponding calibrated PWM value.
/************************************************************************/
/************************************************************************/
// Array to map angle from degrees to pulse-width in order to avoid
// floating point operation. More details in declaration below
/************************************************************************/
/*
const uint16_t  DEG_TO_PWM[91] = {
	1325,1363,1382,1400,1418,1436,1455,1473,1491,1510,1528,1546,
	1564,1583,1601,1619,1637,1656,1674,1692,1711,1729,1747,1765,
	1784,1802,1820,1839,1857,1875,1893,1912,1930,1948,1966,1985,
	2003,2021,2040,2058,2076,2094,2113,2131,2149,2168,2186,2204,
	2222,2241,2259,2277,2295,2314,2332,2351,2369,2387,2405,2424,
	2442,2460,2479,2497,2515,2533,2552,2570,2588,2606,2625,2643,
	2661,2680,2698,2716,2734,2753,2771,2789,2808,2826,2844,2862,
	2881,2899,2917,2935,2954,2972,2960
};*/

const uint16_t  DEG_TO_PWM[91] = {
	1325,1343,1361,1380,1398,1416,1434,1452,1470,1489,1507,1525,
	1543,1561,1579,1598,1616,1634,1652,1670,1688,1707,1725,1743,
	1761,1779,1797,1816,1834,1852,1870,1888,1906,1925,1943,1961,
	1979,1997,2015,2034,2052,2070,2088,2106,2124,2143,2161,2179,
	2197,2215,2234,2252,2270,2288,2306,2324,2343,2361,2379,2397,
	2415,2433,2452,2470,2488,2506,2524,2542,2561,2579,2597,2615,
	2633,2651,2670,2688,2706,2724,2742,2760,2779,2797,2815,2833,
	2851,2869,2888,2906,2924,2942,2960
};
/************************************************************************/
// Initializes PWM for correct pins and timer for the servo
// Non-inverted, fast PWM, clear on compare match, 20ms period, presc 8
// Output: Port H Pin 4
/************************************************************************/
void vServo_init(uint8_t servoAngleDeg){
	/* Clear OCnA/OCnB on Compare Match, set */
	/* OCnA/OCnB at BOTTOM (non-inverting mode) */
	/* Datasheet p.155 Table 17-4 */
	TCCR4A |= (1<<COM4B1) | (0<<COM4B0);
	
	/* Waveform generation mode 14: Fast PWM */
	/* top: ICRn, Update bottom, flag set on top */
	/* Datasheet p.145 Table 17-2 */
	
	TCCR4B |= (1<<WGM43) | (1<<WGM42);
	TCCR4A |= (1<<WGM41) | (0<<WGM40);

	/* Clock select bit description: */
	/* clkI/O/8 (From prescaler) - Datasheet p.157 Table 17-6*/

	TCCR4B |= (0<<CS42) | (1<<CS41) | (0<<CS40);
	
	/* 50Hz 20ms period => 16Mhz/(8clk*50Hz) - 1 = ICR1] */
	/* Datasheet p.125 */
	ICR4 = 39999; // 49999 for 20mhz, 39 999 for 16Mhz
	
	/*Port H Pin 7 as servo PWM Output (OC4B)*/
	servoReg |= (1<<servoPin);
	
	/*  Set angle to desired start angle (usually 0)*/
	vServo_setAngle(servoAngleDeg);
}

/* Sets servo angle to a specific degree */
void vServo_setAngle(uint8_t ServoAngleDeg){
	//debug("vServo_setAngle: %i", ServoAngleDeg);
	/* Ensure feasible values */
	if (ServoAngleDeg >= 90){
		ServoAngleDeg = 90;
		/* Fetch pulse width from array and set to output */
		servoOCR = DEG_TO_PWM[ServoAngleDeg];
	}
	else if(ServoAngleDeg <= 0){
		ServoAngleDeg = 0;
		/* Fetch pulse width from array and set to output */
		servoOCR = DEG_TO_PWM[ServoAngleDeg];
	}
	else if (ServoAngleDeg>0 && ServoAngleDeg<90)
	{
		/* Fetch pulse width from array and set to output */
		servoOCR = DEG_TO_PWM[ServoAngleDeg];
	}
	else{
		//debug("Input error vServo_setAngle in servo.c ");
	}
}