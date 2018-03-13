/************************************************************************/
// File:			main.c
// Author:			Erlend Ese, NTNU Spring 2016
//                  Credit is given where credit is due.
// Purpose:
// AVR Robot with FreeRTOS implementation in the collaborating robots
// project.
// In FreeRTOS each thread of execution is known as tasks
// and are written as C functions
//
// MCU: Arduino Mega with ATmega2560, programmed with USB
//
// TASKS IMPLEMENTED
// Communication:               vMainCommunicationTask
// Sensors:                     vMainSensorTowerTask
// Motor control                vMainMovementTask
// Robot control:               vMainPoseControllerTask
// Position estimator:          vMainPoseEstimatorTask
// Stack overflow handling:     vApplicationStackOverflowHook
//
// See FreeRTOSConfig.h for scheduler settings
// See defines.h for all definitions
//
// GLOBAL VARIABLES:
// See line 84
//
// HARDWARE SETUP
//  Servo pin:  Port B pin 5
//  Sensor pins: Port F pin 0 - 4
//  Motor pins: Port C, pin 5 & 7 and Port H pin 3. Port A pin 2 & 4 and Port E pin 3.
//  Encoder pins: Port D pin 2 & 3
//
// TIMERS USED:
//  Timer0  motorRightPWM   (OCR0B)
//  Timer0  motorLeftPWM    (OCR0A)
//  Timer1  ServoPWM        (OCR1A)
//  Timer3 FreeRTOS time slicer
//
// Interrupt routines located after main function
/************************************************************************/

/* KERNEL INCLUDES */
#include "FreeRTOS.h" /* Must come first */
#include "task.h"     /* RTOS task related API prototypes */
#include "semphr.h"   /* Semaphore related API prototypes */
#include "queue.h"    /* RTOS queue related API prototypes */

/* AVR INCLUDES    */
#include <stdlib.h>             // For itoa();
#include <string.h>             // For stringstuff
#include <util/atomic.h>        // For atomic operation
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

/* Semaphore handles */
SemaphoreHandle_t xScanLock;
SemaphoreHandle_t xPoseMutex;
SemaphoreHandle_t xUartMutex;
SemaphoreHandle_t xTickMutex;
SemaphoreHandle_t xControllerBSem;
SemaphoreHandle_t xCommandReadyBSem;

/* Queues */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t actuationQ = 0;

/* CUSTOM LIBRARIES    */
#include "defines.h"
#include "LED.h"
#include "servo.h"
#include "motor.h"
#include "distSens_g2d12.h"
#include "usart.h"
#include "spi.h"
#include "twi_master.h"
#include "imu_LSM6DS3.h"
#include "com_HMC5883L.h"
#include "functions.h"
#include "arq.h"
#include "simple_protocol.h"
#include "network.h"
#include "server_communication.h"

/* GLOBAL VARIABLES */
// To store ticks from encoder, changed in ISR and motor controller
volatile uint8_t gISR_rightWheelTicks = 0;
volatile uint8_t gISR_leftWheelTicks = 0;

// Flag to indicate connection status. Interrupt can change handshook status
volatile uint8_t gHandshook = FALSE;
volatile uint8_t gPaused = FALSE;

message_t message_in;

// Global robot pose
float gTheta_hat = 0;
int16_t gX_hat = 0;
int16_t gY_hat = 0;

// Global encoder tick values, could probably be replaced by a queue
volatile int16_t gRightWheelTicks = 0;
volatile int16_t gLeftWheelTicks = 0;

/* STRUCTURE */
struct sPolar{
    float heading;
    int16_t distance;
};

struct sCartesian{
	float x;
	float y;
};

/*#define DEBUG*/

#ifdef DEBUG
#warning DEBUG IS ACTIVE
#endif

/*#define tictoc*/
#ifdef tictoc
    // Pin for timing tasks, use tic/toc - PINH5 (Arduino 8) is available
    #define usetictoc DDRH |= (1<<PINH5)
    #define tic PORTH |= (1<<PINH5)
    #define toc PORTH &= ~(1<<PINH5)
#endif

/*  Communication task */
/*  Communication task */
void vMainCommunicationTask( void *pvParameters ){
	// Setup for the communication task
	struct sPolar Setpoint = {0}; // Struct for setpoints from server

	message_t command_in; // Buffer for recieved messages

	server_communication_init();
	if(xTaskCreate(vARQTask, "ARQ", 200, NULL, 3, NULL) != pdPASS) {
		vLED_singleHigh(ledRED);
	}
	uint8_t success = 0;
	
	while(!success) {
		success = server_connect();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		vLED_toggle(ledGREEN);
	}
	
	send_handshake();
	
	while(1){
		if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY) == pdTRUE){
			// We have a new command from the server, copy it to the memory
			vTaskSuspendAll ();       // Temporarily disable context switching
			taskENTER_CRITICAL();
			command_in = message_in;
			taskEXIT_CRITICAL();
			xTaskResumeAll ();      // Enable context switching
			debug("Message received: \n");
			debug("Orientation: %i", command_in.message.order.orientation);
			debug("Distance: %i", command_in.message.order.distance);
			switch(command_in.type){
				case TYPE_CONFIRM:
					taskENTER_CRITICAL();
					gHandshook = TRUE; // Set start flag true
					taskEXIT_CRITICAL();

					break;
					case TYPE_PING:
					send_ping_response();
					break;
				case TYPE_ORDER:
					Setpoint.heading = command_in.message.order.orientation;
					Setpoint.distance = command_in.message.order.distance;
					// Ensure max values are not exceeded
					if (Setpoint.distance > 320){
						Setpoint.distance = 320;
					}
					else if (Setpoint.distance < -320){
						Setpoint.distance = -320;
					}
					Setpoint.heading *= DEG2RAD; // Convert received set point to radians
					vFunc_Inf2pi(&Setpoint.heading);
				
					/* Relay new coordinates to position controller */
					xQueueSend(poseControllerQ, &Setpoint, 100);
					break;
				case TYPE_PAUSE:
					// Stop sending update messages
					taskENTER_CRITICAL();
					gPaused = TRUE;
					taskEXIT_CRITICAL();
					// Stop controller
					Setpoint.distance = 0;
					Setpoint.heading = 0;
					xQueueSend(poseControllerQ, &Setpoint, 100);
					break;
				case TYPE_UNPAUSE:
					taskENTER_CRITICAL();
					gPaused = FALSE;
					taskEXIT_CRITICAL();
					break;
				case TYPE_FINISH:
					taskENTER_CRITICAL();
					gHandshook = FALSE;
					taskEXIT_CRITICAL();
					break;
			}
			// Command is processed
		} // if (xCommandReady) end
	}// While(1) end
}// vMainComtask end

/*  Sensor tower task */
void vMainSensorTowerTask( void *pvParameters){
    /* Task init */
    #ifdef DEBUG
        printf("Tower OK\n");
    #endif 
        
    float thetahat = 0;
    int16_t xhat = 0;
    int16_t yhat = 0;
    
    uint8_t rotationDirection = moveCounterClockwise;
    uint8_t servoStep = 0;
    uint8_t servoResolution = 1;
    uint8_t robotMovement = moveStop;
    
    uint8_t idleCounter = 0;
    int16_t previous_left = 0;
    int16_t previous_right = 0;
    // Initialize the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    
    while(1){
        // Loop
        if ((gHandshook == TRUE) && (gPaused == FALSE)){
            // xLastWakeTime variable with the current time.
            xLastWakeTime = xTaskGetTickCount();
            // Set scanning resolution depending on which movement the robot is executing.
            if (xQueueReceive(scanStatusQ, &robotMovement,150 / portTICK_PERIOD_MS) == pdTRUE){
                // Set servo step length according to movement, 
                // Note that the iterations are skipped while robot is rotating (see further downbelow)
                switch (robotMovement)
                {
                    case moveStop:
                        servoStep *= servoResolution;
                        servoResolution = 1;
                        idleCounter = 1;
                    break;
                    case moveForward:
                    case moveBackward:
                        servoResolution = 5;
                        servoStep /= servoResolution;
                        idleCounter = 0;
                    break;
                    case moveClockwise:
                    case moveCounterClockwise:
                        // Iterations are frozen while rotating, see further down
                        idleCounter = 0;
                    break;
                    default:
                        idleCounter = 0;
                    break;
                }
            }
            vServo_setAngle(servoStep*servoResolution);
            vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_PERIOD_MS); // Wait total of 200 ms for servo to reach set point
            
            uint8_t forwardSensor = ui8DistSens_readCM(distSensFwd);
            uint8_t leftSensor = ui8DistSens_readCM(distSensLeft);
            uint8_t rearSensor = ui8DistSens_readCM(distSensRear);
            uint8_t rightSensor = ui8DistSens_readCM(distSensRight);
            
            xSemaphoreTake(xPoseMutex,40 / portTICK_PERIOD_MS);
                thetahat = gTheta_hat;
                xhat = gX_hat;
                yhat = gY_hat;
            xSemaphoreGive(xPoseMutex);
            
            // Experimental
            if ((idleCounter > 10) && (robotMovement == moveStop)){
                // If the robot stands idle for 1 second, send 'status:idle' in case the server missed it.
                send_idle();
                idleCounter = 1;
            }
            else if ((idleCounter >= 1) && (robotMovement == moveStop)){
                idleCounter++;
            }             

            // Send updates to server
			
			//[Commented out to decrease messages, ]
            send_update(xhat/10,yhat/10,thetahat*RAD2DEG,servoStep*servoResolution,forwardSensor,leftSensor,rearSensor,rightSensor);
            
            
            // Low level anti collision
            uint8_t objectX;
            if ((servoStep*servoResolution) <= 30) objectX = forwardSensor;// * cos(servoStep*5);
            else if((servoStep*servoResolution) >= 60) objectX = rightSensor;// * cos(270 + servoStep*5);
            else objectX = 0;
            

			
            if ((objectX > 0) && (objectX < 20)){
                // Stop controller
                struct sPolar Setpoint = {0, 0};
                xQueueSend(poseControllerQ, &Setpoint, 100);
				
				
            }
            
            // Iterate in a increasing/decreasing manner and depending on the robots movement
            if ((servoStep*servoResolution <= 90) && (rotationDirection == moveCounterClockwise) && (robotMovement < moveClockwise)){
                servoStep++;
            }
            else if ((servoStep*servoResolution > 0) && (rotationDirection == moveClockwise) && (robotMovement < moveClockwise)){
                servoStep --;
            }
            
            if ((servoStep*servoResolution >= 90) && (rotationDirection == moveCounterClockwise)){
                rotationDirection = moveClockwise;
            }
            else if ((servoStep*servoResolution <= 0) && (rotationDirection == moveClockwise)){
                rotationDirection = moveCounterClockwise;
            }    
        }
		
        else{ // Disconnected or unconfirmed
            vServo_setAngle(0);
            // Reset servo incrementation
            rotationDirection = moveCounterClockwise;
            servoStep = 0;
            idleCounter = 0;
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }// While end
}

/*  Calculates new settings for the movement task */
void vMainPoseControllerTask( void *pvParameters ){
    #ifdef DEBUG
        printf("PoseController OK\n");
        uint8_t tellar = 0;
    #endif
    /* Task init */    
    struct sPolar Setpoint = {0}; // Updates from server
    struct sCartesian Error = {0}; // Error values
    struct sPolar oldVal = {0};
    struct sPolar referenceModel = {0};
	float radiusEpsilon = 15; //[mm]The acceptable radius from goal for completion
	uint8_t lastMovement = 0;
	
	uint8_t maxRotateActuation = 75; //The max speed the motors will run at during rotation max is 255
	uint8_t maxDriveActuation = 100; //The max speed the motors will run at during drive max is 255
	uint8_t currentDriveActuation = maxRotateActuation;
	
	/* Controller variables for tuning */
	float rotateThreshold = 0.5235; //[rad] The threshold at which the robot will go from driving to rotation. Equals 10 degrees
	float driveThreshold = 0.0174; // [rad ]The threshold at which the robot will go from rotation to driving. In degrees.
	float driveKp = 600; //Proportional gain for theta control during drive
	float driveKi = 10;//Integral gain for theta during drive
	float speedDecreaseThreshold = 500; //[mm] Distance from goal where the robot will decrease its speed inverse proportionally
	
	/* Current position variables */	
	float thetahat = 0;
	int16_t xhat = 0;
	int16_t yhat = 0;
	
	/* Goal variables*/
	float distance = 0;
	float thetaDiff = 0;
	float xTargt = 0;
	float yTargt = 0;
	
	float prevLeftActuation = 0;
	float prevRightActtion = 0;
	float leftIntError = 0;
	float rightIntError = 0;
	
	uint8_t doneTurning = TRUE;
	
	
	int16_t leftWheelTicks = 0;
	int16_t rightWheelTicks = 0;
	
	uint8_t leftEncoderVal = 0;
	uint8_t rightEncoderVal = 0;
	
	uint8_t gLeftWheelDirection = 0;
	uint8_t gRightWheelDirection = 0;
	
	uint8_t idleSendt = FALSE;
	
	
	
	
	
      
	while(1){
		// Checking if server is ready
		if (gHandshook){
			
			
			
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				leftEncoderVal = gISR_leftWheelTicks;
				gISR_leftWheelTicks = 0;
				rightEncoderVal = gISR_rightWheelTicks;
				gISR_rightWheelTicks = 0;
			}
			
			
			
			
			
			vMotorEncoderLeftTickFromISR(gLeftWheelDirection, &leftWheelTicks, leftEncoderVal);
			vMotorEncoderRightTickFromISR(gRightWheelDirection, &rightWheelTicks, rightEncoderVal);
			
			xSemaphoreTake(xTickMutex,1 / portTICK_PERIOD_MS);
			gLeftWheelTicks = leftWheelTicks;
			gRightWheelTicks = rightWheelTicks;
			xSemaphoreGive(xTickMutex);
			
			
			
			
			
			
			if (xSemaphoreTake(xControllerBSem, portMAX_DELAY) == pdTRUE){    // Wait for synchronization from estimator
				// Get robot pose
				xSemaphoreTake(xPoseMutex,portMAX_DELAY);
					thetahat = gTheta_hat;
					xhat = gX_hat;
					yhat = gY_hat;
				xSemaphoreGive(xPoseMutex);
				
				// Check if a new update is received
				if (xQueueReceive(poseControllerQ, &Setpoint, 0) == pdTRUE){
					xQueueReceive(poseControllerQ, &Setpoint, 20 / portTICK_PERIOD_MS); // Receive theta and radius set points from com task, wait for 20ms if necessary
					Setpoint.distance = Setpoint.distance*10; //Distance is received in cm, convert to mm for continuity
					
					
					
					xTargt = xhat + Setpoint.distance*cos(Setpoint.heading + thetahat);
					yTargt = yhat + Setpoint.distance*sin(Setpoint.heading + thetahat);
					
					
				
				}
				
				distance = (float)sqrt((xTargt-xhat)*(xTargt-xhat) + (yTargt-yhat)*(yTargt-yhat));
				
				//Simple speed controller as the robot nears the target
				if (distance < speedDecreaseThreshold){
					currentDriveActuation = (maxDriveActuation - 0.32*maxDriveActuation)*distance/speedDecreaseThreshold + 0.32*maxDriveActuation; //Reverse proportional + a constant so it reaches. 
				}else{
					currentDriveActuation = maxDriveActuation;
				}
				
				
				
				if(distance > radiusEpsilon){//Not close enough to target
					
					
					idleSendt = FALSE;
					
					float xdiff = xTargt - xhat;
					float ydiff = yTargt - yhat;
					float thetaTargt = atan2(ydiff,xdiff); //atan() returns radians
					float thetaDiff = thetaTargt-thetahat; //Might be outside pi to -pi degrees
					vFunc_Inf2pi(&thetaDiff);
					
					
					
					//Hysteresis mechanics
					if (fabs(thetaDiff) > rotateThreshold){
						doneTurning = FALSE;
						
					}else if (fabs(thetaDiff) < driveThreshold){
						doneTurning = TRUE;
						
					}
					
					int16_t LSpeed = 0;
					int16_t RSpeed = 0;
					
					if (doneTurning){//Start forward movement
						if (thetaDiff >= 0){//Moving left
							LSpeed = currentDriveActuation - driveKp*fabs(thetaDiff) - driveKi*leftIntError; //Simple PI controller for theta 
							
							//Saturation
							if (LSpeed > currentDriveActuation){
								LSpeed = currentDriveActuation;
							}else if(LSpeed < 0){
								LSpeed = 0;
							}
							
							RSpeed = currentDriveActuation;
							
							
						}else{//Moving right
							RSpeed = currentDriveActuation - driveKp*fabs(thetaDiff) - driveKi*rightIntError; //Simple PI controller for theta
							
							//Saturation
							if (RSpeed > currentDriveActuation){
								RSpeed = currentDriveActuation;
								}else if(RSpeed < 0){
								RSpeed = 0;
							}
							
							LSpeed = currentDriveActuation;
							
							
							
						}
						
						leftIntError += thetaDiff;
						rightIntError -= thetaDiff;
						
						
						gRightWheelDirection = motorRightForward;
						gLeftWheelDirection = motorLeftForward;
						lastMovement = moveForward;
						
						
						
					}else{ //Turn within 1 degree of target
						if (thetaDiff >= 0){//Rotating left
							LSpeed = -maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
							gLeftWheelDirection = motorLeftBackward;
							RSpeed = maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
							gRightWheelDirection = motorRightForward;
							lastMovement = moveCounterClockwise;
						}else{//Rotating right
							LSpeed = maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
							gLeftWheelDirection = motorLeftForward;
							RSpeed = -maxRotateActuation*(0.3 + 0.22*(fabs(thetaDiff)));
							gRightWheelDirection = motorRightBackward;
							lastMovement = moveClockwise;
						}
						
						leftIntError = 0;
						rightIntError = 0;
						
					}
					
					
					
					
					
					
					vMotorMovementSwitch(LSpeed,RSpeed, &gLeftWheelDirection, &gRightWheelDirection);
			
				}else{
					
					if (idleSendt == FALSE){
						send_idle();
						idleSendt = TRUE;
					}
					
					
					vMotorBrakeLeft();
					vMotorBrakeRight();
					lastMovement = moveStop;
				}
				
				
				
				xQueueSend(scanStatusQ, &lastMovement, 0); // Send the current movement to the scan task
				
			} // No semaphore available, task is blocking
			
		} //if(gHandshook) end
	}
	
	
	  
	
}

/* Pose estimator task */
void vMainPoseEstimatorTask( void *pvParameters ){
    int16_t previous_ticksLeft = 0;
    int16_t previous_ticksRight = 0;  
    
    const TickType_t xDelay = PERIOD_ESTIMATOR_MS;
    float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0f;
    
    float kalmanGain = 0.5;
    
    float predictedTheta = 0.0;
    float predictedX = 0.0;
    float predictedY = 0.0;
    
    float gyroOffset = 0.0;
    float compassOffset = 0.0;
    
    // Found by using calibration task
    int16_t xComOff = 11; 
    int16_t yComOff = -78;
    
    float variance_gyro = 0.0482f; // [rad] calculated offline, see report
    float variance_encoder = (2.0f * WHEEL_FACTOR_MM) / (WHEELBASE_MM / 2.0f); // approximation, 0.0257 [rad]
    
    float variance_gyro_encoder = (variance_gyro + variance_encoder) * period_in_S; // (Var gyro + var encoder) * timestep
    float covariance_filter_predicted = 0;
    
    #define CONST_VARIANCE_COMPASS 0.3490f
    

    float gyroWeight = 0.5;//encoderError / (encoderError + gyroError);
    uint8_t robot_is_turning = 0;
    
    
    #ifdef DEBUG
        printf("Estimator OK");
        printf("[%i]",PERIOD_ESTIMATOR_MS);
        printf("ms\n");   
        uint8_t printerTellar = 0;     
    #endif
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    while(1){
        // Loop
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );  
        if (gHandshook){ // Check if we are ready    
            int16_t leftWheelTicks = 0;
            int16_t rightWheelTicks = 0;
            
            // Get encoder data, protect the global tick variables
            xSemaphoreTake(xTickMutex, 15 / portTICK_PERIOD_MS);
                leftWheelTicks = gLeftWheelTicks;
                rightWheelTicks = gRightWheelTicks;
            xSemaphoreGive(xTickMutex);
            
            float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
            float dRight =(float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
            previous_ticksLeft = leftWheelTicks;
            previous_ticksRight = rightWheelTicks;
            
			
			
					   
					   
            float dRobot = (dLeft + dRight) / 2;           
            float dTheta = (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
            
			
			
			
			
			
            
            /* PREDICT */
            // Get gyro data:
            float gyrZ = (fIMU_readFloatGyroZ() - gyroOffset);
            //dTheta = gyrZ * period_in_S * DEG2RAD; [COMMENT]I believe this line is not supposed to be here. Residual from broken encoders?
            
            // If the robot is not really rotating we don't include the gyro measurements, to avoid the trouble with drift while driving in a straight line
            if(fabs(gyrZ) < 10){ 
                gyroWeight = 0; // Disregard gyro while driving in a straight line
                robot_is_turning = FALSE; // Don't update angle estimates
                }
            else {
                robot_is_turning = TRUE;
                gyroWeight = 0.85; // Found by experiment, after 20x90 degree turns, gyro seems 85% more accurate than encoders
                
            }
            
            gyrZ *= period_in_S * DEG2RAD; // Scale gyro measurement      
            
			
			
            // Fuse heading from sensors to predict heading:
            dTheta =  (1 - gyroWeight) * dTheta + gyroWeight * gyrZ;
            
            
            // Estimate global X and Y pos
            // Todo; Include accelerator measurements to estimate position and handle wheel slippage
            predictedX = predictedX + (dRobot * cos(predictedTheta + 0.5 * dTheta)); 
            predictedY = predictedY + (dRobot * sin(predictedTheta + 0.5 * dTheta));

            // Predicted (a priori) state estimate for theta
            predictedTheta += dTheta;
                  
            // Predicted (a priori) estimate covariance
            covariance_filter_predicted += variance_gyro_encoder;
            
            /* UPDATE */
            // Get compass data: ( Request and recheck after 6 ms?)
            int16_t xCom, yCom, zCom;
            vCOM_getData(&xCom, &yCom, &zCom);
            // Add calibrated bias
            xCom += xComOff;
            yCom += yComOff;
            // calculate heading
            float compassHeading;
            compassHeading = atan2(yCom, xCom) - compassOffset ; // returns -pi, pi
            // Update predicted state:    
            float error = (compassHeading - predictedTheta);
            vFunc_Inf2pi(&error);
            
            
            //kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
            ///* Commented back in due to fixed encoder
            if (fabs(error) > (0.8727*period_in_S)){ // 0.8727 rad/s is top speed while turning
                // If we have a reading over this, we can safely ignore the compass
                // Ignore compass while driving in a straight line
                kalmanGain = 0;
                vLED_singleLow(ledYELLOW);
            }
            else if ((robot_is_turning == FALSE) && (dRobot == 0)){
                // Updated (a posteriori) state estimate
                kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
                vLED_singleHigh(ledYELLOW);
            }
            else{
                kalmanGain = 0;
                vLED_singleLow(ledYELLOW);
            }            
            //*/
           
            predictedTheta  += kalmanGain*(error);
			vFunc_Inf2pi(&predictedTheta);            
            
            // Updated (a posteriori) estimate covariance
            covariance_filter_predicted = (1 - kalmanGain) * covariance_filter_predicted;  

			

            // Update pose
            xSemaphoreTake(xPoseMutex, 15 / portTICK_PERIOD_MS);
                gTheta_hat = predictedTheta;
                gX_hat = predictedX;
                gY_hat = predictedY;
            xSemaphoreGive(xPoseMutex);
            // Send semaphore to controller
            xSemaphoreGive(xControllerBSem);
        }
        else{
            // Not connected, getting heading and gyro bias
            uint16_t i;
            uint16_t samples = 100;
            float gyro = 0;
            for (i = 0; i<=samples; i++){
                gyro+= fIMU_readFloatGyroZ();
            }
            
            int16_t xCom, yCom, zCom;
            vCOM_getData(&xCom, &yCom, &zCom);
            xCom += xComOff;
            yCom += yComOff;
            
            
            // Initialize pose to 0 and reset offset variables
            predictedX = 0;
            predictedY = 0;
            predictedTheta = 0;
            
            compassOffset = atan2(yCom, xCom);    
            gyroOffset = gyro / (float)i;               
        }
    } // While(1) end
}

/* Handles request from position controller and sets motor pins. */
/* Frequency set by PERIOD_MOTOR_MS in defines.h */
void vMainMovementTask( void *pvParameters ){
    
	
	// Task init 
    uint8_t lastMovement = 0;
    uint8_t movement = 0;
    uint8_t actuation = 0;
    
    int16_t bias_LeftWheelTick = 0;
    int16_t bias_RightWheelTick = 0;
    
    
    // PI control Variabels 
    uint8_t previous_leftEncoderVal = 0;
    uint8_t previous_rightEncoderVal = 0;
    int16_t errorL_I = 0;
    int16_t errorR_I = 0;
    int16_t rightVelocitySP = 0;
    
    
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    #ifdef DEBUG
    printf("Movement OK\n");
    #endif
    
    
    const TickType_t xDelay = PERIOD_MOTOR_MS;
    
    while(1){
		
		
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );  
        
	}// While(1) end
	
	
}


//#define COMPASS_CALIBRATE

#ifdef COMPASS_CALIBRATE
void compassTask(void *par){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("Compass running\n");
    int16_t xComOff = 0;
    int16_t yComOff = 0;
    while(1){
        vTaskDelay(100 / portTICK_PERIOD_MS);
    if (gHandshook){
        int16_t xComMax = -4000, yComMax = -4000;
        int16_t xComMin = 4000, yComMin = 4000;
        int16_t xCom, yCom, zCom;   
// wait until you start moving     
//         while(fabs(zGyr) < 20){
//             zGyr = fIMU_readFloatGyroZ();
//             
//             vTaskDelay(15/portTICK_PERIOD_MS);
//         }
        printf("Rotating in\n...3\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("...2\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("...1\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uint8_t movement;
        movement = moveCounterClockwise;
        xQueueSend(movementQ, &movement, 10);
        
        float heading = 0;
        float gyroHeading = 0;
        float encoderHeading = 0;
        
        gLeftWheelTicks = 0;
        gRightWheelTicks = 0;
        float previous_ticksLeft = 0;
        float previous_ticksRight = 0;
// Storing values for printing later
//         uint8_t tellar = 0;
//         float tabellG[200];
//         float tabellE[200];
        
        TickType_t xLastWakeTime;
        const TickType_t xDelay = 50;
        int counter = 0;
        // Initialise the xLastWakeTime variable with the current time.
        xLastWakeTime = xTaskGetTickCount(); 
        while(heading < 359){
            vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS);
            int16_t leftWheelTicks = 0;
            int16_t rightWheelTicks = 0;
            ATOMIC_BLOCK(ATOMIC_FORCEON){
                leftWheelTicks = gLeftWheelTicks;
                rightWheelTicks = gRightWheelTicks;
            }
            float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
            float dRight =(float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
            if (counter >= 4){
                //printf("%i %i\n",leftWheelTicks,rightWheelTicks);
                counter = 0;
            }
            else counter++;
            
            previous_ticksLeft = leftWheelTicks;
            previous_ticksRight = rightWheelTicks;
            float dTheta = RAD2DEG * (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
            
//            float zGyr = 0.054*(fIMU_readFloatGyroZ() + 1.700); // add offset bias
// Storing values for printing            
//             gyroHeading += zGyr;
//             encoderHeading += dTheta;
//             tabellE[tellar] = dTheta;
//             tabellG[tellar] = zGyr;
//             tellar++;
            heading += dTheta;
            
            vCOM_getData(&xCom, &yCom, &zCom);
            
            if(xCom > xComMax) xComMax = xCom;
            if(yCom > yComMax) yComMax = yCom;
            
            if(xCom < xComMin) xComMin = xCom;
            if(yCom < yComMin) yComMin = yCom;
        }
        movement = moveClockwise;
        xQueueSend(movementQ, &movement, 10);
        movement = moveStop;
        xQueueSend(movementQ, &movement, 10);
// Printing said values
//         int i = 0;
//         for (i = 0; i < tellar; i++){
//             printf("%.1f, %.1f\n",tabellG[i], tabellE[i]);
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//         }
//         printf("gyro %.2f, encoder %.2f \n", gyroHeading, encoderHeading);
        
// Printing new xy cal values
        vUSART_sendS("Old XY-offset values:");
        char dobbel[8];
        itoa(xComOff, dobbel, 10);
        vUSART_sendS(dobbel);
        itoa(yComOff, dobbel, 10);
        vUSART_sendS(",\t ");
        vUSART_sendS(dobbel);
        vUSART_sendS("\n");
        xComOff = ((xComMax - xComMin)/2) - xComMax;
        yComOff = ((yComMax - yComMin)/2) - yComMax;
        
        printf("New XY-offset values:");
        itoa(xComOff, dobbel, 10);
        vUSART_sendS(dobbel);
        itoa(yComOff, dobbel, 10);
        vUSART_sendS(",\t ");
        vUSART_sendS(dobbel);
        vUSART_sendS("\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    else
    vTaskDelay(200 / portTICK_PERIOD_MS); 
        
        
    }
}
#endif

/*  In case of stack overflow, disable all interrupts and handle it  */
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, signed char *pcTaskName){
    cli();
    /*  Handle overflow */
    #ifdef DEBUG
       debug("Overflow\n");
    #endif
    while(1){
        vLED_toggle(ledRED);
        //ledPORT ^= (1<<ledGREEN) | (1<<ledYELLOW) | (1<<ledRED);
    }// While(1) end
}

/*  Main function   */
int main(void){
    /* Setup - Initialize all settings before tasks  */
    /* Initialize LED, pins defined in LED.h   */
    vLED_init();
    vLED_singleHigh(ledRED); // Set red LED on to indicate INIT is ongoing
    /* Initialize USART driver, NB! baud is dependent on nRF51 dongle */
    vUSART_init();
    network_init();
    arq_init();
    simple_p_init(server_receiver);
    // If the MCU resets, the cause can be seen in MCUSR register
    // See page 56 in the data sheet
    #ifdef DEBUG
		uint8_t reg = MCUSR;
        debug("Reboot.\nStatus register: 0b%d\n", reg);     
        MCUSR = 0; // Reset MCUSR
    #endif
    #ifdef tictoc
        usetictoc;
        debug("tictoc!\n");
        tic;
    #endif
    
    /* Initialize servo for sensor tower to zero degrees */
    vServo_init(0);
    /* Initialize sensors */
    vDistSens_init();
    /* Initialize motor controller */
    vMotor_init();
    /* Initialize Inertial Measurement Unit (IMU) and SPI  */
    #ifdef DEBUG
        debug("IMU init..\n");
    #endif
    sIMU_begin(); 
   
    /* Initialize compass */
    /* Connected with I2C, if the chip has no power, MCU will lock. */
    #ifdef DEBUG
            debug("Compass init..\n");
    #endif
    
    //vCOM_init();
    
    /* Initialize RTOS utilities  */
    movementQ = xQueueCreate(2,sizeof(uint8_t)); // For sending movements to vMainMovementTask
    poseControllerQ = xQueueCreate(1, sizeof(struct sPolar)); // For setpoints to controller
    scanStatusQ = xQueueCreate(1,sizeof(uint8_t)); // For robot status
    actuationQ = xQueueCreate(2,sizeof(uint8_t)); // To send variable actuation to motors
    
    xPoseMutex = xSemaphoreCreateMutex(); // Global variables for robot pose. Only updated from estimator, accessed from many
    xUartMutex = xSemaphoreCreateMutex(); // Protected printf with a mutex, may cause fragmented bytes if higher priority task want to print as well
    xTickMutex = xSemaphoreCreateMutex(); // Global variable to hold robot tick values
    
    xControllerBSem = xSemaphoreCreateBinary(); // Estimator to Controller synchronization
    xCommandReadyBSem = xSemaphoreCreateBinary(); // uart ISR to comm task sync
    
    // Todo: Check return variable to ensure RTOS utilities were successfully initialized before continue
    xTaskCreate(vMainMovementTask, "Movement", 300, NULL, 4, NULL); // Independent task, uses ticks from ISR
    xTaskCreate(vMainCommunicationTask, "Comm", 300, NULL, 3, NULL); // Dependant on ISR from UART, sends instructions to other tasks
    
    #ifndef COMPASS_CALIBRATE // If compass calibration task is running dont use these tasks
        xTaskCreate(vMainPoseControllerTask, "PoseCon", 300, NULL, 2, NULL); // Dependant on estimator, sends instructions to movement task
        xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 300, NULL, 5, NULL); // Independent task, uses ticks from ISR
        xTaskCreate(vMainSensorTowerTask,"Tower",300, NULL, 1, NULL); // Independent task, but use pose updates from estimator
    #endif

    #ifdef COMPASS_CALIBRATE
        debug("\n \t WARNING \t !\n");
        debug("COMPASS CALIBRATION!\n");
        debug("Connect to begin!\n");
        xTaskCreate(compassTask, "compasscal", 3500, NULL, 3, NULL); // Task used for compass calibration, dependant on communication and movement task
    #endif
    
    

    sei();
    vLED_singleLow(ledRED);
    #ifdef DEBUG
    debug("Starting scheduler ....\n");
    #endif
    /*  Start scheduler */
    vTaskStartScheduler();

    /*  MCU is out of RAM if the program comes here */
    while(1){
        cli();
        debug("RAM fail\n");
    }
}


/*
 Interrupt Service Routines
 Int2: Left wheel optical encoder
 Int3: Right wheel optical encoder
 Int4: nRF dongle status pin, NB set up in motor.c
 USART0 RX vector: USART with nRF dongle
*/

/* Handle tick from left wheel encoder */
// If ticks generate overly many interrupts, you can 
// connect to T0 and T5 pins and set up  hardware timer overflow to 
// prescale the ticks
ISR(leftWheelCount){
    gISR_leftWheelTicks++;
}

/* Handle tick from right wheel encoder */
ISR(rightWheelCount){
    gISR_rightWheelTicks++;
}

/* Handle change of connection status */
ISR(nRF51_status){
    if (nRFconnected){
        // indicate we are connected
        vLED_singleHigh(ledGREEN);
        vLED_singleHigh(ledYELLOW);
    }
    else{
        // We are not connected or lost connection, reset handshake flag
        gHandshook = FALSE;
        gPaused = FALSE;
        vLED_singleLow(ledGREEN);
        vLED_singleLow(ledYELLOW);
        vLED_singleLow(ledRED);
        xSemaphoreGiveFromISR(xCommandReadyBSem,0); // Let uart parser reset if needed
    }
    xSemaphoreGiveFromISR(xControllerBSem,0); // let the controller reset if needed    
}
