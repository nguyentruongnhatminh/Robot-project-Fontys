/*
 * robotPrj.h
 *
 *  Created on: Nov 18, 2022
 *      Author: nguye
 */
#include <stdlib.h>
#include <time.h>       /* time */

typedef enum states
{
	MOVING_FOWARD,
	TURNING,
	MOVING_BACKWARD
}Robot_States;


#ifndef INC_ROBOTPRJ_H_
#define INC_ROBOTPRJ_H_

#define DIS_TURNING_INTERVAL (10)
#define DIS_RUNNING_INTERVAL (15)

#define CCRx_0_SERVO1 (1480)//0 degree
#define CCRx_0_SERVO2 (1520) //0 degree
#define CCRx_90_SERVO1 (1280) //left servo, 90 degree
#define CCRx_90_SERVO2 (1720) //right servo, 90 degree
#define CCRx_MINUS_90_SERVO1 (1720) //left servo, -90 degree
#define CCRx_MINUS_90_SERVO2 (1280) //right servo, -90 degree
#define MAX_SPEED_LIMIT (100)
#define MIN_SPEED_LIMIT (-100)



volatile int distance;
volatile int risingEdgeEvent;
volatile int fallingEdgeEvent;
volatile int timeCounter;
volatile int timeCounter2;
void GPIO_port_configuration(void);
void TIM4_IRQHandler(void);
void Timer_configuration_servo(uint32_t PSC, uint32_t ARR);
void Timer_configuration_ultrasonic(uint32_t PSC, uint32_t ARR);
void my_PD(int setPoint);
int execute_P(int error, int kP);
int execute_PD(int error, int derivative, int KP, int KD);
float get_sensor_value(void);
void control_servo(int controlValue);
void adjust_robot_state(Robot_States state, int controlValue, int range);
void Generate_Turning_Decision(int inputNumber);
void Adjust_Robot_By_Distance(int controlValue, int range);
void control_first_servo(int controlValue);
void control_second_servo(int controlValue);

#endif /* INC_ROBOTPRJ_H_ */
