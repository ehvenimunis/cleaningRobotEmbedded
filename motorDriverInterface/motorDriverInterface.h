/*
 * motorDriverInterface.h
 *
 *  Created on: Feb 15, 2021
 *      Author: neset
 */

#ifndef MOTORDRIVERINTERFACE_H_
#define MOTORDRIVERINTERFACE_H_

#include "main.h"


#define DRIVERCHANNEL 1 //how many are there driver
#define TIMEOUTVAL 5 //uart timeout value when be long then data transmission is safe


#if DRIVERCHANNEL == 1
#define MDI_channel1 huart1
extern UART_HandleTypeDef MDI_channel1;
#elif DRIVERCHANNEL == 2
#define MDI_channel1 huart2
extern UART_HandleTypeDef MDI_channel1;

#define MDI_channel2 huart1//uart channel
extern UART_HandleTypeDef MDI_channel2;
#endif

void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kd,uint8_t ki,uint8_t kp,uint8_t rate );
void MDI_getDataChannel1(void);



#endif /* MOTORDRIVERINTERFACE_H_ */
