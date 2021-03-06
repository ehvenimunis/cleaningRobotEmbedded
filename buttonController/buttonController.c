/*
 * buttonController.c
 *
 *  Created on: Feb 17, 2021
 *      Author: neset
 */

#include "buttonController.h"
#include "../motorDriverInterface/motorDriverInterface.h"

static uint16_t selectbtncounter=0,plusbtncounter=0,minusbtncounter=0;
mD_interface drv1,drv2;
/* @brief count pressed button use in tim interrupt
 * @param none
 * @return none
 * */
void buttonCounterIncrease(void){
	if(!READ_SELECT_BTN)selectbtncounter++;
	else if(!READ_PLUS_BTN)plusbtncounter++;
	else if(!READ_MINUS_BTN)minusbtncounter++;
}
/* @brief value returner when pressed select button long or short
 * @param none
 * @return 0 -> didnt press
 * @return 1 -> short pressed
 * @return 2 -> long pressed
 * */
uint8_t selectButton_Listenner_For_MenuControl(void) {
	if(READ_SELECT_BTN){
		if(selectbtncounter>=BTN_LONG_PRESS_TIME){ selectbtncounter=0;return 2;}
		if(selectbtncounter>=BTN_SHORT_PRESS_TIME){ selectbtncounter=0;return 1;}
		selectbtncounter=0;
	}
	return 0;
}
/* @brief value returner when pressed plus button long or short
 * @param none
 * @return 0 -> didnt press
 * @return 1 -> short pressed
 * @return 2 -> long pressed
 * */
uint8_t plusButton_Listenner_For_MenuControl(void) {

	if(READ_PLUS_BTN){
		if(plusbtncounter>=BTN_LONG_PRESS_TIME){ plusbtncounter=0;return 2;}
		if(plusbtncounter>=BTN_SHORT_PRESS_TIME){ plusbtncounter=0;return 1;}
		plusbtncounter=0;
	}
	return 0;
}
/* @brief value returner when pressed minus button long or short
 * @param none
 * @return 0 -> didnt press
 * @return 1 -> short pressed
 * @return 2 -> long pressed
 * */
uint8_t minusButton_Listenner_For_MenuControl(void) {
	if(READ_MINUS_BTN){
		if(minusbtncounter>=BTN_LONG_PRESS_TIME){ minusbtncounter=0;return 2;}
		if(minusbtncounter>=BTN_SHORT_PRESS_TIME){ minusbtncounter=0;return 1;}
		minusbtncounter=0;
	}
	return 0;
}
/* @brief drivers parameter values initalize
 * @param none
 * @return none
 * */
void btnParameterInit(void){
	HAL_TIM_Base_Start_IT(&BTNCOUNTER);
	drv1.angle=0;
	drv1.pid_kd=0;
	drv1.pid_ki=0;
	drv1.pid_kp=0;
	drv1.factor=0;
	drv2.angle=0;
	drv2.pid_kd=0;
	drv2.pid_ki=0;
	drv2.pid_kp=0;
	drv2.factor=0;
}
/* @brief set the angle to motor driver 1 interface
 * @param angle -> driver angle value
 * @return none
 * */
void setDriver1AngleValue(uint16_t angle){
	drv1.angle=angle;
}
/* @brief set the angle to motor driver 2 interface
 * @param angle -> driver angle value
 * @return none
 * */
void setDriver2AngleValue(uint16_t angle){
	drv2.angle=angle;
}
/* @brief controlling and sending values of drivers
 * @param none
 * @return none
 * */
void buttonController(void){
	static bool selectedMDI=true; //selected motor driver interface
	static uint8_t selectBtnListenner=0,plusBtnListenner=0,minusBtnListenner=0;
	static item=0;
	if(selectedMDI){
		if(2==selectBtnListenner){ selectedMDI=false; item=0;}
		if(1==selectBtnListenner){
			if(item<ITEM_MAX)item++;
			else if(item==ITEM_MAX)item=0;
		}
		if(1==plusBtnListenner){
			switch(item){
			case 0: if(drv1.pid_kd<255) drv1.pid_kd++; break;
			case 1: if(drv1.pid_ki<255) drv1.pid_ki++; break;
			case 2: if(drv1.pid_kp<255) drv1.pid_kp++; break;
			case 3: if(drv1.factor<255) drv1.factor++; break;
			}
		}
		if(1==minusBtnListenner){
			switch(item){
			case 0: if(drv1.pid_kd>0) drv1.pid_kd--; break;
			case 1: if(drv1.pid_ki>0) drv1.pid_ki--; break;
			case 2: if(drv1.pid_kp>0) drv1.pid_kp--; break;
			case 3: if(drv1.factor>0) drv1.factor--; break;
			}
		}
	}
	else{
		if(2==selectBtnListenner){ selectedMDI=true; item=0;}
		if(1==selectBtnListenner){
			if(item<ITEM_MAX)item++;
			else if(item==ITEM_MAX)item=0;
		}
		if(1==plusBtnListenner){
			switch(item){
			case 0: if(drv2.pid_kd<255) drv2.pid_kd++; break;
			case 1: if(drv2.pid_ki<255) drv2.pid_ki++; break;
			case 2: if(drv2.pid_kp<255) drv2.pid_kp++; break;
			case 3: if(drv2.factor<255) drv2.factor++; break;
			}
		}
		if(1==minusBtnListenner){
			switch(item){
			case 0: if(drv2.pid_kd>0) drv2.pid_kd--; break;
			case 1: if(drv2.pid_ki>0) drv2.pid_ki--; break;
			case 2: if(drv2.pid_kp>0) drv2.pid_kp--; break;
			case 3: if(drv2.factor>0) drv2.factor--; break;
			}
		}
	}

	selectBtnListenner=selectButton_Listenner_For_MenuControl();
	plusBtnListenner=plusButton_Listenner_For_MenuControl();
	minusBtnListenner=minusButton_Listenner_For_MenuControl();
	//"drv1= %x - %x - %x - %x\n",drv1.pid_kd,drv1.pid_ki,drv1.pid_kp,drv1.factor
	//"drv2= %x - %x - %x - %x\n",drv2.pid_kd,drv2.pid_ki,drv2.pid_kp,drv2.factor

}
void transmissionDriver1(void){
	MDI_sendDataChannel1(drv1.angle,drv1.pid_kd,drv1.pid_ki,drv1.pid_kp,drv1.factor);
}
void transmissionDriver2(void){
	MDI_sendDataChannel2(drv2.angle,drv2.pid_kd,drv2.pid_ki,drv2.pid_kp,drv2.factor);
}
