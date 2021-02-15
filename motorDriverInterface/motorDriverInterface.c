/*
 * motorDriverInterface.c
 *
 *  Created on: Feb 15, 2021
 *      Author: neset
 */

#include "motorDriverInterface.h"

/*driverUnits
 * elements:
 * kd
 * ki
 * kp
 * factor*/ //maybe next times change as be struct
#if DRIVERCHANNEL == 1
uint8_t driver1Units[4];
uint16_t driver1angelVal;
uint8_t recBuff[10];
#elif DRIVERCHANNEL == 2
uint8_t driver1Units[4];
uint16_t driver1angelVal;
uint8_t recBuff[10];

uint8_t driver2Units[4];
uint16_t driver2angelVal;
uint8_t rec2Buff[10];
#endif
/**
 * @brief Write command to Motor Driver
 * @param uartChannel -> get uart channel
 * @param cmd -> command to write
 * @return none
 */
static void MDI_writeCommand(UART_HandleTypeDef *uartChannel,uint8_t cmd){
	HAL_UART_Transmit(uartChannel,(uint8_t*) &cmd,sizeof(cmd),TIMEOUTVAL);
}
/**
 * @brief Write small data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param data -> data to write
 * @return none
 */
static void MDI_writeSmallData(UART_HandleTypeDef *uartChannel,uint8_t data){
	HAL_UART_Transmit(uartChannel,(uint8_t*)&data,sizeof(data),TIMEOUTVAL);
}
/**
 * @brief Write big data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param buff -> get data array
 * @param buff_size -> get data array size
 * @return none
 */
static void MDI_writeBigData(UART_HandleTypeDef *uartChannel,uint8_t *buff, size_t buff_size){
	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
		HAL_UART_Transmit(uartChannel, buff, chunk_size, TIMEOUTVAL);
		buff += chunk_size;
		buff_size -= chunk_size;
	}
}
/**
 * @brief Write  2 byte data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param data -> get 2 byte data
 * @return none
 */
void MDI_2byteWriteData(UART_HandleTypeDef *uartChannel,uint16_t data){
	uint8_t arrTmp[] = {data >> 8, data & 0xFF};
	MDI_writeBigData(uartChannel,arrTmp,sizeof(arrTmp));

}
#if DRIVERCHANNEL == 1
/**
 * @brief drive to Motor Driver 1
 * @param angleVal -> get motor angle value
 * @param kd -> get Pid kd value
 * @param ki -> get Pid ki value
 * @param kp -> get Pid kp value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kd,uint8_t ki,uint8_t kp,uint8_t factor ){
	uint16_t checksumTmp=0;
	MDI_writeCommand(&MDI_channel1,0xFF);
	MDI_writeCommand(&MDI_channel1,0xFF); //Data transmission started
	MDI_2byteWriteData(&MDI_channel1,angleVal);
	uint8_t tmpArr[] ={angleVal >> 8, angleVal & 0xFF};
	checksumTmp+=tmpArr[0];
	checksumTmp+=tmpArr[1]; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel1,kd); checksumTmp+=kd; //writed kd
	MDI_writeSmallData(&MDI_channel1,ki); checksumTmp+=ki; //writed ki
	MDI_writeSmallData(&MDI_channel1,kp); checksumTmp+=kp; //writed kp
	MDI_writeSmallData(&MDI_channel1,factor); checksumTmp+=factor; //writed factor
	uint8_t tmp =checksumTmp%256;
	MDI_writeSmallData(&MDI_channel1,tmp); //checksum first byte
	uint8_t tmpComp =~tmp;
	MDI_writeSmallData(&MDI_channel1,tmpComp); //checksum second byte
}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1(void){
	HAL_UART_Receive(&MDI_channel1,(uint8_t*)recBuff,10,50);
	if(0xFF==recBuff[0] && 0xFF==recBuff[1]){
		uint16_t checksumTmp=0;
		for(uint8_t c=2;c<8;c++)checksumTmp+=recBuff[c];
		uint8_t tmp =checksumTmp%256;
		uint8_t tmpComp =~tmp;
		if(tmp == recBuff[8] && tmpComp == recBuff[9]){
			driver1angelVal=((uint16_t)recBuff[2] << 8) | recBuff[3];
			driver1Units[0]=recBuff[4];
			driver1Units[1]=recBuff[5];
			driver1Units[2]=recBuff[6];
			driver1Units[3]=recBuff[7];
		}
		//"angle= %d, kd= %d, ki= %d,kp= %d, factor= %d\n",driver1angelVal,driver1Units[0],driver1Units[1],driver1Units[2],driver1Units[3]
	}	//"%d,%d,%d,%d\n",tmp, recBuff[8],tmpComp,recBuff[9]
	for(uint8_t c=2;c<8;c++)recBuff[c]=0;
}
#elif DRIVERCHANNEL == 2
/**
 * @brief drive to Motor Driver 2
 * @param angleVal -> get motor angle value
 * @param kd -> get Pid kd value
 * @param ki -> get Pid ki value
 * @param kp -> get Pid kp value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel2(uint16_t angleVal,uint8_t kd,uint8_t ki,uint8_t kp,uint8_t factor ){
	uint16_t checksumTmp=0;
	MDI_writeCommand(&MDI_channel2,0xFF); checksumTmp+=0xFF;
	MDI_writeCommand(&MDI_channel2,0xFF); checksumTmp+=0xFF; //Data transmission started
	MDI_2byteWriteData(&MDI_channel2,angleVal); checksumTmp+=angleVal; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel2,kd); checksumTmp+=kd; //writed kd
	MDI_writeSmallData(&MDI_channel2,ki); checksumTmp+=ki; //writed ki
	MDI_writeSmallData(&MDI_channel2,kp); checksumTmp+=kp; //writed kp
	MDI_writeSmallData(&MDI_channel2,factor); checksumTmp+=factor; //writed factor
	uint8_t tmp =checksumTmp%256;
	MDI_writeSmallData(&MDI_channel2,tmp); //checksum first byte
	uint8_t tmpComp =~tmp;
	MDI_writeSmallData(&MDI_channel2,tmpComp); //checksum second byte
}

/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
void MDI_getDataChannel2(void){
	HAL_UART_Receive(&MDI_channel2,(uint8_t*)rec2Buff,10,50);
	if(0xFF==rec2Buff[0] && 0xFF==rec2Buff[1]){
		uint16_t checksumTmp=0;
		for(uint8_t c=2;c<8;c++)checksumTmp+=rec2Buff[c];
		uint8_t tmp =checksumTmp%256;
		uint8_t tmpComp =~tmp;
		if(tmp == rec2Buff[8] && tmpComp == rec2Buff[9]){
			driver2angelVal=((uint16_t)rec2Buff[2] << 8) | rec2Buff[3];
			driver2Units[0]=rec2Buff[4];
			driver2Units[1]=rec2Buff[5];
			driver2Units[2]=rec2Buff[6];
			driver2Units[3]=rec2Buff[7];
		}
	}
	for(uint8_t c=2;c<8;c++)rec2Buff[c]=0;
}
#endif

