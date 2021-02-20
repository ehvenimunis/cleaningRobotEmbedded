import serial
import time
import random
import os

ser1=serial.Serial("/dev/ttyUSB0",57600,timeout=5)
ser2=serial.Serial("/dev/ttyUSB1",57600,timeout=5)
os.system("clear")

print("ser1 val",ser1.read())
print("ser2 val",ser2.read())

def sendVal1():
	arrList=[]
	arrList.append(255)
	arrList.append(255)
	angle=random.randint(0, 1000)
	arrList.append(angle >> 8)
	arrList.append(angle & 0xFF)
	kd=random.randint(0, 255)
	arrList.append(kd)
	ki=random.randint(0, 255)
	arrList.append(ki)
	kp=random.randint(0, 255)
	arrList.append(kp)
	factor=random.randint(0, 255)
	arrList.append(factor)
	temp=arrList[2]+arrList[3]+arrList[4]+arrList[5]+arrList[6]+arrList[7]
	checker=temp%256
	arrList.append(checker)
	checNot=255-checker
	arrList.append(checNot)
	#print('angle->',angle,'arr->',arrList)		
	ser1.write(arrList)
	#print("Sended Vals1=\n","Angle=",angle," kd=",kd," ki=",ki," kp=",kp," factor=",factor)

def sendVal2():
	arrList=[]
	arrList.append(255)
	arrList.append(255)
	angle=random.randint(0, 1000)
	arrList.append(angle >> 8)
	arrList.append(angle & 0xFF)
	kd=random.randint(0, 255)
	arrList.append(kd)
	ki=random.randint(0, 255)
	arrList.append(ki)
	kp=random.randint(0, 255)
	arrList.append(kp)
	factor=random.randint(0, 255)
	arrList.append(factor)
	temp=arrList[2]+arrList[3]+arrList[4]+arrList[5]+arrList[6]+arrList[7]
	checker=temp%256
	arrList.append(checker)
	checNot=255-checker
	arrList.append(checNot)
	#print('angle->',angle,'arr->',arrList)		
	ser2.write(arrList)
	#print("Sended Vals 2=\n","Angle=",angle," kd=",kd," ki=",ki," kp=",kp," factor=",factor)


def readVal1():
	arrList=[]
	type(arrList)
	in_bin1=ser1.read()
	in_hex=hex(int(in_bin1.encode('hex'), 16))
	
	if(hex(255)==in_hex):
		for x in range(9):
			in_bin1=ser1.read()
			in_hex=hex(int(in_bin1.encode('hex'), 16))
			arrList.append(in_hex)	
	
	if(len(arrList)>8):
		temp=int(arrList[1], 16)+int(arrList[2], 16)+int(arrList[3], 16)+int(arrList[4], 16)+int(arrList[5], 16)+int(arrList[6], 16)		
		checker=temp%256
		checNot=255-checker
		if (checker==int(arrList[7], 16) and checNot==int(arrList[8], 16)):
			angle=(int(arrList[1], 16) << 8) | int(arrList[2], 16)
			kd=arrList[3]
			ki=arrList[4]
			kp=arrList[5]
			factor=arrList[6]
			print("Ser1 Read Vals=","Angle=",angle," kd=",kd," ki=",ki," kp=",kp," factor=",factor)

def readVal2():
	arrList=[]
	type(arrList)
	in_bin1=ser2.read()
	in_hex=hex(int(in_bin1.encode('hex'), 16))
	
	if(hex(255)==in_hex):
		for x in range(9):
			in_bin1=ser2.read()
			in_hex=hex(int(in_bin1.encode('hex'), 16))
			arrList.append(in_hex)	
	
	if(len(arrList)>8):
		temp=int(arrList[1], 16)+int(arrList[2], 16)+int(arrList[3], 16)+int(arrList[4], 16)+int(arrList[5], 16)+int(arrList[6], 16)		
		checker=temp%256
		checNot=255-checker
		if (checker==int(arrList[7], 16) and checNot==int(arrList[8], 16)):
			angle=(int(arrList[1], 16) << 8) | int(arrList[2], 16)
			kd=arrList[3]
			ki=arrList[4]
			kp=arrList[5]
			factor=arrList[6]
			print("Ser2 Read Vals=","Angle=",angle," kd=",kd," ki=",ki," kp=",kp," factor=",factor)
#for x in range(150):
#	command = b'\xFF\xFF\x01\x03\x01\x01\x02\x03\x0B\xF4'
	#angle->259, kd->1,ki->1,kp->2,factor->3,mod ->10,tersi->245
#	ser1.write(command)
#	command2 = b'\xFF\xFF\x05\x01\x00\x01\x03\x01\x0B\xF4'
	#angle->1281, kd->0,ki->1,kp->3,factor->1,mod ->10,tersi->245
#	ser2.write(command2)
time.sleep(5)
checkSer=1
while 1:	
	readVal1()
	readVal2()
	#command = b'\xFF\xFF\x01\x03\x01\x01\x02\x03\x0B\xF4'
	#ser1.write(command)
	#angle->259, kd->1,ki->1,kp->2,factor->3,mod ->10,tersi->245
	#command2 = b'\xFF\xFF\x03\x01\x02\x01\x01\x03\x0B\xF4'
	#ser2.write(command)	
	#angle->759, kd->2,ki->1,kp->1,factor->3,mod ->10,tersi->245
	#command = b'\xFF\xFF\x05\x01\x00\x01\x03\x01\x0B\xF4'
	#angle->1281, kd->0,ki->1,kp->3,factor->1,mod ->10,tersi->245
	
	