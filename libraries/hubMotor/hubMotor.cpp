#include "hubMotor.h"

hubMotor::hubMotor(unsigned char VR, unsigned char ZF, unsigned char EL,unsigned int directionFlag){
	VR_ = VR;
	EL_ = EL;
	ZF_ = ZF;
	directionFlag_ = directionFlag;
}

void hubMotor::init(){
	pinMode(VR_,OUTPUT);
	pinMode(EL_,OUTPUT);
	pinMode(ZF_,OUTPUT);
	
	delay(100);
	
	digitalWrite(EL_,HIGH);
	setForward();
	//Serial.println("Initiated");
	
}

void hubMotor::setForward(){
	if(directionFlag_){
		digitalWrite(ZF_,HIGH);
	}else{
		digitalWrite(ZF_,LOW);
	}
	//Serial.println("setFoward");
}

void hubMotor::setReverse(){
	if(directionFlag_){
		digitalWrite(ZF_,LOW);
	}else{
		digitalWrite(ZF_,HIGH);
	}
	//Serial.println("setReverse");
}


void hubMotor::setMotorSpeed(int speedValue){
	if(speedValue != currentMotorSpeed_){
		if(speedValue >= 0){
			if(currentMotorSpeed_ < 0){
				setForward();
			}
			analogWrite(VR_,min(abs(speedValue),255));
			currentMotorSpeed_ = speedValue;
		}else if (speedValue < 0){
			if(currentMotorSpeed_ >= 0){
				setReverse();
			}
			analogWrite(VR_,min(abs(speedValue),255));
			currentMotorSpeed_ = speedValue;
		}
		//Serial.print("currentMotorSpeed_ = ");
		//Serial.println(currentMotorSpeed_);
	}
}

void hubMotor::setBrakes(){
	setMotorSpeed(0);
	//Serial.println("setBrakes");
}