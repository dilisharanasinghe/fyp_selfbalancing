#include "hubMotorV2.h"

hubMotorV2::hubMotorV2(unsigned char CP){
	CP_ = CP;
}

void hubMotorV2::init(){
	pinMode(CP_,OUTPUT);
	analogWrite(CP_,127);
	delay(100);
	
	//Serial.println("Initiated");
	
}


void hubMotorV2::setMotorSpeed(int speedValue){
	if(speedValue != currentMotorSpeed_){
		if(speedValue >= 0){
			//analogWrite(CP_,min(abs(speedValue/2) + 127,255));
			analogWrite(CP_,min(127 - abs(speedValue/2),255));
			currentMotorSpeed_ = speedValue;
		}else if (speedValue < 0){
			//analogWrite(CP_,min(127 - abs(speedValue/2),255));
			analogWrite(CP_,min(abs(speedValue/2) + 127,255));
			currentMotorSpeed_ = speedValue;
		}
		//Serial.print("currentMotorSpeed_ = ");
		//Serial.println(currentMotorSpeed_);
	}
}

void hubMotorV2::setBrakes(){
	setMotorSpeed(0);
	//Serial.println("setBrakes");
}
