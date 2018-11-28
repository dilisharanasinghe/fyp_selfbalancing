#include <Arduino.h>

class hubMotor{
	public:
		hubMotor(unsigned char VR, unsigned char ZF, unsigned char EL,unsigned int directionFlag);
		
		void init();
		void setMotorSpeed(int speedValue);
		void setBrakes();
	
	private:
		unsigned char VR_;	//speed controller
		unsigned char EL_;	//enable pin - high--> enable
		unsigned char ZF_;	//direction control
		unsigned int directionFlag_; //used to overide motor direction
		int currentMotorSpeed_ = 0;
		
		void setForward();
		void setReverse();

};
