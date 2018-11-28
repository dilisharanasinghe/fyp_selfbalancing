#include <Arduino.h>

class hubMotorV2{
	public:
		hubMotorV2(unsigned char CP);
		
		void init();
		void setMotorSpeed(int speedValue);
		void setBrakes();
	private:
		unsigned char CP_;
		int currentMotorSpeed_ = 0;

};
