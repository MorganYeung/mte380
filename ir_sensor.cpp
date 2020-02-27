 #include <Adafruit_MotorShield.h>
class IR_Sensor {
    private:
        const int front = A3; //1 
        const int left1 = A4; //2
        const int left2 = A5; //3
        const int long_front = A6; //4
    
    public:
        int read(int sensor) {
        	switch (sensor){
        		case 1:
        			return -55.89*log(analogRead(front)) + 363.54;
        			break;
        		case 2:
        			return -55.89*log(analogRead(left1)) + 363.54;
        			break;
        	    case 3:
        			return -55.89*log(analogRead(left2)) + 363.54;
        			break;
				case 4:
        			return -279.2*log(analogRead(long_front)) + 1836.7;
        			break;	
        		default:
        			break;
        	}	
		}
};
