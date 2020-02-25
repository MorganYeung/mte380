 #include <Adafruit_MotorShield.h>
class SmallLaser {
    private:
        const int front = A3; //1 
        const int left1 = A4; //2
        const int left2 = A5; //3
    
    public:
        int read(int sensor) {
            return log(analogRead(sensor)/665.9)/-0.018;
        }
};
