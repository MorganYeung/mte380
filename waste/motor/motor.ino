  #include <Adafruit_MotorShield.h>
  
  class Motor {
    private:
      //motor shield
      const int directionA = 12;
      const int directionB = 13;
      const int pwmA = 3;
      const int pwmB = 11;
      const int brakeA = 9;
      const int brakeB = 8;
      const int currentsenseA = A0;
      const int currentsenseB = A1;
      
      //sensor
      const int front = A3;  
      const int left1 = A4;
      const int left2 = A5;
      const int longFront = A6;  
      const int longLeft1 = A7;
      const int longLeft2 = A8;
      
      float frontValue;
      float left1Value;
      float left2Value;
      float longfrontValue;
      float longleft1Value;
      float longleft2Value;
      
    public:
      void motor_setup(){
        pinMode(directionA, OUTPUT);
        pinMode(directionB, OUTPUT); 
        pinMode(pwmA, OUTPUT); 
        pinMode(pwmB, OUTPUT); 
        pinMode(brakeA, OUTPUT); 
        pinMode(brakeB, OUTPUT); 
      }
      
      void move_forwards(int speed){
        digitalWrite(directionA, HIGH);
        digitalWrite(brakeA, LOW);   
        digitalWrite(directionB, HIGH);
        digitalWrite(brakeB, LOW);   
        analogWrite(pwmA, speed);
        analogWrite(pwmB, speed);
      }
      void move_backwards(int speed){
        digitalWrite(directionA, LOW);
        digitalWrite(brakeA, LOW);   
        digitalWrite(directionB, LOW);
        digitalWrite(brakeB, LOW);   
        analogWrite(pwmA, speed);
        analogWrite(pwmB, speed);
      }
      void turn_right(){
        digitalWrite(directionA, HIGH);
        digitalWrite(brakeA, LOW);   
        digitalWrite(directionB, HIGH);
        digitalWrite(brakeB, LOW);   
        analogWrite(pwmA, 255);
        analogWrite(pwmB, 128);
      }
      void turn_left(){
        digitalWrite(directionA, HIGH);
        digitalWrite(brakeA, LOW);   
        digitalWrite(directionB, HIGH);
        digitalWrite(brakeB, LOW);   
        analogWrite(pwmA, 128);
        analogWrite(pwmB, 255);
      }
      void setup() {
        // initialize serial communications at 115200 bps:
        Serial.begin(115200);
        motor_setup();
      }
    };
