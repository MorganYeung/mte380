#include "motor.cpp"
void setup() {
  const int directionA = 12;
  const int directionB = 13;
  const int pwmA = 3;
  const int pwmB = 11;
  const int brakeA = 9;
  const int brakeB = 8;
      
  pinMode(directionA, OUTPUT);
  pinMode(directionB, OUTPUT); 
  pinMode(pwmA, OUTPUT); 
  pinMode(pwmB, OUTPUT); 
  pinMode(brakeA, OUTPUT); 
  pinMode(brakeB, OUTPUT); 
  
}

void loop() {
  Motor motors;

  motors.move_forwards(255);

  delay(3000);

  motors.move_backwards(255);

  delay(3000);

  motors.brake();

  delay(1000);

}
