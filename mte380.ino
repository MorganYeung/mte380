#include "motor.cpp"
#include "small_laser.cpp"
void setup() {
  Serial.begin(115200);
  
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
  SmallLaser front;

  float average = 0;
  for(int x=0; x < 10; x++) {
    average += front.read(1);
  }
  average /= 10;

//  Serial.print(average);
//  Serial.print("\n");
//  delay(1000);

  if(average <= 20) {
      motors.move_backwards(200);
      delay(700);
      motors.turn_right();
      delay(500);
   }
  
  else if(average <= 40) {
    motors.turn_right();
    delay(700);
  }

  else {
    motors.move_forwards(230);
  }
  
//  motors.move_forwards(255);
////  motors.turn_right();
//
//  delay(2000);
//
//  int average = 0;
//  for(int x=0; x < 10; x++) {
//    average += front.read(1);
//  }
//  average /= 10;
//
//  if(average <= 20) {
//    motors.turn_right();
//    delay(2000);
//  }
//  else if(average <= 10) {
//    motors.move_backwards(255);
//    delay(1000);
//    motors.turn_right();
//    delay(2000);
//  }

//  int reading = front.read(1);
//
//  Serial.println(reading);
//  delay(1000);
//
//  if(reading < 20 || reading > 10) {
//    do{
//      motors.turn_right();
//    } while(front.read(1) < 20);
//  }
  
//
//  delay(3000);
//
//  motors.move_backwards(255);
//
//  delay(3000);
//
//  motors.brake();
//
//  delay(1000);

  

}
