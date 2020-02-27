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

  //get average of ToF reading
  float average = 0;
  for(int x=0; x < 10; x++) {
    average += front.read(1);
  }
  average /= 10;

//  Serial.print(average);
//  Serial.print("\n");
//  delay(1000);

  //if too close, do a 3-point turn
  if(average <= 20) {
      motors.move_backwards(200);
      delay(700);
      motors.turn_right();
      delay(500);
   }

  //turn normally
  else if(average <= 40) {
    motors.turn_right();
    delay(700);
  }

  //otherwise just go
  else {
    motors.move_forwards(230);
  }

}
