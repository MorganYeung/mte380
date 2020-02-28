#include "motor.cpp"
#include "ir_sensor.cpp"
#include "MPU9250.h"
MPU9250 IMU(Wire, 0x68);
int status;

long previousMillis = 0; // will store last time LED was updated
long interval = 19; // interval at which to blink (milliseconds), 1 ms shorter than desired (time to finish processing)
long dt; // change in time actual (milliseconds)
const float minGyroValue = 0.25; // min +/-Gyro value, (converted to rad/s)


float checkDeadbandValue (float val, float minVal)
// return value with deadband
{
  float curVal = val;
  if (val < minVal and val > -minVal)
  {
    curVal = 0;
  }
  return curVal;
}


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
  
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  /* Advanced Items:*/
  // IMU.set___(bias, scale)
  //IMU.setAccelCalX( 0.410690795, 1.000380467); // sets the accelerometer bias (m/s/s) and scale factor in the X direction
  //IMU.setAccelCalY( 5.14925E-05, 1.002806316); // sets the accelerometer bias (m/s/s) and scale factor in the Y direction
  //IMU.setAccelCalZ(-0.178361616, 1.009922876); // sets the accelerometer bias (m/s/s) and scale factor in the Z direction

  //IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G); // setting the accelerometer full scale range to +/-8G
  //IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS); // setting the gyroscope full scale range to +/-500 deg/s
  //IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ); // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ); // setting DLPF bandwidth to 5 Hz
  IMU.setSrd(19); // setting SRD to 19 for a 50 Hz update rate

  Serial.println("IMU Check");
  Serial.print("Accelerometer bias in the X direction, m/s/s: "); Serial.println(IMU.getAccelBiasX_mss(), 6);
  Serial.print("Accelerometer scale factor in the X direction "); Serial.println(IMU.getAccelScaleFactorX(), 6);
  Serial.print("Accelerometer bias in the Y direction, m/s/s ");  Serial.println(IMU.getAccelBiasY_mss(), 6);
  Serial.print("Accelerometer scale factor in the Y direction "); Serial.println(IMU.getAccelScaleFactorY(), 6);
  Serial.print("Accelerometer bias in the Z direction, m/s/s ");  Serial.println(IMU.getAccelBiasZ_mss(), 6);
  Serial.print("Accelerometer scale factor in the Z direction "); Serial.println(IMU.getAccelScaleFactorZ(), 6);
  delay(500); 
}

void loop() {
  Motor motors;
  IR_Sensor sensor;
  float data[4];
  float front_distance;
  float side_distance;
  int coordinateX = 1, coordinateY = 1;
  //get average of ToF reading for all sensors
  for(int i = 1; i <5; i++){
    float average = 0;
    for(int x=0; x < 5; x++) {
      average += sensor.read(i);
    }
    data[i-1] = average /= 5;
  }
  
  //switch between extra long range and long range sensors
  if (data[3] > 105) front_distance = data[3];
  else front_distance = data[0];
  side_distance = (data[1] + data[2])/2;
  
  IMU.readSensor();
  //based on orientation this will change
  coordinateX = front_distance/30;
  coordinateY = side_distance/30;
  
//  Serial.print(average);
//  Serial.print("\n");
//  delay(1000);

  //if too close, do a 3-point turn
  if(front_distance <= 20) {
      motors.move_backwards(200);
      delay(700);
      motors.turn_right();
      delay(500);
   }

  //turn normally
  else if(front_distance <= 40) {
    motors.turn_right();
    delay(700);
  }
  //otherwise just go
  else {
    motors.move_forwards(230);
  }

}
