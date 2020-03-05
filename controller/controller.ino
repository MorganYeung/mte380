#include "motor.cpp"
#include "ir_sensor.cpp"
#include "MPU9250.h"
#include "movingAvg.h"
MPU9250 IMU(Wire, 0x68);
int status;

float pi = 3.141592654;
long previousMillis = 0; // will store last time LED was updated
float delta_max = pi/2; // max steering angle
float traj_angle = 0; // TODO: set trajectory angle. It's depedent on direction of IMU. basically the direction of what's forward for our robot

movingAvg avgAccelX(5);
movingAvg avgAccelY(5);
movingAvg avgGyroZ(5);
movingAvg avgIR1(20);
movingAvg avgIR2(20);
float prevVelX = 0;
float prevVelY = 0;
float prevHeading = 0;

long interval = 19; // interval at which to blink (milliseconds), 1 ms shorter than desired (time to finish processing)
long dt; // change in time actual (milliseconds)
int k = 5; // gain for controller
const float minGyroValue = 0.25; // min +/-Gyro value, (converted to rad/s)

// TODO: Adjust these values
float dist_bt_IR_sens = 1;
float robot_length = 1;
float dist_from_wall = 20;


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

float distanceToLineSegment(float len1, float len2){
  // returns the distance robot is from the wall
  // TODO: maybe do a running average?
  return (len1+len2)/2;
}

float integrate(float intVal, float dt, float prevSpeed){
  // integrate based off of running average
  dt = dt/1000; 
  return (intVal * dt) + prevSpeed;
}

float angleWrap(float angle) {
  // A function that receives an angle and binds it between [-pi, pi].
  // TODO: proper angle wrap function
  //return (angle + pi)%(2*pi) - pi;
  return angle;
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

  // start moving average
  avgAccelX.begin();
  avgAccelY.begin();
  avgGyroZ.begin();
  avgIR1.begin();
  avgIR2.begin();
  
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  /* Advanced Items:*/
  //IMU.set___(bias, scale)
  //IMU.calibrateAccel();
  IMU.setAccelCalX( 0.83, 1.000380467); // sets the accelerometer bias (m/s/s) and scale factor in the X direction
  IMU.setAccelCalY( 0.66, 1.002806316); // sets the accelerometer bias (m/s/s) and scale factor in the Y direction
  //  Left is positive y, Forward is positive x
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
  // initialize vars
  float accelX;
  float accelY;
  float new_angular_vel;
  float delta;
  Motor motors;
  IR_Sensor ir_sensor;
  float left_sensor1; //front dist sensor
  float left_sensor2; //back dist sensor
  float front_sensor;
  float speedX;
  float speedY;
  float velocity;
  float currHeading;
  float line_follow;
  float steer_ratio;
  float curr_angular_vel;
  
  unsigned long currentMillis = millis();

  // TODO: convert m/s to cm/s or vice versa
  if (currentMillis - previousMillis > interval) {
    dt = currentMillis - previousMillis;
    previousMillis = currentMillis; // save the last time you blinked the LED

    

    // read sensor values here and convert to cm/s
    IMU.readSensor();
    accelX = IMU.getAccelX_mss();
    accelY = IMU.getAccelY_mss();
    left_sensor1 = ir_sensor.read(2);
    left_sensor2 = ir_sensor.read(3);
    front_sensor = ir_sensor.read(1);
    curr_angular_vel = checkDeadbandValue(IMU.getGyroZ_rads(), minGyroValue);

    line_follow = (left_sensor1+left_sensor2)/2 + dist_from_wall; // line to follow from wall
    
    // integrates here
    speedX = integrate(accelX, dt, prevVelX);
    prevVelX = speedX;
    speedY = integrate(accelY, dt, prevVelY);
    prevVelY = speedY;
    //velocity = sqrt(pow(speedX,2) + pow(speedY,2));
    velocity = 1;

    // currHeading = atan2((left_sensor1-left_sensor2),dist_bt_IR_sens);
    currHeading = integrate(curr_angular_vel, dt, prevHeading);
    prevHeading = currHeading;

    // Figure out steering angle
    delta = max(-delta_max, min(delta_max, angleWrap(traj_angle - currHeading) + atan2(-k*line_follow, velocity)));
    // Convert steering angle to angular velocity
    new_angular_vel =  velocity*tan(delta/robot_length);

    // Control the left and right motors
    //motors.move_forwards(255);
    if(new_angular_vel < 0){
      steer_ratio = max(0.5,  1 + (new_angular_vel*dist_bt_IR_sens/velocity));
      motors.move_left_forwards(255);
      motors.move_right_forwards(int(255*steer_ratio));
      Serial.print("left_steer_ratio: 1");
      Serial.print("\t");
      Serial.print("right_steer_ratio: ");
      Serial.print(steer_ratio);
      Serial.print("\t");
      Serial.print(left_sensor1, 6);
      Serial.print("\t");
      Serial.print(left_sensor2, 6);
    } else {
      steer_ratio = max(0.5, 1 - (new_angular_vel*dist_bt_IR_sens/velocity));
      motors.move_left_forwards(int(255*steer_ratio));
      motors.move_right_forwards(255);
      Serial.print("left_steer_ratio: ");
      Serial.print(steer_ratio);
      Serial.print("\t");
      Serial.print("right_steer_ratio: 1");
      Serial.print("\t");
      Serial.print(left_sensor1, 6);
      Serial.print("\t");
      Serial.print(left_sensor2, 6);
    }
    Serial.print("\t");
    Serial.print(currHeading);
    Serial.print("\t");
    Serial.print(velocity);
    Serial.println();
    // Printing data here
    /*
    Serial.print(dt);
    Serial.print("\t");
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(speedX, 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print(speedY, 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print(left_sensor1, 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print(left_sensor2, 6);
    Serial.print("\t");
    Serial.print(front_sensor, 6);
    Serial.println();
    */
    
  }
}
