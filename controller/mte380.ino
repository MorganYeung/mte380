#include "motor.cpp"
#include "ir_sensor.cpp"
#include "MPU9250.h"
#include "movingAvg.h"
MPU9250 IMU(Wire, 0x68);
int status;

long previousMillis = 0; // will store last time LED was updated
float delta_max = pi/2; % max steering angle
float traj_angle = 0 // TODO: set trajectory angle. It's depedent on direction of IMU. basically the direction of what's forward for our robot

movingAvg avgAccelX(5);
movingAvg avgAccelY(5);
movingAvg avgGyroZ(5);
long prevVelX = 0;
long prevVelY = 0;

long interval = 19; // interval at which to blink (milliseconds), 1 ms shorter than desired (time to finish processing)
long dt; // change in time actual (milliseconds)
int k = 5 // gain for controller
const float minGyroValue = 0.25; // min +/-Gyro value, (converted to rad/s)

// TODO: Adjust these values
float dist_bt_IR_sens = 1;
float robot_length = 1;
float dist_from_wall = 5;


int runningAverage(float Average)
{
  static float LM[10];      // LastMeasurements
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;
 
  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = Average;
  sum += LM[index];
  index = index % 10;
  if (count < 10) count++;

  return sum / count;
}


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
  return (len1+len2)/2
}

float integrate(float intVal, float dt, float prevSpeed){
  // integrate based off of running average
  dt = dt/1000
  return (intVal * dt) + prevSpeed;
}

float angleWrap(float angle) {
  // A function that receives an angle and binds it between [-pi, pi].
  float pi = 3.141592654
  return mod(angle + pi)%(2*pi) - pi;
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
  // TODO: convert m/s to cm/s or vice versa
  unsigned long currentMillis = millis(); // read the sensor

  if (currentMillis - previousMillis > interval) {
    dt = currentMillis - previousMillis;
    previousMillis = currentMillis; // save the last time you blinked the LED

    Serial.print(dt);

    line_follow = (dist1+dist2)/2 + dist_from_wall

    IMU.readSensor();
    accelX = avgAccelX.reading(IMU.getAccelX_mss());
    accelY = avgAccelY.reading(IMU.getAccelY_mss());
    angularVel = avgGyroZ.reading(checkDeadbandValue(IMU.getGyroZ_rads(), minGyroValue));

    // integrates here
    speedX = integrate(accelX, dt, prevVelX);
    speedY = integrate(accelY, dt, prevVelY);
    speed = sqrt(pow(speedX,2), pow(speedY,2))
    currHeading = atan2((dist1-dist2)/dist_bt_IR_sens)



    // Figure out steering angle
    float delta = max(-delta_max, min(delta_max, angleWrap(traj_angle - currHeading) + atan2(-k*line_follow, speed))
    float angular_vel =  speed*tan(delta/robot_length);

    // Convert steering angle to angular velocity
    if(angular_vel < 0){
      steer_ratio = max(0.5,  1 + (angular_vel*dist_bt_IR_sens/velocity))
      // left_motor = fullpower;
      // right_motor = fullpower*steer_ratio
    } else {
      steer_ratio = max(0.5, 1 - (angular_vel*dist_bt_IR_sens/velocity));
      // left_motor = fullpower*steer_ratio;
      // right_motor = fullpower;
    }

    Serial.print("\t");
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print(checkDeadbandValue(IMU.getGyroX_rads(), minGyroValue), 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print(checkDeadbandValue(IMU.getGyroY_rads(), minGyroValue), 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print(checkDeadbandValue(IMU.getGyroZ_rads(), minGyroValue), 6);

    /*
      Serial.print("\t");
      Serial.print(IMU.getMagX_uT(),6);
      Serial.print("\t");
      Serial.print(IMU.getMagY_uT(),6);
      Serial.print("\t");
      Serial.print(IMU.getMagZ_uT(),6);
      Serial.print("\t");
      Serial.println(IMU.getTemperature_C(),6);
    */
    Serial.println();
  }
}
