
// 2017-04-12 -- ADD: motorRunDiscretePosZ (motor, dir, posZ) gyroZangle
// 2017-04-12 -- ADD: motorRunDiscretePosY (motor, dir, posY)
// 2017-04-12 -- MOD: motorRunTime (with Pos)
// 2017-04-12 -- ADD: motorRunPos

// 2017-04-11 -- TODO:  updatePosition insert into motorRun
// 2017-04-11 -- ISSUE: TimerOne Interrupt doesn't work.


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Kalman.h"

void trackSurface(int angle);
long usonicDistanceCm();
void initMotorH();
void initMotorR();
void testMotorAll();
void setMotorDirection(uint8_t motor, uint8_t motorDir);
void motorOff(uint8_t motor, uint8_t reason);
uint8_t motorRun(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed);
uint8_t motorRunTime(uint8_t motor, uint8_t motorDir, int runTime);
uint8_t motorRunTimeUSonic(uint8_t motor, uint8_t motorDir, int runTime);
uint8_t motorRunUSonic(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed);
uint8_t motorRunUSonicDist(uint8_t motor, uint8_t motorDir, int runDist);


void intButton();
long usonicDistanceValue = 100;

#define PIN_LED 13

// MOTORs

// Delay for single motor run, ms
#define STEP_MS 10

// Motor names
#define MOTOR_G 0
#define MOTOR_H 1
#define MOTOR_R 2
#define MOTOR_V 3
#define MOTOR_ALL 255

// Motor Directions names
#define DIR_LOOSE 0
#define DIR_TIGHT 1
#define DIR_BACK 0
#define DIR_FRWD 1
#define DIR_UP 0
#define DIR_DN 1
#define DIR_CW 0
#define DIR_CCW 1

// Motor OFF Reasons
#define STOP_REASON_BLOCK 0
#define STOP_REASON_SENS_IR 1
#define STOP_REASON_TIME 2
#define STOP_REASON_POS 3
#define STOP_REASON_BUTTON 4
#define STOP_REASON_USONIC 5

// IFRARED SENSOR
#define PIN_IR 4

// ULTRASONIC SENSOR
#define PIN_TRIG 7
#define PIN_ECHO 8
#define DISTANCE_MIN 7

// SHIFT REGISTER
#define PIN_SER 13
#define PIN_LATCH 12
#define PIN_CLK 11

// MOTORs PWM and CURRENT SENSORS
uint8_t pinPWM[4] = {5, 6, 9, 10};
uint8_t pinCS[4] = {0, 1, 2, 3};

// MOTOR DIRECTIONS NAMES
char* motorDirName[] = {"LOOSE", "TIGHT", "BACK ", "FRWD ", "CW   ", "CCW  "};

// MOTOR NAMES
char motorName[]= {'G', 'H', 'R'};

// MOTOR MATRIX EXAMPLES
// SpeedMatrix Format Example:
// motorSpeedStart[8] = {MOTOR_G__DIR_LOOSE, MOTOR_G__DIR_TIGHT,
//                       MOTOR_H__DIR_BACK,  MOTOR_H__DIR_FRWD,
//                       MOTOR_R__DIR_CW, MOTOR_R__DIR__CCW,
//                       MOTOR_V__DIR_UP,  MOTOR_V__DIR_DN,};

// MOTOR CURRENT MATRIX
uint8_t motorCurrentMax[8] =    {150, 150,    150, 160,   90, 90,   100, 100};

// MOTOR TIME MATRIX ms
int motorTimeMax[8] =    {3000, 3000, 3000, 3000, 3000, 3000};

// MOTOR SPEED MATRIX
uint8_t motorSpeedStart[8] = { 120,  120,    80,  80,       50,   50,    20, 20};
uint8_t motorSpeedMax[8] =   {180, 180,    110,  110,     80,   60,    40, 40};
uint8_t motorSpeedMin[8] =   {120, 120,    30,  30,       60,   50,    30, 30};
uint8_t motorSpeedInc[4] = {1, 1, 1, 1};
uint8_t motorSpeedDelay[4] = {250, 250, 250, 250};

uint8_t motorDir[4] = {DIR_LOOSE, DIR_BACK, DIR_UP, DIR_CW};
int motorCurrentSensorValue;

// MPU Initialization

// Comment out to restrict roll to ±90deg
#define RESTRICT_PITCH

// Create the Kalman instances
Kalman kalmanX, kalmanY, kalmanZ;

const uint8_t MPU6050 = 0x68;
const uint8_t HMC5883L = 0x1E;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
int16_t tempRaw;

// Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer
double roll, pitch, yaw;
double startY, endY;
double startZ, endZ;

// Angle calculate using the gyro only
double gyroXangle, gyroYangle, gyroZangle;

// Calculated angle using a complementary filter
double compAngleX, compAngleY, compAngleZ;

// Calculated angle using a Kalman filter
double kalAngleX, kalAngleY, kalAngleZ;

uint32_t timer;
// Buffer for I2C data
uint8_t i2cData[14];

// Calibrated Values
#define MAG0MAX 540
#define MAG0MIN -376

#define MAG1MAX 303
#define MAG1MIN -628

#define MAG2MAX 531
#define MAG2MIN -367

float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];

// Used to check for errors in I2C communication
const uint16_t I2C_TIMEOUT = 1000;


volatile bool flagIntButton = true;

// OFF Button Interrupt
void intButton()
{
  Serial.println();
  Serial.println(F(" INT: BUTTON "));
  digitalWrite(PIN_LED, LOW);
  motorOff(MOTOR_ALL, STOP_REASON_BUTTON);
}


// I2C Functions
uint8_t i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(address, registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
    Serial.println("STOP");
    delay(10000);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    delay(10000);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(address, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}


// MPU Functions
void updateMPU6050() {
  while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = -((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = -(i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = -(i2cData[12] << 8) | i2cData[13];
}

void updateHMC5883L() {
  while (i2cRead(HMC5883L, 0x03, i2cData, 6)); // Get magnetometer values
  magX = ((i2cData[0] << 8) | i2cData[1]);
  magZ = ((i2cData[2] << 8) | i2cData[3]);
  magY = ((i2cData[4] << 8) | i2cData[5]);
}

void updatePitchRoll() {
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;

  magX *= magGain[0];
  magY *= magGain[1];
  magZ *= magGain[2];

  magX -= magOffset[0];
  magY -= magOffset[1];
  magZ -= magOffset[2];

  double rollAngle = kalAngleX * DEG_TO_RAD;
  double pitchAngle = kalAngleY * DEG_TO_RAD;

  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

  yaw *= -1;
}

void calibrateMag() { // Inspired by: https://code.google.com/p/open-headtracker/
  i2cWrite(HMC5883L, 0x00, 0x11, true);
  delay(100); // Wait for sensor to get ready
  updateHMC5883L(); // Read positive bias values

  double magPosOff[3] = { magX, magY, magZ };

  i2cWrite(HMC5883L, 0x00, 0x12, true);
  delay(100); // Wait for sensor to get ready
  updateHMC5883L(); // Read negative bias values

  double magNegOff[3] = { magX, magY, magZ };

  i2cWrite(HMC5883L, 0x00, 0x10, true); // Back to normal

  magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);
  magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);


  magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);

#if 1
  Serial.print("Mag cal: ");
  Serial.print(magNegOff[0] - magPosOff[0]);
  Serial.print(",");
  Serial.print(magNegOff[1] - magPosOff[1]);
  Serial.print(",");
  Serial.println(magNegOff[2] - magPosOff[2]);
  Serial.print("Gain: ");
  Serial.print(magGain[0]);
  Serial.print(",");
  Serial.print(magGain[1]);
  Serial.print(",");
  Serial.println(magGain[2]);
#endif
}

void initMPU() {
  delay(100); // Wait for sensors to get ready

  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  Wire.beginTransmission(0x68);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  while (i2cRead(MPU6050, 0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  // Configure device for continuous mode
  while (i2cWrite(HMC5883L, 0x02, 0x00, true));
  calibrateMag();

  // Wait for sensors to stabilize
  delay(100);

  // STARTING ANGLE
  updateMPU6050();
  updateHMC5883L();
  updatePitchRoll();
  updateYaw();

  // X, ROLL
  kalmanX.setAngle(roll);
  gyroXangle = roll;
  compAngleX = roll;

  // Y, PITCH
  kalmanY.setAngle(pitch);
  gyroYangle = pitch;
  compAngleY = pitch;

  // Z, YAW
  kalmanZ.setAngle(yaw);
  gyroZangle = yaw;
  compAngleZ = yaw;

  timer = micros(); // Initialize the timer
}

void updatePosition()
{
  updateMPU6050();
  updateHMC5883L();

  // Calculate delta time
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  /* Roll and pitch estimation */
  updatePitchRoll();

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif


  /* Yaw estimation */
  updateYaw();
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s
  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);
    compAngleZ = yaw;
    kalAngleZ = yaw;
    gyroZangle = yaw;
  } else
  // Calculate the angle using a Kalman filter
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);


  // Estimate angles using gyro only
  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;

  // Calculate gyro angle using the unbiased rate from the Kalman filter
  gyroXangle += kalmanX.getRate() * dt;
  gyroYangle += kalmanY.getRate() * dt;
  gyroZangle += kalmanZ.getRate() * dt;

  // Estimate angles using complimentary filter
  // Calculate the angle using a Complimentary filter
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;
}

void printPosition()
{
  Serial.print("  \tX: ");
  Serial.print(roll); Serial.print(" ");
  //Serial.print(gyroXangle); Serial.print(" ");
  //Serial.print(compAngleX); Serial.print(" ");
  //Serial.print(kalAngleX); Serial.print(" ");

  Serial.print("\tY: ");
  Serial.print(pitch); Serial.print(" ");
  //Serial.print(gyroYangle); Serial.print(" ");
  // Serial.print(compAngleY); Serial.print(" ");
  // Serial.print(kalAngleY); Serial.print(" ");

  Serial.print("\tZ: ");
  Serial.print(yaw); Serial.print(" ");
  Serial.print(gyroZangle); Serial.print(" ");
  Serial.print(compAngleZ); Serial.print(" ");
  Serial.print(kalAngleZ); Serial.print(" ");

  // After gain and offset compensation
  /*
  Serial.print("\tMagXYZ: ");
  Serial.print(magX); Serial.print(" ");
  Serial.print(magY); Serial.print(" ");
  Serial.print(magZ); Serial.print(" ");
  Serial.print("\tT: ");
  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature);
  */
}

// SETUP

void setup()
{

  // INIT Serial Debug
  Serial.begin(115200);
  Serial.println(F("START"));

  // INIT PINs
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SER, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);

  // Interrupt Button
  attachInterrupt(1, intButton, RISING);
  Serial.println(F("Button interrupt attach DONE"));

  // INIT MPU
  initMPU();
  Serial.println(F("MPU Init DONE"));

// ****************************************
// ****************************************


updatePosition();
startZ = gyroZangle;
startY = 10;
endY = 60;
/*
motorRunDiscretePosY(MOTOR_H, DIR_FRWD, endY);
delay(1000);

motorRunCurrent(MOTOR_G, DIR_LOOSE);
while (digitalRead(PIN_IR));
motorRunCurrent(MOTOR_G, DIR_TIGHT);

delay(1000);
motorRunDiscretePosY(MOTOR_H, DIR_BACK, startY);

delay(1000);
*/
motorRunDiscretePosZ(MOTOR_R, DIR_CCW, startZ-50);
delay(1000);
motorRunDiscretePosZ(MOTOR_R, DIR_CW, startZ);

Serial.println(F("DONE"));

// ****************************************
// ****************************************
}


void loop()
{
//trackSurface(10);
//delay(250);
}


void motorOff(uint8_t motor, uint8_t reason)
{

 digitalWrite(PIN_LATCH, LOW);
 shiftOut(PIN_SER, PIN_CLK, LSBFIRST, B00000000);
 digitalWrite(PIN_LATCH, HIGH);
 analogWrite(pinPWM[MOTOR_G], 0);
 analogWrite(pinPWM[MOTOR_H], 0);
 analogWrite(pinPWM[MOTOR_R], 0);

 Serial.print(F("MOTOR: "));
 if (motor == MOTOR_ALL)
   Serial.print(F("ALL "));
 else
   Serial.print(motorName[motor]);

 switch (reason)
 {
  case STOP_REASON_BLOCK:
    Serial.println(F(" STOP: BLOCK"));
    break;
  case STOP_REASON_SENS_IR:
    Serial.println(F(" STOP: IR"));
    break;
  case STOP_REASON_TIME:
    Serial.println(F(" STOP: TIME"));
    break;
  case STOP_REASON_POS:
    Serial.println(F(" STOP: POSITION"));
    break;
  case STOP_REASON_BUTTON:
    Serial.println(F(" STOP: BUTTON"));
    break;
  case STOP_REASON_USONIC:
    Serial.println(F(" STOP: USONIC"));
    break;
  default:
    Serial.println(F(" STOP: NA"));
  }

}

void setMotorDirection(uint8_t motor, uint8_t motorDir)
{
   uint8_t shiftDir;
   digitalWrite(PIN_LATCH, LOW);
   shiftOut(PIN_SER, PIN_CLK, MSBFIRST, 0);
   digitalWrite(PIN_LATCH, HIGH);

   shiftDir = (1 << ((2 * motor) + motorDir));
   digitalWrite(PIN_LATCH, LOW);
   shiftOut(PIN_SER, PIN_CLK, MSBFIRST, shiftDir);
   digitalWrite(PIN_LATCH, HIGH);

//   Serial.print(F("  MOTOR: "));
//   Serial.print(motor);
//   Serial.print(F(" SET SHIFT DIR: "));
//   Serial.println(shiftDir);
}


uint8_t motorRun(uint8_t motor, uint8_t motorDir, uint8_t motorSpeed)
{
    int motorCurrentSensorValue;
    Serial.print(F("  MOTOR: "));
    Serial.print(motorName[motor]);
    Serial.print(F("  DIR: "));
    Serial.print(motorDirName[2*motor+motorDir]);
    Serial.print(F("  SPEED: "));
    Serial.print(motorSpeed);

    setMotorDirection(motor, motorDir);
    analogWrite(pinPWM[motor], motorSpeed);
    delay(STEP_MS);
    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.print(motorCurrentSensorValue);

    return motorCurrentSensorValue;
}

uint8_t motorRunPos(uint8_t motor, uint8_t motorDir, uint8_t motorSpeed)
{
    int motorCurrentSensorValue;
    Serial.print(F("  MOTOR: "));
    Serial.print(motorName[motor]);
    Serial.print(F("  DIR: "));
    Serial.print(motorDirName[2*motor+motorDir]);
    Serial.print(F("  SPEED: "));
    Serial.print(motorSpeed);

    setMotorDirection(motor, motorDir);
    analogWrite(pinPWM[motor], motorSpeed);
    delay(STEP_MS);
    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.print(motorCurrentSensorValue);

    updatePosition();
    printPosition();
    
    return motorCurrentSensorValue;
}


uint8_t motorRunCurrent(uint8_t motor, uint8_t motorDir)
{
  uint8_t motorSpeed;
  uint8_t motorCurrentSensorValue;
  boolean flagStart;
  int runTime;

  Serial.print(F("START MOTOR_"));
  Serial.print(motorName[motor]);
  Serial.print(F(" "));
  Serial.print(motorDirName[motorDir]);
  Serial.print(F(" CURRENT"));
  printPosition();
  Serial.println();

  motorCurrentSensorValue = analogRead(pinCS[motor]);
  motorSpeed = motorSpeedStart[2*motor+motorDir];
  flagStart = true;
  runTime = 0;
  while (motorCurrentSensorValue < motorCurrentMax[2*motor+motorDir])
  {
    if (runTime < motorTimeMax[2*motor+motorDir])
      runTime += STEP_MS;
    else
      {
        motorOff(motor, STOP_REASON_TIME);
        return STOP_REASON_TIME;
      }
    motorCurrentSensorValue = motorRun(motor, motorDir, motorSpeed);
    Serial.println();
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor+motorDir]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor+motorDir];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_BLOCK);
  return STOP_REASON_BLOCK;
 }

uint8_t motorRunDiscretePosY (uint8_t motor, uint8_t motorDir, double posY)
{
  int motorSpeed = motorSpeedStart[2*motor];

  Serial.print(F("START MOTOR_"));
  Serial.print(motorName[motor]);
  Serial.print(F(" "));
  Serial.print(motorDirName[2*motor+motorDir]);
  Serial.print(F(" PosY: "));
  Serial.print(posY);
  Serial.println();

  if (motorDir == DIR_FRWD)
  {
    while (pitch < posY)
    {
      motorRunTime(motor, motorDir, 50);
      delay(50);
    }
  }
  else
  {
    while (pitch > posY)
    {
      motorRunTime(motor, motorDir, 50);
      delay(50);
    }
  }
  motorOff(motor, STOP_REASON_POS);
  return STOP_REASON_POS;
}


uint8_t motorRunDiscretePosZ (uint8_t motor, uint8_t motorDir, double posZ)
{
  int motorSpeed = motorSpeedStart[2*motor];

  Serial.print(F("START MOTOR_"));
  Serial.print(motorName[motor]);
  Serial.print(F(" "));
  Serial.print(motorDirName[2*motor+motorDir]);
  Serial.print(F(" PosZ: "));
  Serial.print(posZ);
  Serial.println();

  if (motorDir == DIR_CW)
  {
    while (gyroZangle < posZ)
    {
      motorRunTime(motor, motorDir, 50);
      delay(50);
    }
  }
  else
  {
    while (gyroZangle > posZ)
    {
      motorRunTime(motor, motorDir, 50);
      delay(50);
    }
  }
  motorOff(motor, STOP_REASON_POS);
  return STOP_REASON_POS;
}


uint8_t motorRunUSonicDist(uint8_t motor, uint8_t motorDir, int runDist)
{
  int motorSpeed = motorSpeedStart[2*motor];
  boolean flagStart = true;

  Serial.print(F("START MOTOR_"));
  Serial.print(motorName[motor]);
  Serial.print(F(" "));
  Serial.print(motorDirName[motorDir]);
  Serial.print(F(" DISTANCE: "));
  Serial.print(runDist);
  Serial.println();

  usonicDistanceValue = usonicDistanceCm();
  while ( usonicDistanceValue > runDist)
   {
     usonicDistanceValue = usonicDistanceCm();
     motorCurrentSensorValue = motorRun(motor, motorDir, motorSpeed);
     Serial.print(F("  DISTANCE: "));
     Serial.println(usonicDistanceValue);
     if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }
     if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
       motorSpeed++;
     else
      {
       motorSpeed = motorSpeedMin[2*motor];
       flagStart = false;
      }
    }
  motorOff(motor, STOP_REASON_USONIC);
  return STOP_REASON_USONIC;
}

uint8_t motorRunTime(uint8_t motor, uint8_t motorDir, int runTime)
{
  int i;
  int motorSpeed = motorSpeedStart[2*motor];
  int motorCurrentSensorValue;
  boolean flagStart = true;

  Serial.print(F("START MOTOR_"));
  Serial.print(motorName[motor]);
  Serial.print(F(" "));
  Serial.print(motorDirName[2*motor+motorDir]);
  Serial.print(F(" TIME: "));
  Serial.print(runTime);
  Serial.println();

  for(i=0; i<runTime; i+=STEP_MS)
  {
    motorCurrentSensorValue = motorRunPos(motor, motorDir, motorSpeed);
    Serial.print(F(" TIME: "));
    Serial.println(i);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_TIME);
  return STOP_REASON_TIME;
}

long usonicDistanceCm()
{
  long duration;
  long distanceCm;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  pinMode(PIN_ECHO, INPUT);
  duration = pulseIn(PIN_ECHO, HIGH);
  distanceCm = (duration/2) / 29.1;
  return distanceCm;
}

uint8_t motorRunUSonic(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed)
{
    int motorCurrentSensorValue;
    Serial.print(F("  MOTOR: "));
    Serial.print(motor);
    if (curMotorDir == DIR_LOOSE)
    {
      Serial.print(F("  DIR: L/B/CW "));
    }
    else
    {
      Serial.print(F("  DIR: T/F/CCW"));
    }
    Serial.print(F("  SPEED: "));
    Serial.print(curMotorSpeed);

    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.print(motorCurrentSensorValue);

    usonicDistanceValue = usonicDistanceCm();

    Serial.print("  USONIC: ");
    Serial.println(usonicDistanceValue);

    setMotorDirection(motor, curMotorDir);
    analogWrite(pinPWM[motor], curMotorSpeed);
    delay(STEP_MS);
    return motorCurrentSensorValue;
}

uint8_t motorRunTimeUSonic(uint8_t motor, uint8_t motorDir, int runTime)
{
  int i;
  int motorSpeed = motorSpeedStart[2*motor];
  boolean flagStart = true;

  for(i=0; i<runTime; i+=STEP_MS)
  {

    motorCurrentSensorValue = motorRunUSonic(motor, motorDir, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_TIME);
  return STOP_REASON_TIME;
}

/*
uint8_t motorRunGyroZ(uint8_t motor, uint8_t motorDir, float posZ)
{
  uint8_t motorSpeed;
  uint8_t motorCurrentSensorValue;
  boolean flagStart;
  int runTime;

  Serial.print(F("START MOTOR_"));
  Serial.print(motorName[motor]);
  Serial.print(F(" "));
  Serial.print(motorDirName[motorDir]);
  Serial.println(F(" GYRO_Z"));

  motorSpeed = motorSpeedStart[2*MOTOR_H];
  flagStart = true;
  while (!posZ)
  {
    if (mpuInterrupt)
    {
      while(!getMPUGyroZ());
      curMPUGyroZ = mpuGyroZ;
      Serial.print(F("  CurMpuGyroZ = "));
      Serial.print(curMPUGyroZ);
      if (curMPUGyroZ >= GYRO_MAX_UP)
        startPosition = true;
     }
    else
    {
     motorCurrentSensorValue = motorRun(MOTOR_H, DIR_BACK, motorSpeed);
     if (flagStart && (motorSpeed < motorSpeedMax[2*MOTOR_H]))
       motorSpeed++;
     else
     {
      motorSpeed = motorSpeedMin[2*MOTOR_H];
      flagStart = false;
     }
    }
  }

  motorOff(MOTOR_H, STOP_REASON_GYRO);
  return STOP_REASON_GYRO;

 }
*/

/*
void trackSurface(int angle)
{

  // Track surface look most higher point on it
  // Rotate while Angle

  boolean flagStart = true;
  boolean flagStartPosition = true;
  long lowUSonicDistanceCm;
  long curUSonicDistanceCm;
  float startMPUGyroX;
  float endMPUGyroX;
  float curMPUGyroX;
  float lowMPUGyroX;
  uint8_t motorSpeed;
  uint8_t motor = MOTOR_R;

  // Save init gyro position

  if (mpuInterrupt && flagStartPosition)
    {
      while(!getMPUGyroXYZ());
      startMPUGyroX = mpuGyroX;
      if (startMPUGyroX != 0)
        {
          flagStartPosition = false;
          Serial.print(F("  startMpuGyroX = "));
          Serial.println(startMPUGyroX);
        }
    }

  // End gyro Position
  endMPUGyroX = startMPUGyroX + angle;
  if (endMPUGyroX > 180)
    endMPUGyroX -= endMPUGyroX - 360;
  Serial.print(F("  endMpuGyroX = "));
  Serial.print(endMPUGyroX);

  // Save cur_usonicDistanceCm
   lowUSonicDistanceCm = usonicDistanceCm();
   Serial.print(F("  LowUSonic = "));
   Serial.println(lowUSonicDistanceCm);


  // RUN MOTOR_R CW till endMPUGyroX
  motorSpeed = motorSpeedStart[2*motor];

  curMPUGyroX = startMPUGyroX;
  lowMPUGyroX = startMPUGyroX;
  while(curMPUGyroX < endMPUGyroX)
  {
     if (mpuInterrupt)
      {
        while(!getMPUGyroXYZ());
        curMPUGyroX = mpuGyroX;
        if (startMPUGyroX != 0)
          {
            Serial.print(F("  curMpuGyroX = "));
            Serial.print(curMPUGyroX);
          }
      }

   curUSonicDistanceCm = usonicDistanceCm();
   Serial.print(F("  curUSonic = "));
   Serial.println(curUSonicDistanceCm);

   // Find lowest_usonicDistanceCm
   if  (curUSonicDistanceCm < lowUSonicDistanceCm)
    {
      lowUSonicDistanceCm = curUSonicDistanceCm;
      lowMPUGyroX = curMPUGyroX;
    }
    motorCurrentSensorValue = motorRun(motor, DIR_CW, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
      }
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_GYRO);
   Serial.print(F("  lowUSonicDistanceCm = "));
   Serial.print(lowUSonicDistanceCm);
   Serial.print(F("  lowMPUGyroX = "));
   Serial.println(lowMPUGyroX);

   delay(1500);

  // RUN MOTOR_R CCW till lowest_gyro_x

  while(curMPUGyroX > (lowMPUGyroX + 15))
  {
     if (mpuInterrupt)
      {
        while(!getMPUGyroXYZ());
        curMPUGyroX = mpuGyroX;
        if (startMPUGyroX != 0)
          {
            Serial.print(F("  curMpuGyroX = "));
            Serial.print(curMPUGyroX);
          }
      }

    motorCurrentSensorValue = motorRun(motor, DIR_CCW, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor+1])
      {
        motorOff(motor, STOP_REASON_BLOCK);
      }
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor+1]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor+1];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_GYRO);
  // Read gyro_x and usonicDistanceCm

  // Find lowest_usonicDistanceCm
  // Save lowest_gyro_x for lowest_usonicDistanceCm

  // RUN MOTOR_R CCW till lowest_gyro_x

  // LOOSE MOTOR_G

  motorSpeed = motorSpeedStart[2*MOTOR_G];

  while (motorCurrentSensorValue < motorCurrentMax[2*MOTOR_G])
  {
    motorCurrentSensorValue = motorRun(MOTOR_G, DIR_LOOSE, motorSpeed);
  }
  motorOff(MOTOR_G, STOP_REASON_BLOCK);

  motorRunTime(MOTOR_G, DIR_TIGHT, 40);
  motorOff(MOTOR_G, STOP_REASON_TIME);
  delay(500);
  // FRWD MOTOR_H


  // **** MOTOR_G TIGHT ****
  Serial.println(F("START MOTOR_G TIGHT"));
  motorCurrentSensorValue = analogRead(pinCS[MOTOR_G]);
  motorSpeed = motorSpeedStart[2*MOTOR_G+1];
  flagStart = true;
  while (motorCurrentSensorValue < motorCurrentMax[2*MOTOR_G+1])
  {

    motorCurrentSensorValue = motorRun(MOTOR_G, DIR_TIGHT, motorSpeed);

    if (flagStart && (motorSpeed < motorSpeedMax[2*MOTOR_G+1]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*MOTOR_G+1];
      flagStart = false;
    }
  }

  motorOff(MOTOR_G, STOP_REASON_BLOCK);
  delay(500);

  // TIGHT MOTOR_G

  // BACK MOTOR_H

}
*/

// ****** TESTS AND INITS

void initMotorR()
{
  Serial.println(F("INIT MOTOR_R"));

 // motorRunTimeUSonic(MOTOR_R, DIR_CW, 1500);
  delay(500);
   motorRunTimeUSonic(MOTOR_R, DIR_CCW, 1000);
  motorRunTimeUSonic(MOTOR_R, DIR_CW, 1500);
  delay(500);
   motorRunTimeUSonic(MOTOR_R, DIR_CCW, 1500);

 // motorRunTimeUSonic(MOTOR_R, DIR_CW, 1500);
  delay(500);
   motorRunTimeUSonic(MOTOR_R, DIR_CCW, 3500);

  Serial.print("DONE INIT MOTOR_R");
  delay(1000);
}

void initMotorH()
{

  Serial.println(F("INIT MOTOR_H"));
  motorRunTime(MOTOR_H, DIR_FRWD, 100);
  //motorRunTime(MOTOR_H, DIR_BACK, 100);

  //  delay(2000);
//motorRunTime(MOTOR_H, DIR_BACK, 1000);

 // motorRunUSonicDist(MOTOR_H, DIR_FRWD, 8);
  Serial.print("DONE INIT MOTOR_H");
  delay(1000);
}


void testMotorAll ()
{

  motorRun(MOTOR_H, DIR_FRWD,  70);
  delay(1000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(100);

  motorRun(MOTOR_G, DIR_LOOSE,  50);
  delay(2000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(100);
  motorRun(MOTOR_G, DIR_TIGHT,  50);
  delay(2000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

  motorRun(MOTOR_R, DIR_CW,  100);
  delay(750);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);
  motorRun(MOTOR_R, DIR_CCW,  70);
  delay(750);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

  motorRun(MOTOR_H, DIR_BACK,  50);
  delay(900);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

}

