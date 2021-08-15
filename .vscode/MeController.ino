/**
 *
 */
#include <Filters.h>
#include "MeMegaPi.h"
#include <Wire.h>

// Parameters
  double setPoint = -4.5;
  double kP = 10;
  double kI = 0.02;
  double kD = .00;
  int maxAngle = 15; //+- angle range that will attempt to balance
  int deadBand = 0.0; //+- angle range that will cause no response
  int loopTime = 10;
  double pidMax = 50; //Max PID value before saturation
  
  int motorMax = 255; // motorMin-255
  int motorMin = 20; // 0 - motorMax
  
  float accelLPFfreq = 2; //Hz
 
  int joystickXoffset = 10;
  int joystickYoffset = 20;
  int joystickGain = 256/motorMax;
MeGyro gyro;
MePotentiometer myPotentiometer(PORT_7);
MeMegaPiDCMotor motor1(PORT1B);
MeMegaPiDCMotor motor2(PORT2B);
MeJoystick joystick(PORT_8);
FilterOnePole filterOneLowpass( LOWPASS, accelLPFfreq);
// RunningStatistics filterStats; 

  int motorSpeed = 0;
  int16_t x = 0;    /* a variable for the Joystick's x value */
  int16_t y = 0;    /* a variable for the Joystick's y value */

  int accScale = 1675;
  double loopRate = 0.1;
  double currentOffset = 0; //0 - 972
  double prevOffset = 0; //0 - 972
  double P = 0;
  double I = 0;
  double D = 0;
  double pidOut = 0;
  double clampedOut = 0;
  double currentError = 0;
  double prevError = 0;
  unsigned long prevTime = millis();
  unsigned long currentTime = 0;
  double angX = 0;
  double angY = 0;
  double angZ = 0;
  double accX = 0;
  double accY = 0;
  double accZ = 0;
  double accYf = 0;
void setup()
{
  Serial.begin(115200);
  gyro.begin();
}

void loop()
{
  gyro.update();
  angX = gyro.getAngleX();
  angY = gyro.getAngleY();
  angZ = gyro.getAngleZ();
  accX = gyro.getAccX()/accScale;
  accY = gyro.getAccY()/accScale;
  accZ = gyro.getAccZ()/accScale;
  x = (joystick.readX()+joystickXoffset)*joystickGain;
  y = (joystick.readY()+joystickYoffset)*joystickGain;
  filterOneLowpass.input( accY );
  accYf = filterOneLowpass.output();
  currentOffset = myPotentiometer.read();
  currentTime = millis();
  currentError = setPoint - angY;
  setPoint = setPoint + (currentOffset-prevOffset)/972*5;
  // PID CONTROL BALANCE
  P = kP * currentError;
  I = I + kI * currentError * (currentTime - prevTime);
  D = kD * (currentError - prevError)/(currentTime - prevTime);
  pidOut = P + I + D;
  //Saturation
  if(I > pidMax) I = pidMax; 
  else if(I < -pidMax) I = -pidMax;

  // Deadband
  if(currentError > -setPoint-deadBand && currentError < setPoint+deadBand) motorSpeed = y;
  // Give up if over max angle
  else if(angY< -maxAngle || angY > maxAngle) motorSpeed = 0;
  // Else adjust speed according to PID
  else motorSpeed = pidOut * (motorMax - motorMin)/pidMax + motorMin +y;
  
  // motorSpeed = pidOut * motorMax / maxAngle;
  #if 1 // disable motor output
  motor1.run(-motorSpeed+x); /* value: between -255 and 255. */
  motor2.run(motorSpeed+x); /* value: between -255 and 255. */
  #endif
  prevOffset = currentOffset;
  prevTime = currentTime;
  prevError = currentError;
#if 0 
  Serial.print(" P: ");
  Serial.print(P);
  Serial.print(" I: ");
  Serial.print(I);
  Serial.print(" D: ");
  Serial.print(D);
  Serial.print(" PID: ");
  Serial.print(pidOut);
  Serial.print(" Pot: ");
  Serial.print(setPoint);
  Serial.print(" Angle: ");
  Serial.println(angY);
#endif
#if 1 
  Serial.print(" Pot: ");
  Serial.print(setPoint);
  Serial.print(" Angle: ");
  Serial.print(angY);
  Serial.print(" PID: ");
  Serial.println(motorSpeed/10);
#endif
#if 0
Serial.print(" Xang:");
  Serial.print(angX);
  Serial.print(" Yang:");
  Serial.print(angY);
  Serial.print(" Zang:");
  Serial.println(angZ);
#endif
#if 0
  Serial.print(" Xacc:");
  Serial.print(accX/accScale );
  Serial.print(" Yacc:");
  Serial.print(accY/accScale);
  Serial.print(" Zacc:");
  Serial.println(accZ()/accScale );
#endif
#if 0
  Serial.print(" joyY");
  Serial.println(y);
//  Serial.print(" Yaccf:");
//  Serial.println(accYf );
#endif
  delay(loopTime);
}
