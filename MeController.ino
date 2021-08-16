
#include "MeMegaPi.h"
#include <Wire.h>
#include <Filters.h>

#define REVSP 0  //1.52
#define FWDSP 0  //3.72
#define MOTORTYPE 0  //set 1 for DC motor, 0 for EncoderMotor.
//#define TILTFWD -1.83

enum States{STOP, RUN, REVERSE, PAUSE};
enum States currentState = STOP;
enum States prevState = STOP;

MeGyro gyro;
// MeUltrasonicSensor ultraSensor(PORT_7);
MePotentiometer myPotentiometer(PORT_7);
MePotentiometer myPotentiometer2(PORT_8);
Me7SegmentDisplay disp(PORT_5);
MeEncoderOnBoard motor1(SLOT1);
MeEncoderOnBoard motor2(SLOT2);
double distTravelled;
//PID params
double pidBias = 0.5; //0: angle only, 1: speed only, 0.5: equal weight
    //Angle
double setPoint1 = REVSP; //deg
  double currentOffset1 = 0; //0 - 972
  double currentOffset2 = 0; //0 - 972
  double prevOffset = 0; //0 - 972

double kP1 = 11;
double kI1 = 0.1;
double kD1 = 0.0;
double pidMax1 = 100; //Max PID value before saturation

int maxAngle = 30; //+- angle range that will attempt to balance
    //Speed
double setPoint2 = 8;  //
double kP2 = 15.0;
double kI2 = 0.15;
double kD2 = 0.0;
double pidMax2 = 255; //Max PID value before saturation
//Motor params
double wheelCorrect=1.12;
double motorMaxRPM = 255;

//flags
unsigned int flagEnterState = false;
unsigned int flagExitState = FALSE;
unsigned int flagHasTravelled5m = FALSE;
unsigned int flagHasTipped = FALSE;
unsigned int flagHasTurned180 = false;
unsigned int flagObstacleDetected = FALSE;
unsigned int flagRunEnded = FALSE;
//PID vars
unsigned long prevTime = millis();
unsigned long currentTime = 0;
double pidOutFinal = 0;
    //Angle
double P1 = 0;
double I1 = 0;
double D1 = 0;
double sumError1 = 0;
double currentError1 = 0;
double prevError1 = 0;
double pidOut1 = 0;
    //Speed
double P2 = 0;
double I2 = 0;
double D2 = 0;
double sumError2 = 0;
double currentError2 = 0;
double prevError2 = 0;
double pidOut2 = 0;
//Gyro vars
double angX = 0;
double angY = 0;
double angZ = 0;
double accY = 0;
//Motor vars
int motorSpeed = 0;
double motorAvSpeed = 0;
double LPFmotorAvSpeed = 0;
FilterOnePole filterOneLowpass( LOWPASS, 0.25);
//functions
void resetFlags(){
    //flagEnterState = FALSE;
    //flagExitState = FALSE;
    flagHasTravelled5m = FALSE;
    flagHasTurned180 = FALSE;
    flagObstacleDetected = FALSE;
}
//int checkDistance();
//int checkTurnAngle();
//int checkObstacle();
void turn180();
void isr_process_encoder1(void)
{
  if(digitalRead(motor1.getPortB()) == 0)
  {
    motor1.pulsePosMinus();
  }
  else
  {
    motor1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(motor2.getPortB()) == 0)
  {
    motor2.pulsePosMinus();
  }
  else
  {
    motor2.pulsePosPlus();
  }
}
void setup(){
    attachInterrupt(motor1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(motor2.getIntNum(), isr_process_encoder2, RISING);

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    Serial.begin(115200);
    gyro.begin();
}

void loop(){
    //Loop Functions:
    gyro.update();
    currentOffset1 = myPotentiometer.read();
    currentOffset2 = myPotentiometer2.read();
    
    motor1.updateSpeed();
    motor2.updateSpeed();
    motor1.updateCurPos();
    angX = gyro.getAngleX();
    angY = gyro.getAngleY();
    angZ = gyro.getAngleZ();
    accY = gyro.getAccY();
    //setPoint1 = REVSP + (currentOffset1 - prevOffset)/972*2;
    //setPoint1 = REVSP + (currentOffset2 - prevOffset)/972*2;

    //update Flags
    flagHasTipped = (currentError1< -maxAngle || currentError1 > maxAngle)? TRUE : FALSE;
    if (distTravelled>50) flagHasTravelled5m = TRUE;
    if (distTravelled<0 && flagHasTravelled5m) flagHasTurned180 = TRUE;
    // flagHasTurned180 = 0 ? flagHasTravelled5m &&distTravelled<0 : FALSE;
    // flagHasTravelled5m = checkDistance() ? TRUE : FALSE;
    // flagHasTurned180 = checkTurnAngle() ? TRUE : FALSE;
    flagObstacleDetected = 0 ? TRUE : FALSE;
    // flagObstacleDetected = (ultraSensor.distanceCm() < 10.0) ? TRUE : FALSE;

    currentTime = millis();
    // PID CONTROL BALANCE
    currentError1 = setPoint1 - angY;
    sumError1 += currentError1;
    // sumError Saturation
    if(sumError1 > pidMax1) sumError1 = pidMax1; 
    else if(sumError1 < -pidMax1) sumError1 = -pidMax1;
    
    P1 = kP1 * currentError1;
    I1 = kI1 * sumError1 * (currentTime - prevTime);
    D1 = kD1 * (currentError1 - prevError1)/(currentTime - prevTime);
    pidOut1 = P1 + I1 + D1;
    
    // PID Saturation
    if(pidOut1 > pidMax1) pidOut1 = pidMax1; 
    else if(pidOut1 < -pidMax1) pidOut1 = -pidMax1;

    // PID CONTROL SPEED
    motorAvSpeed = (motor1.getCurrentSpeed()+motor2.getCurrentSpeed())/2;
    filterOneLowpass.input(motorAvSpeed);
    LPFmotorAvSpeed = -filterOneLowpass.output();
    currentError2 = setPoint2 - LPFmotorAvSpeed;
    sumError2 += currentError1;
    // sumError Saturation
    if(sumError2 > pidMax1) sumError2 = pidMax1; 
    else if(sumError2 < -pidMax1) sumError2 = -pidMax1;
    P2 = kP2 * currentError2;
    I2 = kI2 * sumError2 * (currentTime - prevTime);
    D2 = kD2 * (currentError2 - prevError2)/(currentTime - prevTime);
    pidOut2 = P2 + I2 + D2;
    //Speed I Saturation
    if(I2 > pidMax2) I2 = pidMax2; 
    else if(I2 < -pidMax2) I2 = -pidMax2;
    //Mix PID Outputs
    // pidOutFinal = pidOut1/pidMax1 + pidBias*pidOut2/pidMax2;
    pidOutFinal = (1-pidBias)*pidOut1/pidMax1 + pidBias*pidOut2/pidMax2;
 

    #if 1
    // Serial.print(currentState);
    // Serial.print(angY);
    Serial.print(" sp1: ");
    Serial.print(setPoint1);
    Serial.print(" sp2: ");
    Serial.print(setPoint2);
    // // Serial.print(" MotorSpd: ");
    // // Serial.print(motorSpeed);
    // // Serial.print(" spdout: ");
    // // Serial.print(pidOut2);
    Serial.print("\n");
    Serial.print(" ER1:");
    Serial.print(currentError1);
    Serial.print(" ER2:");
    Serial.print(currentError2);
    // Serial.print(" MTRSPD:");
    // Serial.print(LPFmotorAvSpeed);
    // Serial.print(" PIDang:");
    //  Serial.print(pidOut1);
    //  Serial.print(" PIDspd:");
    //  Serial.print(pidOut2);
    // Serial.print(" PIDfnl:");
    // Serial.println(pidOutFinal);
    #endif

    // Give up if over max angle
    // Else adjust speed according to PID
    
    distTravelled = ((double)motor1.getCurPos())/170;
    // if (distTravelled>10) flagHasTravelled5m = true;
    disp.display(distTravelled);

    // prevOffset = currentOffset1;
    // prevOffset = currentOffset2;
    prevTime = currentTime;
    prevError1 = currentError1;
    prevOffset = currentOffset1;
    prevOffset = currentOffset2;
    ///////////////////////////////////////////////////////////////////////////

    
    //Finite State Machine:
    switch (currentState)
    {
    case STOP:
        //Transition Conditions:
        if (!flagRunEnded){
            prevState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        //Enter Action:
        if (flagEnterState){
            //add actions below
            resetFlags();
        //    Serial.println("entering STOP State");
            if (prevState == RUN) flagRunEnded = TRUE;
            // setLED(LEFT,colour);
            //add actions above
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions below
                        // setPoint2 = 0;
            // setPoint1 = 0;
                motor1.setMotorPwm(0);
    motor2.setMotorPwm(0);

            if (flagHasTipped && flagRunEnded) flagRunEnded = FALSE; //Tip to reset after run
            //add actions above

        //Exit Actions:
        if (flagExitState){
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case RUN:
        //Transition Conditions:
        if (flagHasTravelled5m && !flagHasTurned180){
            prevState = currentState;
            currentState = REVERSE;
            flagExitState = TRUE;
        }
        else if (flagObstacleDetected){
            prevState = currentState;
            currentState = PAUSE;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions below
            // setLED(LEFT,colour);
            //add actions above
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions below
        //    Serial.println("RUN State");
            setPoint1 = currentOffset1/200*10-10;
            if(flagHasTipped) motorSpeed = 0;
    else motorSpeed = pidOutFinal*motorMaxRPM;
    motor1.setMotorPwm(motorSpeed);
    motor2.setMotorPwm(-motorSpeed*wheelCorrect);
            //add actions above

        //Exit Actions:
        if (flagExitState){
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case REVERSE:
        //Transition Conditions:
        //if (flagHasTurned180){
         //   prevState = currentState;
         //   currentState = STOP;
         //   flagExitState = TRUE;
       // }
      //  else if (flagObstacleDetected){
      //      prevState = currentState;
      //      currentState = PAUSE;
      //      flagExitState = TRUE;
      //  }

        //Enter Action:
        if (flagEnterState){
            //add actions below
            // setLED(LEFT,colour);
//            turn180();
            //add actions above
            flagEnterState = FALSE;
        } 
        //During Actions:
            //add actions below
        //    Serial.println("REVERSE State");
            setPoint2 = -5;
            setPoint1 = currentOffset1/200*10-7;
                  if(flagHasTipped) motorSpeed = 0;
                else motorSpeed = pidOutFinal*motorMaxRPM;
    motor1.setMotorPwm(motorSpeed);
    motor2.setMotorPwm(-motorSpeed);
            //add actions above

        //Exit Actions:
        if (flagExitState){
            flagRunEnded = true;
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case PAUSE:
        //Transition Conditions:
        if (!flagObstacleDetected && prevState == RUN){
            prevState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        else if (!flagObstacleDetected && prevState == REVERSE){
            prevState = currentState;
            currentState = REVERSE;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions below
        //    Serial.println("entering PAUSE State");
            // setLED(LEFT,colour);
            //add actions above
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions below
            
            //add actions above

        //Exit Actions:
        if (flagExitState){
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    default:
        prevState = currentState;
        currentState = STOP;
        break;
    }
}
