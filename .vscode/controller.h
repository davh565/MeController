// data
struct PID{
    /* data */
};


// functions
    // Control
    double pid (double input, struct PID);
    void balance();
    // Gyro
    double getHeading(); //return +- 180 deg
    // Motor
    void moveDistance(double dist); //+- dist m
    void turnInPlace(double angle); //+-180 deg
    // Joystick

    // Ultrasonic
    double getObstacle();

// states
    // MANUAL
    // STOP
    // FWD
    // TURN
    // PAUSE

// ouputs
    int motorOut(double balance, double movement);
// transition

// EXISTING FUNCTIONS
// Megyro.h
    // MeGyro(uint8_t port);
    // gyro.begin();
    // gyro.update();
    // double getAngleX(void) const;
    // double getAngleZ(void) const;
    // double getGyroX(void) const;
    // double getGyroY(void) const;
    // double getAccX(void) const;
    // double getAccY(void) const;
    // double getAccZ(void) const;
// MePotentiometer.h
    // MePotentiometer(uint8_t port);
    // uint16_t read(void);
// MeJoystick.h
    // MeJoystick(uint8_t port);
    // uint16_t readX(void);
    // uint16_t readY(void);
    // void CalCenterValue(int16_t = 0, int16_t = 0);
// MeUltrasonicSensor.h
    // MeUltrasonicSensor(uint8_t port);
    // double distanceCm(uint16_t = 400);
    // long measure(unsigned long = 30000);
// Me7SegmentDisplay.h;
    //  Me7SegmentDisplay(uint8_t port);
    // void reset(uint8_t port);
    // void init(void); // Clear display
    // void write(uint8_t SegData[]);
    // void display(uint16_t value);
    // void clearDisplay(void);
// MeEncoderMotor.h
    // MeEncoderMotor(uint8_t addr,uint8_t slot);
    // void begin();
    // boolean reset();
    // boolean move(float angle, float speed);
    // boolean moveTo(float angle, float speed);
    // boolean runTurns(float turns, float speed);
    // boolean runSpeed(float speed);
    // boolean runSpeedAndTime(float speed, float time);
    // float getCurrentSpeed();
    // float getCurrentPosition();
    // void request(byte *writeData, byte *readData, int wlen, int rlen);
// FilterOnePole.h
    // FilterOnePole( FILTER_TYPE ft=LOWPASS, float fc=1.0, float initialValue=0 );
    // float input( float inVal );
    // float output();