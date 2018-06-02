// all the libraries are in 2018 Formula --> Design 154 Powertrain --> Electric Shifting
#include <MsTimer2.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>

//**Pinout**//
#define POSITION_SENSOR_PIN A3  // analog pin that the potentiometer is attached to
#define DOWNSHIFT_PIN       5   // upshift/downshift buttons have pulldown resistors
#define UPSHIFT_PIN         3
#define RPM_IN_PIN          A5
#define IGNITION_CUT_PIN    13
#define GEAR_OUT_PIN        11

//**Setup**//
#define SENSOR_MIN      98      // reading in neutral (actual=105) - backlash (50 for now) (new is actually 272)
#define SENSOR_MAX      795     // reading in 4th (actual=790) + backlash (50 for now) (new is actually 982)
#define PWM_MIN         -400    // VNH5019 shield PWM min & max
#define PWM_MAX         400     // Min/Max are -400/400, set low for safety
#define RATE            1       //ms, or 1kHz. Sets PID rate
#define CUTOFF_CURRENT  9000    // the max amperage before it detects a jam (in mA)

DualVNH5019MotorShield motorDriver; //(2, 7, 6, A0, 2, 7, 12, A1); // motor drivers MUST be wired in PARALLEL

unsigned long time;
unsigned long neutralTimer;
unsigned int getM1CurrentMilliamps();
unsigned int getM2CurrentMilliamps();
unsigned int motorCurrentMilliamps();
unsigned char getM1Fault();
unsigned char getM2Fault();
int sensorValue;  // value read from the pot
int currentGear; // gear output to display
int targetGear; 
int motorValue = 0;
int upshiftState = 0; //current button state
int downshiftState = 0;
int lastUpshiftState = 0; //previous button state
int lastDownshiftState = 0;
int meanCurrent = 0;
long map(long x, long in_min, long in_max, long out_min, long out_max) {  //This equation is needed for gears to be mapped proportionally
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
int gearPos[5] = {98,284,442,616,795}; //(N is actually 1st. If using 5spd, add N=174)

//**PID gain values**//
float Kp = 6;
float Ki = 0.5;
float Kd = 0.0;
double targetPos, currentPos, Output;
PID shiftPID(&currentPos, &Output, &targetPos, Kp, Ki, Kd, DIRECT); // direction is either DIRECT or REVERSE


void setup() {
    Serial.begin(9600);
    
    motorDriver.init();
    motorDriver.setM1Speed(0x0000);
    
    shiftPID.SetMode(AUTOMATIC);
    shiftPID.SetSampleTime(RATE);
    shiftPID.SetOutputLimits(PWM_MIN, PWM_MAX);
    
    MsTimer2::set(RATE, controlLoop);
    MsTimer2::start();
    
    pinMode(UPSHIFT_PIN, INPUT);
    pinMode(DOWNSHIFT_PIN, INPUT);
    
    targetGear = currentGear; //not needed?
}

void loop() {
    
    //  meanCurrent = (motorDriver.getM1CurrentMilliamps() + motorDriver.getM2CurrentMilliamps()) / 2;
      
    upshiftState = digitalRead(UPSHIFT_PIN); // read the pushbutton input pin
    downshiftState = digitalRead(DOWNSHIFT_PIN);
    
    currentPos = analogRead(POSITION_SENSOR_PIN); // read the analog in value
    currentGear = map(currentPos, SENSOR_MIN, SENSOR_MAX, 0, 4); // maps the sensor to a range of 0-4 (0=N)
    
    if (upshiftState != lastUpshiftState) {              // checks to see if button state changed
        if ((upshiftState == HIGH) && (currentGear < 4)) { // if the current state is HIGH then the button went from off to on
        targetGear = currentGear + 1;                                   // for each press of upshift, targetGear will shift up a gear while <4
        //**ignition-cut code goes here**//
    
        }
        delay(5);  // debounce
    }
    lastUpshiftState = upshiftState;         // save the current state as the last state, for next time through the loop
    
    if (downshiftState != lastDownshiftState) {
        if (downshiftState == HIGH) {
            if (currentGear > 1) {
                targetGear = currentGear - 1;  //instead of targetGear --
            } else {                    
                neutralTimer = millis();
                delay(1000);
                time = millis();
                if ((digitalRead(DOWNSHIFT_PIN) == HIGH) && (time - neutralTimer >= 1000)) {
                    targetGear = 0;
                }
            }
            delay(5);  // debounce
        }
    }
    lastDownshiftState = downshiftState;
    
    // if (currentGear == targetGear) {  // experimental; trying to figure out how to cut power while not shifting.
    //   shiftPID.SetMode(MANUAL);
    // } else {
    //   shiftPID.SetMode(AUTOMATIC);
    // }
    
    // print the results to the serial monitor:
    Serial.print("current (mA) = ");
    Serial.print(meanCurrent);
    Serial.print("\t sensor = ");
    Serial.print(currentPos);
    Serial.print("\t target pos = "); //temp
    Serial.print(targetPos);  //temp
    Serial.print("\t current gear = ");
    Serial.print(currentGear);
    Serial.print("\t target gear = ");
    Serial.println(targetGear);
    
    delay(10);  // wait 5ms
}

void controlLoop() {  // Motor PID loop
    
    currentPos = analogRead(POSITION_SENSOR_PIN);
    meanCurrent = motorDriver.getM1CurrentMilliamps();// + motorDriver.getM2CurrentMilliamps()) / 2;  // read current (in mA)
    if (meanCurrent > CUTOFF_CURRENT) {    // CUTOFF_CURRENT is defined in setup
        targetPos = gearPos[currentGear];
        //Serial.print("JAM DETECTED");   // for debugging
    } else {
        targetPos = gearPos[targetGear];       
    }
      
    shiftPID.Compute();                    
    int motorValue = int(Output);           
    motorDriver.setM1Speed(motorValue);
    //motorDriver.setM2Speed(motorValue);
    // stopIfFault();
}

