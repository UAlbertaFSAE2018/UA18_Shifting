// All the libraries are in: 2018 Formula → Design 154 Powertrain → Electric Shifting
#include <MsTimer2.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>


#define DEBUG_MODE          true
#define DUAL_CHANNEL        false
#define BAUD_RATE           9600

// Pinout
#define POSITION_SENSOR_PIN A3  // analog pin that the potentiometer is attached to
#define DOWNSHIFT_PIN       5   // upshift/downshift buttons have pulldown resistors
#define UPSHIFT_PIN         3
#define RPM_IN_PIN          A5
#define IGNITION_CUT_PIN    13
#define GEAR_OUT_PIN        11

// Setup
#define SENSOR_MIN          98      // reading in neutral (actual=105) - backlash (50 for now) (new is actually 272)
#define SENSOR_MAX          795     // reading in 4th (actual=790) + backlash (50 for now) (new is actually 982)
#define GEAR_MIN            0       // for neutral
#define GEAR_MAX            4       // 4th gear
#define PWM_MIN             -400    // VNH5019 shield PWM min & max
#define PWM_MAX             400     // Min/Max are -400/400, set low for safety
#define RATE                1       // ms, or 1kHz. Sets PID rate
#define CUTOFF_CURRENT      9000    // the max amperage before it detects a jam (in mA

#define DEBOUNCE_TIME       5       // 5ms
#define NEUTRAL_LOCK_TIME   1000    // 1000ms / 1s
#define MAX_CURRENT_ERROR   100     // 100mA


bool upshiftPressed = false;
bool downshiftPressed = false;
// ~50 days to roll over so should be fine
unsigned long timeUpshiftReleased = millis();
unsigned long timeDownshiftReleased = millis();
unsigned long timeDownshiftPressed = millis();

unsigned long time;
unsigned long neutralTimer;
unsigned int getM1CurrentMilliamps();
unsigned int getM2CurrentMilliamps();
unsigned int motorCurrentMilliamps();
unsigned char getM1Fault();
unsigned char getM2Fault();
int sensorValue; // value read from the pot
int currentGear;
int targetGear; 
int motorValue = 0;
int upshiftState = 0; //current button state
int downshiftState = 0;
int lastUpshiftState = 0; //previous button state
int lastDownshiftState = 0;
int meanCurrent = 0;
int lastWorkingGear = 0; // is it okay to assume that this is Neutral?
int gearPos[5] = {98,284,442,616,795}; //(N is actually 1st. If using 5spd, add N=174)

double currentPosition;
double Output;
double targetPosition;
// PID gain values
float Kp = 6;
float Ki = 0.5;
float Kd = 0.0;


PID shiftPID(&currentPosition, &Output, &targetPosition, Kp, Ki, Kd, DIRECT); // direction is either DIRECT or REVERSE
DualVNH5019MotorShield motorDriver; //(2, 7, 6, A0, 2, 7, 12, A1); // motor drivers MUST be wired in PARALLEL


void setup() {
    Serial.begin(BAUD_RATE);
    
    pinMode(UPSHIFT_PIN, INPUT);
    pinMode(DOWNSHIFT_PIN, INPUT);
    
    motorDriver.init();
    motorDriver.setM1Speed(0x0000);
    
    shiftPID.SetMode(AUTOMATIC);
    shiftPID.SetSampleTime(RATE);
    shiftPID.SetOutputLimits(PWM_MIN, PWM_MAX);
    
    MsTimer2::set(RATE, controlLoop);
    MsTimer2::start();
    
    targetGear = currentGear; //not needed?
}


void loop() {
    // determine current state
    currentPosition = analogRead(POSITION_SENSOR_PIN); // read the analog in value
    currentGear = mapGear(currentPosition, SENSOR_MIN, SENSOR_MAX, GEAR_MIN, GEAR_MAX); // maps the sensor to a range of 0-4 (0=N)
    if(digitalRead(UPSHIFT_PIN)) timeUpshiftReleased = millis();
    if(digitalRead(DOWNSHIFT_PIN)) timeDownshiftReleased = millis();
    
    // debouncing
    if(!digitalRead(UPSHIFT_PIN) && upshiftPressed && millis() - timeUpshiftReleased >= DEBOUNCE_TIME){
        upshiftPressed = false;
    }
    if(!digitalRead(DOWNSHIFT_PIN) && downshiftPressed && millis() - timeDownshiftReleased >= DEBOUNCE_TIME){
        downshiftPressed = false;
    }

    // read buttons
    if(digitalRead(UPSHIFT_PIN) && !upshiftPressed && currentGear < GEAR_MAX){
        upshiftPressed = true;
        targetGear += 1;
    }
    if(digitalRead(DOWNSHIFT_PIN) && !downshiftPressed && currentGear > GEAR_MIN){
        downshiftPressed = true;
        // Neutral lock
        if(currentGear > GEAR_MIN + 1)
            targetGear -= 1;
        else
            timeDownshiftPressed = millis();
    }
    // Neutral lock
    if(digitalRead(DOWNSHIFT_PIN) && millis() - timeDownshiftPressed >= NEUTRAL_LOCK_TIME && currentGear == GEAR_MIN + 1){
        targetGear = GEAR_MIN;
    }
}


// This function is needed for gears to be mapped proportionally
long mapGear(long x, long in_min, long in_max, long out_min, long out_max) {
    // default map behavior:
    // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}


// Motor PID loop
void controlLoop() {
    // determine current state
    currentPosition = analogRead(POSITION_SENSOR_PIN); // read the analog in value
    currentGear = mapGear(currentPosition, SENSOR_MIN, SENSOR_MAX, GEAR_MIN, GEAR_MAX); // maps the sensor to a range of 0-4 (0=N)
    
    // Read current (in mA)
    if(DUAL_CHANNEL)
        meanCurrent = (motorDriver.getM1CurrentMilliamps() + motorDriver.getM2CurrentMilliamps()) / 2;
    else
        meanCurrent = motorDriver.getM1CurrentMilliamps();
    
    // Check for and handle jam.
    if (meanCurrent > CUTOFF_CURRENT) {
        if(DEBUG_MODE) Serial.println("JAM DETECTED");
        targetGear = lastWorkingGear;
    }
    
    targetPosition = gearPos[targetGear];
      
    // Use PID to set motor speed
    double motorValue = 0;
    if(currentGear != targetGear || meanCurrent > MAX_CURRENT_ERROR){
        shiftPID.Compute();
        motorValue = int(Output);
    }
    else
        lastWorkingGear = currentGear;
    
    motorDriver.setM1Speed(motorValue);
    if(DUAL_CHANNEL)
        motorDriver.setM2Speed(motorValue);
    
    //stopIfFault();

    if(DEBUG_MODE) {
        // print the results to the serial monitor:
        Serial.print("current (mA) = ");
        Serial.print(meanCurrent);
        Serial.print("\t sensor = ");
        Serial.print(currentPosition);
        Serial.print("\t target pos = "); //temp
        Serial.print(targetPosition);  //temp
        Serial.print("\t current gear = ");
        Serial.print(currentGear);
        Serial.print("\t target gear = ");
        Serial.println(targetGear);
    }
}

