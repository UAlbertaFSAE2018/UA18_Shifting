// All the libraries are in: 2018 Formula → Design 154 Powertrain → Electric Shifting
#include <MsTimer2.h>
#include <DualVNH5019MotorShield.h>
#include <PID_v1.h>


#define DEBUG_MODE          true
#define DUAL_CHANNEL        false   // false if single channel
#define BAUD_RATE           9600

// Pinout
#define POSITION_SENSOR_PIN A3      // analog pin that the potentiometer is attached to
#define DOWNSHIFT_PIN       5       // upshift/downshift buttons have pulldown resistors
#define UPSHIFT_PIN         3
#define RPM_IN_PIN          A5
#define IGNITION_CUT_PIN    13
#define GEAR_OUT_PIN        11

// Setup
#define SENSOR_MIN          98      // reading in neutral (actual=105) - backlash (50 for now) (new is actually 272)
#define SENSOR_MAX          795     // reading in 4th (actual=790) + backlash (50 for now) (new is actually 982)
#define SENSOR_ERROR_MAX    10      // when the motor stops trying to change position
#define GEAR_MIN            0       // for neutral
#define GEAR_MAX            4       // 4th gear
#define PWM_MIN             -400    // VNH5019 shield PWM min & max
#define PWM_MAX             400     // Min/Max are -400/400, set low for safety
#define RATE                1       // ms (or 1/kHz). Sets PID rate
#define CUTOFF_CURRENT      20000   // the max amperage before it detects a jam (in mA)

#define DEBOUNCE_TIME       5       // ms
#define NEUTRAL_LOCK_TIME   1000    // ms
#define IGNITION_CUT_TIME   100     // ms
#define MOTOR_STOPPED       0x0000


//unsigned int getM1CurrentMilliamps();
//unsigned int getM2CurrentMilliamps();
//unsigned int motorCurrentMilliamps();
//unsigned char getM1Fault();
//unsigned char getM2Fault();


bool upshiftRecentlyPressed = false;
bool downshiftRecentlyPressed = false;
int awaitingUpshift = 0; // works like a semaphore
// ~50 days to roll over so should be fine
unsigned long timeUpshiftReleased = millis();
unsigned long timeUpshiftPressed = millis();
unsigned long timeDownshiftReleased = millis();
unsigned long timeDownshiftPressed = millis();
unsigned long currentMillis = millis();

int sensorValue; // value read from the pot
//int currentGear;
int targetGear; 
int lastWorkingGear;
int upshiftButtonDepressed = 0; //current button state
int downshiftButtonDepressed = 0;
int gear2position[5] = {98,284,442,616,795}; //(N is actually 1st. If using 5spd, add N=174)

double meanCurrent;
double motorSpeed = MOTOR_STOPPED;
double currentPosition;
double Output;
double targetPosition;
// PID gain values
float Kp = 6.0;
float Ki = 0.1;
float Kd = 0.0;


PID shiftPID(&currentPosition, &Output, &targetPosition, Kp, Ki, Kd, DIRECT); // direction is either DIRECT or REVERSE
DualVNH5019MotorShield motorDriver; //(2, 7, 6, A0, 2, 7, 12, A1); // motor drivers MUST be wired in PARALLEL


void setup() {
    Serial.begin(BAUD_RATE);
    
    pinMode(UPSHIFT_PIN, INPUT);
    pinMode(DOWNSHIFT_PIN, INPUT);
    pinMode(IGNITION_CUT_PIN, OUTPUT);
    pinMode(GEAR_OUT_PIN, OUTPUT);
    
    motorDriver.init();
    motorDriver.setM1Speed(MOTOR_STOPPED);
    if(DUAL_CHANNEL) {
        motorDriver.setM2Speed(MOTOR_STOPPED);
    }
    
    shiftPID.SetMode(AUTOMATIC);
    shiftPID.SetSampleTime(RATE);
    shiftPID.SetOutputLimits(PWM_MIN, PWM_MAX);
    
    MsTimer2::set(RATE, controlLoop);
    MsTimer2::start();
    
    // determine current state
    currentPosition = analogRead(POSITION_SENSOR_PIN); // read the analog in value
    int currentGear = mapGear(currentPosition, SENSOR_MIN, SENSOR_MAX, GEAR_MIN, GEAR_MAX); // maps the sensor to a range of 0-4 (0=N)
    targetGear = currentGear;
    lastWorkingGear = currentGear;
    analogWrite(GEAR_OUT_PIN, currentPosition);
    digitalWrite(IGNITION_CUT_PIN, LOW);
    
    controlLoop(); // initialize values
}


void loop() {
    upshiftButtonDepressed = digitalRead(UPSHIFT_PIN);
    downshiftButtonDepressed = digitalRead(DOWNSHIFT_PIN);
    currentMillis = millis();

    analogWrite(GEAR_OUT_PIN, currentPosition);
    
    // continually updates time until released
    if(upshiftButtonDepressed) {
        timeUpshiftReleased = currentMillis;
    }
    if(downshiftButtonDepressed) {
        timeDownshiftReleased = currentMillis;
    } else {
        timeDownshiftPressed = currentMillis; // ensures downshift button must be held to downshift
    }
    
    // debouncing
    if(!upshiftButtonDepressed && upshiftRecentlyPressed && currentMillis - timeUpshiftReleased >= DEBOUNCE_TIME) {
        upshiftRecentlyPressed = false;
    }
    if(!downshiftButtonDepressed && downshiftRecentlyPressed && currentMillis - timeDownshiftReleased >= DEBOUNCE_TIME) {
        downshiftRecentlyPressed = false;
    }

    // read buttons
    if(upshiftButtonDepressed && !upshiftRecentlyPressed && targetGear < GEAR_MAX) {
        upshiftRecentlyPressed = true;
        timeUpshiftPressed = currentMillis;
        // no throttle cut for 1st
        if(targetGear > GEAR_MIN) { // this is the OLD target gear
            Serial.println("boomshakalakalaka");
            MsTimer2::stop();
            digitalWrite(IGNITION_CUT_PIN, HIGH);
            awaitingUpshift++;
        }
        targetGear += 1; // needs to be after timer is stopped
    }
    if(downshiftButtonDepressed && !downshiftRecentlyPressed && targetGear > GEAR_MIN) {
        downshiftRecentlyPressed = true;
        timeDownshiftPressed = currentMillis;
        // Neutral lock
        if(targetGear > GEAR_MIN + 1) {
            targetGear -= 1;
        }
    }
    
    // Neutral lock
    if(downshiftButtonDepressed && currentMillis - timeDownshiftPressed >= NEUTRAL_LOCK_TIME && targetGear == GEAR_MIN + 1) {
        targetGear = GEAR_MIN;
    }
    // Throttle cut
    if(awaitingUpshift && currentMillis - timeUpshiftPressed >= IGNITION_CUT_TIME) {
        MsTimer2::start();
        awaitingUpshift--;
        if(!awaitingUpshift) {
            digitalWrite(IGNITION_CUT_PIN, LOW);
        }
    }

    // can't print during interrupt
    if(DEBUG_MODE) {
        currentPosition = analogRead(POSITION_SENSOR_PIN);
        int currentGear = mapGear(currentPosition, SENSOR_MIN, SENSOR_MAX, GEAR_MIN, GEAR_MAX); // maps the sensor to a range of 0-4 (0=N)
        // print the results to the serial monitor:
        Serial.print("ctrl ");
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


// Motor PID loop
// Should be as short as possible
void controlLoop() {
    // determine current state
    currentPosition = analogRead(POSITION_SENSOR_PIN); // read the analog in value
    
    // Read current (in mA)
    meanCurrent = motorDriver.getM1CurrentMilliamps();
    if(DUAL_CHANNEL)
        meanCurrent = (meanCurrent + motorDriver.getM2CurrentMilliamps()) / 2;
    
    // Check for and handle jam.
    if (meanCurrent > CUTOFF_CURRENT) {
        targetGear = lastWorkingGear;
    }
    
    targetPosition = gear2position[targetGear];
      
    // Use PID to set motor speed
    motorSpeed = MOTOR_STOPPED;
    if(abs(currentPosition - targetPosition) > SENSOR_ERROR_MAX) {
        shiftPID.Compute();
        motorSpeed = Output;
    }
    else{
        lastWorkingGear = mapGear(currentPosition, SENSOR_MIN, SENSOR_MAX, GEAR_MIN, GEAR_MAX);
        //MsTimer2::stop();
    }
    
    motorDriver.setM1Speed(motorSpeed);
    if(DUAL_CHANNEL) {
        motorDriver.setM2Speed(motorSpeed);
    }
    
    //stopIfFault();
}


// Old motor PID loop
/*void controlLoopOld() {
    currentPosition = analogRead(POSITION_SENSOR_PIN);
    meanCurrent = motorDriver.getM1CurrentMilliamps();// + md.getM2CurrentMilliamps()) / 2;  // read current (in mA)
    if (meanCurrent > CUTOFF_CURRENT) {    // cutoffCurrent is defined in setup
        targetPosition = gear2position[currentGear];
    } else {
        targetPosition = gear2position[targetGear];       
    }
      
    shiftPID.Compute();                    
    int motorValue = int(Output);           
    motorDriver.setM1Speed(motorValue);
    //md.setM2Speed(motorValue);
    // stopIfFault();
}*/


// This function is needed for gears to be mapped proportionally
long mapGear(long x, long in_min, long in_max, long out_min, long out_max) {
    // default map behavior:
    // return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if(x > in_max) return out_max;
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

