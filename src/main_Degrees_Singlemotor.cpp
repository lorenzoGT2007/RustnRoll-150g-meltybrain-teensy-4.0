/* Notes:
Motor dshot (edited library): 0 = full stop, 1 - 1000 spin clockwise, 1001 - 2000 spin counterclockwise

max measured rpm of rust'n'roll 150g melty is 3500rpm

This code only works for motors that are 180deg apart

channel mixing code source: https://www.instructables.com/Understanding-Channel-Mixing/


Data Type Sizes on Teensy 4.0 (ARM Cortex-M7)
Data Type	Size (Bytes)	Notes
double	        8	IEEE 754 double-precision floating point
float	        4	IEEE 754 single-precision floating point
long	        4	Signed 32-bit integer
unsigned long	4	Unsigned 32-bit integer
int	            4	Signed 32-bit integer
short	        2	Signed 16-bit integer
char	        1	8-bit character


EEPROM used addresses: 0 - 3 getmaxRPM, 4 - 11 Kp, 12 - 19 Ki, 20 - 27 Kd
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <PulsePosition.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <DShot.h>
#include <math.h>
#include <PID_v1.h>
#include <InterpolationLib.h>

// #define SERIALCHECKaccelerometer_LOOP // accelerometer values printed out
// #define SERIALCHECKchannels_LOOP // all channel raw data (us)
// #define SERIALCHECK2_LOOP // channels mapped data
// #define SERIALCHECK1_SETUP // accelerometer settings and startup printed out
// #define GETMAXRPM //Serial needs to be activated
// #define LOOPFREQUENCY  //write loop frequency in serial
// #define SERIALCHECKPID // write saved pid settings in serial
// #define SERIALCHECKDEGCAL //write calibrated deg values in serial

#ifdef GETMAXRPM
bool firstRun = true;
unsigned long previousPeriodTest = 0;
unsigned long lastEepromWriteTest = 0;
#endif

/* values that need to be manually inserted */
const unsigned long minChosenMotorPulseWidth = 25'000;           // in millideg, this value will be directly scaled as the RPM increases
const unsigned long maxChosenMotorPulseWidth = 100'000;          // in millideg
const unsigned long chosenLedWidth = 40'000;                     // in millideg
double radius = 0.0473;                                          // meters
const unsigned long maxRPM = 3500;                               // max goal rpm that will be used for full ch3 throttle, in the case of 2300 a number like 2000 as max is recommended
const unsigned long maxCalRPM = 2500;                            // max rpm for calibration
const unsigned long minCalRPM = 1000;                            // min rpm for calibration
const unsigned minTimeBetweenActivations = 150;                  // in ms, it's the min time that needs to pass between each motor activation, this will be the max value possible once mapped to ch2
const unsigned long periodUsISR = 100;                           // period that sets the frequency of the isr for motor throttle
const long recieverFailsafeValues[4] = {1500, 1500, 1000, 1000}; // values that will be set when reciever signal is lost. ch1, ch2, ch3, ch4

/* global variables */
unsigned long failsafeTimer = 0;
unsigned long ledWidth = chosenLedWidth;
unsigned long deltaDeg = 0;
unsigned long angPos = 0;
bool motorActivationOngoing = false;
unsigned long periodUs = 0;
unsigned long initialMicros = 0;
unsigned long lastTimeDrift = 0;
unsigned long startCalibrationDeg = 0;
bool degCalibration = false;
bool degCalibratingNow = false;
unsigned long motorsCalibrationDegRaw = 0;
unsigned long lastMotorActivation = 0;
bool motorActive = false;
bool preventActivationChopping = true;
bool previousMotorState = motorActive;
bool PIDCalibration = false;
bool PIDcalibratingNow = false;
unsigned long startCalibrationDegPID = 0;
long previousCh4Value = 0;
double calRangeRPM = maxCalRPM - minCalRPM;
uint8_t PIDcoeff = 1;
uint8_t calPoint = 1;
bool failsafeOn = false;

/* PID library */
double RPMgoal;     // your setpoint
double RPM;         // measured RPM from sensor
double motorOutput; // PID output, 0..1000

double consKp = 0, consKi = 0, consKd = 0;                               // conservative tuning when close to the desired range
double aggKp = 0, aggKi = 0, aggKd = 0;                                  // aggressive tuning when far from the desired range
PID myPID(&RPM, &motorOutput, &RPMgoal, consKp, consKi, consKd, DIRECT); // initializing PID parameters

/* Point interpolation library */
const uint8_t numberOfPoints = 5;                                                                                                                    // number of points
double calRPM[numberOfPoints] = {0, minCalRPM + (calRangeRPM / 4), minCalRPM + (calRangeRPM * 2 / 4), minCalRPM + (calRangeRPM * 3 / 4), maxCalRPM}; // RPM values where each deg advance will be calculated
double calDeg[numberOfPoints] = {0};                                                                                                                 // this will be filled later on with the values stored in eeprom

/* Pin declarations */
const uint8_t pinESC1 = 1; // ESC pins. pin 1 = Serial1. pin 17 = Serial4
const uint8_t pinESC2 = 17;
const uint8_t pinCH1 = 14; // reciever signal PPM pin
const uint8_t pinLED = 16; // LED NPN transistor pin

#define H3LIS331_SCK 13 // accelerometer SPI pins
#define H3LIS331_MISO 12
#define H3LIS331_MOSI 11
#define H3LIS331_CS 10

/* adafruit declaration of accelerometer object */
Adafruit_H3LIS331 accel = Adafruit_H3LIS331();

/* ESC Dshot declaration */
DShot ESC1(&Serial1, DShotType::DShot600);
DShot ESC2(&Serial4, DShotType::DShot600);
IntervalTimer sendThrottle;

/* esc signal variables */
long ch1Value = 0;
long ch2Value = 0;
long ch3Value = 0;
long ch4Value = 0;

/* PPM communication object definition */
PulsePositionInput ReceiverInput(RISING);
long receiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0}; // array for channels values
int ChannelNumber = 0;

/* interrupt function for motor throttle */
volatile uint16_t throttle1 = 1048;
volatile uint16_t throttle2 = 1048;
void updateThrottle()
{
    ESC1.sendThrottle(throttle1, false); // 0 min to 2000 max. the first 48 values are for esc setup. center value between min and max is 1000
    ESC2.sendThrottle(throttle2, false); //(throttle value, telemetry from esc requested)
}

void setup()
{

#if defined(SERIALCHECKchannels_LOOP) || defined(SERIALCHECK2_LOOP) || defined(SERIALCHECK1_SETUP) || defined(GETMAXRPM) || defined(LOOPFREQUENCY) || defined(SERIALCHECKPID) || defined(SERIALCHECKDEGCAL)
    Serial.begin(115200);
#endif

    while (!accel.begin_SPI(H3LIS331_CS)) // wait until the accelerometer is found
    {
        digitalWrite(pinLED, HIGH); // turn on LED while waiting for the accelerometer to be found
        delay(1000);
    }

    accel.setRange(H3LIS331_RANGE_400_G);
    accel.setDataRate(LIS331_DATARATE_1000_HZ);
    sensors_event_t event;
    accel.getEvent(&event);

    pinMode(pinLED, OUTPUT); // LED pin
    digitalWrite(pinLED, LOW);

    ReceiverInput.begin(pinCH1); // begin communication with receiver

    EEPROM.get(4, consKp);
    EEPROM.get(12, consKi);
    EEPROM.get(20, consKd);
    aggKp = consKp * 1.6; // aggressive tuning when far from the desired range
    aggKi = consKi * 1.2;
    aggKd = consKd * 1.2;
    myPID.SetMode(AUTOMATIC);       // turn PID on
    myPID.SetOutputLimits(1, 1000); // limit output to motor range

    for (uint8_t i = 0; i < numberOfPoints - 1; i++) // read the calibrated values skipping the first one that will always be zero
    {
        unsigned long EEPROMStartAddress = 28;
        EEPROM.get(EEPROMStartAddress + i * sizeof(calDeg[0]), calDeg[i + 1]); // write the eeprom values into the corresponding calDeg arrays

#ifdef SERIALCHECKDEGCAL
        Serial.print(calDeg[i + 1]);
        Serial.print("    ");
#endif
    }

    for (size_t i = 0; i < 4000; i++) // arming sequence for ESC using dshot
    {
        ESC1.sendCommand(0, false); // 0 command MUST be sent for dshot arming
        ESC2.sendCommand(0, false);
        delayMicroseconds(1'000);
    }

    bool interruptStart = sendThrottle.begin(updateThrottle, periodUsISR); // start motor throttle interrupt
    if (!interruptStart)
    {
        // Failed to start interrupt, blink LED
        while (true)
        {
            digitalWrite(pinLED, HIGH);
            delay(100);
            digitalWrite(pinLED, LOW);
            delay(100);
        }
    }

#ifdef SERIALCHECKPID
    delay(10000); // wait so the user have time to connect to serial once turned on
    Serial.print(consKp);
    Serial.print(" consKp          ");
    Serial.print(consKi);
    Serial.print(" consKi          ");
    Serial.print(consKd);
    Serial.print(" consKd          \r");
#endif

#ifdef GETMAXRPM
    delay(10000); // wait so you have time to connect to serial once turned on
    EEPROM.get(0, periodUs);
    unsigned long rpm = 60.0 * 1000000.0 / periodUs;
    Serial.print(rpm);
    Serial.print(" rpm    ");
    Serial.print(periodUs);
    Serial.println(" Period Us");
#endif

#ifdef SERIALCHECK1_SETUP

    Serial.println("H3LIS331 test!");

    if (!accel.begin_SPI(H3LIS331_CS))
    {
        Serial.println("Couldn't start");
        while (1)
            yield();
    }

    Serial.println("H3LIS331 found!");

    Serial.print("Range set to: ");
    accel.setRange(H3LIS331_RANGE_200_G);
    switch (accel.getRange())
    {
    case H3LIS331_RANGE_100_G:
        Serial.println("100 g");
        break;
    case H3LIS331_RANGE_200_G:
        Serial.println("200 g");
        break;
    case H3LIS331_RANGE_400_G:
        Serial.println("400 g");
        break;
    }

    accel.setDataRate(LIS331_DATARATE_1000_HZ);
    Serial.print("Data rate set to: ");
    switch (accel.getDataRate())
    {

    case LIS331_DATARATE_POWERDOWN:
        Serial.println("Powered Down");
        break;
    case LIS331_DATARATE_50_HZ:
        Serial.println("50 Hz");
        break;
    case LIS331_DATARATE_100_HZ:
        Serial.println("100 Hz");
        break;
    case LIS331_DATARATE_400_HZ:
        Serial.println("400 Hz");
        break;
    case LIS331_DATARATE_1000_HZ:
        Serial.println("1000 Hz");
        break;
    case LIS331_DATARATE_LOWPOWER_0_5_HZ:
        Serial.println("0.5 Hz Low Power");
        break;
    case LIS331_DATARATE_LOWPOWER_1_HZ:
        Serial.println("1 Hz Low Power");
        break;
    case LIS331_DATARATE_LOWPOWER_2_HZ:
        Serial.println("2 Hz Low Power");
        break;
    case LIS331_DATARATE_LOWPOWER_5_HZ:
        Serial.println("5 Hz Low Power");
        break;
    case LIS331_DATARATE_LOWPOWER_10_HZ:
        Serial.println("10 Hz Low Power");
        break;
    }

#endif
}

void failsafe()
{
    for (uint8_t i = 0; i < (sizeof(recieverFailsafeValues) / sizeof(recieverFailsafeValues[0])); i++)
    {
        receiverValue[i] = recieverFailsafeValues[i];
    }
}

void readReceiver()
{
    ChannelNumber = ReceiverInput.available();
    if (ChannelNumber > 0)
    {
        failsafeOn = false;
        for (int i = 1; i <= ChannelNumber; i++)
        {
            receiverValue[i - 1] = ReceiverInput.read(i);
        }

        failsafeTimer = millis(); // last time a valid signal was received
    }
    else
    {
        if (millis() - failsafeTimer > 1000) // if no signal is detected for more than a tot of ms failsafe is activated
        {
            failsafeOn = true;
            failsafe();
        }
    }
}

int convertThrottle(int speed) // converts throttle for dshot library. input is from -1000 to 1000, output from 0 to 2000 for dshot library. 0 sends the full stop command
{
    if (speed == 0)
    {
        return 0;
    }
    if (speed > 0)
    {
        return speed;
    }
    //  if (speed < 0) {
    return 1000 - speed;
}

int processChannelValue(int CH, int min, int max)
{
    CH = map(CH, 1000, 2000, min, max); // map for convertThrottle function
    CH = (abs(CH) <= 10) ? 0 : CH;      // signal deadband when in neutral position
    CH = constrain(CH, min, max);
    return CH;
}

void mixEscSignals(int x, int y)
{ // this mixing algorith from a -1000 and 1000 input range spits out a -2000 and 2000 range if not mapped. but it gives the possibility to completely map a square joystick
    ch1Value = x + y;
    ch2Value = y - x;
    int diff = abs(x) - abs(y);

    if (ch1Value < 0)
    {
        ch1Value = ch1Value - abs(diff);
    }
    else
    {
        ch1Value = ch1Value + abs(diff);
    }

    if (ch2Value < 0)
    {
        ch2Value = ch2Value - abs(diff);
    }
    else
    {
        ch2Value = ch2Value + abs(diff);
    }

    ch1Value = map(ch1Value, -2000, 2000, -500, 500); // remap for -500 and 500 range, instead of max values so it is easier to control
    ch2Value = map(ch2Value, -2000, 2000, -500, 500);
    ch1Value = constrain(ch1Value, -1000, 1000);
    ch2Value = constrain(ch2Value, -1000, 1000);
}

float centripetalAccel() // since x and y axis are flared 45 deg out from the line that goes from the origin to the center of the melty the real accel needs to be evaluated
{
    sensors_event_t event;
    accel.getEvent(&event);
    float accelAverage = fabs((event.acceleration.y + event.acceleration.x) / 2); // fabs is the abs for float variables
    return accelAverage * sqrt(2);                                                // µm/µs²  if the main acceleration direction is 45 degrees offset from both axis it is simply a square diagonal.
}

int angularVariation()
{
    float angVelocity = sqrt(centripetalAccel() / radius);

    periodUs = (unsigned long)((2.0f * 3.14159265f / angVelocity) * 1'000'000.0f); // periodUs = 2π / angVelocity * 1,000,000. for microseconds per revolution.

    unsigned long now = micros();
    unsigned long deltaMicros = now - initialMicros;
    initialMicros = now;

    return (angVelocity * (deltaMicros * 1e-6f) * (180.0f / 3.14159265f)) * 1'000UL; // delta in seconds, casting to return an Unsigned Long value in millidegrees
}

double fmap(long x, long in_min, long in_max, double out_min, double out_max)
{
    return (double)(x - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

// Function to update LED width based on RPM and calibration point
void updateLedWidthCal(double desiredRPM, double Range)
{
    double minRange = desiredRPM - Range;
    if (minRange < 0.0)
    {
        minRange = 0.0;
    }
    double maxRange = desiredRPM + Range;

    if (RPM >= minRange && RPM <= maxRange)
    {
        ledWidth = chosenLedWidth / 4;
    }
    else
    {
        ledWidth = (chosenLedWidth * 2) % 360'000;
    }
}

void loop()
{
#ifdef LOOPFREQUENCY
    unsigned long start = micros();
#endif

    readReceiver(); // read and store reciever channel values
    ch1Value = processChannelValue(receiverValue[0], -1000, 1000);
    ch2Value = processChannelValue(receiverValue[1], -1000, 1000);
    ch3Value = processChannelValue(receiverValue[2], 0, 1000);
    ch4Value = processChannelValue(receiverValue[3], -100, 100); // ch4 is slave to the 3 position switch. it evaluates to -100, 0 or 100
    // ch5 is for calibration so it won't be mapped to increase performance
    // same for ch6
    if (ch3Value == 0) // tank mode, CH3 is deadbanded for the first 5 values so they result in 0
    {
        mixEscSignals(ch1Value, ch2Value);
        if (failsafeOn == true)// if failsafe is on the led is always off
        {
            digitalWrite(pinLED, LOW); // LED always on in tank mode
        }
        else
        {
            digitalWrite(pinLED, HIGH); // LED always on in tank mode
        }
        delayMicroseconds(1000); // slow down the loop, speed is not required in this case

        noInterrupts();
        throttle1 = convertThrottle(-ch1Value); // changing the sign inverts the motor
        throttle2 = convertThrottle(ch2Value);
        interrupts();
    }
    else if (ch3Value > 0 && ch3Value <= 1000) // spin mode
    {
        deltaDeg = angularVariation();     // in microdegrees
        RPM = 60.0 * 1000000.0 / periodUs; // periodUs is computed inside angular variation         RPM = 60.0 * 1000000.0 / periodUs;

        /* Angular position drift calibration and directional LED stick control */
        unsigned long now = micros();
        unsigned long driftFreq = periodUs / 200; // how many times every rotation it will be applied. if the loop runs at 10kHz and the robot is at 200rpm you get a max of 300 frames per revolution
        if (now - lastTimeDrift > driftFreq)      // drift calibration needs to be in function of rotations. so every rotation a certain amout of degrees is added/subtracted
        {
            if (!degCalibratingNow && !PIDcalibratingNow) // if not calibrating anything else
            {
                radius = fmap(receiverValue[4], 1000, 2000, 0.02, 0.07); // edit the radius size, since the only variable that can add a costant drift is the radius when calculating angular velocity
            }
            deltaDeg += map(ch1Value, -1000, 1000, 100, -100); // directional led stick control
            lastTimeDrift = now;
        }

        /* Angular position wrapping */
        angPos += deltaDeg;   // angular position counting
        if (angPos > 360'000) // wrap around if over 360 deg
        {
            angPos -= 360'000;
        }

        /* LED activation */
        if (angPos > 360'000 - ledWidth / 2 || angPos < ledWidth / 2) // 0 degrees should be in the exact middle of the led line
        {
            digitalWrite(pinLED, HIGH);
        }
        else
        {
            digitalWrite(pinLED, LOW);
        }

        /* Motor advance manual calibration */
        unsigned long nowCalibrationDeg = millis();
        if (ch4Value >= -50 && ch4Value <= 50 && degCalibration == false && PIDcalibratingNow == false)
        {
            degCalibration = true;
            startCalibrationDeg = nowCalibrationDeg;
        }

        if (nowCalibrationDeg - startCalibrationDeg > 3000 && degCalibration == true)
        {
            degCalibratingNow = true;
            if (abs(ch4Value - previousCh4Value) > 50) // start calibrating next Point only if ch4Value changed more than 50, so there's a little bit of play
            {
                calPoint++;
            }

            updateLedWidthCal(calRPM[calPoint], 50);                                 // desired rpm where LED line will get shorter and the other value is the +-range
            motorsCalibrationDegRaw = map(receiverValue[4], 1000, 2000, 0, 360'000); // delay in degrees manually assigned at a certain rpm range
            calDeg[calPoint] = motorsCalibrationDegRaw;                              // assign the deg value to the current point that is being calibrated

            if (calPoint > numberOfPoints - 1) // 1 is subtracted since numberOfPoints includes the 0 too
            {
                degCalibration = false;
                degCalibratingNow = false;
                calPoint = 1;
                for (uint8_t i = 0; i < numberOfPoints - 1; i++) // write the 4 calibrated value skipping the first one that will always be zero
                {
                    unsigned long EEPROMStartAddress = 28;
                    EEPROM.put(EEPROMStartAddress + i * sizeof(calDeg[0]), calDeg[i + 1]); // write the value stored in calDeg array in the corresponding EEPROM address
                }
            }
        }
        else if ((ch4Value < -50 || ch4Value > 50) && degCalibratingNow == false) // if the first "if" is waiting for the timer to be true but the ch4 changes everything resets
        {
            degCalibration = false;
            degCalibratingNow = false;
        }

        /* PID Manual Calibration */
        unsigned long nowCalibrationDegPID = millis();
        if (ch4Value > 50 && PIDCalibration == false && degCalibratingNow == false) // starts counting timer to start PID calibration
        {
            PIDCalibration = true;
            startCalibrationDegPID = nowCalibrationDegPID;
        }

        if (nowCalibrationDegPID - startCalibrationDegPID > 3000 && PIDCalibration == true) // if the switch stays in the PID calibration start range for more than 3 seconds calibration starts
        {
            PIDcalibratingNow = true;
            if (abs(ch4Value - previousCh4Value) > 50) // start calibrating next PID coefficient only if ch4Value changed more than 50, so there's a little bit of play
            {
                PIDcoeff++;
            }

            switch (PIDcoeff) // map pid coefficient to calibration potentiometer (same used for motor)
            {
            case 1:
                consKp = fmap(receiverValue[4], 1000, 2000, 0, 10); // in PID Kp defines how aggressively the PID reacts to the current error with respect to the setpoint
                aggKp = consKp * 1.6;
                ledWidth = 180'000;
                consKi = 0;
                consKd = 0;
                aggKi = 0;
                aggKd = 0;
                break;
            case 2:
                consKi = fmap(receiverValue[4], 1000, 2000, 0, 50); // in PID Ki defines how aggressively the PID reacts to the sum of all previous errors with respect to the setpoint
                aggKi = consKi * 1.2;
                ledWidth = 90'000;
                break;
            case 3:
                consKd = fmap(receiverValue[4], 1000, 2000, 0, 10); // in PID Kd defines how aggressively the PID reacts to the prediction of future errors with respect to the setpoint
                aggKd = consKd * 1.2;
                ledWidth = 10'000;
                break;
            default:
                PIDcalibratingNow = false;
                PIDCalibration = false;
                PIDcoeff = 1;
                EEPROM.put(4, consKp);
                EEPROM.put(12, consKi);
                EEPROM.put(20, consKd);
                break;
            }
        }
        else if (ch4Value < 50 && PIDcalibratingNow == false) // if the first "if" is waiting for the timer to be true but the ch4 changes everything resets
        {
            PIDCalibration = false;
        }

        previousCh4Value = ch4Value;
        if (!degCalibratingNow && !PIDcalibratingNow) // if none of the calibrations are running use the standard led width
        {
            ledWidth = chosenLedWidth;
        }

        /* Deciding if the motor should be on or off */
        unsigned long motorPulseWidth = map(RPM, 0, maxRPM, minChosenMotorPulseWidth, maxChosenMotorPulseWidth); // motor pulse width mapped to the current RPM
        unsigned long motorsCalibrationDeg = Interpolation::Linear(calRPM, calDeg, numberOfPoints, RPM, true);   //(x values, y values, number of points, x value to find the corresponding y, no extrapolation if true)
        unsigned long motorActivation = (360000UL - (motorsCalibrationDeg % 360000UL)) % 360000UL;               // defining motor activation
        if (ch2Value < 0)
        {
            motorActivation = (motorActivation + 180000UL) % 360000UL; // motor activation is 180 deg later so it goes in reverse
        }
        unsigned long motorEnd = (motorActivation + motorPulseWidth) % 360000UL; // defining motor activation end

        if (motorActivation < motorEnd) // this if statement decides if the motor is in its activation range
        {
            // Normal case: example 340 - 350
            motorActive = (angPos >= motorActivation && angPos < motorEnd);
        }
        else
        {
            // Wrapped case: example 350 - 20
            motorActive = (angPos >= motorActivation || angPos < motorEnd);
        }

        /* Timer that stops any motor activation for a chosen amount of ms*/
        unsigned long MappedTimeBetweenActivations = map(abs(ch2Value), 0, 1000, minTimeBetweenActivations, 0); // send more activations if the ch2 value is bigger
        unsigned long nowMotorTimer = millis();
        if (nowMotorTimer - lastMotorActivation >= MappedTimeBetweenActivations) // if a defined amount of ms passed then the next motor Activation can be sent
        {
            if (motorActive == true && preventActivationChopping == true) // if the motor was supposed to be active it won't run, to prevent partially activating it
            {
                motorActive = false;
            }
            else
            {
                preventActivationChopping = false; // when the motor is off the real values will be sent, so it will now fully activate next time it is on
            }

            if (preventActivationChopping == false && previousMotorState != motorActive && motorActive == false) // once the activation is completed the timer will reset
            {
                preventActivationChopping = true;
                lastMotorActivation = nowMotorTimer;
            }

            previousMotorState = motorActive;
        }
        else // if the selected amount of millis has still not passed do not activate the motor
        {
            previousMotorState = motorActive; // needed so the first loop if the timer condition is true recieves the true motor previous state and not the overwritten one.
            motorActive = false;
        }

        /* Decide to use aggressive PID or conservative PID */
        RPMgoal = map(ch3Value, 0, 1000, 0, maxRPM);
        double gap = abs(RPMgoal - RPM);
        if (gap < 100)
        { // close to setpoint
            myPID.SetTunings(consKp, consKi, consKd);
        }
        else
        { // far from setpoint
            myPID.SetTunings(aggKp, aggKi, aggKd);
        }
        myPID.Compute(); // calculate the correct motorOutput value

        /* chose to activate motor based on receiver input */
        int signal1;
        int signal2;
        if (receiverValue[5] > 1500) // depending on the switch in ch6 spin is decided, when the robot flips over
        {
            signal1 = motorOutput;
            signal2 = motorOutput;
        }
        else
        {
            signal1 = -motorOutput;
            signal2 = -motorOutput;
        }

        if (motorActive && ch2Value != 0) // Decide signals for translation motor
        {
            signal1 = 0; // brake
        }

        /* Write the chosen signal in the motor ISR */
        noInterrupts(); // stop the throttle ISR
        throttle1 = convertThrottle(signal1);
        throttle2 = convertThrottle(signal2);
        interrupts(); // restart the ISR

#ifdef GETMAXRPM

        if (firstRun)
        {
            previousPeriodTest = periodUs; // periodTest gets initialized with the first periodUS value read
            firstRun = false;
        }

        unsigned long nowTest = millis();  // millis reference
        if (periodUs < previousPeriodTest) // if the new period is maller than the last written one
        {
            if (nowTest - lastEepromWriteTest > 300) // and if 300ms passed to not use eeprom too many times.
            {
                EEPROM.put(0, periodUs); // write the new value in eeprom
                previousPeriodTest = periodUs;
                lastEepromWriteTest = nowTest;
            }
        }
#endif
    }
/*
    Serial.print("calPoint=");
    Serial.print(calPoint);
    Serial.print(" RPM=");
    Serial.print(RPM);
    Serial.print(" desiredRPM=");
    Serial.print(calRPM[calPoint]);
    Serial.print(" min=");
    Serial.print(calRPM[calPoint] - 50.0);
    Serial.print(" max=");
    Serial.print(calRPM[calPoint] + 50.0);
    Serial.print(" ledWidth=");
    Serial.print(ledWidth);
    Serial.print("        ");
    Serial.print(calDeg[0]);
    Serial.print(" slot 0     ");
    Serial.print(calDeg[1]);
    Serial.print(" slot 1     ");
    Serial.print(calDeg[2]);
    Serial.print(" slot 2     ");
    Serial.print(calDeg[3]);
    Serial.print(" slot 3     ");
    Serial.print(calDeg[4]);
    Serial.print(" slot 4     ");
    Serial.print(degCalibratingNow);
    Serial.print(" degCalibratingNow     ");
    Serial.print(degCalibration);
    Serial.println(" degcalibration    ");
    delay(50);
*/
#ifdef SERIALCHECK2_LOOP
    delay(50);
    Serial.print(ch2Value);
    Serial.print("  ch2Value       ");
    Serial.print(ch1Value);
    Serial.print("  ch1Value       ");
    Serial.print(ch3Value);
    Serial.print("  ch3Value       ");
    Serial.print(ch4Value);
    Serial.print("  ch4Value      \r ");
#endif
#ifdef SERIALCHECKchannels_LOOP

    delay(50);
    Serial.print("      Number of channels: ");
    Serial.print(ChannelNumber);
    Serial.print("     CH1 [µs]: ");
    Serial.print(receiverValue[0]);
    Serial.print("     CH2 [µs]: ");
    Serial.print(receiverValue[1]);
    Serial.print("     CH3 [µs]: ");
    Serial.print(receiverValue[2]);
    Serial.print("     CH4 [µs]: ");
    Serial.print(receiverValue[3]);
    Serial.print("     CH5 [µs]: ");
    Serial.print(receiverValue[4]);
    Serial.print("     CH6 [µs]: ");
    Serial.print(receiverValue[5]);
    Serial.print("     CH7 [µs]: ");
    Serial.print(receiverValue[6]);
    Serial.print("     CH8 [µs]: ");
    Serial.print(receiverValue[7]);
    Serial.print("\r");

#endif
#ifdef SERIALCHECKaccelerometer_LOOP
    delay(50);

    sensors_event_t event;
    accel.getEvent(&event);
    Serial.print("\t\tX: ");
    Serial.print(event.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(event.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(event.acceleration.z);
    Serial.print(" m/s^2    \r");
#endif
#ifdef LOOPFREQUENCY
    unsigned long elapsedtime = micros() - start;
    delay(50);
    Serial.print(1 / (elapsedtime * 1e-6));
    Serial.print("  frequency          ");
    Serial.print(elapsedtime);
    Serial.print("  elapsed time      \r ");
#endif
}
