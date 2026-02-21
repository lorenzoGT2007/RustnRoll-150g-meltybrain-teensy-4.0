/* Notes:
Motor dshot (edited library): 0 = full stop, 1 - 1000 spin clockwise, 1001 - 2000 spin counterclockwise

max measured rpm of rust'n'roll 150g melty is 3500rpm

This code only works for motors that are 180deg apart

channel mixing code source: https://www.instructables.com/Understanding-Channel-Mixing/

ch3 is the rpm control, ch1 and ch2 are the directional controls, ch4 is for calibration and ch5 is for the radius calibration that corrects drift.
ch6 is for selecting the direction of spin in spin mode

Data Type Sizes on Teensy 4.0 (ARM Cortex-M7)
Data Type	Size (Bytes)	Notes
double	        8	IEEE 754 double-precision floating point
float	        4	IEEE 754 single-precision floating point
long	        4	Signed 32-bit integer
unsigned long	4	Unsigned 32-bit integer
int	            4	Signed 32-bit integer
short	        2	Signed 16-bit integer
char	        1	8-bit character


EEPROM used addresses: 0-3 getmaxRPM (4 Byte), 4-11 Kp (8B), 12-19 Ki (8B), 20-27 Kd (8B), 28+ calDeg
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
// #define SERIALCHECKDEGCAL //write calibrated deg values in serial
// #define MOTOR_OUTPUT // check the motor output value for debugging

#ifdef GETMAXRPM
bool firstRun = true;
unsigned int previousPeriodTest = 0;
unsigned int lastEepromWriteTest = 0;
#endif

/* values that need to be manually inserted */
const double minChosenMotorPulseWidth = 15.0;                   // in degrees, this value will be directly scaled as the RPM increases
constexpr double maxChosenMotorPulseWidth = 345.0;                  // in degrees, can't be bigger than 355 degrees because the sinusoidal function would overflow
const double chosenLedWidth = 40.0;                             // in degrees
double radius = 0.0473;                                         // meters, this is not a constant because it is used as a calibration value for the led drift
const double maxRPM = 3500.0;                                   // max goal rpm that will be used for full ch3 throttle, in the case of 2300 a number like 2000 as max is recommended
const unsigned int maxCalRPM = 2500;                            // max rpm for calibration
const unsigned int minCalRPM = 1000;                            // min rpm for calibration
const unsigned int maxTimeBetweenActivations = 500;             // in ms, it's the max time that needs to pass between each motor activation, this will be the max value possible once mapped to ch2
const unsigned int periodSecondsISR = 100;                      // period that sets the frequency of the isr for motor throttle
const int recieverFailsafeValues[4] = {1500, 1500, 1000, 1000}; // values that will be set when reciever signal is lost. ch1, ch2, ch3, ch4

/* global variables */
bool degCalibration = false;
bool degCalibratingNow = false;
bool PIDCalibration = false;
bool PIDcalibratingNow = false;
bool failsafeOn = false;
bool reversed = false;
bool peakReached = false;

int motorOutput = 0;
int lastMotorDerivative = 0;
int previousCh4Value = 0;

uint8_t PIDcoeff = 1;
uint8_t calPoint = 1;

unsigned int lastMotorActivation = 0;
unsigned int failsafeTimer = 0;
unsigned int initialMicros = 0;
unsigned int lastTimeDrift = 0;
unsigned int startCalibrationDeg = 0;
unsigned int startCalibrationDegPID = 0;

double ledWidth = chosenLedWidth;
double deltaDeg = 0;
double angPos = 0;
double periodSeconds = 0;
double motorsCalibrationDegRaw = 0;
double calRangeRPM = maxCalRPM - minCalRPM;

/* constants */
const double SQRT2 = 1.41421356;
const double maxMotorThrottle = 1000.0;
const double degToRad = PI / 180.0;

/* PID library */
double RPMgoal;        // your setpoint
double RPM;            // measured RPM from sensor
double targetThrottle; // PID output, 0..1000, double because the PID library expects doubles

double consKp = 0, consKi = 0, consKd = 0;                                  // conservative tuning when close to the desired range
double aggKp = 0, aggKi = 0, aggKd = 0;                                     // aggressive tuning when far from the desired range
PID myPID(&RPM, &targetThrottle, &RPMgoal, consKp, consKi, consKd, DIRECT); // initializing PID parameters

/* Point interpolation library */
const uint8_t numberOfPoints = 5;      // number of points
double calRPM[numberOfPoints] = {0};   // RPM values for calibration points, they will be equally spaced between minCalRPM and maxCalRPM and computed in setup
double calDeg[numberOfPoints] = {0.0}; // this will be filled later on with the values stored in eeprom

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
int ch1Value = 0;
int ch2Value = 0;
int ch3Value = 0;
int ch4Value = 0;

/* PPM communication object definition */
PulsePositionInput ReceiverInput(RISING);
int receiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0}; // array for channels values
int ChannelNumber = 0;

/* interrupt function for motor throttle */
volatile uint16_t throttle1 = 1048;
volatile uint16_t throttle2 = 1048;

static_assert(maxChosenMotorPulseWidth <= 355.0, 
  "CRITICAL ERROR: maxChosenMotorPulseWidth is too high and will cause a long int overflow!");//this is done to avoid overflows, the code will not compile

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
    unsigned int EEPROMStartAddress = 28;
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

  // Initialize RPM calibration points evenly distributed from minCalRPM to maxCalPM
  for (uint8_t i = 1; i < numberOfPoints; i++)
  {
    calRPM[i] = minCalRPM + (calRangeRPM * i / (numberOfPoints - 1));
  }

  bool interruptStart = sendThrottle.begin(updateThrottle, periodSecondsISR); // start motor throttle interrupt
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

#ifdef GETMAXRPM
  delay(10000); // wait so you have time to connect to serial once turned on
  EEPROM.get(0, periodSeconds);
  unsigned int rpm = 60.0 * 1000000.0 / periodSeconds;
  Serial.print(rpm);
  Serial.print(" rpm    ");
  Serial.print(periodSeconds);
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

  if (std::abs(CH) <= 10) // signal deadband when in neutral position
  {
    CH = 0;
  }
  CH = constrain(CH, min, max);
  return CH;
}

void mixEscSignals(int x, int y)
{ // this mixing algorith from a -1000 and 1000 input range spits out a -2000 and 2000 range if not mapped. but it gives the possibility to completely map a square joystick
  ch1Value = x + y;
  ch2Value = y - x;
  int diff = std::abs(x) - std::abs(y);

  if (ch1Value < 0)
  {
    ch1Value = ch1Value - std::abs(diff);
  }
  else
  {
    ch1Value = ch1Value + std::abs(diff);
  }

  if (ch2Value < 0)
  {
    ch2Value = ch2Value - std::abs(diff);
  }
  else
  {
    ch2Value = ch2Value + std::abs(diff);
  }

  ch1Value = map(ch1Value, -2000, 2000, -300, 300); // remap for -500 and 500 range, instead of max values so it is easier to control
  ch2Value = map(ch2Value, -2000, 2000, -300, 300);
  ch1Value = constrain(ch1Value, -1000, 1000);
  ch2Value = constrain(ch2Value, -1000, 1000);
}

double centripetalAccel() // since x and y axis are flared 45 deg out from the line that goes from the origin to the center of the melty the real accel needs to be evaluated
{
  sensors_event_t event;
  accel.getEvent(&event);
  double accelAverage = std::abs((event.acceleration.y + event.acceleration.x) / 2);
  return accelAverage * SQRT2; // µm/µs²  if the main acceleration direction is 45 degrees offset from both axis it is simply a square diagonal.
}

double angularVariation()
{
  double angVelocity = sqrt(centripetalAccel() / radius);

  periodSeconds = (2.0 * PI / angVelocity); // periodSeconds = 2π / angVelocity. for seconds per revolution.

  unsigned int now = micros();
  unsigned int deltaMicros = now - initialMicros;
  initialMicros = now;
  double deltaSeconds = deltaMicros * 1e-6;
  return angVelocity * deltaSeconds * (180.0 / PI); // degrees
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to update LED width based on RPM and calibration point
void updateLedWidthCal(double desiredRPM)
{
  double difference = std::abs(desiredRPM - RPM);                            // the closer it gets to the desired range the smaller the number becomes
  difference = constrain(difference, 0.0, maxRPM - maxRPM / 3.0);            // constrain the difference so maxrpm - maxrpm/3. this is an arbitrary value
  ledWidth = mapDouble(difference, 0.0, maxRPM - maxRPM / 3.0, 30.0, 360.0); // the bigger the difference the wider it gets, when on the value it reaches 10 degrees minum of width
}

void loop()
{
#ifdef LOOPFREQUENCY
  unsigned int start = micros();
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
    if (receiverValue[5] > 1500) // motors are reversed with a switch, when the robot flips over
    {
      ch1Value = -ch1Value;
    }
    mixEscSignals(ch1Value, ch2Value);
    if (failsafeOn == true) // if failsafe is on the led is always off
    {
      digitalWrite(pinLED, LOW);
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
  else if (ch3Value > 0 && ch3Value <= 1000) // spin mode, the last else is for failsafe
  {

    deltaDeg = angularVariation(); // in degrees

    if (periodSeconds > 0) // compute rpm if only periodSeconds is bigger than 0 to avoid dividing by zero
    {
      RPM = 60.0 / periodSeconds; // periodSeconds is computed inside angular variation
    }
    else
    {
      RPM = 0;
    }

    /* Angular position drift calibration and directional LED stick control */
    unsigned int now = micros();
    unsigned int driftFreq = periodSeconds / 200; // how many times every rotation it will be applied. if the loop runs at 10kHz and the robot is at 200rpm you get a max of 300 frames per revolution
    if (now - lastTimeDrift > driftFreq * 1000)   // drift calibration needs to be in function of rotations. so every rotation a certain amout of degrees is added/subtracted
    {
      if (reversed == true)
      {
        deltaDeg += mapDouble(ch1Value, -1000, 1000, 0.07, -0.07); // directional led stick control
      }
      else
      {
        deltaDeg -= mapDouble(ch1Value, -1000, 1000, 0.07, -0.07);
      }
      lastTimeDrift = now;
    }
    if (!degCalibratingNow && !PIDcalibratingNow) // if not calibrating anything else
    {
      radius = mapDouble(receiverValue[4], 1000, 2000, 0.02, 0.07); // edit the radius size, since the only variable that can add a costant drift is the radius when calculating angular velocity
    }

    /* Angular position evaluation and wrapping */
    angPos += deltaDeg; // angular position counting
    if (angPos > 360.0) // wrap around if over 360 deg
    {
      angPos -= 360.0;
    }

    /* LED activation */
    if (angPos > 360.0 - ledWidth / 2.0 || angPos < ledWidth / 2.0) // 0 degrees should be in the exact middle of the led line
    {
      digitalWrite(pinLED, HIGH);
    }
    else
    {
      digitalWrite(pinLED, LOW);
    }

    /* Motor advance manual calibration */
    unsigned int nowCalibrationDeg = millis();
    if (ch4Value >= -50 && ch4Value <= 50 && degCalibration == false && PIDcalibratingNow == false)
    {
      degCalibration = true;
      startCalibrationDeg = nowCalibrationDeg;
    }

    if (nowCalibrationDeg - startCalibrationDeg > 3000 && degCalibration == true)
    {
      degCalibratingNow = true;
      if (std::abs(ch4Value - previousCh4Value) > 50) // start calibrating next Point only if ch4Value changed more than 50, so there's a little bit of play
      {
        calPoint++;
      }

      updateLedWidthCal(calRPM[calPoint]);                                                       // visual feedback of the calibration point you are on, the led width gets smaller as you get closer to the desired rpm range of that calibration point
      motorsCalibrationDegRaw = mapDouble((double)receiverValue[4], 1000.0, 2000.0, 0.0, 360.0); // delay in degrees manually assigned at a certain rpm range
      calDeg[calPoint] = motorsCalibrationDegRaw;                                                // assign the deg value to the current point that is being calibrated

      if (calPoint > numberOfPoints - 1) // 1 is subtracted since numberOfPoints includes the 0 too
      {
        degCalibration = false;
        degCalibratingNow = false;
        calPoint = 1;
        for (uint8_t i = 0; i < numberOfPoints - 1; i++) // write the 4 calibrated value skipping the first one that will always be zero
        {
          unsigned int EEPROMStartAddress = 28;
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
    unsigned int nowCalibrationDegPID = millis();
    if (ch4Value < 50 && PIDCalibration == false && degCalibratingNow == false) // starts counting timer to start PID calibration
    {
      PIDCalibration = true;
      startCalibrationDegPID = nowCalibrationDegPID;
    }

    if (nowCalibrationDegPID - startCalibrationDegPID > 3000 && PIDCalibration == true) // if the switch stays in the PID calibration start range for more than 3 seconds calibration starts
    {
      PIDcalibratingNow = true;
      if (std::abs(ch4Value - previousCh4Value) > 50) // start calibrating next PID coefficient only if ch4Value changed more than 50, so there's a little bit of play
      {
        PIDcoeff++;
      }

      switch (PIDcoeff) // map pid coefficient to calibration potentiometer (same used for motor)
      {
      case 1:
        consKp = mapDouble(receiverValue[4], 1000.0, 2000.0, 0.0, 10.0); // in PID Kp defines how aggressively the PID reacts to the current error with respect to the setpoint
        aggKp = consKp * 1.6;
        ledWidth = 180'000;
        consKi = 0;
        consKd = 0;
        aggKi = 0;
        aggKd = 0;
        break;
      case 2:
        consKi = mapDouble(receiverValue[4], 1000, 2000, 0, 50); // in PID Ki defines how aggressively the PID reacts to the sum of all previous errors with respect to the setpoint
        aggKi = consKi * 1.2;
        ledWidth = 90'000;
        break;
      case 3:
        consKd = mapDouble(receiverValue[4], 1000, 2000, 0, 10); // in PID Kd defines how aggressively the PID reacts to the prediction of future errors with respect to the setpoint
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

    /* Decide to use aggressive PID or conservative PID and compute target Throttle*/
    RPMgoal = map(ch3Value, 0, 1000, 0, maxRPM);
    double gap = std::abs(RPMgoal - RPM);
    if (gap < 1000.0)
    { // close to setpoint
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    { // far from setpoint
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute(); // calculate the correct targetThrottle value

    /* function defined so a sinewave is used to determine the activation period

    motorThrottle= (maxMotorThrottle/(1+cos(motorPulseWidth/2)))(sin(angPos-calibratedDeg)+cos(motorPulseWidth/2))           with the conditions:

    targetthrottle <= maxMotorThrottle             this is defined as a constrain in the PID evaluation or just as max if directly scaled from a channel value
    motorThrottle = targetThrottle     if          motorThrottle>targetSpeed
    motorThrottle = maxMotorThrottle   if          motorThrottle>maxMotorThrottle. this is not possible to happen with this function, only if instead of (maxMotorThrottle/(1+cos(motorPulseWidth/2))), this is used: maxMotorThrottle
    motorThrottle = -maxMotorThrottle  if          motorThrottle<-maxMotorThrottle
    */
    double motorPulseWidth = mapDouble(RPM, 0.0, maxRPM, minChosenMotorPulseWidth, maxChosenMotorPulseWidth); // the pulse width gets bigger as the rpm increases to account for the delays in the motor response
    long int motorThrottle = (maxMotorThrottle/(1+cos((motorPulseWidth/2)*degToRad))) * (sin((angPos - motorsCalibrationDegRaw) * degToRad) + cos((motorPulseWidth / 2) * degToRad));
    long int motorThrottle2 = (maxMotorThrottle/(1+cos((motorPulseWidth/2)*degToRad))) * (sin((angPos - motorsCalibrationDegRaw - 180.0) * degToRad) + cos((motorPulseWidth / 2) * degToRad)); //second motor activates 180 degrees out of phase with the first one

    if (motorThrottle < -maxMotorThrottle)
    {
      motorThrottle = -maxMotorThrottle;
    }

    if (motorThrottle2 < -maxMotorThrottle)
    {
      motorThrottle2 = -maxMotorThrottle;
    }
    /*
    if the targetthrottle <= maxMotorThrottle is correctly constrained in the PID evaluation or just as max if directly scaled from a channel value, the condition
    below is not needed since the target throttle will never be higher than the max throttle

    if(motorThrottle>maxMotorThrottle){
      motorThrottle = maxMotorThrottle;
    }
    */

    /* Timer that stops any motor activation for a chosen amount of ms so translation can be faster or slower

     It will check the value of the derivative of the sinewave to detect the peaks, the best moment to chop translation or activate it.
     If the value goes from positive from negative, so when a peak is reached, it will start using the sinusoidal modulation.
     once it goes again from positive to negative, if the directional stick is near zero, it will stop using sinusoidal modulation
    */
    double motorThrottleDerivative = maxMotorThrottle * cos((angPos - motorsCalibrationDegRaw) * degToRad); // derivative of motorThrottle to know when the peak is reached, this is when the sinusoidal modulation will be used for tra

    if (std::abs(ch2Value) > 200) // if the directional stick is near zero the sinusoidal modulation is not used
    {
      if (lastMotorDerivative >= 0 && motorThrottleDerivative <= 0) // if the peak of the sinusoidal modulation is reached, this means that the motor is in the best position to be activated for a translation aka using sinusoidal modulation
      {
        peakReached = true;
      }
    }
    else if (peakReached == false) // if the peak is yet not reached it won't use the sinusoidal modulation, and will wait until the next peak
    {
      motorThrottle = targetThrottle;
      motorThrottle2 = targetThrottle;
    }

    if (peakReached == true && std::abs(ch2Value) <= 200) // once the directional stick is near zero it will wait for the next peak before stopping using sinusoidal modulation
    {
      if (lastMotorDerivative >= 0 && motorThrottleDerivative <= 0) 
      {
        peakReached = false;
      }
    }

    /* reverse the rotation direction */
    int signal2 = motorThrottle2;// it is mapped from 0 to 1000 so it is rotating only in a single direction
    int signal1 = motorThrottle;//this is the value for translation
    if (receiverValue[5] > 1500) // depending on the switch in ch6 spin is decided, when the robot flips over
    {
      reversed = false;
    }
    else
    {
      reversed = true;
      signal1 = -signal1;
      signal2 = -signal2;
    }

    /* Write the chosen signal in the motor ISR */
    noInterrupts(); // stop the throttle ISR
    throttle1 = convertThrottle(signal1);
    throttle2 = convertThrottle(signal2);
    interrupts(); // restart the ISR

#ifdef GETMAXRPM

    if (firstRun)
    {
      previousPeriodTest = periodSeconds; // periodTest gets initialized with the first periodSeconds value read
      firstRun = false;
    }

    unsigned int nowTest = millis();        // millis reference
    if (periodSeconds < previousPeriodTest) // if the new period is maller than the last written one
    {
      if (nowTest - lastEepromWriteTest > 300) // and if 300ms passed to not use eeprom too many times.
      {
        EEPROM.put(0, periodSeconds); // write the new value in eeprom
        previousPeriodTest = periodSeconds;
        lastEepromWriteTest = nowTest;
      }
    }
#endif
  }
  else // safety stop without any esc signal on ch3 using PPM
  {
    noInterrupts();
    throttle1 = 0;
    throttle2 = 0;
    interrupts();
  }
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
  unsigned int elapsedtime = micros() - start;
  delay(50);
  Serial.print(1 / (elapsedtime * 1e-6));
  Serial.print("  frequency          ");
  Serial.print(elapsedtime);
  Serial.print("  elapsed time microseconds \r ");
#endif
#ifdef MOTOR_OUTPUT
  delay(50);
  Serial.print(throttle1);
  Serial.print("  throttle1       ");
  Serial.print(throttle2);
  Serial.println("  throttle2   ");
#endif
}
