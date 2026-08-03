/* ============================================================================
   TEST BUILD — revamped single motor firmware (2026-08-02 bugfix pass)
   PID RPM control + on/off motor activation window for translation
   (the main build uses sinusoidal throttle modulation instead).

   NOT the competition firmware: that is src/main_Degrees_Singlemotor_Sinusoidal.cpp.

       pio run -e test_Degrees_Singlemotor_sinusoidal
   ============================================================================ */

/* ============================ CONTROLLER GUIDE (sticks / switches / knobs) ============================

CH1 (stick, left/right):
    - tank mode: steering
    - spin mode: rotates the LED heading line left/right (heading trim while spinning)
CH2 (stick, forward/back):
    - tank mode: forward/backwards. full power when driving straight, power blends down to
      minTankPowerPercent while steering so turns stay controllable
    - spin mode: translation. the further from center, the more frequent the motor activations
CH3 (throttle knob): spin RPM target. at zero = tank mode, above the deadband = spin mode, full = maxRPM
CH4 (3-position switch): calibration control, only works in spin mode
    - hold MIDDLE for 3s -> motor advance (deg) calibration starts
    - hold HIGH  for 3s -> PID calibration starts
    - while calibrating, every flip to a different position advances to the next point/coefficient
CH5 (knob):
    - normal spinning: radius trim (0.02 - 0.07 m), turn it until the LED heading line stops drifting
    - deg calibration: sets the advance angle (0 - 360 deg) of the current calibration point
    - PID calibration: sets the current coefficient value
CH6 (2-position switch): spin/motor direction, flip it when the robot is driving upside down

MOTOR ADVANCE (DEG) CALIBRATION - point RPM targets: 1375 / 1750 / 2125 / 2500
    1. enter spin mode, hold CH4 in the MIDDLE for 3 seconds -> point 1 is active
    2. bring the RPM close to the point target with CH3: the LED line gets NARROWER the closer you are
    3. turn CH5 until the robot translates in the direction of the LED line (push CH2 to test)
    4. flip CH4 to any other position -> next point
    5. after setting the 4th point, one more flip saves everything to EEPROM and exits

PID CALIBRATION - LED width shows the active coefficient: Kp = 180 deg, Ki = 90 deg, Kd = 10 deg
    1. enter spin mode, hold CH4 HIGH for 3 seconds -> Kp is active (Ki and Kd are reset to 0 here)
    2. set the value with CH5 (Kp: 0-10, Ki: 0-50, Kd: 0-10)
    3. flip CH4 -> Ki, flip again -> Kd
    4. one more flip saves Kp/Ki/Kd to EEPROM and exits

FAILSAFE: after 1s without receiver signal the controls are neutralized (motors stop, LED off in tank mode)

========================================================================================================

Notes:
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


EEPROM used addresses: 0 - 3 getmaxRPM, 4 - 11 Kp, 12 - 19 Ki, 20 - 27 Kd, 28+ calDeg

REVAMP (2026-08-02) — summary of the edits, each one is also marked with an "EDIT:" comment where it happens:
 - startup cut from ~4.5s to ~0.55s (ESC arming 4000->500 frames, accel retry 1000ms->10ms)
 - fixed LED never lighting during the accelerometer wait (pinMode was after the wait loop)
 - fixed throttle init of 1048 (= ~5% reverse throttle in the 3D dshot mapping, motors crept at boot)
 - fixed full-throttle window at boot when no receiver signal (receiverValue now starts at failsafe values)
 - fixed out-of-bounds write to calRPM[5]/calDeg[5] during motor advance calibration
 - fixed unsigned underflow of deltaDeg at low RPM and single-step angPos wrap
 - fixed first angularVariation() integrating the whole setup time
 - guards: div-by-zero when not spinning, NaN from never-written EEPROM cells
 - perf: digitalWriteFast, float-only math in the accel path, constants precomputed, PID sample time 100ms->10ms
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

/* ------------------------- serial debug switches --------------------------
   uncomment one or more to enable the matching Serial prints (Serial is
   started automatically in setup when any of these is defined) */
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
const unsigned minTimeBetweenActivations = 500;                  // in ms, it's the min time that needs to pass between each motor activation, this will be the max value possible once mapped to ch2
const int minTankPowerPercent = 50;                              // EDIT: tank mode power left at full steering, in percent. driving straight is always 100%, steering blends down to this value (50 = the old fixed reduction)
const unsigned long periodUsISR = 100;                           // period that sets the frequency of the isr for motor throttle
const long recieverFailsafeValues[4] = {1500, 1500, 1000, 1000}; // values that will be set when reciever signal is lost. ch1, ch2, ch3, ch4

/* precomputed constants (EDIT: computed once instead of every loop) */
const float SQRT2_F = 1.41421356f;         // sqrt(2), was computed with sqrt() on every accelerometer read
const float RAD_TO_MILLIDEG = 57295.7795f; // (180 / pi) * 1000, rad -> millideg in a single multiply
const float TWO_PI_F = 6.28318531f;

/* global variables */
unsigned long failsafeTimer = 0;
unsigned long ledWidth = chosenLedWidth;
long deltaDeg = 0; // EDIT: was unsigned long. the led stick drift correction subtracts from it, at low RPM it underflowed and wrapped to ~4 billion millideg
long angPos = 0;   // EDIT: signed too so a negative deltaDeg can be wrapped correctly
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
bool reversed = false;
// EDIT: removed motorActivationOngoing, it was never used anywhere

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
// EDIT: was all zeros. with the shorter startup the loop now starts before the receiver has sent a frame
// and before the 1000ms failsafe timeout: raw zeros mapped to full stick deflection in tank mode, which
// meant FULL THROTTLE on both motors until the first PPM frame or the failsafe kicked in.
// starting from the failsafe/neutral values keeps the motors stopped until real data arrives.
long receiverValue[] = {1500, 1500, 1000, 1000, 1500, 1000, 1500, 1500}; // array for channels values
int ChannelNumber = 0;

/* interrupt function for motor throttle */
// EDIT: was 1048. in the 3D dshot mapping of the edited library 1048 is ~5% throttle in reverse,
// so both motors crept as soon as the arming sequence finished. 0 = full stop.
volatile uint16_t throttle1 = 0;
volatile uint16_t throttle2 = 0;
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

    pinMode(pinLED, OUTPUT);        // EDIT: moved BEFORE the accelerometer wait. it was configured after it, so the "waiting" LED write went to an unconfigured pin and never lit
    digitalWriteFast(pinLED, HIGH); // turn on LED while waiting for the accelerometer to be found

    while (!accel.begin_SPI(H3LIS331_CS)) // wait until the accelerometer is found
    {
        delay(10); // EDIT: was 1000ms per retry, begin_SPI is fast so this only added up to a second of dead time at boot
    }
    digitalWriteFast(pinLED, LOW);

    accel.setRange(H3LIS331_RANGE_400_G);
    accel.setDataRate(LIS331_DATARATE_1000_HZ);
    sensors_event_t event;
    accel.getEvent(&event);

    ReceiverInput.begin(pinCH1); // begin communication with receiver

    EEPROM.get(4, consKp);
    EEPROM.get(12, consKi);
    EEPROM.get(20, consKd);
    // EDIT: a never-written EEPROM cell reads 0xFF..FF which is NaN as a double. NaN would flow
    // through the PID into the throttle. fall back to 0 (PID off) until a calibration is saved.
    if (isnan(consKp))
        consKp = 0;
    if (isnan(consKi))
        consKi = 0;
    if (isnan(consKd))
        consKd = 0;
    aggKp = consKp * 1.6; // aggressive tuning when far from the desired range
    aggKi = consKi * 1.2;
    aggKd = consKd * 1.2;
    myPID.SetMode(AUTOMATIC);       // turn PID on
    myPID.SetOutputLimits(1, 1000); // limit output to motor range
    myPID.SetSampleTime(10);        // EDIT: default is 100ms, so the PID only reacted 10 times/s on a robot spinning at up to 58 rev/s.
                                    // the library rescales Ki/Kd internally so the saved tunings keep the same meaning. delete this line to get the old behavior back

    for (uint8_t i = 0; i < numberOfPoints - 1; i++) // read the calibrated values skipping the first one that will always be zero
    {
        unsigned long EEPROMStartAddress = 28;
        EEPROM.get(EEPROMStartAddress + i * sizeof(calDeg[0]), calDeg[i + 1]); // write the eeprom values into the corresponding calDeg arrays
        if (isnan(calDeg[i + 1]))                                              // EDIT: same NaN guard as the PID gains
            calDeg[i + 1] = 0;

#ifdef SERIALCHECKDEGCAL
        Serial.print(calDeg[i + 1]);
        Serial.print("    ");
#endif
    }

    // EDIT: 4000 -> 500 frames (4s -> 0.5s). dshot ESCs arm after a few hundred ms of continuous stop
    // commands, 500 x 1ms is the value already proven on this robot by the non-test sinusoidal build.
    // this is the physical limit of the ESC firmware, don't go much lower or the ESCs may not arm.
    for (size_t i = 0; i < 500; i++) // arming sequence for ESC using dshot
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
            digitalWriteFast(pinLED, HIGH);
            delay(100);
            digitalWriteFast(pinLED, LOW);
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
    // EDIT: removed the second begin_SPI and the setRange(H3LIS331_RANGE_200_G) that was here:
    // it silently overrode the 400G range configured above whenever this debug flag was enabled
    Serial.print("Range set to: ");
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

    initialMicros = micros(); // EDIT: baseline for the first angularVariation(). without it the first delta
                              // integrated the whole setup time (~90,000+ millideg jump on the first loop)
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
        // EDIT(review): PulsePosition can report up to 16 channels if noise injects extra edges into a
        // frame, but receiverValue only has 8 slots. every channel is still read (so the library clears
        // its frame flag) but only the first 8 are stored, the rest went past the end of the array
        const int maxCh = (int)(sizeof(receiverValue) / sizeof(receiverValue[0]));
        for (int i = 1; i <= ChannelNumber; i++)
        {
            long value = ReceiverInput.read(i);
            if (i <= maxCh)
            {
                receiverValue[i - 1] = value;
            }
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

    // EDIT: was a fixed map to -500..500 that halved the power everywhere, which made straight driving slow.
    // now the mix is mapped to the full -1000..1000 motor range and then reduced only as a function of how
    // much steering is applied: stick centered = 100% power, full steering = minTankPowerPercent (the old feel)
    ch1Value = map(ch1Value, -2000, 2000, -1000, 1000);
    ch2Value = map(ch2Value, -2000, 2000, -1000, 1000);
    int powerScale = map(constrain(abs(x), 0, 1000), 0, 1000, 100, minTankPowerPercent); // steering amount -> power percentage
    ch1Value = ch1Value * powerScale / 100;
    ch2Value = ch2Value * powerScale / 100;
    ch1Value = constrain(ch1Value, -1000, 1000);
    ch2Value = constrain(ch2Value, -1000, 1000);
}

float centripetalAccel() // since x and y axis are flared 45 deg out from the line that goes from the origin to the center of the melty the real accel needs to be evaluated
{
    sensors_event_t event;
    accel.getEvent(&event);
    float accelAverage = fabsf((event.acceleration.y + event.acceleration.x) * 0.5f); // EDIT: fabsf + multiply, stays in single precision
    return accelAverage * SQRT2_F;                                                    // EDIT: constant instead of calling sqrt(2) on every read. if the main acceleration direction is 45 degrees offset from both axis it is simply a square diagonal.
}

long angularVariation()
{
    float angVelocity = sqrtf(centripetalAccel() / (float)radius); // EDIT: sqrtf keeps the whole chain in single precision (the M7 FPU does doubles at half speed)

    // EDIT: guard. when the robot is not spinning angVelocity is ~0 and 2pi/angVelocity was cast from
    // inf to unsigned long (undefined behavior on ARM). report "not spinning" and keep the time baseline fresh
    if (angVelocity < 0.5f) // 0.5 rad/s is about 5 rpm
    {
        periodUs = 0; // sentinel: RPM is computed as 0 when periodUs == 0
        initialMicros = micros();
        return 0;
    }

    periodUs = (unsigned long)((TWO_PI_F / angVelocity) * 1'000'000.0f); // periodUs = 2π / angVelocity * 1,000,000. for microseconds per revolution.

    unsigned long now = micros();
    unsigned long deltaMicros = now - initialMicros;
    initialMicros = now;

    return (long)(angVelocity * (deltaMicros * 1e-6f) * RAD_TO_MILLIDEG); // delta in millidegrees. EDIT: single multiply by a precomputed rad->millideg constant
}

double fmap(long x, long in_min, long in_max, double out_min, double out_max)
{
    return (double)(x - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

// Function to update LED width based on RPM and calibration point
void updateLedWidthCal(double desiredRPM)
{
    double difference = fabs(desiredRPM - RPM);                        // the closer it gets to the desired range the smaller the number becomes
    difference = constrain(difference, 0, maxRPM - maxRPM / 3);        // constrain the difference so maxrpm - maxrpm/3. this is an arbitrary value
    ledWidth = map(difference, 0, maxRPM - maxRPM / 3, 30'000, 360'000); // the bigger the difference the wider it gets, when on the value it reaches 10 degrees minum of width
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
    /* =============================== TANK MODE ================================
       CH3 at zero (it is deadbanded, so the first few values still count as 0) */
    if (ch3Value == 0)
    {
        mixEscSignals(ch1Value, ch2Value);
        if (failsafeOn == true) // if failsafe is on the led is always off
        {
            digitalWriteFast(pinLED, LOW);
        }
        else
        {
            digitalWriteFast(pinLED, HIGH); // LED always on in tank mode
        }
        delayMicroseconds(1000); // slow down the loop, speed is not required in this case

        noInterrupts();
        throttle1 = convertThrottle(-ch1Value); // changing the sign inverts the motor
        throttle2 = convertThrottle(ch2Value);
        interrupts();
    }
    /* =============================== SPIN MODE ================================ */
    else if (ch3Value > 0 && ch3Value <= 1000)
    {
        deltaDeg = angularVariation();                              // in millidegrees
        RPM = (periodUs > 0) ? (60.0 * 1000000.0 / periodUs) : 0.0; // EDIT: guard, periodUs is 0 when the robot is not spinning (see angularVariation)

        /* Angular position drift calibration and directional LED stick control */
        unsigned long now = micros();
        unsigned long driftFreq = periodUs / 200;                  // how many times every rotation it will be applied. if the loop runs at 10kHz and the robot is at 200rpm you get a max of 300 frames per revolution
        if (periodUs != 0 && now - lastTimeDrift > driftFreq)      // EDIT(review): skip when not spinning. periodUs 0 (the sentinel) made driftFreq 0, so the led stick drift was applied every single loop below ~5rpm and scrambled the heading during spin-up
        {
            if (!degCalibratingNow && !PIDcalibratingNow) // if not calibrating anything else
            {
                radius = fmap(receiverValue[4], 1000, 2000, 0.02, 0.07); // edit the radius size, since the only variable that can add a costant drift is the radius when calculating angular velocity
            }

            if (reversed == true)
            {
                deltaDeg += map(ch1Value, -1000, 1000, 100, -100); // directional led stick control
            }
            else
            {
                deltaDeg -= map(ch1Value, -1000, 1000, 100, -100);
            }
            lastTimeDrift = now;
        }

        /* Angular position wrapping */
        angPos += deltaDeg; // angular position counting
        // EDIT: modulo wrap instead of a single -= 360000. the old version could not recover from a
        // delta bigger than one revolution and could not handle a negative deltaDeg (led stick at low RPM)
        angPos %= 360'000;
        if (angPos < 0)
        {
            angPos += 360'000;
        }

        /* LED activation */
        if ((unsigned long)angPos > 360'000UL - ledWidth / 2 || (unsigned long)angPos < ledWidth / 2) // 0 degrees should be in the exact middle of the led line
        {
            digitalWriteFast(pinLED, HIGH);
        }
        else
        {
            digitalWriteFast(pinLED, LOW);
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

            // EDIT: completion check moved BEFORE the array writes. calPoint reaches numberOfPoints (5) here
            // and the old order wrote calRPM[5]/calDeg[5] first: an out-of-bounds write that corrupted memory
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
            else
            {
                updateLedWidthCal(calRPM[calPoint]);                                                              // desired rpm where LED line will get shorter and the other value is the +-range
                motorsCalibrationDegRaw = map(constrain(receiverValue[4], 1000L, 2000L), 1000, 2000, 0, 360'000); // EDIT(review): constrained, transmitters go below 1000us at full deflection and a negative map() result wrapped to ~4 billion millideg in this unsigned variable (and got saved to EEPROM)
                calDeg[calPoint] = motorsCalibrationDegRaw;                              // assign the deg value to the current point that is being calibrated
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
        unsigned long motorPulseWidth = map(RPM, 0, maxRPM, minChosenMotorPulseWidth, maxChosenMotorPulseWidth);      // motor pulse width mapped to the current RPM
        motorPulseWidth = constrain(motorPulseWidth, minChosenMotorPulseWidth, maxChosenMotorPulseWidth);             // EDIT: map extrapolates when RPM overshoots maxRPM, keep the width in its intended range
        unsigned long motorsCalibrationDeg = Interpolation::Linear(calRPM, calDeg, numberOfPoints, RPM, true);        //(x values, y values, number of points, x value to find the corresponding y, no extrapolation if true)
        unsigned long motorActivation = (360000UL - (motorsCalibrationDeg % 360000UL)) % 360000UL;                    // defining motor activation
        if (ch2Value < 0)
        {
            motorActivation = (motorActivation + 180000UL) % 360000UL; // motor activation is 180 deg later so it goes in reverse
        }
        unsigned long motorEnd = (motorActivation + motorPulseWidth) % 360000UL; // defining motor activation end

        if (motorActivation < motorEnd) // this if statement decides if the motor is in its activation range
        {
            // Normal case: example 340 - 350
            motorActive = ((unsigned long)angPos >= motorActivation && (unsigned long)angPos < motorEnd);
        }
        else
        {
            // Wrapped case: example 350 - 20
            motorActive = ((unsigned long)angPos >= motorActivation || (unsigned long)angPos < motorEnd);
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
        if (gap < 200)
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
            reversed = false;
            signal1 = motorOutput;
            signal2 = motorOutput;
        }
        else
        {
            reversed = true;
            signal1 = -motorOutput;
            signal2 = -motorOutput;
        }

        if (motorActive && ch2Value != 0) // Decide signals for translation motor
        {
            signal2 = reversed ? 1000 : -1000; // EDIT(review): brake must oppose the CURRENT spin direction. a fixed -1000 was full throttle in the spin direction when the robot was flipped (ch6), accelerating instead of braking
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
    // EDIT: removed a ~30 line commented-out Serial debug block that was here, the same data is available through the SERIALCHECK defines

    /* ------- debug prints, enabled by the switches at the top of the file ------- */
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
