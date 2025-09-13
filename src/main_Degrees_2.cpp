/* Notes:
Motor dshot (edited library): 0 = full stop, 1 - 1000 spin clockwise, 1001 - 2000 spin counterclockwise

This code only works for motors that are 180deg apart

max rpm measured rust'n'roll 150g melty: 2312

https://www.instructables.com/Understanding-Channel-Mixing/
*/

#include <Arduino.h>
#include <PulsePosition.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <DShot.h>
#include <math.h>
#include <EEPROM.h>

// #define SERIALCHECK1_LOOP //all channel raw data (us)
// #define SERIALCHECK2_LOOP // channels mapped data
// #define SERIALCHECK3_LOOP // accelerometer data
// #define SERIALCHECK1_SETUP //accelerometer settings and startup printed out

// #define GETMAXRPM // reads and stores in eeprom the max rpm value while the robot was spinning

#ifdef GETMAXRPM
bool firstRun = true;
unsigned long previousPeriodTest = 0;
unsigned long lastEepromWriteTest = 0;
#endif
/* values that needs to be manually inserted*/
unsigned long motorPulseWidth = 10'000; // in millideg
unsigned long chosenLedWidth = 40'000;  // in millideg
float radius = 0.0473;                  // meters

/* global variables */
unsigned long ledWidth = chosenLedWidth;
unsigned long deltaDeg = 0;
unsigned long angPos = 0;
unsigned long periodUs = 0;
unsigned long initialMicros = 0;
unsigned long lastTime = 0;
unsigned long startCalibration = 0;
bool calibration = false;
bool calibratingNow = false;
unsigned long motorsCalibrationDegRaw = 0;

int ch1Value = 0;
int ch2Value = 0;
int ch3Value = 0;
int ch4Value = 0;

// Pin declarations
const uint8_t pinESC1 = 1; // ESC pins. pin 1 = Serial1. pin 17 = Serial4
const uint8_t pinESC2 = 17;
const uint8_t pinCH1 = 14; // reciever signal PPM pin
const uint8_t pinLED = 16; // LED NPN transistor pin

#define H3LIS331_SCK 13 // accelerometer SPI pins
#define H3LIS331_MISO 12
#define H3LIS331_MOSI 11
#define H3LIS331_CS 10

// adafruit declaration of accelerometer object
Adafruit_H3LIS331 accel = Adafruit_H3LIS331();

// ESC Dshot declaration
DShot ESC1(&Serial1, DShotType::DShot600);
DShot ESC2(&Serial4, DShotType::DShot600);
IntervalTimer sendThrottle;

// PPM communication object definition
PulsePositionInput ReceiverInput(RISING);
long receiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0}; // array for channels values
int ChannelNumber = 0;

// interrupt function for motor throttle
volatile uint16_t throttle1 = 1048;
volatile uint16_t throttle2 = 1048;
void updateThrottle()
{
  ESC1.sendThrottle(throttle1, false); // 0 min to 2000 max. the first 48 values are for esc setup. center value between min and max is 1000
  ESC2.sendThrottle(throttle2, false); //(throttle value, telemetry from esc requested)
}

void setup()
{

  // Serial.begin(115200);

  accel.begin_SPI(H3LIS331_CS); // accelerometer parameters
  accel.setRange(H3LIS331_RANGE_200_G);
  accel.setDataRate(LIS331_DATARATE_1000_HZ);
  sensors_event_t event;
  accel.getEvent(&event);

  pinMode(pinLED, OUTPUT); // LED pin
  digitalWrite(pinLED, LOW);

  ReceiverInput.begin(pinCH1); // begin communication with receiver

  for (size_t i = 0; i < 4000; i++)
  {
    ESC1.sendCommand(0, false); // 0 command MUST be sent for dshot arming
    ESC2.sendCommand(0, false);
    delayMicroseconds(1'000);
  }

  bool interruptStart = sendThrottle.begin(updateThrottle, 500); // start motor throttle interrupt
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

void readReceiver()
{
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0)
  {
    for (int i = 1; i <= ChannelNumber; i++)
    {
      receiverValue[i - 1] = ReceiverInput.read(i);
    }
  }
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

int convertThrottle(int speed) // converts throttle for dshot library. values go from 0 to 2000
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

void loop()
{
  readReceiver(); // read and store reciever channel values
  ch1Value = processChannelValue(receiverValue[0], -1000, 1000);
  ch2Value = processChannelValue(receiverValue[1], -1000, 1000);
  ch3Value = processChannelValue(receiverValue[2], 0, 1000);
  ch4Value = processChannelValue(receiverValue[6], 0, 1000); // ch4 is slave of the aux switch on ch7
  // ch5 is for calibration so it won't be mapped to increase performance
  // same for ch6
  if (ch3Value == 0) // tank mode, CH3 is deadbanded for the first 5 values so they result in 0
  {
    mixEscSignals(ch1Value, ch2Value);
    digitalWrite(pinLED, LOW);

    noInterrupts();
    throttle1 = convertThrottle(-ch1Value); // changing the sign inverts the motor
    throttle2 = convertThrottle(ch2Value);
    interrupts();
  }
  else if (ch3Value > 0 && ch3Value <= 1000) // spin mode
  {
    deltaDeg = angularVariation(); // in microdegrees

    unsigned long now = micros();
    unsigned long driftFreq = periodUs / 200; // how many times every rotation it will be applied. if the loop runs at 10kHz and the robot is at 200rpm you get a max of 300 frames per revolution
    if (now - lastTime > driftFreq)           // drift calibration needs to be in function of rotations. so every rotation a certain amout of degrees is added/subtracted
    {
      deltaDeg += map(receiverValue[4], 1000, 2000, -200, 200); // 1 deg = 1'000'000 microdeg.
      deltaDeg += map(ch1Value, -1000, 1000, -100, 100);        // directional led stick control
      lastTime = now;
    }

    angPos += deltaDeg;   // angular position counting
    if (angPos > 360'000) // wrap around if over 360 deg
    {
      angPos -= 360'000;
    }

    if (angPos > 360'000 - ledWidth / 2 || angPos < ledWidth / 2) // 0 degrees should be in the exact middle of the led line
    {
      digitalWrite(pinLED, HIGH);
    }
    else
    {
      digitalWrite(pinLED, LOW);
    }

    unsigned long periodAtCalibration = 40'000;                 // µs at 1500 RPM, manually defined as optimal calibration rpm
    unsigned long periodMin = (60UL * 1000000UL) / (1500 + 50); // 1505 RPM
    unsigned long periodMax = (60UL * 1000000UL) / (1500 - 50); // 1495 RPM

    unsigned long nowCalibration = millis();
    if (ch4Value >= -50 && ch4Value <= 50 && calibration == false) // if switch is at center position activate calibration
    {
      calibration = true;
      startCalibration = nowCalibration;
    }

    if (nowCalibration - startCalibration > 3000 && calibration == true) // if the switch stays at center position for 3 seconds motorsCalibrationDegRaw starts changing
    {
      calibratingNow = true;
      motorsCalibrationDegRaw = map(receiverValue[5], 1000, 2000, 0, 360'000); // advance in degrees manually assigned

      if (periodUs >= periodMin && periodUs <= periodMax) // if the robot get to the ideal rpm range the led width changes
      {
        ledWidth = 10'000;
      }
      else
      {
        ledWidth = chosenLedWidth;
      }

      if (ch4Value < -50) // if the switch is set to low it saves the new settings
      {
        calibratingNow = false;
        calibration = false;
        // write in eeprom motors calibrationdeg
      }
    }
    else if ((ch4Value < -50 || ch4Value > 50) && !calibratingNow) // if the switch is moved when calibration is not active the timer is reset
    {
      calibration = false;
      calibratingNow = false;
    }

    unsigned long delay_ratio = periodAtCalibration / periodUs;                 // ratio between the calibrated period and the period at this moment
    unsigned long motorsCalibrationDeg = motorsCalibrationDegRaw * delay_ratio; // degrees anticipation * ratio

    int signal1 = -ch3Value;
    int signal2 = -ch3Value;

    unsigned long degStart = 360'000 - motorsCalibrationDeg;
    unsigned long degEnd = degStart + motorPulseWidth;
    unsigned long degOvf = fmax(degEnd - 360'000, 0);

    if ((angPos > degStart && angPos < degEnd) || angPos < degOvf)
    {
      signal1 = -ch2Value;
    }

    degStart = (degStart + 180'000) % 360;
    degEnd = degStart + motorPulseWidth;
    degOvf = fmax(degEnd - 360'000, 0);

    if ((angPos > degStart && angPos < degEnd) || angPos < degOvf)
    {
      signal2 = -ch2Value;
    }

    if (ch2Value == 0)
    {
      signal2 = -ch3Value;
      signal1 = -ch3Value;
    }

    noInterrupts();
    throttle1 = convertThrottle(signal1); // changing the sign inverts the motor
    throttle2 = convertThrottle(signal2);
    interrupts();

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
  else // safety stop without any esc signal on ch3 using PPM
  {
    noInterrupts();
    throttle1 = 0;
    throttle2 = 0;
    interrupts();
  }

#ifdef SERIALCHECK2_LOOP
  delay(50);
  Serial.print(ch1Value);
  Serial.print("  ch1Value       ");
  Serial.print(ch2Value);
  Serial.print("  ch2Value       ");
  Serial.print(ch3Value);
  Serial.print("  ch3Value       ");
  Serial.print(ch7Value);
  Serial.print("  ch7Value      \r ");
#endif
#ifdef SERIALCHECK1_LOOP
  // Display the results (acceleration is measured in m/s^2)
  delay(80);

  readReceiver();
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
  Serial.print("\r");
#endif
#ifdef SERIALCHECK3_LOOP
  sensors_event_t event;
  accel.getEvent(&event);
  Serial.print("\t\tX: ");
  Serial.print(event.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(event.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(event.acceleration.z);
  Serial.print(" m/s^2    ");
#endif
}
