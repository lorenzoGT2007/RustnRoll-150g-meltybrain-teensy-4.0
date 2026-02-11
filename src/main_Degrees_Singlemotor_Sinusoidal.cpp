/* Notes:
Motor dshot (edited library): 0 = full stop, 1 - 1000 spin clockwise, 1001 - 2000 spin counterclockwise

https://www.instructables.com/Understanding-Channel-Mixing/
*/

#include <Arduino.h>
#include <PulsePosition.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <DShot.h>
#include <math.h>

// #define SERIALCHECK1_LOOP //all channel raw data (us) and accelerometer printed out
// #define SERIALCHECK2_LOOP //channels mapped data
// #define SERIALCHECK1_SETUP //accelerometer settings and startup printed out

unsigned long measuredRadius = 47'300; // micrometers 1mm = 1000um. Doesn't need to be precise, it will be manually calibrated with the transmitter
int LEDwidth = 30;                     // degrees

unsigned long previousTimeLED = 0;
unsigned long LEDinterval = 0;
bool LEDon;

unsigned long calibratedRadius = 0;
unsigned long periodUs = 0;
unsigned long initialMicros = 0;
unsigned long lastTimeDrift = 0;
unsigned long ledOnMicros;
unsigned long now;
unsigned long periodStart;

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
// esc signal variables
int ch1Value = 0;
int ch2Value = 0;
int ch3Value = 0;
int ch4Value = 0;
int ch5Value = 0;
int ch6Value = 0;

// PPM communication object definition
PulsePositionInput ReceiverInput(RISING);
int receiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0}; // array for channels values
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
  delay(500);

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

int processChannelValue(int CH, int min, int max)
{
  CH = map(CH, 1000, 2000, min, max); // map for convertThrottle function
  CH = (abs(CH) <= 5) ? 0 : CH;       // signal deadband when in neutral position
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
  return accelAverage * sqrt(2);                                                // if the main acceleration direction is 45 degrees offset from both axis it is simply a square diagonal.
}

unsigned long calcPeriodUs()
{
  float angVelocity = sqrt(centripetalAccel() / ((float)calibratedRadius / 1'000'000.0f));

  return (unsigned long)((2.0f * 3.14159265f / angVelocity) * 1'000'000.0f); // periodUs = 2π / angVelocity * 1,000,000. for microseconds per revolution. used for drift calibration
}

void loop()
{
  readReceiver(); // read and store reciever channel values
  ch1Value = processChannelValue(receiverValue[0], -1000, 1000);
  ch2Value = processChannelValue(receiverValue[1], -1000, 1000);
  ch3Value = processChannelValue(receiverValue[2], 0, 1000);
  ch5Value = processChannelValue(receiverValue[4], -1000, 1000);
  ch6Value = processChannelValue(receiverValue[5], -1000, 1000);

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
    periodUs = calcPeriodUs();                                    // in microseconds
    
    ledOnMicros = (LEDwidth * periodUs) / 360; // width of led line in Us based on a simple proportion
    unsigned long now = micros();

    if (now - previousTimeLED >= LEDinterval)
    {
      LEDon = !LEDon;
      previousTimeLED = now;
    }

    if (LEDon)
    {
      digitalWrite(pinLED, HIGH);
      LEDinterval = ledOnMicros;
    }
    else
    {
      digitalWrite(pinLED, LOW);
      LEDinterval = periodUs - ledOnMicros;
    }

    noInterrupts();
    throttle1 = convertThrottle(-ch3Value); // changing the sign inverts the motor
    throttle2 = convertThrottle(-ch3Value);
    interrupts();
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
  Serial.print("  ch3Value      \r ");
#endif
#ifdef SERIALCHECK1_LOOP
  // Display the results (acceleration is measured in m/s^2)
  delay(50);
  sensors_event_t event;
  accel.getEvent(&event);
  Serial.print("\t\tX: ");
  Serial.print(event.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(event.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(event.acceleration.z);
  Serial.print(" m/s^2    ");

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
}
