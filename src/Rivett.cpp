/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/john/Rivett/src/Rivett.ino"
#include <Wire.h>

void setup();
void loop();
void tach_interrupt();
#line 3 "/Users/john/Rivett/src/Rivett.ino"
int buttonPin = D0;         // the number of the input pin
int ledPin = D1;       // the number of the output pin

int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin
int count = -1;

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long mytime = 0;         // the last time the output pin was toggled
unsigned long debounce = 50;   // the debounce time, increase if the output flickers

//Tachometer
// these constants won't change:
int irEmit = D5;
int irRec = D4;
int rpm = 0;

long lastUpdate = 10;  // for timing display updates
volatile long accumulator = 0;  // sum of last 8 revolution times
volatile unsigned long startTime = 0; // start of revolution in microseconds
volatile unsigned int revCount = 0; // number of revolutions since last display update
// these variables will change:
int sensorReading = 0;


void setup()
{
  //Tachometer
  pinMode(irRec, INPUT_PULLUP);// declare the ledPin as as OUTPUT
  Serial.begin(9600);  // use the serial port
  attachInterrupt(digitalPinToInterrupt(irRec), tach_interrupt, FALLING);
  //digitalWrite(irEmit, HIGH);
  Particle.variable("rpm", rpm);
  Particle.variable("count",count);

  
  //Switch Counter - Part Counter Setup
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);

}

void loop()
{

   // read the sensor and store it in the variable sensorReading:
  
  //Serial.print("irREC: ");
  //Serial.println(digitalRead(irRec));
  
  delay(100);  // delay to avoid overloading the serial port buffer

  if (millis() - lastUpdate > 1000) // update every second
  {
    rpm = 0;
    // divide number of microseconds in a minute, by the average interval.
    if (revCount > 0)
    {
      rpm = 60000000 / (accumulator>>3);
    }
    //Particle.publish("rpm",rpm);
    Serial.print("RPM: ");
    Serial.println(rpm);
    Particle.publish("rpm");
    //matrix.println(rpm);
    //matrix.writeDisplay();
    
    lastUpdate = millis();
    revCount = 0;
  }
// Switch Counter - Part Counter
  Particle.variable("count",count);
  reading = digitalRead(buttonPin);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
if (reading == HIGH  && previous == LOW && (millis() - mytime) > debounce) {
  //Serial.println("MYTIME:");
  //Serial.println(mytime);
  //Serial.println(millis() - mytime);
  //Serial.println(reading);
  Serial.println(count);
  Particle.publish("count");
  if (state == HIGH)
  {
    count = count + 1;
    state = LOW;
    mytime = millis();
  }
  else
  {
    count = count + 1;
    state = HIGH;
    mytime = millis();    
  }
}


else
{
  if (state == LOW)
  {
    
    state = LOW;
    //Serial.println("LOW");
    digitalWrite(ledPin,state);
  }
  else 
  {
    
    state = HIGH;
    //Serial.println("HIGH");
    digitalWrite(ledPin, state);
  }
}
 //Serial.println(buttonPin);
  // 
   //Serial.println(reading);
previous = reading;
} 

void tach_interrupt()
{
  // calculate the microseconds since the last interrupt
  long usNow = micros();
  long elapsed = usNow - startTime;
  startTime = usNow;  // reset the clock
  
  // Accumulate the last 8 interrupt intervals
  accumulator -= (accumulator >> 3);
  accumulator += elapsed;
  revCount++;
}