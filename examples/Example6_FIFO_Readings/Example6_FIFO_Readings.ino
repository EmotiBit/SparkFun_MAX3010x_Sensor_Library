/*
  MAX30105 Breakout: Take readings from the FIFO
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  Outputs all Red/IR/Green values at 25Hz by polling the FIFO

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
 
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "MAX30105.h"
#include "wiring_private.h"

MAX30105 particleSensor;

long startTime;
long samplesTaken = 0; //Counter for calculating the Hz or read rate
bool usingArduinoPlotter = false; // toggle to false if not using the plotter. When toggled off, it prints IR sensor as well
bool enableSerial = false; 
uint32_t maxTimeToCheck = 0;
uint32_t maxTimeToGet = 0;

// emotibit i2s definitaions
#if defined ARDUINO_FEATHER_ESP32
int timeToCheckIndicatorPin = 12;
int timeTogetIndicatorPin = 33;
#elif defined ADAFRUIT_FEATHER_M0
int timeToCheckIndicatorPin = 12;
int timeTogetIndicatorPin = 10;
#endif
uint8_t availablePinState = LOW;
TwoWire* _EmotiBit_i2c = nullptr;
int hibernatePin = 6;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");
#if defined ARDUINO_FEATHER_ESP32
  _EmotiBit_i2c = new TwoWire(0);
  Serial.print("Setting up I2C(For ESP32)....");
  bool status = _EmotiBit_i2c->begin(27, 13);
  //status = _EmotiBit_i2c->begin(EmotiBitVersionController::EMOTIBIT_I2C_DAT_PIN, EmotiBitVersionController::EMOTIBIT_I2C_CLK_PIN);
  //status = _EmotiBit_i2c->begin(27, 13);
  if (status)
  {
	  Serial.println("I2c setup complete");
  }
  else
  {
	  Serial.println("I2c setup failed");
  }
#else
  _EmotiBit_i2c = new TwoWire(&sercom1, 11,13);
  _EmotiBit_i2c->begin();
  // ToDo: detect if i2c init fails
  //Serial.println("I2C interface initialized");
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
#endif
  // set i2c clock
  _EmotiBit_i2c->setClock(400000);
  // Initialize sensor
  if (particleSensor.begin(*_EmotiBit_i2c) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //Setup to sense up to 18 inches, max LED brightness
  byte ledBrightness = 0x22;  // for MAX30100 -> 4 bits for Red and 4 bits for IR
  byte sampleAverage = 4; // irrelevant for MAX30100
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 50; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 1600; //Options: 200, 400, 800, 1600 
  int adcRange = 2048; //Options: 2048, 4096, 8192, 16384
  if (sampleRate > 100)
	  enableSerial = false; // disable serial if running the sesnor at high sampling rates, The serial starts to lag because of number of samples
  Serial.print("Sampling Rate: "); Serial.println(sampleRate);
  Serial.print("Pulse Width: "); Serial.println(pulseWidth);
  delay(2000);
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
//  particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive

  // set GPIO Pins
  pinMode(timeToCheckIndicatorPin, OUTPUT);
  pinMode(timeTogetIndicatorPin, OUTPUT);
  startTime = millis();
}

void loop()
{
	uint32_t timeToCheck = micros();
	digitalWrite(timeToCheckIndicatorPin, HIGH);
	particleSensor.check(); //Check the sensor, read up to 2 samples
	digitalWrite(timeToCheckIndicatorPin, LOW);
	timeToCheck = micros() - timeToCheck;
	if (maxTimeToCheck < timeToCheck)
	{
		maxTimeToCheck = timeToCheck;
	}
	while (particleSensor.available()) //do we have new data?
	{
		// to track the sampling rate in the digital oscilloscope
		samplesTaken++;
		uint32_t irData, redData;
		uint32_t timeToGetData = micros();
		digitalWrite(timeTogetIndicatorPin, HIGH);
		irData = particleSensor.getFIFOIR();
		redData = particleSensor.getFIFORed();
		digitalWrite(timeTogetIndicatorPin, LOW);
		timeToGetData = micros() - timeToGetData;
		if (maxTimeToGet < timeToGetData)
		{
			maxTimeToGet = timeToGetData;
		}
		if (enableSerial)
		{
			if (!usingArduinoPlotter)
			{
				Serial.print("IR:");
				Serial.print(irData);
				Serial.print(",");

			}
			Serial.print("Red:");
			Serial.print(redData);
			if (!usingArduinoPlotter)
			{
				Serial.print("\tHz[");
				Serial.print((float)samplesTaken / ((millis() - startTime) / 1000.0), 2);
				Serial.print("]");

			}
			Serial.println();
		}
		uint32_t timeForNextSample = micros();
		particleSensor.nextSample(); //We're finished with this sample so move to next sample
		timeForNextSample = micros() - timeForNextSample;
		if (Serial.available())
		{
			char input = Serial.read();
			// flush serial buffer
			while (Serial.available())
			{
				Serial.read();
			}
			Serial.print("maxTimeToCheck: "); Serial.print(maxTimeToCheck); Serial.println(" uS");
			Serial.print("maxTimeToGet: "); Serial.print(maxTimeToGet); Serial.println(" uS");
			Serial.print("timeForNextSample: "); Serial.print(timeForNextSample); Serial.println(" uS");
		}
		
	}
  
}
