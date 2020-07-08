// -----------------------------------------------------------------
//
//	Temp Humid Alt HP Time App
//	Copyright 2013 Tim Rotunda, Austin, Texas
//
// -----------------------------------------------------------------
//
//  Included Libraries
// -----------------------------------------------------------------
//
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "RTClib.h"
//
// -----------------------------------------------------------------
//  Defined parameters
// -----------------------------------------------------------------
//
#define HOLD 2000  
#define HOLD2 5000  
#define HOLD3 15000  

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
//
// -----------------------------------------------------------------
//  Assignments
// -----------------------------------------------------------------
//
Adafruit_BME280 bme; // I2C
RTC_DS3231 rtc;
//
// -----------------------------------------------------------------
//  Vars
// -----------------------------------------------------------------
//
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//
// -----------------------------------------------------------------
//  Sub routines
// -----------------------------------------------------------------
//
void displaySensorDetails(void)
{

	sensor_t sensor;
	bmp.getSensor(&sensor);
	Serial.print("----------------------------\n");
	Serial.print("Sensor:       "); Serial.println(sensor.name);
	Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
	Serial.print("Driver Ver:   "); Serial.println(sensor.version);
	Serial.print("-----------------------------\n\n");
	delay(HOLD);
}
//
//  Setups
//
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
	Wire.begin();
	Serial.begin(115200);
	byte error, address;
	int nDevices;

	Serial.println("Scanning I2C Devices...");

 	nDevices = 0;
	for(address = 1; address < 127; address++ ) 
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
//		Serial.print("I2C return value: ");			// Debug
//		Serial.print(error);Serial.print("\n");		// Debug
		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16) 
				Serial.print("0");
				Serial.print(address,HEX);
				Serial.print("\n");
				nDevices++;
			}
			else if (error==4) 
			{
		Serial.print("Unknow error at address 0x");
		if (address<16) 
			Serial.print("0");
			Serial.println(address,HEX);
		}    
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.print("Scan complete, ");
		Serial.print(nDevices);
		Serial.print(" I2C devices found\n");

	/* Initialise the sensor */
	if(!bmp.begin())
	{
		/* There was a problem detecting the BMP085 ... check your connections */
		Serial.print("Ooops, no BMP085 detected, program terminating ... Check your wiring or I2C ADDR!");
		while(1);
	}
  
	/* Display some basic information on this sensor */
	displaySensorDetails();


	#ifndef ESP8266
		while (!Serial); // wait for serial port to connect. Needed for native USB
	#endif

	if (! rtc.begin()) {
		Serial.println("Couldn't find RTC");
	while (1);
	}

	if (rtc.lostPower()) {
		Serial.println("RTC lost power, let's set the time!");
		// When time needs to be set on a new device, or after a power loss, the
		// following line sets the RTC to the date & time this sketch was compiled
 		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
 		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	}

	// When time needs to be re-set on a previously configured device, the
	// following line sets the RTC to the date & time this sketch was compiled
	// rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	// This line sets the RTC with an explicit date & time, for example to set
	// January 21, 2014 at 3am you would call:
	// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
	/* Get a new sensor event */ 
	sensors_event_t event;
	bmp.getEvent(&event);
	DateTime now = rtc.now();

 
	/* Display the results (barometric pressure is measure in hPa) */
	if (event.pressure)
	{
		/* Display atmospheric pressue in hPa */
		Serial.print("Pressure:    ");
		Serial.print(event.pressure, 1);
 		Serial.print(" hPa\t\t");
		Serial.print((event.pressure * (0.02952998751 * 1)), 2);
		Serial.print(" inMg\n");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
		float temperature;
		bmp.getTemperature(&temperature);
		Serial.print("Temperature: ");
		Serial.print(temperature, 1);
		Serial.print(" c\t\t");
		Serial.print(temperature * 1.8 + 32, 1);
		Serial.print(" f\n");

	/* Then convert the atmospheric pressure, and SLP to altitude         */
	/* Update this next line with the current SLP for better results      */
		float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
		Serial.print("Altitude:    "); 

		Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure), 0); 
		Serial.print(" m\t\t");

		Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure) / 0.3048, 0); 
		Serial.print("'\n\n");
	}
	else
//
// -------------------------------------------------------------------------
// Subroutines
// -------------------------------------------------------------------------
//
void scanIRQ()
{
   byte error, address;
   int nDevices;

   Serial.println("Scanning I2C Devices...");

   nDevices = 0;
   for(address = 1; address < 127; address++ ) 
   {
     // The i2c_scanner uses the return value of
     // the Write.endTransmisstion to see if
     // a device did acknowledge to the address.
     Wire.beginTransmission(address);
     error = Wire.endTransmission();

     if (error == 0)
     {
       Serial.print("I2C device found at address 0x");
       if (address<16) 
         Serial.print("0");
       Serial.println(address,HEX);

       nDevices++;
     }
     else if (error==4) 
     {
       Serial.print("Unknow error at address 0x");
       if (address<16) 
         Serial.print("0");
       Serial.println(address,HEX);
     }    
   }
   if (nDevices == 0)
     Serial.println("\nNo I2C devices found\n");
   else
     Serial.println("Scan Complete!\n");

//   delay(5000);           // wait 5 seconds for next scan
}


void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}
	{
		Serial.println("Sensor error");
	}



	Serial.print("It is " );
	Serial.print(now.hour(), DEC);
	Serial.print(":");
	Serial.print(now.minute(), DEC);
	Serial.print(":");
	Serial.print(now.second(), DEC);
	Serial.print(" ");
	Serial.print("on ");
	Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
	Serial.print(", ");
	Serial.print(now.month(), DEC);
	Serial.print(".");
	Serial.print(now.day(), DEC);
	Serial.print(".");
	Serial.print(now.year(), DEC);
	Serial.print("\nRTC Compensation Temp = ");
	Serial.print(rtc.getTemperature(), 1);
	Serial.print(" c\n\n\n");

	delay(HOLD2);
}


