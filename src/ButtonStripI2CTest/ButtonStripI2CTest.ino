#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <Bounce.h>
#include <ADC.h>

const int I2C_ADDRESS(222); 

void setup()
{
  Serial.println("Setup BEGIN");
  Serial.begin(9600);

  pinMode( LED_BUILTIN, OUTPUT );

  Wire.begin();

  digitalWrite( LED_BUILTIN, HIGH );
  
  delay(1000);

  Serial.println("Setup END");
}

void loop()
{ 
  Serial.println("");

  static bool led_on = true;
  led_on = !led_on;
  if( led_on )
  { 
    digitalWrite( LED_BUILTIN, HIGH );
  }
  else
  {    
    digitalWrite( LED_BUILTIN, LOW );
  }

  //const uint16_t time = 200;
  delay(100);

  // read switch values
  Wire.requestFrom(I2C_ADDRESS, 1);
  const byte switch_values = Wire.read();

  // write led values
//  const byte led_values = 0xFF;
//  Wire.beginTransmission(I2C_ADDRESS);
//  Wire.write(led_values);
//  Wire.endTransmission();

  Serial.print( "switch values:");
  Serial.print( switch_values, BIN );
}

