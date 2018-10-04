#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <Bounce.h>
#include <ADC.h>

const int I2C_ADDRESS(0x01); 

void setup()
{
  Serial.println("Setup BEGIN");
  Serial.begin(9600);

  pinMode( LED_BUILTIN, OUTPUT );

  Wire.begin();

  digitalWrite( LED_BUILTIN, HIGH );
  
  delay(100);

  Serial.println("Setup END");
}

void loop()
{ 
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

  Serial.println("Loop");

  // read switch values
  Wire.requestFrom(I2C_ADDRESS, 1, false);
  //while( !Wire.available() );
  const byte switch_values = Wire.read();

  Serial.print( "switch values:");
  Serial.println( switch_values, BIN );

  //delay(10);

  // write led values
  const byte led_values = 0xFF;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(led_values);
  Wire.endTransmission();

  //Serial.print( "switch values:");
  //Serial.println( switch_values, BIN );
  //Serial.println( v2, BIN );
  //Serial.print( switch_values );
}

