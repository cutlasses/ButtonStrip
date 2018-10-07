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

  static byte switch_values = 0;
  static byte led_values = 0;
  static byte step_num = 0;

  constexpr const uint64_t step_time_ms = 300;
  static uint64_t next_step_time_stamp = 0;

  constexpr const int NUM_STEPS(8);
  const uint64_t current_time = millis();
  if( current_time > next_step_time_stamp )
  {
    if( ++step_num >= NUM_STEPS )
    {
      step_num = 0;
    }

    next_step_time_stamp = current_time + step_time_ms;
  }

  led_values = (1 << step_num);

  for( int i = 0; i < NUM_STEPS; ++i )
  {
    const uint8_t bit_on = 1 << i;
    if( switch_values & bit_on )
    {
      step_num = i;
      //next_step_time_stamp = millis() + step_time_ms;
      break; // only interested in the lowest button
    }
  }

  // read switch values
  Wire.requestFrom(I2C_ADDRESS, 1, false);
  switch_values = Wire.read();

  //Serial.println(led_values, BIN);
  //Serial.print( "switch values:");
  //Serial.println( switch_values, BIN );

  // write led values
  Wire.beginTransmission(I2C_ADDRESS);
  //const uint8_t last_bit = 1 << 7;
  //Wire.write(last_bit);
  Wire.write(led_values);
  Wire.endTransmission();
}

