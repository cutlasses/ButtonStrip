#define _XTAL_FREQ 16000000

#include <xc.h>

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define I2C_ADDRESS 111
#define DATA_SIZE_BYTES 12
#define DATA_SIZE_WORDS 6

#define NUM_LED_SWITCHES 8


typedef unsigned char byte;

byte led_values[NUM_LED_SWITCHES];
byte switch_values[NUM_LED_SWITCHES];

void init_PIC()
{
	// osc control / 16MHz / internal
	OSCCON      = 0b01111010;

	// configure io
	TRISA       = 0b11111111;                    
	TRISC       = 0b11000011;    // set 5,4,3,2 as output         
	ANSELA      = 0b11111111;
	ANSELC      = 0;             // set all PORT C to digital

	// timer0... configure source and prescaler
	OPTION_REG  = 0b10000100;

	// turn off the ADC
	ADCON1      = 0;
	ADCON0      = 0;    
}

void main()
{
    init_PIC();
    
    for( int i = 0; i < NUM_LED_SWITCHES; ++i )
    {
        led_values[i]       = 1;
        switch_values[i]    = 0;
    }
    
    // clear the shift register
    PORTCbits.RC5 = 0;      // DATA
    for( int i = 0; i < 8; ++i )
    {       
       PORTCbits.RC3 = 0;   // SHIFT CLOCK
       PORTCbits.RC3 = 1;   // SHIFT CLOCK
       
       PORTCbits.RC4 = 0;   // STORE CLOCK
       PORTCbits.RC4 = 1;   // STORE CLOCK
    }
    
	while(1) 
	{
        for( int i = 0; i < NUM_LED_SWITCHES; ++i )
        {
            if( i == 0 )
            {
                // turn first bit on, and shift down
                PORTCbits.RC5   = 1;      // DATA
            }
            else
            {
                // then turn it off
                PORTCbits.RC5   = 0;      // DATA
            }
                        
            PORTCbits.RC3       = 0;      // SHIFT CLOCK
            PORTCbits.RC3       = 1;      // SHIFT CLOCK
       
            PORTCbits.RC4       = 0;      // STORE CLOCK
            PORTCbits.RC4       = 1;      // STORE CLOCK
 
            switch_values[i]    = PORTAbits.RA5;
            
            //PORTCbits.RC2       = led_values[i]; // set RC2 high to turn on LEDs (sinking current through the transistor)
            PORTCbits.RC2       = switch_values[i];

            __delay_ms(1);
        }
	}
}

