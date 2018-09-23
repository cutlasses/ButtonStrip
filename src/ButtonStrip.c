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

#define I2C_ADDRESS 222
//#define DATA_SIZE_BYTES 2
//#define DATA_SIZE_WORDS 1

#define NUM_LED_SWITCHES 8


typedef unsigned char byte;

byte led_values;
byte switch_values;

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

void interrupt ISR(void)
{
    //////////////////////////////////////////////////////////
    // I2C SLAVE INTERRUPT
    // Called when there is activity on the I2C bus
    if( PIR1bits.SSP1IF )
    {      
        PIR1bits.SSP1IF = 0; // clear interrupt flag

        if( !SSP1STATbits.D_nA ) // master has sent our slave address
        {
            byte d = SSP1BUF; // read and discard address to clear BF flag

            // Is the master setting up a data READ?
            if( SSP1STATbits.R_nW )
            {      
                // send the switch values
                SSP1BUF = switch_values;
            }
            else
            {
                // read the led values
                //led_values = SSP1BUF;
                SSP1BUF = 0;
            }
        }
        else // DATA (will this be reached if only read/writing a single byte)
        {
            if( SSP1STATbits.R_nW ) // MASTER IS READING FROM SLAVE
            {
                // send the switch values
                SSP1CON1bits.WCOL = 0; // clear write collision bit
                
                SSP1BUF = switch_values;
            }
            else // MASTER IS WRITING TO SLAVE
            {                                              
                // read the led values
                //led_values = SSP1BUF;
            }
        }
        
        SSP1CON1bits.CKP = 1; // release clock
    }
}

////////////////////////////////////////////////////////////
//
// I2C SLAVE INITIALISATION
//
////////////////////////////////////////////////////////////

void i2c_init( byte addr )
{
    SSP1CON1    	= 0b00100110;   // I2C slave mode, with 7 bit address, enable i2c
    SSP1MSK     	= 0b01111111;   // address mask bits 0-6
    SSP1ADD     	= addr<<1;      // set slave address
    PIE1bits.SSP1IE	= 1;
	PIR1bits.SSP1IF	= 0;
    
    // global enable interrupts
	INTCONbits.GIE  = 1;
    INTCONbits.PEIE = 1;
}

void main()
{
    led_values = 0;
    switch_values = 0;
    
    init_PIC();
    
    i2c_init( I2C_ADDRESS );
       
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
 
            //switch_values[i]    = PORTAbits.RA5;
            if( PORTAbits.RA5 )
            {
                switch_values   |= (PORTAbits.RA5 << i);
            }
            else
            {
                switch_values   &= ~(PORTAbits.RA5 << i);
            }
            
            led_values          = switch_values; // temp - set led on when switch is on
            //PORTCbits.RC2       = led_values[i]; // set RC2 high to turn on LEDs (sinking current through the transistor)
            
            if( led_values & (PORTAbits.RA5 << i) )
            {
                PORTCbits.RC2   = 1; // set RC2 high to turn on LEDs (sinking current through the transistor)
            }
            else
            {
                PORTCbits.RC2    = 0; // set RC2 low to turn off LEDs (transistor will not conduct)
            }

            __delay_ms(1);
        }
	}
}

