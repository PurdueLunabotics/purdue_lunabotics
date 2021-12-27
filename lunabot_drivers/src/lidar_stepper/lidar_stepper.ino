// This code demonstrates how to generate two output signals
// with variable phase shift between them using an AVR Timer 

// The output shows up on Arduino pin 9, 10

// More AVR Timer Tricks at http://josh.com

void setup() {
  pinMode( 0, OUTPUT);
  pinMode( 1, OUTPUT);

  
  pinMode( 9 , OUTPUT );    // Arduino Pin  9 = OCR1A
  pinMode( 10 , OUTPUT );   // Arduino Pin 10 = OCR1B

  // Both outputs in toggle mode  
  TCCR1A = _BV( COM1A0 ) |_BV( COM1B0 );


  // CTC Waveform Generation Mode
  // TOP=ICR1  
  // Note clock is left off for now

  TCCR1B = _BV( WGM13) | _BV( WGM12);

  OCR1A = 0;    // First output is the base, it always toggles at 0


}

// prescaler of 1 will get us 8MHz - 488Hz
// User a higher prescaler for lower freqncies

#define PRESCALER 1
#define PRESCALER_BITS 0x01

#define CLK 16000000UL    // Default clock speed is 16MHz on Arduino Uno

// Output phase shifted wave forms on Arduino Pins 9 & 10
// freq = freqnecy in Hertz (  122 < freq <8000000 )
// shift = phase shift in degrees ( 0 <= shift < 180 )

// Do do shifts 180-360 degrees, you could invert the OCR1B by doing an extra toggle using FOC

/// Note phase shifts will be rounded down to the next neared possible value so the higher the frequency, the less phase shift resolution you get. At 8Mhz, you can only have 0 or 180 degrees because there are only 2 clock ticks per cycle.  

int setWaveforms( unsigned long freq , int shift ) {

  // This assumes prescaler = 1. For lower freqnecies, use a larger prescaler.

  unsigned long clocks_per_toggle = (CLK / freq) / 2;    // /2 becuase it takes 2 toggles to make a full wave

  ICR1 = clocks_per_toggle;

  unsigned long offset_clocks = (clocks_per_toggle * shift) / 180UL; // Do mult first to save precision

  OCR1B= offset_clocks;

  // Turn on timer now if is was not already on
  // Clock source = clkio/1 (no prescaling)
  // Note: you could use a prescaller here for lower freqnencies
  
  TCCR1B = 0x02;

}

// Demo by cycling through some phase shifts at 50Khz  

void loop() {
  digitalWrite(0,HIGH);
  digitalWrite(1,HIGH);
  setWaveforms( 100 , 90 ); 
}
