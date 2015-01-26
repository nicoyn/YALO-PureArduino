/*
fht_adc.pde
guest openmusiclabs.com 9.5.12
example sketch for testing the fht library.
it takes in data on ADC0 (Analog0) and processes them
with the fht. the data is sent out over the serial
port at 115.2kb.  there is a pure data patch for
visualizing the data.
*/

// Fixed definitions cannot change on the fly.
#define LED_DT 11 // Data pin to connect to the strip.
#define COLOR_ORDER RGB // Are they RGB, GRB or what??
#define LED_TYPE WS2811 // What kind of strip are you using?
#define NUM_LEDS 50 // Number of LED's.
#include "FastLED.h" // FastLED library. Preferably the latest copy of FastLED 2.1.

// Initialize changeable global variables.
uint8_t max_bright = 128; // Overall brightness definition. It can be changed on the fly.
struct CRGB leds[NUM_LEDS]; // Initialize our LED array.

#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // set to 256 point fht
#include <FHT.h> // include the library

#define  IR_AUDIO  0 // ADC channel to capture

#define qsub(a, b) ((a>b)?a:0) // Unsigned subtraction macro. if result <0, then => 0.

volatile  int16_t position = 0; // Warning, byte is < 255  and so FHT_N can not be 256 !
volatile  uint16_t zero = 0;

#define nbBand 6  //Nbre of band to view
float noiseMatrix[nbBand] = {1.65, 1.5, 1.5, 1.25, 0.75, 0.25}; // Treshold by band, by testing -_-'
float noiseFactor;
byte noiselevel = 0;
byte noiseMin = 30; //Minimum noise level

//byte range[nbBand] = {2, 2, 4, 8, 12, 36}; //Bin repartition amongst band, should be = FTH_N/2
byte range[nbBand] = {1, 1, 2, 4, 8, 48}; //Bin repartition amongst band, should be = FTH_N/2
uint16_t data[nbBand]; //données actuelles

uint16_t maxPeak = 1;
uint16_t minPeak = 0;

byte Hue = 0;
byte HueDelta = 21;

void setup() {
  //Serial.begin(57600); // use the serial port
  
  LEDS.addLeds<LED_TYPE, LED_DT, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(max_bright);

  adcInit();
  adcCalb();
}

void loop() {
  if (position >= FHT_N)
  {
    uint16_t tmp = 0;
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fft
    fht_run(); // process the data in the fft
    fht_mag_log(); // take the output of the fft
    
    //Serial.write(255); // send a start byte
    //Serial.println(255); // send a start byte
    byte idx=0;
    noiseFactor = noiseMatrix[idx];
    noiselevel = noiseMin * noiseFactor;
    byte k=0;
    bool newMaxPeak = false;
    for (byte i = 0; i < FHT_N/2; i++){ //Only half of the bins are usable (NYQUIST)
            tmp = qsub(fht_log_out[i], noiseMin);
            //Serial.write(tmp); //Pour une vision des 64 bandes "brutes" sur la liaison RS
            /*
            Serial.print(i);
            Serial.print(':');
            Serial.println(tmp); // send a start byte
            */
            if ( tmp > maxPeak ){
              maxPeak = tmp;
              newMaxPeak = true;
            }
            if ( tmp < minPeak ) minPeak = tmp;

            if ( k == range[idx] || i == (FHT_N/2 - 1)){ //When we reached the numbers of bin to add, or if it's the last one
               maxPeak = maxPeak == minPeak ? 1 : maxPeak ; //beurk, permet de ne pas avoir maxpeak = minpeak
               data[idx] = qsub(data[idx], noiselevel) * ( 1 / noiseFactor );
               data[idx] = map(data[idx], minPeak, maxPeak, minPeak, 255) ;
               
               /*
               //Tracé du grap via processing si necessaire
               for ( byte j = 0; j < 10; j++ ){      
                  Serial.write(data[idx]);
               }
               */
               
               uint8_t avgLight = leds[idx].getAverageLight() * 3;
               
               /*
               Serial.print(i);
               Serial.print(')');
               Serial.print(k);
               Serial.print(':');
               Serial.print(idx);
               Serial.print(':');
               Serial.print(avgLight);
               Serial.print(':');
               Serial.println(data[idx]); // send a start byte
               */
               
               for ( byte x=idx; x < NUM_LEDS; x = x + nbBand )
               {
                  leds[x].nscale8(224); // Let's fade the whole thing over time as well.
                  if ( data[idx] > avgLight ) {
                     leds[x] = CHSV(Hue , 255, data[idx]); // Note how we really cranked up the tmp value to get BRIGHT LED's. Also increment the hue for fun.
                  }
               }
               Hue += HueDelta; // compute new hue
               data[idx]=0; //I'm starting with a new data set from this idx               
               idx++;
               noiseFactor = noiseMatrix[idx];
               noiselevel = noiseMin * noiseFactor;
               k=0;
            }
            data[idx] += tmp;
            k++;
       }
       if (! newMaxPeak){ //Max peak will decrease if no noise is detected
          maxPeak *= 0.9;
          if (maxPeak < minPeak ) maxPeak = minPeak;
       }
       /*
       Serial.write(0);
       Serial.write(0);
       Serial.write(0);
       Serial.write(0);
       */
       position = 0;
       FastLED.show();
    }
}

// free running ADC fills capture buffer
ISR(ADC_vect)
{
  if (position >= FHT_N) return;
  fht_input[position] = ADC + zero;
  position++;
}

void adcInit(){
  /*  REFS0 : VCC use as a ref, IR_AUDIO : channel selection, ADEN : ADC Enable, ADSC : ADC Start, ADATE : ADC Auto Trigger Enable, ADIE : ADC Interrupt Enable,  ADPS : ADC Prescaler  */
  // free running ADC mode, f = ( 16MHz / prescaler ) / 13 cycles per conversion 
  ADMUX = _BV(REFS0) | IR_AUDIO; // | _BV(ADLAR); 
  //ADMUX = _BV(REFS0) | IR_AUDIO | _BV(ADLAR); //ADLAR left shift for 8 bit instead of 16 reading
  //ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //prescaler 32 : 19231 Hz - 300Hz per 64 divisions
  ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) ; //prescaler 64 : 19231 Hz - 300Hz per 64 divisions
  //ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // prescaler 128 : 9615 Hz - 150 Hz per 64 divisions, better for most music
  sei();
}

void adcCalb(){ //Get global offset
  long midl = 0;
  // get 5 measurment at 0,5 sec
  // on ADC input must be NO SIGNAL!!!
  for (byte i = 0; i < 5; i++)
  {
    position = 0;
    delay(10);
    midl += fht_input[0];
    delay(190);
  }
  zero = -midl/5;
}


