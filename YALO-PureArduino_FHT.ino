/*
fht_adc.pde
guest openmusiclabs.com 9.5.12
example sketch for testing the fht library.
it takes in data on ADC0 (Analog0) and processes them
with the fht. the data is sent out over the serial
port at 115.2kb.  there is a pure data patch for
visualizing the data.
*/

#include "FastLED.h" // FastLED library. Preferably the latest copy of FastLED 2.1.
// Fixed definitions cannot change on the fly.
#define LED_DT 11 // Data pin to connect to the strip.
#define COLOR_ORDER RGB // Are they RGB, GRB or what??
#define LED_TYPE WS2811 // What kind of strip are you using?
#define NUM_LEDS 50 // Number of LED's.
// Initialize changeable global variables.
uint8_t max_bright = 128; // Overall brightness definition. It can be changed on the fly.
struct CRGB leds[NUM_LEDS]; // Initialize our LED array.

#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // set to 256 point fht
#include <FHT.h> // include the library

#define  IR_AUDIO  0 // ADC channel to capture

volatile  int16_t  position = 0;
volatile  uint16_t zero = 0;
uint16_t calibration[FHT_N/2];

#define nbBand 6
float noiseMatrix[nbBand] = {2.0, 1.5, 1.5, 1.25, 0.90, 0.60};
uint8_t range[FHT_N/2] = {1, 2, 2, 7, 13, 30}; //Repartition des bandes dans le spectre

uint8_t data[nbBand]; //données actuelles
uint8_t lastData[nbBand];  //run précédent
byte calibrated = 0;
uint8_t noiselevel = 0;
uint8_t maxPeak = 0;
uint8_t minPeak = 0;
uint8_t noiseMin = 40;

uint8_t Hue = 0;
uint8_t HueDelta = 42;
//uint8_t  HueMax = 64; //Pas besoin si HueMax = 255

void setup() {
  //Serial.begin(57600); // use the serial port
  LEDS.addLeds<LED_TYPE, LED_DT, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(max_bright);
  adcInit();
  adcCalb();
}

void loop() {
  int tmp = 0;
  if (position >= FHT_N)
  {
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fft
    fht_run(); // process the data in the fft
    fht_mag_log(); // take the output of the fft
    
    if (calibrated < 5){
       for (int16_t i = 0; i < FHT_N/2; i++){
         calibration[i] += fht_log_out[i];
       }
       delay(10);
       calibrated++;
    } else {
       //Serial.write(255); // send a start byte
       byte idx=0;
       float noiseFactor=noiseMatrix[idx];
       byte k=0;
       bool newMaxPeak = false;
       for (int16_t i = 0; i < FHT_N/2; i++){
            calibration[i] = (calibration[i] / 5 ) * noiseFactor;
            noiselevel = calibration[i] > 0 ? (calibration[i] / 5 ) * noiseFactor : noiseMin;
            
            tmp = fht_log_out[i] < noiselevel ? 0 : fht_log_out[i] ; 
            if ( tmp > maxPeak){
              maxPeak=tmp;
              newMaxPeak = true;
            }
            if ( tmp < minPeak ) minPeak=tmp;
            if ( k > range[idx] ){
               idx++;
               noiseFactor=noiseMatrix[idx];
            }
            data[idx]+=tmp;
            k++;
            //Serial.write(tmp); //Pour une vision des 64 bandes "brutes" sur la liaison RS
       }
       if (! newMaxPeak){
          maxPeak *= 0.9;
          if (maxPeak <= minPeak  + 10) maxPeak = minPeak;
       }
       //Affichage des datas réparties sur 64 bandes
       //uint16_t Hue = 0;
       for ( byte i = 0; i < nbBand; i++ ){
         maxPeak = maxPeak == minPeak ? 10 : maxPeak ;
         //data[i] =  data[i] < minPeak ? 0 : map(data[i],minPeak,maxPeak,0,255) ;
         data[i] =  data[i] < noiseMin * noiseMatrix[i] ? 0 : map(data[i],noiseMin,maxPeak,0,255) ;
         if ( lastData[i] > data[i] ) data[i]=lastData[i]*0.9; //decay
         /*
         //Tracé du grap via processing si necessaire
         for ( byte j = 0; j < 10; j++ ){      
           Serial.write(data[i]);
         }
         */
         uint8_t avgLight = leds[i].getAverageLight() * 2;
         if ( data[i] > avgLight  ) {
           for ( byte x=i; x < NUM_LEDS; x = x + nbBand )
           {
              leds[x] = CHSV(Hue + x, 255, data[i]); // Note how we really cranked up the tmp value to get BRIGHT LED's. Also increment the hue for fun.
              leds[x].nscale8(224); // Let's fade the whole thing over time as well.

              //leds[i+1] = CHSV(Hue + HueDelta, 255, (data[i] + data[i+1] ) / 2); // Note how we really cranked up the tmp value to get BRIGHT LED's. Also increment the hue for fun.
              //leds[i+1].nscale8(224); // Let's fade the whole thing over time as well.
           }
         }
         Hue += HueDelta; // compute new hue
         //Hue += HueDelta; // compute new hue
         //Hue = Hue % HueMax; //Pas besoin si HueMax = 255
         lastData[i]=data[i];
         data[i]=0;         
       }

    }
    position = 0;
    FastLED.show();
  }
}

// free running ADC fills capture buffer
ISR(ADC_vect)
{
  if (position >= FHT_N)
      return;
  fht_input[position] = abs(ADC + zero);
  position++;
}

void adcInit(){
  /*  REFS0 : VCC use as a ref, IR_AUDIO : channel selection, ADEN : ADC Enable, ADSC : ADC Start, ADATE : ADC Auto Trigger Enable, ADIE : ADC Interrupt Enable,  ADPS : ADC Prescaler  */
  // free running ADC mode, f = ( 16MHz / prescaler ) / 13 cycles per conversion 
  ADMUX = _BV(REFS0) | IR_AUDIO; // | _BV(ADLAR); 
  //ADMUX = _BV(REFS0) | IR_AUDIO | _BV(ADLAR); //ADLAR left shift for 8 bit instead of 16 reading
  //ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); //prescaler 32 : 19231 Hz - 300Hz per 64 divisions
  //ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) ; //prescaler 64 : 19231 Hz - 300Hz per 64 divisions
  ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // prescaler 128 : 9615 Hz - 150 Hz per 64 divisions, better for most music
  sei();
}

void adcCalb(){
  //Serial.println("Start to calc zero");
  long midl = 0;
  // get 5 meashurment at 0,5 sec
  // on ADC input must be NO SIGNAL!!!
  for (byte i = 0; i < 5; i++)
  {
    position = 0;
    delay(50);
    midl += fht_input[0];
    delay(100);
  }
  zero = -midl/5;
  //Serial.println(zero);
  //Serial.println("Done.");
}



      //C'est la que ca rame grave :(
     

