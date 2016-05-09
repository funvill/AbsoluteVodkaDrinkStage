/** 
 *  Created by: Steven Smethurst
 *  Last updated: May 5, 2016 
 *  
 *  Requirments 
 *  - The LED rings have two states, a "close" and a "far" state. The close state is triggered when a 
 *    cup is placed ontop of the bar and is msured with a distance sensor. Each state has its own 
 *    animation effect. 
 *    
 *  - If all the LED Rings have the state of close. A speical animation will occure. In this case 
 *    A HSV rainbow. 
 * 
 *  - ToDo: Sound EQ with head phone input    
 *  
 *  Hardware 
 *  - 1x Teency 3.2 https://www.pjrc.com/store/teensy32.html
 *  - 4x NeoPixel Ring 24 LEDs https://www.adafruit.com/products/1586, 
 *    All LED rings are daisy chain together, in a single series of LEDS
 *  - 4x Pololu Carrier with Sharp GP2Y0D805Z0F Digital Distance Sensor 5cm https://www.pololu.com/product/1132/specs
 *  - 5 meters NeoPixel WS2812b LED strip 60 LEDS per meter. 
 *  - 1x 10 Amp 5v power supply 
 *  - Sound sensor ? 
 *  
 *  Libaries 
 *  - FastLED - http://fastled.io/ a very good LED libary that uses low memory. 
 */

#include "FastLED.h"
FASTLED_USING_NAMESPACE

const int PIN_DISTANCE_SENSOR_ONE   = 12 ; 
const int PIN_DISTANCE_SENSOR_TWO   = 11 ; 
const int PIN_DISTANCE_SENSOR_THREE = 10 ; 
const int PIN_DISTANCE_SENSOR_FOUR  =  9 ; 
const int PIN_SOUND_SENSOR          =  0 ; 
const int PIN_RING_LEDS             =  2 ;
const int PIN_STRIP_LEDS             = 6 ;


// Ring LEDs
#define COUNT_LED_RINGS        4
#define NUM_LEDS_RING         24
CRGB ringLeds [( NUM_LEDS_RING        * COUNT_LED_RINGS )];

// Strip LEDs
#define COUNT_LED_METER        5
#define NUM_LEDS_STRIP_METER  60
CRGB stripLeds[( NUM_LEDS_STRIP_METER * COUNT_LED_METER )];

// Global Settings 
#define SETTING_GLOBAL_BRIGHTNESS          100

// Sample window 
// The sample window consumes time from the rest of the program 
// It can not be run as a interval base, because of this the other interval bases can not 
// be less then the sample window in mS. 
// The sameple window width is in mS (50 mS = 20Hz) 
#define SETTING_EQ_SAMPLE_WINDOW    50

// Falling rate
// How fas the peek of the EQ falls. A high value of 100 or more will be so fast that the 
// user will not notice any delay in the falling. A value of 5 is nice. The lower this value is 
// The higher the SETTING_EQ_PEEK_FALLOFF_LENGTH should be 
#define SETTING_EQ_FALLING_RATE     100 

#define SETTING_EQ_PEEK_FALLOFF_LENGTH           1 
#define SETTING_EQ_ROLLING_AVERAGE_SAMPLES       40 


// https://stackoverflow.com/questions/10990618/calculate-rolling-moving-average-in-c/10990656#10990656
float approxRollingAverage (float avg, float new_sample) {
    avg -= avg / SETTING_EQ_ROLLING_AVERAGE_SAMPLES;
    avg += new_sample / SETTING_EQ_ROLLING_AVERAGE_SAMPLES;
    return avg;
}


class CEQ {
  public:
    CRGB * m_data ; 
    unsigned short m_pixelCount ;  
    unsigned short m_sensorPin;

    
    int lastMaxSoundLevel ; 
    
    float rollingAverageSoundLevel;
    float aveMin;
    float aveMax;

    void Init( CRGB * data, unsigned short pixelCount, unsigned short sensorPin ) {
      this->m_data = data ; 
      this->m_pixelCount = pixelCount ; 
      this->m_sensorPin = sensorPin ; 
      Serial.println("CEQ.Init() sensorPin="+ String(this->m_sensorPin) +", pixelCount=" + String(this->m_pixelCount) );

      lastMaxSoundLevel = 0 ; 
      rollingAverageSoundLevel = 0 ;
    }
    

    unsigned short SoundLevel() {

      unsigned int signalMax = 0 ; 
      unsigned int signalMin = 1024 ; 
      unsigned long startMillis = millis();  // Start of sample window

      while (millis() - startMillis < SETTING_EQ_SAMPLE_WINDOW ) {
        unsigned int sample = analogRead(this->m_sensorPin);
        if (sample < 1024) { // toss out spurious readings
          if (sample > signalMax){
            signalMax = sample;  // save just the max levels
          } else if (sample < signalMin) {
            signalMin = sample;  // save just the min levels
          }
        }
        // Serial.print(".");
      } 

      // We have all the samples we need. update the lastREading 
      int soundLevelRaw = signalMax - signalMin ;  // max - min = peak-peak amplitude
      unsigned short soundLevelLast = map(soundLevelRaw, 0, 1023, 0, this->m_pixelCount);



      /*
      // This shit don't work. 
      // 
      // This will keep a running average of the entire show min and max. Then it will attempt 
      // to make the EQ levels spread out across the range of the LEDS. This mades it so that the 
      // sound levels react to the type of music over time. 
      // This is based on a https://en.wikipedia.org/wiki/Moving_average 
      // 
      
      this->aveMin = approxRollingAverage( this->aveMin, signalMin ) ;
      this->aveMax = approxRollingAverage( this->aveMax, signalMax ) ;
      float avePeekToPeek = this->aveMax - this->aveMin ;  // max - min = peak-peak amplitude

      int testing = map( soundLevelRaw, avePeekToPeek/2, avePeekToPeek*2, 0, this->m_pixelCount );

      Serial.print(String( millis()) + " signalMax="+ String(signalMax) + " signalMin="+ String(signalMin) );
      Serial.print(" aveMin="+ String(aveMin) + " aveMax="+ String(aveMax) );
      Serial.print(" avePeekToPeek="+ String(avePeekToPeek)    );      
      Serial.print(" soundLevelRaw="+ String(soundLevelRaw) + " testing="+ String(testing)  );      
      Serial.println(); 
      
      // this->rollingAverageSoundLevel = approxRollingAverage( this->rollingAverageSoundLevel, soundLevelRaw ) ;
      // int testing = map( soundLevelRaw, 0,1024, 0, this->m_pixelCount );
      if( testing <= 0 ) {
        testing = 0 ; 
      }
      return testing; 
      */
      
      Serial.print(String( millis()) + " signalMax="+ String(signalMax) + " signalMin="+ String(signalMin) );
      Serial.print(" Raw=" + String(soundLevelRaw) + " Last="+ String(soundLevelLast) );
      Serial.println(); 
      return soundLevelLast; 
      
    }

    void Loop() {
      // Read the sound level from the mic 
      unsigned short soundLevel = this->SoundLevel();

      // Put a PEEK LED that falls as time progresses. 
      if( lastMaxSoundLevel < soundLevel ) {
        lastMaxSoundLevel = soundLevel ; 
      } else {
        lastMaxSoundLevel -= SETTING_EQ_FALLING_RATE; 
        if( lastMaxSoundLevel <= 0 ) {
          lastMaxSoundLevel = 0 ; 
        }
      }      

      // Depending on the sound level make the EQ bounce. 
      for( unsigned short offset = 0 ; offset < this->m_pixelCount ; offset++ ) {
        if(offset < soundLevel ) {          
          // Hue of 160 is a nice absolut vodka blue 
          unsigned char hue = map( offset, this->m_pixelCount, 0, 0, 255 ); 
          this->m_data[offset].setHSV(hue, 255 , 255); 
        } else {
          this->m_data[offset].setHSV(0, 0, 0); 
        }
      }

      
      // We want something to show off the peek of the sound level 
      // Lets make it white 
      for( unsigned short offset = 0 ; offset < SETTING_EQ_PEEK_FALLOFF_LENGTH ; offset++ ) {
        unsigned short offsetPixel = soundLevel-offset ; 
        if( offsetPixel <= 0 ) {
          break ; 
        }
        this->m_data[offsetPixel].setHSV(255, 0, (255/SETTING_EQ_PEEK_FALLOFF_LENGTH) * offset ); 
      }
      
      return ; 
    }
};
















class CLEDRing
{
  private: 
    CRGB * m_data ; 
    unsigned short m_pixelCount ;
    unsigned short m_sensorPin;

    // Pulse bright and dark in blue. 
    unsigned short direction ; 
    unsigned short amount ;
    unsigned short ticker ; 
    unsigned long lastProgres ; 
    unsigned long lastTickerProgres ;
    
    bool lastState ;  

  public:
    void Init( CRGB * data, unsigned short pixelCount, unsigned short sensorPin  ) {
      this->m_data = data ; 
      this->m_pixelCount = pixelCount ; 
      this->m_sensorPin = sensorPin ; 
      pinMode(this->m_sensorPin, INPUT); 

      Serial.println("CLEDRing.Init() sensorPin="+ String(this->m_sensorPin) +", pixelCount=" + String(this->m_pixelCount) );

      // Pulse bright and dark in blue. 
      direction = 3 ;  // How fast 
      amount = 30 ; 
      ticker = 0 ; 
      lastProgres = random( 0, 1000*5) ; 
      lastTickerProgres = 0; 

      lastState = false ; 
    }

    bool GetState() {
      return ! lastState ; 
    }

    void Loop() {
      // Read the sensor 
      bool state = digitalRead(this->m_sensorPin);
      // Display the result 
      if( state == HIGH) {
        this->Far(); 
      } else {
        this->Closes();         
      }

      // See if the state just changed 
      if( lastState != state ) {
        lastState = state ; 
        Serial.println("RingLED["+ String( this->m_sensorPin ) +"] State Changed to [" + String(state) + "]" );
      }
      


      // Time proccess 
      if( lastProgres + 20 < millis() ) {
        lastProgres = millis() ;         
        amount += direction ;    
        if( amount > 255 ) {
          direction = (-1)*direction ; 
          amount = 254 ; 
        } else if( amount < 30 ) {
          direction = (-1)*direction ;           
        }
      }

      unsigned short rotationSpeed = 74 ; // 20 
      if( lastTickerProgres + rotationSpeed < millis() ) {
        lastTickerProgres = millis() ;
        ticker++; 
      } 
    }

    void Closes() {
      unsigned short fadeAmount = 255 / 4 ; 
      for( unsigned short offset = 0 ; offset < this->m_pixelCount ; offset++ ) {
        if( (ticker + offset) % 8 == 0 ) {
          this->m_data[offset].setRGB(0,0,255);
          // this->m_data[offset].setRGB(fadeAmount*5,fadeAmount*5,fadeAmount*5);  
        } else if( (ticker + offset) % 8 == 1 ) {
          this->m_data[offset].setRGB(fadeAmount*4,fadeAmount*4,fadeAmount*4);  
        } else if( (ticker + offset) % 8 == 2 ) {
          this->m_data[offset].setRGB(fadeAmount*3,fadeAmount*3,fadeAmount*3+60);  
        } else if( (ticker + offset) % 8 == 3 ) {
          this->m_data[offset].setRGB(fadeAmount*2,fadeAmount*2,fadeAmount*2+90);  
        } else if( (ticker + offset) % 8 == 4 ) {
          this->m_data[offset].setRGB(fadeAmount*1,fadeAmount*1,fadeAmount*1+120);  
        } else {
          this->m_data[offset].setRGB(0,0,amount);
        }
      }
    }
    
    void Far() {
      for( unsigned short offset = 0 ; offset < this->m_pixelCount ; offset++ ) {
        this->m_data[offset].setRGB(0,0,amount);
      }
    }
    
} ;








CEQ eq ; 
CLEDRing rings[COUNT_LED_RINGS]; 


void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  delay(1000); 
  
  FastLED.addLeds<NEOPIXEL, PIN_RING_LEDS> (ringLeds,  (NUM_LEDS_RING * COUNT_LED_RINGS) );
  FastLED.addLeds<NEOPIXEL, PIN_STRIP_LEDS>(stripLeds, (NUM_LEDS_STRIP_METER * COUNT_LED_METER));

  // Set master brightness control
  FastLED.setBrightness(SETTING_GLOBAL_BRIGHTNESS);

  // Set up the Ring LEDs and distance sensors 
  rings[0].Init( ringLeds + (0 * NUM_LEDS_RING), NUM_LEDS_RING, PIN_DISTANCE_SENSOR_ONE   );  
  rings[1].Init( ringLeds + (1 * NUM_LEDS_RING), NUM_LEDS_RING, PIN_DISTANCE_SENSOR_TWO   );  
  rings[2].Init( ringLeds + (2 * NUM_LEDS_RING), NUM_LEDS_RING, PIN_DISTANCE_SENSOR_THREE );  
  rings[3].Init( ringLeds + (3 * NUM_LEDS_RING), NUM_LEDS_RING, PIN_DISTANCE_SENSOR_FOUR  );  


  // eq
  // eq.Init( stripLeds, (COUNT_LED_METER * NUM_LEDS_STRIP_METER), PIN_SOUND_SENSOR );  
  eq.Init( stripLeds, 100, PIN_SOUND_SENSOR );  

  pinMode(13, OUTPUT); 
}



void loop() {

  // Check the state of the LED rings and do animations based on the state. 
  unsigned short speicalAnimation = 0 ;
  for( int offset = 0 ; offset < COUNT_LED_RINGS ; offset++ ) {
    rings[offset].Loop(); 
    if( rings[offset].GetState() ) {
      speicalAnimation++ ;
    }
  }

  // Check for special state when there are four cups down 
  if( speicalAnimation >= 3 ) {
    // Moving rainbow mode! 
    static unsigned short movingRainbow = 0 ; 
    for( int offset = 0 ; offset < NUM_LEDS_RING * COUNT_LED_RINGS ; offset++ ) {
     ringLeds[offset].setHue( offset % 255 + movingRainbow);
    }
    movingRainbow++; 
  }

  // EQ 
  eq.Loop(); 

  
  FastLED.show(); 

  static bool togleLED = HIGH ; 
  unsigned long lastBlink = millis() ; 
  if( lastBlink + 500 < millis() ) {    
    lastBlink = millis() ; 
    togleLED = !togleLED ; 
    digitalWrite(13, togleLED);
  }


  
}
