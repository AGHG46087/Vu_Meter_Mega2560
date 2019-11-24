#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include "water_torture.h"
#include <math.h>
#include <SoftwareSerial.h>
#include <fix_fft.h>

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif


#define FORCE_TEST_PATTERNS false             // Force test a specific set of patterns
#define PATTERN_NUMBER 18                    // 1 all VU, 2 all StandyBy, the rest 3-29 are individual patterns
/* HANS Original Patterns
 *  12: goodOleRedGreenBLue();
 *  14: multiStdRainbow();
 *  18: multiAlternatingFFT();
 *  
 *  
 */

#define OLED_RESET 4 // MAYBE 13
#define LOGO16_GLCD_HEIGHT 32
#define LOGO16_GLCD_WIDTH  128
Adafruit_SSD1306 ssdDisplay(OLED_RESET);
//Adafruit_SSD1306 ssdDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define POT_BRIGHT_PIN A4
#define PATTERN_SWITCH_SECONDS 30            // Time to wait in seconds before pattern switch in cycles modes 1 & 2
#define N_PIXELS  60                         // Number of pixels in strand
#define N_PIXELS_HALF (N_PIXELS/2)           // Half off the Pixels - center
#define MIC_PIN   A5                         // Microphone is attached to this analog pin
#define LED_PIN    6                         // NeoPixel LED strand is connected to this pin
#define SAMPLE_WINDOW   10                   // Sample window for average level
#define PEAK_HANG 24                         // Time of pause before peak dot falls
#define PEAK_FALL 20                         // Rate of falling peak dot
#define PEAK_FALL2 8                         // Rate of falling peak dot
#define INPUT_FLOOR 10                       // Lower range of analogRead input
#define INPUT_CEILING 300                    // Max range of analogRead input, the lower the value the more sensitive (1023 = max)300 (150)
#define DC_OFFSET  0                         // DC offset in mic signal - if unusure, leave 0
#define NOISE     10                         // Noise/hum/interference in mic signal
#define SAMPLES   60                         // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS + 2)             // Allow dot to go slightly off scale
#define SPEED .20                            // Amount to increment RGB color by each cycle
#define TOP2      (N_PIXELS + 1)             // Allow dot to go slightly off scale
#define LAST_PIXEL_OFFSET N_PIXELS-1
#define PEAK_FALL_MILLIS 10                  // Rate of peak falling dot
#define CENTER_IN_PEAK_MIN 75                // If the peak to Peak is less than this value Just return do nothing.
#define POT_PIN    5
#define BG 0
#define LAST_PIXEL_OFFSET N_PIXELS-1
#define SPARKING 50
#define COOLING  55
#define FRAMES_PER_SECOND 60
#define NUM_BALLS         4                  // Number of bouncing balls you want (recommend < 7, but 20 is fun in its own way)
#define GRAVITY           -9.81              // Downward (negative) acceleration of gravity in m/s^2
#define h0                1                  // Starting height, in meters, of the ball (strip length)


#define BRIGHTNESS  255
#define LED_TYPE    WS2812 //WS2812B         // Only use the LED_PIN for WS2812's
#define COLOR_ORDER GRB
#define COLOR_MIN           0
#define COLOR_MAX         255
#define DRAW_MAX          100
#define SEGMENTS            4                // Number of segments to carve amplitude bar into
#define COLOR_WAIT_CYCLES  10                // Loop cycles to wait between advancing pixel origin
#define qsubd(x, b)  ((x>b)?b:0)
#define qsuba(x, b)  ((x>b)?x-b:0)           // Analog Unsigned subtraction macro. if result <0, then => 0. By Andrew Tuline.
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define REVERSE true

uint16_t brightness = BRIGHTNESS;
String patternName = "";


  //config for balls

float h[NUM_BALLS] ;                         // An array of heights
float vImpact0 = sqrt( -2 * GRAVITY * h0 );  // Impact velocity of the ball when it hits the ground if "dropped" from the top of the strip
float vImpact[NUM_BALLS] ;                   // As time goes on the impact velocity will change, so make an array to store those values
float tCycle[NUM_BALLS] ;                    // The time since the last time the ball struck the ground
int   pos[NUM_BALLS] ;                       // The integer position of the dot on the strip (LED index)
long  tLast[NUM_BALLS] ;                     // The clock time of the last ground strike
float COR[NUM_BALLS] ;                       // Coefficient of Restitution (bounce damping)

struct CRGB leds[N_PIXELS];


Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

static uint16_t dist;                        // A random number for noise generator.
uint16_t scale = 30;                         // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
uint8_t maxChanges = 48;                     // Value for blending between palettes.

  //CRGBPalette16 currentPalette(CRGB::Black);
CRGBPalette16 currentPalette(OceanColors_p);
CRGBPalette16 targetPalette(CloudColors_p);
  // Water torture
WaterTorture water_torture = WaterTorture(&strip);


// goodOleRedGreenBlue variables
int fadeCol = 0, total = 0, counter = 0, volLast = 0,fadeAmt = 0;

// multiAlternatingFFT
bool invertFFT = false;
int freqCounter = 0;
int rotateFreqs = 13;
char im[128], data[128];
char data_avgs[14];

  //new ripple vu
uint8_t timeval = 20;                        // Currently 'delay' value. No, I don't use delays, I use EVERY_N_MILLIS_I instead.
uint16_t loops = 0;                          // Our loops per second counter.
bool     samplepeak = 0;                     // This sample is well above the average, and is a 'peak'.
uint16_t oldsample = 0;                      // Previous sample is used for peak detection and for 'on the fly' values.
bool thisdir = 0;
  //new ripple vu

  // Modes
enum {
} MODE;



byte volCount  = 0;                        // Frame counter for storing past volume data

int reading,
    vol[SAMPLES],                          // Collection of prior volume samples
    lvl       = 10,                        // Current "dampened" audio level
    minLvlAvg = 0,                         // For dynamic adjustment of graph low & high
    maxLvlAvg = 512;
float greenOffset = 30, blueOffset = 150;

int CYCLE_MIN_MILLIS = 2;
int CYCLE_MAX_MILLIS = 1000;
int cycleMillis = 20;
bool paused = false;
long lastTime = 0;
bool boring = true;
int  myhue =   0;
bool reverseDirection = false;

//vu ripple
uint8_t colour;
uint8_t myfade = 255;                      // Starting brightness.
#define maxsteps 16                        // Case statement wouldn't allow a variable.
int peakspersec = 0;
int peakcount = 0;
uint8_t bgcol = 0;
int thisdelay = 20;

// FOR SYLON ETC
uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   2;                    // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255;                     // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255;

//FOR JUGGLE
uint8_t numdots = 4;                       // Number of dots in use.
uint8_t faderate = 2;                      // How long should the trails be. Very low value = longer trails.
uint8_t hueinc = 16;                       // Incremental change in hue between each dot.
uint8_t thishue = 0;                       // Starting hue.
uint8_t curhue = 0;
uint8_t thisbright = 255;                  // How bright should the LED/display be.
uint8_t basebeat = 5;

// Twinkle
float redStates[N_PIXELS];
float blueStates[N_PIXELS];
float greenStates[N_PIXELS];
float Fade = 0.96;

//Samples
#define NSAMPLES 64
unsigned int samplearray[NSAMPLES];
unsigned long samplesum = 0;
unsigned int sampleavg = 0;
int samplecount = 0;
unsigned int sample; // This could be local to function
unsigned long oldtime = 0;
unsigned long newtime = 0;

//Ripple variables
int color;
int center = 0;
int step = -1;
int maxSteps = 16;
float fadeRate = 0.80;
int diff;

//vu 8 variables
int origin = 0,
    color_wait_count = 0,
    scroll_color = COLOR_MIN,
    last_intensity = 0,
    intensity_max = 0,
    origin_at_flip = 0;
uint32_t draw[DRAW_MAX];
boolean growing = false, fall_from_left = true;



//background color
uint32_t currentBg = random(256);
uint32_t nextBg = currentBg;
//CRGBPalette16 currentPalette;
//CRGBPalette16 targetPalette;
TBlendType    currentBlending;

const int buttonPin = 0;                  // the number of the pushbutton pin

  //Variables will change:
int buttonPushCounter = 0;                // counter for the number of button presses
int buttonState = 0;                      // current state of the button
int lastButtonState = 0;


byte peak = 16;                           // Peak level of column; used for falling dots
byte dotCount = 0;                        // Frame counter for peak dot
byte dotHangCount = 0;                    // Frame counter for holding peak dot

void ssdTextLine(String text, int xPos, int yPos, int textSize) {
  ssdDisplay.setTextSize(textSize);
  ssdDisplay.setTextColor(WHITE);
  ssdDisplay.setCursor(xPos, yPos);
  ssdDisplay.println(text);
}
void displayText() {
  String text = "";
  int txtSize = 1;
  int xPos = 4;
  int yPos = 2;
  int rowHeight = 9;
  
  ssdDisplay.clearDisplay();
  // Display will support 29 chars Max starting from position 3, lets call it 28

  ssdTextLine(patternName, xPos, yPos, txtSize);
  yPos += rowHeight;  
  
  text = "Button Counter: " + String(buttonPushCounter);
  ssdTextLine(text, xPos, yPos, txtSize);  yPos += rowHeight;
  
  text = "Brightness:     " + String(brightness);
  ssdTextLine(text, xPos, yPos, txtSize);
  
  
  ssdDisplay.drawRect(1, 1, 126,31, WHITE);
  
  ssdDisplay.display();
}
uint16_t getBrightness() {
  int readValue = analogRead(POT_BRIGHT_PIN);
  uint16_t localBright = (255.0 / 1023.0 ) * readValue;  // Normalize via brute force approach
  uint16_t mapBright = map(readValue, 30, 1023, 0, BRIGHTNESS); // Normalize via map function

  // Both normalized values still have values that become mapped to a higher base, I want them to zero
  mapBright = (mapBright < 35) ? 0 : mapBright; 
  localBright = (localBright < 25) ? 0 : localBright;
  
  String potText = "POT_BRIGHTNESS: " + String(localBright);
  String mapText = "MAP_BRIGHTNESS: " + String(mapBright);
  
  // Looks like localBright ( brute force ) has better range value - use this one.
  brightness = localBright;


  return brightness;
}

void setup() {

  analogReference(EXTERNAL);            // set aref to external

  pinMode(POT_BRIGHT_PIN, INPUT);
  
  pinMode(buttonPin, INPUT);
    //initialize the buttonPin as output
  digitalWrite(buttonPin, HIGH);

  strip.begin();
  strip.show(); // all pixels to 'off'
  

  Serial.begin(57600);
  
  delay(3000);   // SPOOL UP TIME 

  ssdDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  ssdDisplay.display();

  ssdDisplay.clearDisplay();

  LEDS.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds,N_PIXELS).setCorrection(TypicalLEDStrip);
  LEDS.setBrightness(getBrightness());
  FastLED.setBrightness(brightness);
  strip.setBrightness(brightness);
  dist = random16(12345);               // A semi-random number for our noise generator. GEEK: Check this use.

  
  for (int i = 0 ; i < NUM_BALLS ; i++) {// Initialize variables
    tLast[i] = millis();
    h[i] = h0;
    pos[i] = 0;                         // Balls start on the ground
    vImpact[i] = vImpact0;              // And "pop" up at vImpact0
    tCycle[i] = 0;
    COR[i] = 0.90 - float(i)/pow(NUM_BALLS,2);
  }

}


void loop() {
  
  LEDS.setBrightness(getBrightness()); FastLED.setBrightness(brightness); strip.setBrightness(brightness);
    // read the pushbutton input pin:
  int updateBtnChange = false;
  buttonState = digitalRead(buttonPin);
    // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
      // if the state has changed, increment the counter
    if (buttonState == HIGH) {
        // if the current state is HIGH then the button
        // wend from off to on:

        // Increment the button push counter for the switch statment and 
        // change the state of the flag for the button push, Used to update display at end of switch
      buttonPushCounter++;
      updateBtnChange = true;
      
      if(buttonPushCounter==30) {
        buttonPushCounter=1;}
    }
    else {
        // if the current state is LOW then the button
        // wend from on to off:
    }
  }
    // save the current state as the last state,
    // for next time through the loop
  lastButtonState = buttonState;
  

#if FORCE_TEST_PATTERNS
  buttonPushCounter = PATTERN_NUMBER;
#endif

  switch (buttonPushCounter){

    case 1: {
      buttonPushCounter==1;
      fftVuPatterns();
      break;}

    case 2: {
      buttonPushCounter==2;
      standByPatterns();
      break;}

    case 3: {
      buttonPushCounter==3;
      stdRainbow(); // NORMAL - Double CHECK 
      break;}

    case 4: {
      buttonPushCounter==4;
      stdCenterOut(); // Centre out Double CHECK
      break;}

    case 5: {
      buttonPushCounter==5;
      stdCenterIn(); // Closest Match classicNormal();
      break;}

    case 6: {
      buttonPushCounter==6;
      stdColorChanging(); // normalRainbow Double CHECK
      break;}

    case 7: {
      buttonPushCounter==7;
      colorChangingCenter(); // Centre rainbow - CHECK
      break;}

    case 8: {
      buttonPushCounter==8;
      shootingStar(); // Shooting Star - CHECK
      break;}

    case 9: {
      buttonPushCounter==9;
      fallingStar(); // Falling star - CHECK
      break;}

    case 10: {
      buttonPushCounter==10;
      rippleWithBackground(); // Ripple with background - CHECK
      break;}

    case 11: {
      buttonPushCounter==11;
      shatter(); // Shatter //  - CHECK
      break;}
    case 12: {
      buttonPushCounter==12;
      goodOleRedGreenBLue(); //  - CHECK
      break;}
    case 13: {
      buttonPushCounter==13;
      pulseWithGlitter(); // Pulse - CHECK
      break;}
    case 14: {
      buttonPushCounter==14;
      multiStdRainbow(); // - CHECK
      break;}
    case 15: {
      buttonPushCounter==15;
      rippleNoBackground(); // Ripple without Background - CHECK
      break;}

    case 16: {
      buttonPushCounter==16;
      juggleWithGlitter(); // Juggle With Glitter - CHECK
      break;}

    case 17: {
      buttonPushCounter==17;
      streamWithGlitter(); // Stream With Glitter- GEEK - Copy this
      break;}
    case 18: {
      buttonPushCounter==18;
      multiAlternatingFFT(); // multi FFT blocks alternates between 6 and 10 frequency ranges - CHECK
      break;}
        //-------------------------------------------------------//
    case 19: {
      buttonPushCounter==19;
      rainbow(20);
      break;}

    case 20: {
      buttonPushCounter==21; // Done
      ripple();
      break;}

    case 21: {
      buttonPushCounter==21; // Done
      ripple2();
      break;}

    case 22: {
      buttonPushCounter==22;
      Twinkle();
      break;}

    case 23: {
      buttonPushCounter==23; // Done
      pattern2(); // sylon
      break;}

    case 24: {
      buttonPushCounter==24; // Done
      pattern3();
      break;}

    case 25: {
      buttonPushCounter==25;
      juggle2();
      break;}

    case 26: {
      buttonPushCounter==26; // Done
      blur();
      break;}

    case 27: {
      buttonPushCounter==27; // Done
      Balls(); //
      break;}

    case 28: {
      buttonPushCounter==28;
      Drip(); //
      break;}

    case 29: {
      buttonPushCounter==29; // Done
      fireblu();
      break;}

    case 30: {
      buttonPushCounter==30; // Done
      fire();
      break;}

    case 31: {
      buttonPushCounter==31; // Done
      colorWipe(strip.Color(0, 0, 0), 10); // Black
      break;}
  }

  // Button was identifed as being pressed, update the display now to capture pattern name and button counter
  if (updateBtnChange) {
      displayText();
  }


}
float count = 0;
void setPatternName(String text) {
  /*
  int ARRAY_LENGTH = 27;
  char* buffer = new char[ARRAY_LENGTH+1]; // 27 + 1 for null.
  memset( buffer, '\0', sizeof(char)*(ARRAY_LENGTH+1) );

  strncpy ( buffer, text, ARRAY_LENGTH );

  delete[] buffer;
  */

  patternName = text;
  patternName.trim();
  if ( text.length() > 27 ) {
    patternName = patternName.substring (0,27); 
  }
  
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    if (digitalRead(buttonPin) != lastButtonState) {  // <------------- add this
      return;                                         // <------------ and this
    }
    delay(wait);
  }
}

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


    // condition curve parameter
    // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function


    // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

    // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float


    // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;

}

void stdRainbow() {
  setPatternName("VU: Std Rainbow");

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(MIC_PIN);             // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET);        // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);  // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;            // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


    // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {
    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));

  }


    // Draw peak dot
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));

  strip.show(); // Update strip

    // Every few frames, make the peak pixel drop by 1:

  if(++dotCount >= PEAK_FALL) { // fall rate

    if(peak > 0) peak--;
    dotCount = 0;
  }


  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


void stdCenterOut() {
  setPatternName("VU: stdCenterOut");

  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(MIC_PIN);                        // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top

    // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS_HALF; i++) {
    if(i >= height) {
      strip.setPixelColor(N_PIXELS_HALF-i-1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF+i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,30,150));
      strip.setPixelColor(N_PIXELS_HALF-i-1,color);
      strip.setPixelColor(N_PIXELS_HALF+i,color);
    }

  }

    // Draw peak dot
  if(peak > 0 && peak <= N_PIXELS_HALF-1) {
    uint32_t color = Wheel(map(peak,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS_HALF-peak-1,color);
    strip.setPixelColor(N_PIXELS_HALF+peak,color);
  }

  strip.show(); // Update strip

// Every few frames, make the peak pixel drop by 1:

  if(++dotCount >= PEAK_FALL) { //fall rate

    if(peak > 0) peak--;
    dotCount = 0;
  }



  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}



void stdCenterIn()  {
  setPatternName("VU: stdCenterIn");

  unsigned long startMillis= millis();  // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;
  unsigned int tempSample;

  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    tempSample = sample = analogRead(MIC_PIN);
    sample /= 4;
    if (sample < 1024)
    {
      if (sample > signalMax)
      {
        signalMax = sample;
      }
      else if (sample < signalMin)
      {
        signalMin = sample;
      }
    }
  }
  peakToPeak = signalMax - signalMin;

  if ( peakToPeak > CENTER_IN_PEAK_MIN ) { // value 75


    for (int i=0;i<=N_PIXELS_HALF-1;i++){
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,30,150));
      strip.setPixelColor(N_PIXELS-i,color);
      strip.setPixelColor(0+i,color);
    }


    c = fscale(INPUT_FLOOR, INPUT_CEILING, N_PIXELS_HALF, 0, peakToPeak, 2);

    if(c < peak) {
      peak = c;            // Keep dot on top
      dotHangCount = 15;   // make the dot hang before falling PEAK_HANG is 24
    }
    if (c <= strip.numPixels()) { // Fill partial column with off pixels
      drawLine(N_PIXELS_HALF, N_PIXELS_HALF-c, strip.Color(0, 0, 0));
      drawLine(N_PIXELS_HALF, N_PIXELS_HALF+c, strip.Color(0, 0, 0));
    }




    y = N_PIXELS_HALF - peak;
    uint32_t color1 = Wheel(map(y,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(y-1,color1);

    y = N_PIXELS_HALF + peak;
    strip.setPixelColor(y,color1);

    strip.show();

      // Frame based peak dot animation
    if(dotHangCount > PEAK_HANG) { //Peak pause length
      if(++dotCount >= PEAK_FALL2) { //Fall rate
        peak++;
        dotCount = 0;
      }
    }
    else {
      dotHangCount++;
    }
  } // end if ( peakToPeak > CENTER_IN_PEAK_MIN )

}


void stdColorChanging() {
  setPatternName("VU: stdColorChanging");
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(MIC_PIN);             // Raw reading from mic
  n = abs(n - 512 - DC_OFFSET);        // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE);  // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;          // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;      // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top

  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;

    // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++) {
    if (i >= height) {
      strip.setPixelColor(i, 0, 0, 0);
    } else {
      strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, (int)greenOffset, (int)blueOffset)));
    }
  }
    // Draw peak dot
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));

  strip.show(); // Update strip

    // Every few frames, make the peak pixel drop by 1:

  if(++dotCount >= PEAK_FALL) { //fall rate

    if(peak > 0) peak--;
    dotCount = 0;
  }
  strip.show();  // Update strip

  vol[volCount] = n;
  if (++volCount >= SAMPLES) { 
    volCount = 0;
  }

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) {
      minLvl = vol[i];
    } else if (vol[i] > maxLvl) {
      maxLvl = vol[i];
    }
  }

    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) {
    maxLvl = minLvl + TOP;
  }
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}


void colorChangingCenter() {
  setPatternName("VU: ColorChangingCenter");
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(MIC_PIN);             // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET);        // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);  // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;            // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;

    // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS_HALF; i++) {
    if(i >= height) {
      strip.setPixelColor(N_PIXELS_HALF-i-1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF+i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,(int)greenOffset, (int)blueOffset));
      strip.setPixelColor(N_PIXELS_HALF-i-1,color);
      strip.setPixelColor(N_PIXELS_HALF+i,color);
    }

  }

    // Draw peak dot
  if(peak > 0 && peak <= N_PIXELS_HALF-1) {
    uint32_t color = Wheel(map(peak,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS_HALF-peak-1,color);
    strip.setPixelColor(N_PIXELS_HALF+peak,color);
  }

  strip.show(); // Update strip

    // Every few frames, make the peak pixel drop by 1:

  if(++dotCount >= PEAK_FALL) { // fall rate

    if(peak > 0) peak--;
    dotCount = 0;
  }


  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

void shootingStar()  {
  setPatternName("VU: shootingStar");
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(MIC_PIN);            // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET);       // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;           // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP2 * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP2) height = TOP2;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


#ifdef CENTERED
    // Color pixels based on rainbow gradient
  for(i=0; i<(N_PIXELS/2); i++) {
    if(((N_PIXELS/2)+i) >= height)
    {
      strip.setPixelColor(((N_PIXELS/2) + i),   0,   0, 0);
      strip.setPixelColor(((N_PIXELS/2) - i),   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(((N_PIXELS/2) + i),Wheel(map(((N_PIXELS/2) + i),0,strip.numPixels()-1,30,150)));
      strip.setPixelColor(((N_PIXELS/2) - i),Wheel(map(((N_PIXELS/2) - i),0,strip.numPixels()-1,30,150)));
    }
  }

    // Draw peak dot
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((N_PIXELS/2) + peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((N_PIXELS/2) - peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
    // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++)
  {
    if(i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
    }
  }

    // Draw peak dot
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak,255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }

#endif

    // Every few frames, make the peak pixel drop by 1:

  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    strip.show(); // Update strip

      //fall rate
    if(peak > 0) peak--;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++)
  {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP2) maxLvl = minLvl + TOP2;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void fallingStar() {
  setPatternName("VU: fallingStar");
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(MIC_PIN);            // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET);       // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE); // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;           // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP2 * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP2) height = TOP2;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


#ifdef CENTERED
    // Draw peak dot
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((N_PIXELS/2) + peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((N_PIXELS/2) - peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
    // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++)
  {
    if(i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    { // GEEK What should I do here.
    }
  }

    // Draw peak dot
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak,0,0,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }

#endif

    // Every few frames, make the peak pixel drop by 1:

  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    strip.show(); // Update strip

      //fall rate
    if(peak > 0) peak--;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++)
  {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP2) maxLvl = minLvl + TOP2;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void rippleWithBackground() {
  setPatternName("VU: rippleWithBackground");

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                   // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                             // Reset the counter every second.
  }

  soundripper(false);

  EVERY_N_MILLISECONDS(20) {
    ripple3();
  }

  show_at_max_brightness_for_power();

} // loop()


void ripple3() {
  for (int i = 0; i < N_PIXELS; i++) {
    leds[i] = CHSV(bgcol, 255, sampleavg*2);  // Set the background colour.
  }

  switch (step) {

    case -1:                                  // Initialize ripple variables.
      center = random(N_PIXELS);
      colour = (peakspersec*10) % 255;        // More peaks/s = higher the hue colour.
      step = 0;
      bgcol = bgcol+8;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);  // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                            // At the end of the ripples.
        // step = -1;
      break;

    default:                                  // Middle of the ripples.
      leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);
      step ++;                                // Next step.
      break;
  } // switch step
} // ripple()


void shatter() {
  setPatternName("VU: shatter");
  int intensity = calculateIntensity();
  updateOrigin(intensity);
  assignDrawValues(intensity);
  writeSegmented();
  updateGlobals();
}

int calculateIntensity() {
  int      intensity;

  reading   = analogRead(MIC_PIN);            // Raw reading from mic
  reading   = abs(reading - 512 - DC_OFFSET); // Center on zero
  reading   = (reading <= NOISE) ? 0 : (reading - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + reading) >> 3;           // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  intensity = DRAW_MAX * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  return constrain(intensity, 0, DRAW_MAX-1);
}

void updateOrigin(int intensity) {
    // detect peak change and save origin at curve vertex
  if (growing && intensity < last_intensity) {
    growing = false;
    intensity_max = last_intensity;
    fall_from_left = !fall_from_left;
    origin_at_flip = origin;
  } else if (intensity > last_intensity) {
    growing = true;
    origin_at_flip = origin;
  }
  last_intensity = intensity;

    // adjust origin if falling
  if (!growing) {
    if (fall_from_left) {
      origin = origin_at_flip + ((intensity_max - intensity) / 2);
    } else {
      origin = origin_at_flip - ((intensity_max - intensity) / 2);
    }
      // correct for origin out of bounds
    if (origin < 0) {
      origin = DRAW_MAX - abs(origin);
    } else if (origin > DRAW_MAX - 1) {
      origin = origin - DRAW_MAX - 1;
    }
  }
}

void assignDrawValues(int intensity) {
    // draw amplitue as 1/2 intensity both directions from origin
  int min_lit = origin - (intensity / 2);
  int max_lit = origin + (intensity / 2);
  if (min_lit < 0) {
    min_lit = min_lit + DRAW_MAX;
  }
  if (max_lit >= DRAW_MAX) {
    max_lit = max_lit - DRAW_MAX;
  }
  for (int i=0; i < DRAW_MAX; i++) {
      // if i is within origin +/- 1/2 intensity
    if (
        (min_lit < max_lit && min_lit < i && i < max_lit) // range is within bounds and i is within range
        || (min_lit > max_lit && (i > min_lit || i < max_lit)) // range wraps out of bounds and i is within that wrap
        ) {
      draw[i] = Wheel(scroll_color);
    } else {
      draw[i] = 0;
    }
  }
}

void writeSegmented() {
  int seg_len = N_PIXELS / SEGMENTS;

  for (int s = 0; s < SEGMENTS; s++) {
    for (int i = 0; i < seg_len; i++) {
      strip.setPixelColor(i + (s*seg_len), draw[map(i, 0, seg_len, 0, DRAW_MAX)]);
    }
  }
  strip.show();
}

void writeToStrip(uint32_t* draw) {
  for (int i = 0; i < N_PIXELS; i++) {
    strip.setPixelColor(i, draw[i]);
  }
  strip.show();
}

void updateGlobals() {
  uint16_t minLvl, maxLvl;

    //advance color wheel
  color_wait_count++;
  if (color_wait_count > COLOR_WAIT_CYCLES) {
    color_wait_count = 0;
    scroll_color++;
    if (scroll_color > COLOR_MAX) {
      scroll_color = COLOR_MIN;
    }
  }

  vol[volCount] = reading;                    // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0;     // Advance/rollover sample counter

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(uint8_t i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < N_PIXELS) maxLvl = minLvl + N_PIXELS;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}




void soundble() {                             // Quick and dirty sampling of the microphone.

  int tmp = analogRead(MIC_PIN) - 512 - DC_OFFSET;
  sample = abs(tmp);

} 



void sndwave() {

  leds[N_PIXELS/2] = ColorFromPalette(currentPalette, sample, sample*2, currentBlending); // Put the sample into the center

  for (int i = N_PIXELS - 1; i > N_PIXELS/2; i--) {       // move to the left:  Copy to the left, and let the fade do the rest.
    leds[i] = leds[i - 1];
  }

  for (int i = 0; i < N_PIXELS/2; i++) {                  // move to the right: Copy to the right, and let the fade to the rest.
    leds[i] = leds[i + 1];
  }
  addGlitter(sampleavg);
}
/***** GEEK multiStdRainbow Examples Here ***********************/
void multiStdRainbow() {
  setPatternName("VU: multiStdRainbow");
  const int SEG = N_PIXELS/10;
  const int HEIGHT = N_PIXELS/6;
  uint8_t  i,j;
  uint16_t minLvl, maxLvl;
  int      n, height, pos;

  n   = analogRead(MIC_PIN);             // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET);  // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);  // Remove noise/hum
  
  lvl = (((lvl * 7) + n) >> 3 % 5);            // "Dampened" reading (else looks twitchy)

    // Calculate bar height based on dynamic min/max levels (fixed point):
  height = HEIGHT * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > HEIGHT) height = HEIGHT;
  if(height > peak)     peak = height+ 3; // Keep 'peak' dot at top

  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;

    // Color pixels based on rainbow gradient
      for(i=0; i<HEIGHT; i++) {
        if(i >= height) {
          for( j = 0; j < SEG; j++) {
            pos = (j*HEIGHT) + i;
            strip.setPixelColor(pos,   0,   0, 0);
          }
        }
        else {
          for( j = 0; j < SEG; j++ ) {
            pos = (j*HEIGHT) + i;
            strip.setPixelColor(pos,Wheel(map(i,0,N_PIXELS_HALF-1,(int)greenOffset, (int)blueOffset)));
          }
        }
      }


    // Draw peak dot
  if(peak > 0 && peak <= HEIGHT-1) {
    for ( j = 0; j < SEG; j++ ){
      pos = (j * HEIGHT) + peak;
      strip.setPixelColor(pos,Wheel(map(i,0,N_PIXELS_HALF-1,(int)greenOffset, (int)blueOffset))); //Wheel(map(peak,0,strip.numPixels()-1,30,150))
    }
  }

  strip.show(); // Update strip

    // Every few frames, make the peak pixel drop by 1:

  if(++dotCount >= (PEAK_FALL)) { // fall rate

    if(peak > 0) peak--;
    dotCount = 0;
  }


  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

    // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
    // minLvl and maxLvl indicate the volume range over prior frames, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)


}
void multiAlternatingFFT() {
  setPatternName("VU: multiAlternatingFFT");
  
  int HEIGHT = N_PIXELS/10;
  int SEG = N_PIXELS/6;
  int ledIndex = 0;
  int lclVal = 0;
  int i = 0;
   
  if ( ++freqCounter > 1024 ) { rotateFreqs--;  }
  if( freqCounter > 512 ) {
    HEIGHT = N_PIXELS/6;
    SEG = N_PIXELS/10;
  }
  if (rotateFreqs <= 0 ) {
    rotateFreqs = 13;
    freqCounter = 0;
    HEIGHT = N_PIXELS/10;
    SEG = N_PIXELS/6;
  }
  // Every so often change the direction of the array, on a changing segment 
  EVERY_N_SECONDS( SEG ) { invertFFT = !invertFFT; }


  for (i=0; i < 128; i++){ 
    lclVal = 5000 * analogRead(MIC_PIN); //shin: analog sig not detected from laptop audio out. scale to 5000  
    lclVal = abs((int)lclVal % 128);
    data[i] = lclVal;
    im[i] = 0;
  };

  fix_fft(data,im,7,0);

  for (i=0; i< 64;i++){                                      
    data[i] = sqrt(data[i] * data[i] + im[i] * im[i]);  // this gets the absolute value of the values in the array, so we're only dealing with positive numbers
  };     
  
  
  // average bars together
  for (i=0; i<14; i++) {
    data_avgs[i] = data[i*4] + data[i*4 + 1] + data[i*4 + 2] + data[i*4 + 3];   // average together 
    data_avgs[i] = map(data_avgs[i], 0, 30, 0, HEIGHT); // 9                        // remap values for LoL
  }
  
  
  
  // set LoLShield
  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;
  
  for (int x=0; x < SEG; x++) {
    for (int y=0; y < HEIGHT; y++) {
      ledIndex = (x * HEIGHT) + y; 
      if ( invertFFT ) {
        ledIndex = ((x+1) * HEIGHT) - y;
      }
//      if (y < data_avgs[13-x]) { // 13-x reverses the bars so low to high frequences are represented from left to right.
//      if (y < data_avgs[x]) { // 13-x reverses the bars so low to high frequences are represented from left to right.
      if (y < data_avgs[rotateFreqs-x]) { // 13-x reverses the bars so low to high frequences are represented from left to right.
        strip.setPixelColor(ledIndex, Wheel(map(data_avgs[y],0,HEIGHT,(int)greenOffset, (int)blueOffset)));
        strip.setBrightness(brightness);
      } else {
        strip.setPixelColor(ledIndex, strip.Color(0,0,0));
      }
    } 
  }

  strip.show();
  
}
void goodOleRedGreenBLue() {
  setPatternName("VU: goodOleRedGreenBLue");
  
  fadeCol = 0;
  total = 0;
  int HEIGHT = N_PIXELS / 2;
  int i;
  int lclVol = 0;

  for (i = 0; i < N_PIXELS; i++){
    counter = 0;
    do{
      lclVol = analogRead(MIC_PIN);
 
      counter = counter + 1;
      if (counter > 500){
        rainbowCycle(10);
        
      }
    }while (lclVol == 0);
    
    total = total + lclVol;
 
  }
  
  lclVol = abs((int)total % 256);
  
  lclVol = map(lclVol,20,255,0,20);
  
  
  if (volLast > lclVol) {
    lclVol = volLast - 4;
  }
  
  volLast = lclVol;
  fadeAmt = 0;
   
  for (i = 0; i<strip.numPixels()/2;i++){
    if (i < lclVol){ 
      strip.setPixelColor((i+strip.numPixels()/2), strip.Color(0,255,lclVol));
      strip.setPixelColor((strip.numPixels()/2-i), strip.Color(0,255,lclVol));
    }
    else if (i < (lclVol + 8)) {
      
      strip.setPixelColor((i+strip.numPixels()/2), strip.Color(255,lclVol,lclVol));
      strip.setPixelColor((strip.numPixels()/2-i), strip.Color(255,lclVol,lclVol));
    }
    else
    {
      strip.setPixelColor((i+strip.numPixels()/2), strip.Color(0,0,255));
      strip.setPixelColor((strip.numPixels()/2-i), strip.Color(0,0,255));
    }
  }
  strip.show();
  
}
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  int lclVol = 0;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    
    delay(wait);
    lclVol = analogRead(MIC_PIN);
    if (lclVol> 10) {
      return;
    }
  }
}


void rippleNoBackground() {
  setPatternName("VU: rippleNoBackground");

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                              // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                        // Reset the counter every second.
  }

  soundripper(false);

  EVERY_N_MILLISECONDS(20) {
    rippled();
  }

  FastLED.show();

} // loop()



void rippled() {

  fadeToBlackBy(leds, N_PIXELS, 64);                      // 8 bit, 1 = slow, 255 = fast

  switch (step) {

    case -1:                                              // Initialize ripple variables.
      center = random(N_PIXELS);
      colour = (peakspersec*10) % 255;                    // More peaks/s = higher the hue colour.
      step = 0;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);              // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                        // At the end of the ripples.
        // step = -1;
      break;

    default:                                              // Middle of the ripples.
      leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);
      step ++;                                            // Next step.
      break;
  } // switch step

} // ripple()


  //Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for(int i=from; i<=to; i++){
    strip.setPixelColor(i, c);
  }
}

void pulseWithGlitter() {
  setPatternName("VU: pulseWithGlitter");
    //currentBlending = LINEARBLEND;
  currentPalette = OceanColors_p;             // Initial palette.
  currentBlending = LINEARBLEND;
  EVERY_N_SECONDS(5) {                        // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++) {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(100) {                 // AWESOME palette blending capability once they do change.
    uint8_t maxChanges = 24;
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }


  EVERY_N_MILLIS_I(thistimer,20) {            // For fun, let's make the animation have a variable rate.
    uint8_t timeval = beatsin8(10,20,50);     // Use a sinewave for the line below. Could also use peak/beat detection.
    thistimer.setPeriod(timeval);             // Allows you to change how often this routine runs.
    fadeToBlackBy(leds, N_PIXELS, 16);        // 1 = slow, 255 = fast fade. Depending on the faderate, the LED's further away will fade out.
// NERD    fadeToBlackBy(ledsRight, N_PIXELS, 16);        // 1 = slow, 255 = fast fade. Depending on the faderate, the LED's further away will fade out.
    sndwave();
    soundble();
  }
  FastLED.setBrightness(brightness);
  FastLED.show();

} // pulseWithGlitter

void juggleWithGlitter() {
  setPatternName("VU: juggleWithGlitter");

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                              // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                        // Reset the counter every second.
  }

  soundripper(false);

  EVERY_N_MILLISECONDS(20) {
    rippvu();
  }

  FastLED.show();

} // loop()

void rippvu() {                                           // Display ripples triggered by peaks.

  fadeToBlackBy(leds, N_PIXELS, 64);                      // 8 bit, 1 = slow, 255 = fast

  switch (step) {

    case -1:                                              // Initialize ripple variables.
      center = random(N_PIXELS);
      colour = (peakspersec*10) % 255;                    // More peaks/s = higher the hue colour.
      step = 0;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);              // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                        // At the end of the ripples.
        // step = -1;
      break;

    default:                                              // Middle of the ripples.
      leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);
      step ++;                                            // Next step.
      break;
  } // switch step
  addGlitter(sampleavg);
} // ripple()



void streamWithGlitter() {                                // The >>>>>>>>>> L-O-O-P <<<<<<<<<<<<<<<  is buried here!!!11!1!
  setPatternName("VU: streamWithGlitter");

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                             // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                       // Reset the counter every second.
  }

  soundripper(true);

  EVERY_N_MILLISECONDS(20) {
    jugglep();
  }

  FastLED.show();

} // loop()

// Rolling average counter - means we don't have to go through an array each time.
// This function is consumed by multiple other functions. To reduce duplication needed to add
// a parameter to call juggle2 manipulator for one use case.  pass in true/false.
void soundripper(int juggleFunc) {
  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

// GEEK:  Check the use against MIC_PIN rather than POT_PIN
  int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60);

  samplesum = samplesum + sample - samplearray[samplecount];  // Add the new sample and remove the oldest sample in the array
  sampleavg = samplesum / NSAMPLES;                      // Get an average
  samplearray[samplecount] = sample;                     // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;            // Update the counter for the array

  if (newtime > (oldtime + 200)) digitalWrite(13, LOW);  // Turn the LED off 200ms after the last peak.

    // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
  if ((sample > (sampleavg + potin)) && (newtime > (oldtime + 60)) ) {
    step = -1;
    peakcount++;
    oldtime = newtime;
      // Change the current pattern function periodically.
    if( juggleFunc ) { 
      jugglep();
    }
  }

} // soundripper()


void jugglep() {                                         // Use the juggle routine, but adjust the timebase based on sampleavg for some randomness.

    // Persistent local variables
  static uint8_t thishue=0;

  timeval = 40;                                          // Our EVERY_N_MILLIS_I timer value.

  leds[0] = ColorFromPalette(currentPalette, thishue++, sampleavg, LINEARBLEND);

  for (int i = N_PIXELS-1; i >0 ; i-- ) leds[i] = leds[i-1];

  addGlitter(sampleavg/2);                              // Add glitter based on sampleavg. By Andrew Tuline.

} // matrix()


void Balls() {
  setPatternName("SB: Balls");
  for (int i = 0 ; i < NUM_BALLS ; i++) {
      // Calculate the time since the last time the ball was on the ground
    tCycle[i] =  millis() - tLast[i] ;

      // A little kinematics equation calculates positon as a function of time, acceleration (gravity) and intial velocity
    h[i] = 0.5 * GRAVITY * pow( tCycle[i]/1000 , 2.0 ) + vImpact[i] * tCycle[i]/1000;

    if ( h[i] < 0 ) {
      h[i] = 0;                            // If the ball crossed the threshold of the "ground," put it back on the ground
      vImpact[i] = COR[i] * vImpact[i] ;   // and recalculate its new upward velocity as it's old velocity * COR
      tLast[i] = millis();

      if ( vImpact[i] < 0.01 ) vImpact[i] = vImpact0;  // If the ball is barely moving, "pop" it back up at vImpact0
    }
    pos[i] = round( h[i] * (N_PIXELS - 1) / h0);       // Map "h" to a "pos" integer index position on the LED strip
  }

    //Choose color of LEDs, then the "pos" LED on
  for (int i = 0 ; i < NUM_BALLS ; i++) leds[pos[i]] = CHSV( uint8_t (i * 40) , 255, 255);
  FastLED.show();
    //Then off for the next loop around
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    leds[pos[i]] = CRGB::Black;
  }
}



void ripple() {
  setPatternName("SB: Ripple");
  if (currentBg == nextBg) {
    nextBg = random(256);
  }
  else if (nextBg > currentBg) {
    currentBg++;
  } else {
    currentBg--;
  }
  for(uint16_t l = 0; l < N_PIXELS; l++) {
    leds[l] = CHSV(currentBg, 255, 50);           // strip.setPixelColor(l, Wheel(currentBg, 0.1));
  }

  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }

  if (step == 0) {
    leds[center] = CHSV(color, 255, 255);         // strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  }
  else {
    if (step < maxSteps) {

      leds[wrap(center + step)] = CHSV(color, 255, pow(fadeRate, step)*255);
      leds[wrap(center - step)] = CHSV(color, 255, pow(fadeRate, step)*255);
        //strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
        //strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        leds[wrap(center + step - 3)] = CHSV(color, 255, pow(fadeRate, step - 2)*255);
        leds[wrap(center - step + 3)] = CHSV(color, 255, pow(fadeRate, step - 2)*255);
          //strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
          //strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    }
    else {
      step = -1;
    }
  }

  LEDS.show();
  delay(50);
}


int wrap(int step) {
  if(step < 0) return N_PIXELS + step;
  if(step > N_PIXELS - 1) return step - N_PIXELS;
  return step;
}


void one_color_allHSV(int ahue, int abright) {  // SET ALL LEDS TO ONE COLOR (HSV)
  for (int i = 0 ; i < N_PIXELS; i++ ) {
    leds[i] = CHSV(ahue, 255, abright);
  }
}


void ripple2() {
  setPatternName("SB: Ripple2");
  if (BG){
    if (currentBg == nextBg) {
      nextBg = random(256);
    }
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
  } else {
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }


  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }



  if (step == 0) {
    strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  }
  else {
    if (step < maxSteps) {
      strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    }
    else {
      step = -1;
    }
  }

  strip.show();
  delay(50);
}

void fire() {
  setPatternName("SB: Fire");
#define FRAMES_PER_SECOND 40
  random16_add_entropy( random());


// Array of temperature readings at each simulation cell
  static byte heat[N_PIXELS];

    // Step 1.  Cool down every cell a little
  for( int i = 0; i < N_PIXELS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / N_PIXELS) + 2));
  }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= N_PIXELS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }

    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160,255) );
  }

    // Step 4.  Map from heat cells to LED colors
  for( int j = 0; j < N_PIXELS; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
    byte colorindex = scale8( heat[j], 240);
    CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Red, CRGB::Yellow,  CRGB::White), colorindex);
    int pixelnumber;
    if( reverseDirection ) {
      pixelnumber = (N_PIXELS-1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;

  }
  FastLED.show();
}


void fireblu() {
  setPatternName("SB: Fireblue");
#define FRAMES_PER_SECOND 40
  random16_add_entropy( random());


// Array of temperature readings at each simulation cell
  static byte heat[N_PIXELS];

    // Step 1.  Cool down every cell a little
  for( int i = 0; i < N_PIXELS; i++) {
    heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / N_PIXELS) + 2));
  }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= N_PIXELS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
  }

    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if( random8() < SPARKING ) {
    int y = random8(7);
    heat[y] = qadd8( heat[y], random8(160,255) );
  }

    // Step 4.  Map from heat cells to LED colors
  for( int j = 0; j < N_PIXELS; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
    byte colorindex = scale8( heat[j], 240);
    CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Blue, CRGB::Aqua,  CRGB::White), colorindex);
    int pixelnumber;
    if( reverseDirection ) {
      pixelnumber = (N_PIXELS-1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;

  }
  FastLED.show();
}


void Drip()  {
  setPatternName("SB: Drip");

  MODE_WATER_TORTURE:
  if (cycle())
  {
    strip.setBrightness(BRIGHTNESS); // off limits
    water_torture.animate(REVERSE);
    strip.show();

  }
}


bool cycle()
 {
   if (paused)
   {
     return false;
   }

   if (millis() - lastTime >= cycleMillis)
   {
     lastTime = millis();
     return true;
   }
   return false;
 }


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, float opacity) {

  if(WheelPos < 85) {
    return strip.Color((WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity, 0);
  }
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color((255 - WheelPos * 3) * opacity, 0, (WheelPos * 3) * opacity);
  }
  else {
    WheelPos -= 170;
    return strip.Color(0, (WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity);
  }
}


void pattern2() {
  setPatternName("SB: p2->sinelon");

  sinelon();                                                  // Call our sequence.
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
} // loop()


void sinelon() {
    // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, N_PIXELS, thisfade);
  int pos1 = beatsin16(thisbeat,0,N_PIXELS);
  int pos2 = beatsin16(thatbeat,0,N_PIXELS);
  leds[(pos1+pos2)/2] += CHSV( myhue++/64, thissat, thisbri);
}
// Pattern 3 - JUGGLE
void pattern3() {
  setPatternName("SB: p3->juggle");
  ChangeMe();
  juggle();
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
} // loop()


void juggle() {                                               // Several colored dots, weaving in and out of sync with each other
  curhue = thishue;                                           // Reset the hue values.
  fadeToBlackBy(leds, N_PIXELS, faderate);
  for( int i = 0; i < numdots; i++) {
    leds[beatsin16(basebeat+i+numdots,0,N_PIXELS)] += CHSV(curhue, thissat, thisbright);   //beat16 is a FastLED 3.1 function
    curhue += hueinc;
  }
} // juggle()


void ChangeMe() {                                             // A time (rather than loop) based demo sequencer. This gives us full control over the length of each sequence.
  uint8_t secondHand = (millis() / 1000) % 30;                // IMPORTANT!!! Change '30' to a different value to change duration of the loop.
  static uint8_t lastSecond = 99;                             // Static variable, means it's only defined once. This is our 'debounce' variable.
  if (lastSecond != secondHand) {                             // Debounce to make sure we're not repeating an assignment.
    lastSecond = secondHand;
    if (secondHand ==  0)  {numdots=1; faderate=2;}           // You can change values here, one at a time , or altogether.
    if (secondHand == 10)  {numdots=4; thishue=128; faderate=8;}
      // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
    if (secondHand == 20)  {hueinc=48; thishue=random8();}
  }
} // ChangeMe()

void juggle2() {                                              // Several colored dots, weaving in and out of sync with each other
  setPatternName("SB: juggle2");
    // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, N_PIXELS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,N_PIXELS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
  FastLED.show();
}

void addGlitter( fract8 chanceOfGlitter) {                    // Let's add some glitter, thanks to Mark

  if( random8() < chanceOfGlitter) {
    leds[random16(N_PIXELS)] += CRGB::White;
  }

} // addGlitter()




void Twinkle() {
  setPatternName("SB: Twinkle");

  if (random(25) == 1) {
    uint16_t i = random(N_PIXELS);
    if (redStates[i] < 1 && greenStates[i] < 1 && blueStates[i] < 1) {
      redStates[i] = random(256);
      greenStates[i] = random(256);
      blueStates[i] = random(256);
    }
  }

  for(uint16_t l = 0; l < N_PIXELS; l++) {
    if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1) {
      strip.setPixelColor(l, redStates[l], greenStates[l], blueStates[l]);

      if (redStates[l] > 1) {
        redStates[l] = redStates[l] * Fade;
      } else {
        redStates[l] = 0;
      }

      if (greenStates[l] > 1) {
        greenStates[l] = greenStates[l] * Fade;
      } else {
        greenStates[l] = 0;
      }

      if (blueStates[l] > 1) {
        blueStates[l] = blueStates[l] * Fade;
      } else {
        blueStates[l] = 0;
      }

    } else {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }
  strip.show();
  delay(10);

}

void blur() {
  setPatternName("SB: Blur");

  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, N_PIXELS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.

  uint8_t  i = beatsin8(  9, 0, N_PIXELS);
  uint8_t  j = beatsin8( 7, 0, N_PIXELS);
  uint8_t  k = beatsin8(  5, 0, N_PIXELS);

    // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);

  FastLED.show();

} // loop()




void rainbow(uint8_t wait) {
  setPatternName("SB: Rainbow");
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
      // check if a button pressed
    if (digitalRead(buttonPin) != lastButtonState) {  // <------------- add this
      return;                                         // <------------ and this
    }
    delay(wait);
  }
}

// List of StandBy patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { ripple, ripple2, Twinkle, pattern2, juggle2, pattern3, blur, Balls, Drip, fireblu, fire};
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current

void nextStandByPattern()
 {
     // add one to the current pattern number, and wrap around at the end
   gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
   
   gPatterns[gCurrentPatternNumber]();
   displayText();   
 }
void standByPatterns()
 {
     // Call the current pattern function once, updating the 'leds' array
   gPatterns[gCurrentPatternNumber]();
   EVERY_N_SECONDS( PATTERN_SWITCH_SECONDS ) { nextStandByPattern(); } // change patterns periodically
 }
// second list

// List of VU patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList qPatterns = { stdRainbow, stdCenterOut, stdCenterIn, stdColorChanging,
                                colorChangingCenter, shootingStar, fallingStar, rippleWithBackground,
                                shatter, goodOleRedGreenBLue, pulseWithGlitter, multiStdRainbow, 
                                rippleNoBackground, juggleWithGlitter, streamWithGlitter, multiAlternatingFFT };
uint8_t qCurrentPatternNumber = 0; // Index number of which pattern is current

void nextFFTVuPattern()
 {
     // add one to the current pattern number, and wrap around at the end
     // Pattern Number has changed, redisplay 
   qCurrentPatternNumber = (qCurrentPatternNumber + 1) % ARRAY_SIZE( qPatterns);
   
   qPatterns[qCurrentPatternNumber]();
   displayText();
 }
void fftVuPatterns()
 {
     // Call the current pattern function once, updating the 'leds' array
   qPatterns[qCurrentPatternNumber]();
   EVERY_N_SECONDS( PATTERN_SWITCH_SECONDS ) { nextFFTVuPattern(); } // change patterns periodically
 }
