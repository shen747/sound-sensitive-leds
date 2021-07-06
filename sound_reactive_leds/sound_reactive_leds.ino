#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
#include <math.h>

//LED Strip - [WS2812B]
#define LED_DATA_PIN  6
#define LED_COUNT 150            // Number of pixel LEDs in strand
#define HALF_POINT (LED_COUNT/2)
#define LAST_PIXEL_OFFSET (LED_COUNT-1)
#define TOP (LED_COUNT + 2)     // Allow dot to go slightly off scale
#define LED_TYPE WS2812B        // LED string type [WS2812B]
#define COLOR_ORDER GRB         // Colour order of LED strip 
#define PEAK_FALL 10            // Rate of peak falling dot
#define PEAK_HANG 24            //Time of pause before peak dot falls
#define PEAK_FALL_MILLIS  10    // Rate of peak falling dot
#define SPEED .20               // Amount to increment RGB color by each cycle
#define NSAMPLES 64
#define MAXSTEPS 16
#define COLOR_WAIT_CYCLES 10    // Loop cycles to wait between advancing pixel origin
#define COLOR_MIN 0
#define COLOR_MAX 255
#define DRAW_MAX  100
#define SEGMENTS  4             // Number of segments to carve amplitude bar into

unsigned int samplearray[NSAMPLES];
float greenOffset = 30;
float blueOffset = 150;
unsigned int sample;
long lastTime = 0;
int peakspersec = 0;
int peakcount = 0;
unsigned long oldtime = 0;
unsigned long newtime = 0;
unsigned long samplesum = 0;
unsigned int sampleavg = 0;
int samplecount = 0;
int center = 0;
int step = -1;
uint8_t bgcol = 0;
uint8_t colour;
uint8_t myfade = 255;
int color_wait_count = 0;
int scroll_color = COLOR_MIN;
uint32_t draw[DRAW_MAX];
boolean growing = false;
boolean fall_from_left = true;
int last_intensity = 0;
int intensity_max = 0;
int origin_at_flip = 0;
int origin = 0;

//Anaglog Sound Sensor
#define SOUND_SENSOR  A0
#define DC_OFFSET 0             // DC offset in mic signal - if unusure, leave 0
#define NOISE 10                // Noise/hum/interference in mic signal
#define SAMPLES 60              // Length of buffer for dynamic level adjustment [60]
#define MEAN 512                //Mean value used for centering on Zero
#define SAMPLE_WINDOW 10        // Sample window for average level
#define INPUT_FLOOR 10          //Lower range of analogRead input
#define INPUT_CEILING 300       //Max range of analogRead input, the lower the value the more sensitive (1023 = max)300 (150)

int  reading;
int lvl = 0;                    // Current "dampened" audio level
int minLvlAvg = 0;              // For dynamic adjustment of graph low & high
int maxLvlAvg = 512;
byte peak = 16;                 // Peak level of column; used for falling dots
byte dotCount = 0;              //Frame counter for peak dot
byte volCount  = 0;             // Frame counter for storing past volume data
byte dotHangCount = 0;          //Frame counter for holding peak dot
int  vol[SAMPLES];

struct CRGB leds[LED_COUNT];
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// cycle through patterns - Each pattern is defined as a separate function below
#define PATTERN_CYCLE_TIME 15 //pattern cycle time in seconds

typedef void (*SimplePatternList[])();
uint8_t qCurrentPatternNumber = 0;

//Common Config
#define POWER_UP_SAFETY_DELAY 3000 // power-up safety delay
#define SERIAL_BAUD_RATE 9600
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void setup() {

  strip.begin();
  strip.show(); // all pixels to 'off'

  pinMode(SOUND_SENSOR, INPUT);
  Serial.begin(SERIAL_BAUD_RATE);
  delay(POWER_UP_SAFETY_DELAY);

  FastLED.addLeds < LED_TYPE, LED_DATA_PIN, COLOR_ORDER > (leds, LED_COUNT).setCorrection(TypicalLEDStrip);
}

void loop() {
  //function that runs all patterns alrernatively every 30 seconds
  allPatterns();
}

void vu1() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(SOUND_SENSOR);         // Raw reading from mic
  n   = abs(n - MEAN - DC_OFFSET);        // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);   // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;             // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;      // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top
  Serial.println(height);

  // Color pixels based on rainbow gradient
  for (i = 0; i < LED_COUNT; i++) {
    if (i >= height)  strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 2, LED_COUNT)));
  }

  // Draw peak dot
  if (peak > 0 && peak <= LED_COUNT - 1) strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:
  if (++dotCount >= PEAK_FALL) { //fall rate

    if (peak > 0) peak--;
    dotCount = 0;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void vu2() {
  int intensity = calculateIntensity();
  updateOrigin(intensity);
  assignDrawValues(intensity);
  writeSegmented();
  updateGlobals();
}

void vu3()
{
  unsigned long startMillis = millis(); // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;

  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    sample = analogRead(SOUND_SENSOR);
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

  for (int i = 0; i <= HALF_POINT - 1; i++) {
    uint32_t color = Wheel(map(i, 0, HALF_POINT - 1, 30, 150));
    strip.setPixelColor(LED_COUNT - i, color);
    strip.setPixelColor(0 + i, color);
  }

  c = fscale(INPUT_FLOOR, INPUT_CEILING, HALF_POINT, 0, peakToPeak, 2);

  if (c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= strip.numPixels()) { // Fill partial column with off pixels
    drawLine(HALF_POINT, HALF_POINT - c, strip.Color(0, 0, 0));
    drawLine(HALF_POINT, HALF_POINT + c, strip.Color(0, 0, 0));
  }

  y = HALF_POINT - peak;
  uint32_t color1 = Wheel(map(y, 0, HALF_POINT - 1, 30, 150));
  strip.setPixelColor(y - 1, color1);
  //strip.setPixelColor(y-1,Wheel(map(y,0,N_PIXELS_HALF-1,30,150)));

  y = HALF_POINT + peak;
  strip.setPixelColor(y, color1);
  //strip.setPixelColor(y+1,Wheel(map(y,0,N_PIXELS_HALF+1,30,150)));

  strip.show();

  // Frame based peak dot animation
  if (dotHangCount > PEAK_HANG) { //Peak pause length
    if (++dotCount >= PEAK_FALL) { //Fall rate
      peak++;
      dotCount = 0;
    }
  }
  else {
    dotHangCount++;
  }
}

void vu4() {
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;

  n = analogRead(SOUND_SENSOR);             // Raw reading from mic
  n = abs(n - MEAN - DC_OFFSET);        // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE);  // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

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
  for (i = 0; i < LED_COUNT; i++) {
    if (i >= height) {
      strip.setPixelColor(i, 0, 0, 0);
    } else {
      strip.setPixelColor(i, Wheel(
                            map(i, 0, strip.numPixels() - 1, (int)greenOffset, (int)blueOffset)
                          ));
    }
  }
  // Draw peak dot
  if (peak > 0 && peak <= LED_COUNT - 1) strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL) { //fall rate

    if (peak > 0) peak--;
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

void vu5()
{
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(SOUND_SENSOR);                        // Raw reading from mic
  n   = abs(n - MEAN - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top


#ifdef CENTERED
  // Color pixels based on rainbow gradient
  for (i = 0; i < (LED_COUNT / 2); i++) {
    if (((LED_COUNT / 2) + i) >= height)
    {
      strip.setPixelColor(((LED_COUNT / 2) + i),   0,   0, 0);
      strip.setPixelColor(((LED_COUNT / 2) - i),   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(((LED_COUNT / 2) + i), Wheel(map(((LED_COUNT / 2) + i), 0, strip.numPixels() - 1, 30, 150)));
      strip.setPixelColor(((LED_COUNT / 2) - i), Wheel(map(((LED_COUNT / 2) - i), 0, strip.numPixels() - 1, 30, 150)));
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((LED_COUNT / 2) + peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((LED_COUNT / 2) - peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
  // Color pixels based on rainbow gradient
  for (i = 0; i < LED_COUNT; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 30, 150)));
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak, 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }

#endif

  // Every few frames, make the peak pixel drop by 1:

  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    strip.show(); // Update strip

    //fall rate
    if (peak > 0) peak--;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void vu6()
{
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = analogRead(SOUND_SENSOR);                        // Raw reading from mic
  n   = abs(n - MEAN - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top


#ifdef CENTERED
  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((LED_COUNT / 2) + peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((LED_COUNT / 2) - peak), 255, 255, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
  // Color pixels based on rainbow gradient
  for (i = 0; i < LED_COUNT; i++)
  {
    if (i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    {
    }
  }

  // Draw peak dot
  if (peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak, 0, 0, 255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }

#endif

  // Every few frames, make the peak pixel drop by 1:

  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    strip.show(); // Update strip

    //fall rate
    if (peak > 0) peak--;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++)
  {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

SimplePatternList qPatterns = { vu1, vu2, vu3, vu4, vu5, vu6 };
void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  qCurrentPatternNumber = (qCurrentPatternNumber + 1) % ARRAY_SIZE( qPatterns);
}

void allPatterns()
{
  // Call the current pattern function once, updating the 'leds' array
  qPatterns[qCurrentPatternNumber]();
  EVERY_N_SECONDS( PATTERN_CYCLE_TIME ) {
    nextPattern();  // change patterns periodically
  }
}

/************* AUX Functions used by VU  Pattern functions *************/

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

  vol[volCount] = reading;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (uint8_t i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < LED_COUNT) maxLvl = minLvl + LED_COUNT;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void writeSegmented() {
  int seg_len = LED_COUNT / SEGMENTS;

  for (int s = 0; s < SEGMENTS; s++) {
    for (int i = 0; i < seg_len; i++) {
      strip.setPixelColor(i + (s * seg_len), draw[map(i, 0, seg_len, 0, DRAW_MAX)]);
    }
  }
  strip.show();
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
  for (int i = 0; i < DRAW_MAX; i++) {
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

int calculateIntensity() {
  int      intensity;

  reading   = analogRead(SOUND_SENSOR);                        // Raw reading from mic
  reading   = abs(reading - MEAN - DC_OFFSET); // Center on zero
  reading   = (reading <= NOISE) ? 0 : (reading - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + reading) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  intensity = DRAW_MAX * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  return constrain(intensity, 0, DRAW_MAX - 1);
}


void ripple3() {
  for (int i = 0; i < LED_COUNT; i++) leds[i] = CHSV(bgcol, 255, sampleavg * 2); // Set the background colour.

  switch (step) {

    case -1:                                                          // Initialize ripple variables.
      center = random(LED_COUNT);
      colour = (peakspersec * 10) % 255;                                           // More peaks/s = higher the hue colour.
      step = 0;
      bgcol = bgcol + 8;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
      step ++;
      break;

    case MAXSTEPS:                                                    // At the end of the ripples.
      // step = -1;
      break;

    default:                                                             // Middle of the ripples.
      leds[(center + step + LED_COUNT) % LED_COUNT] += CHSV(colour, 255, myfade / step * 2);   // Simple wrap from Marc Miller.
      leds[(center - step + LED_COUNT) % LED_COUNT] += CHSV(colour, 255, myfade / step * 2);
      step ++;                                                         // Next step.
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
  for (int i = from; i <= to; i++) {
    strip.setPixelColor(i, c);
  }
}

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {

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

  if (newEnd > newBegin) {
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

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;

}


// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < HALF_POINT) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < LED_COUNT) {
    WheelPos -= HALF_POINT;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= LED_COUNT;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
