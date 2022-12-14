
#include <Adafruit_DotStar.h>

// There is only one pixel on the board
#define NUMPIXELS 1

//Use these pin definitions for the ItsyBitsy M4
#define DATAPIN    8
#define CLOCKPIN   6

//Enable one of the following
#define TEST_RAINBOW
//#define TEST_LED

const int ledPin = 11;
int led = 1;

#ifdef TEST_LED
  #define LED_ENABLED
#endif

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

void setup() {
#ifdef TEST_RAINBOW
  strip.begin(); // Initialize pins for output
  strip.setBrightness(80);
  strip.show();  // Turn all LEDs off ASAP
#endif

#ifdef TEST_LED
  pinMode(ledPin, OUTPUT);
#endif
}


void SetLED( int val)
{
  if (val == 1) {
#ifdef LED_ENABLED
    digitalWrite(ledPin, HIGH);   // set the LED on
#endif
    led = 1;
  } else {
#ifdef LED_ENABLED
    digitalWrite(ledPin, LOW);    // set the LED off
#endif
    led = 0;
  }
}

void ToggleLED()
{
  // Put this in setup() when we want to use it
  // --------------------
  // Initialize the digital pin as an output.
  //pinMode(ledPin, OUTPUT);
  //digitalWrite(ledPin, HIGH);   // set the LED on
  // --------------------

  if (led == 0) {
#ifdef LED_ENABLED
    digitalWrite(ledPin, HIGH);   // set the LED on
#endif
    led = 1;
  } else {
#ifdef LED_ENABLED
    digitalWrite(ledPin, LOW);    // set the LED off
#endif
    led = 0;
  }
}



void loop() {
#ifdef TEST_RAINBOW
  rainbow(10);             // Flowing rainbow cycle along the whole strip
#endif

#ifdef TEST_LED
  strip.setPixelColor(0, 255, 0, 0);  //red
  strip.show();
  SetLED(1);
  delay(1000);
  strip.setPixelColor(0, 0, 255, 0);  //green
  strip.show();
  SetLED(0);
  delay(1000);
  strip.setPixelColor(0, 0, 0, 255);  //blue
  strip.show();
  SetLED(1);
  delay(1000);
  SetLED(0);
  delay(1000);
#endif
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
