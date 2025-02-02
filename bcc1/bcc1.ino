#include <Adafruit_NeoPixel.h>

// LED matrix configuration
#define LED_PIN       6     // Pin to which the WS2812 matrix is attached
#define NUM_LEDS      64    // Total number of LEDs in the 8x8 matrix

// Button pins (assumed wiring; modify as needed)
#define BTN_START_PIN    2
#define BTN_ADD_MIN_PIN  3
#define BTN_ADD_10_PIN   4

// Debounce parameters
#define DEBOUNCE_DELAY   20   // milliseconds for button debouncing
#define LONG_PRESS_TIME 2000  // milliseconds for detecting long press on start button

// Maximum time in seconds (64 minutes)
#define MAX_TIME (64 * 60)

bool detectFallingEdge(uint8_t pin, int &lastState, unsigned long &lastDebounceTime) {
  int reading = digitalRead(pin);
  if (reading != lastState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY && lastState == HIGH && reading == LOW) {
    lastState = reading;
    return true;
  }
  lastState = reading;
  return false;
}

bool detectRisingEdge(uint8_t pin, int &lastState, unsigned long &lastDebounceTime) {
  int reading = digitalRead(pin);
  if (reading != lastState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY && lastState == LOW && reading == HIGH) {
    lastState = reading;
    return true;
  }
  lastState = reading;
  return false;
}

// NeoPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Global timer variables (in seconds)
unsigned long presetTime = 60;     // initial preset time (can be increased before starting)
unsigned long originalTime = 60;   // captures preset time when clock first started
unsigned long remainingTime = 60;  // remaining time (in seconds)

// Timing for countdown update
unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // update countdown every 1 second

// State variables
bool running = false;
bool hasStarted = false;  // indicates if the timer has ever started (to capture presetTime)

// Variables for button debouncing and long press detection
unsigned long lastDebounceStart = 0;
int lastBtnStartState = HIGH;
bool btnStartPressed = false;
unsigned long btnStartPressTime = 0;

unsigned long lastDebounceAddMin = 0;
int lastBtnAddMinState = HIGH;

unsigned long lastDebounceAdd10 = 0;
int lastBtnAdd10State = HIGH;

void setup() {
  // Initialize serial monitor for debugging
  Serial.begin(9600);

  // Initialize button pins as input with pullup
  pinMode(BTN_START_PIN, INPUT_PULLUP);
  pinMode(BTN_ADD_MIN_PIN, INPUT_PULLUP);
  pinMode(BTN_ADD_10_PIN, INPUT_PULLUP);

  // Initialize the LED strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  // Initialize initial display
  updateDisplay();
}

void loop() {
  unsigned long currentMillis = millis();

  // Start Button handling
  if (detectFallingEdge(BTN_START_PIN, lastBtnStartState, lastDebounceStart)) {
    btnStartPressTime = millis();
    btnStartPressed = true;
  }
  if (btnStartPressed && detectRisingEdge(BTN_START_PIN, lastBtnStartState, lastDebounceStart)) {
    unsigned long pressDuration = millis() - btnStartPressTime;
    if (pressDuration >= LONG_PRESS_TIME) {
      running = false;
      remainingTime = originalTime;
      Serial.println("Long press: Resetting timer to original preset.");
    } else {
      if (!hasStarted) {
        originalTime = presetTime;
        remainingTime = originalTime;
        hasStarted = true;
        running = true;
        previousMillis = millis();
        Serial.println("Timer started for the first time.");
      } else {
        running = !running;
        if (running) {
          previousMillis = millis();
          Serial.println("Timer resumed.");
        } else {
          Serial.println("Timer paused.");
        }
      }
    }
    btnStartPressed = false;
  }

  // Add 1 Minute Button handling
  if (detectFallingEdge(BTN_ADD_MIN_PIN, lastBtnAddMinState, lastDebounceAddMin)) {
    if (!hasStarted) {
      presetTime += 60;
      if (presetTime > MAX_TIME) {
        presetTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Preset time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 1 minute (preset mode).");
      }
    } else {
      remainingTime += 60;
      if (remainingTime > MAX_TIME) {
        remainingTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Remaining time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 1 minute (running/paused).");
      }
    }
    updateDisplay();
  }

  // Add 10 Minutes Button handling
  if (detectFallingEdge(BTN_ADD_10_PIN, lastBtnAdd10State, lastDebounceAdd10)) {
    if (!hasStarted) {
      presetTime += 600;
      if (presetTime > MAX_TIME) {
        presetTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Preset time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 10 minutes (preset mode).");
      }
    } else {
      remainingTime += 600;
      if (remainingTime > MAX_TIME) {
        remainingTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Remaining time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 10 minutes (running/paused).");
      }
    }
    updateDisplay();
  }

  // Countdown update
  unsigned long now = millis();
  if (running && (now - previousMillis >= interval)) {
    if (remainingTime > 0) {
      remainingTime--;
      previousMillis = now;  // update previousMillis to current time
      updateDisplay();
    } else {
      running = false;
      Serial.println("Countdown complete.");
      // Optionally you could flash the display here to indicate the end.
    }
  }
}

// Update the LED matrix display based on remainingTime.
// It lights full brightness white LEDs for each full minute and one LED with adjusted brightness for the fractional minute.
void updateDisplay() {
  // Clear the strip (set all pixels to off)
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, 0); // off
  }
  
  // Determine the number of full minutes and fraction 
  unsigned long fullMinutes = remainingTime / 60;
  int secondsLeft = remainingTime % 60;
  
  // Light up full minute LEDs (each full minute = full white LED)
  for (int i = 0; i < fullMinutes && i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(170, 170, 170)); // reduced brightness white
  }
  
  // For the fractional minute, if there is space and if there are remaining seconds:
  if (fullMinutes < NUM_LEDS && secondsLeft > 0) {
    // Calculate brightness proportional to the seconds left (range 0-170)
    uint8_t brightness = (uint8_t)((secondsLeft / 60.0) * 170);
    strip.setPixelColor(fullMinutes, strip.Color(brightness, brightness, brightness));
  }
  
  // Finally, update the LED matrix
  strip.show();
}

// Function to flash all LEDs very fast 3 times as feedback for max time reached
void flashMaxFeedback() {
  for (int i = 0; i < 3; i++) {
    // Turn all LEDs on with blue color
    for (int j = 0; j < NUM_LEDS; j++) {
      strip.setPixelColor(j, strip.Color(0, 0, 170));
    }
    strip.show();
    delay(100); // on for 100 ms
    // Turn all LEDs off
    for (int j = 0; j < NUM_LEDS; j++) {
      strip.setPixelColor(j, 0);
    }
    strip.show();
    delay(100); // off for 100 ms
  }
  // Restore the current display after flashing
  updateDisplay();
}