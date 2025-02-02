#include <Adafruit_NeoPixel.h>

// LED matrix configuration
#define LED_PIN       6     // Pin to which the WS2812 matrix is attached
#define NUM_LEDS      64    // Total number of LEDs in the 8x8 matrix

// Button pins (assumed wiring; modify as needed)
#define BTN_START_PIN    2
#define BTN_ADD_MIN_PIN  3
#define BTN_ADD_10_PIN   4

// Maximum time in seconds (64 minutes)
#define MAX_TIME (64 * 60)

// Maximum LED brightness (1/3 of 255)
#define MAX_BRIGHTNESS 85

// Simple debounce delay in milliseconds
#define DEBOUNCE_DELAY 50

// Long press detection time in milliseconds
#define LONG_PRESS_TIME 2000

// Simple edge detection functions with debounce
bool detectFallingEdge(uint8_t pin, int &lastState, unsigned long &unused) {
  int currentState = digitalRead(pin);
  
  // First check if we have a falling edge
  if (currentState == LOW && lastState == HIGH) {
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.println(" Falling edge!");
    delay(DEBOUNCE_DELAY);
    lastState = currentState;  // Update state after debounce
    return true;
  }
  
  lastState = currentState;
  return false;
}

bool detectRisingEdge(uint8_t pin, int &lastState, unsigned long &unused) {
  int currentState = digitalRead(pin);
  
  // First check if we have a rising edge
  if (currentState == HIGH && lastState == LOW) {
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.println(" Rising edge!");
    delay(DEBOUNCE_DELAY);
    lastState = currentState;  // Update state after debounce
    return true;
  }
  
  lastState = currentState;
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
bool countdownFinished = false;
unsigned long lastFlashMillis = 0;
bool flashState = false;

// Variables for button states
int lastBtnStartState = HIGH;
bool btnStartPressed = false;
unsigned long btnStartPressTime = 0;

int lastBtnAddMinState = HIGH;
int lastBtnAdd10State = HIGH;

void setup() {
  // Initialize serial monitor for debugging
  Serial.begin(115200);
  Serial.println("Bedtime counter hello.");

  // Initialize button pins as input with pullup
  pinMode(BTN_START_PIN, INPUT_PULLUP);
  pinMode(BTN_ADD_MIN_PIN, INPUT_PULLUP);
  pinMode(BTN_ADD_10_PIN, INPUT_PULLUP);

  // Initialize button states to their pulled-up state
  lastBtnStartState = HIGH;
  lastBtnAddMinState = HIGH;
  lastBtnAdd10State = HIGH;
  Serial.println("Button states initialized to HIGH");

  // Initialize the LED strip
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  // Show initial 1-minute value
  Serial.println("Initializing display with 1 minute.");
  updateDisplay();
}

void loop() {
  unsigned long currentMillis = millis();

  // Countdown update
  unsigned long now = millis();
  if (running && (now - previousMillis >= interval)) {
    if (remainingTime > 0) {
      remainingTime--;
      previousMillis = now;  // update previousMillis to current time
      updateDisplay();
    } else {
      running = false;
      countdownFinished = true;  // Start the continuous flash
      Serial.println("Countdown complete - resetting timer to preset.");
      remainingTime = presetTime;  // Reset to preset time
    }
  }

  // Handle finish flash if countdown is complete
  if (countdownFinished) {
    handleFinishFlash();
    
    // Check any button press to stop flashing
    if (digitalRead(BTN_START_PIN) == LOW || 
        digitalRead(BTN_ADD_MIN_PIN) == LOW || 
        digitalRead(BTN_ADD_10_PIN) == LOW) {
      countdownFinished = false;
      Serial.println("Flash stopped by button press");
      updateDisplay();  // Restore normal display
      delay(DEBOUNCE_DELAY);  // Simple debounce to avoid immediate re-trigger
      while ((digitalRead(BTN_START_PIN) == LOW || 
        digitalRead(BTN_ADD_MIN_PIN) == LOW || 
        digitalRead(BTN_ADD_10_PIN) == LOW)) {
        // wait until button releases
      }
    }
  }

  // Start Button handling

  // Always check for rising edge when button is pressed
  if (btnStartPressed) {
    if (detectRisingEdge(BTN_START_PIN, lastBtnStartState, btnStartPressTime)) {
      unsigned long pressDuration = millis() - btnStartPressTime;
      Serial.print("Button released! Press duration: ");
      Serial.print(pressDuration);
      Serial.println(" ms");
      
      if (pressDuration >= LONG_PRESS_TIME) {
        running = false;
        remainingTime = 60;  // Restore to 1 minute
        Serial.println("Long press: Resetting timer to 1 minute.");
        updateDisplay();
      } else {
        if (!hasStarted) {
          originalTime = presetTime;
          remainingTime = originalTime;
          hasStarted = true;
          running = true;
          previousMillis = millis();
          Serial.println("Timer started for the first time.");
          flashWhite();  // Flash white when timer starts
          updateDisplay();
        } else {
          running = !running;
          if (running) {
            previousMillis = millis();
            Serial.println("Timer resumed.");
            updateDisplay();
          } else {
            Serial.println("Timer paused.");
            flashBlue();  // Flash blue when timer is paused
            updateDisplay();
          }
        }
      }
      btnStartPressed = false;
    }
  } else {
    if (detectFallingEdge(BTN_START_PIN, lastBtnStartState, btnStartPressTime)) {
      btnStartPressTime = millis();
      btnStartPressed = true;
      Serial.println("Start button pressed, waiting for release...");
    }
  }

  // Add 1 Minute Button handling
  if (detectFallingEdge(BTN_ADD_MIN_PIN, lastBtnAddMinState, btnStartPressTime)) {
    Serial.println("Add 1 minute button pressed.");
    if (!hasStarted) {
      presetTime += 60;
      if (presetTime > MAX_TIME) {
        presetTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Preset time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 1 minute (preset mode).");
      }
      remainingTime = presetTime;  // Update remainingTime to show current preset
      updateDisplay();             // Show the new time
    } else {
      remainingTime += 60;
      if (remainingTime > MAX_TIME) {
        remainingTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Remaining time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 1 minute (running/paused).");
      }
      updateDisplay();
    }
  }

  // Add 10 Minutes Button handling
  if (detectFallingEdge(BTN_ADD_10_PIN, lastBtnAdd10State, btnStartPressTime)) {
    Serial.println("Add 10 minutes button pressed.");
    if (!hasStarted) {
      presetTime += 600;
      if (presetTime > MAX_TIME) {
        presetTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Preset time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 10 minutes (preset mode).");
      }
      remainingTime = presetTime;  // Update remainingTime to show current preset
      updateDisplay();             // Show the new time
    } else {
      remainingTime += 600;
      if (remainingTime > MAX_TIME) {
        remainingTime = MAX_TIME;
        flashMaxFeedback();
        Serial.println("Remaining time reached maximum of 64 minutes. Feedback flashed.");
      } else {
        Serial.println("Added 10 minutes (running/paused).");
      }
      updateDisplay();
    }
  }
}

void handleFinishFlash() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastFlashMillis >= 500) {  // Flash every 500ms
    lastFlashMillis = currentMillis;
    flashState = !flashState;
    
    for (int i = 0; i < NUM_LEDS; i++) {
      if (flashState) {
        strip.setPixelColor(i, strip.Color(MAX_BRIGHTNESS, 0, 0));  // Red at max brightness
      } else {
        strip.setPixelColor(i, 0);  // Off
      }
    }
    strip.show();
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
    strip.setPixelColor(i, strip.Color(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS)); // reduced brightness white
  }
  
  // For the fractional minute, if there is space and if there are remaining seconds:
  if (fullMinutes < NUM_LEDS && secondsLeft > 0) {
    // Calculate brightness proportional to the seconds left
    uint8_t brightness = (uint8_t)((secondsLeft / 60.0) * MAX_BRIGHTNESS);
    strip.setPixelColor(fullMinutes, strip.Color(brightness, brightness, brightness));
  }
  
  // Finally, update the LED matrix
  strip.show();

  Serial.print("Remaining time displayed: ");
  Serial.println(remainingTime);
}

// Function to flash all LEDs very fast 3 times as feedback for max time reached
void flashMaxFeedback() {
  for (int i = 0; i < 3; i++) {
    // Turn all LEDs on with blue color
    for (int j = 0; j < NUM_LEDS; j++) {
      strip.setPixelColor(j, strip.Color(0, 0, MAX_BRIGHTNESS));
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

void flashColor(uint8_t red, uint8_t green, uint8_t blue, int flashCount) {
    for (int i = 0; i < flashCount; i++) {
        // Turn all LEDs on with the specified color
        for (int j = 0; j < NUM_LEDS; j++) {
            strip.setPixelColor(j, strip.Color(red, green, blue));
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
}

void flashWhite() {
    flashColor(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS, 1); // Flash white once
}

void flashBlue() {
    flashColor(0, 0, MAX_BRIGHTNESS, 1); // Flash blue once
}