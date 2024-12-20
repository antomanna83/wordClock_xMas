#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "RTClib.h"
#include <LedMatrixDefinition.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"

/////////////////////////////////////////////
///////////////// Functions /////////////////
/////////////////////////////////////////////
void button_ISR();
void updateFrame();
void updateStrip(uint8_t *message, uint8_t len, uint8_t row, uint32_t color);
int16_t getAmbientLight();
void my_homekit_setup();
void my_homekit_loop();
void setupHomekit();

///////////////////////////////////////////
///////////////// DEFINES /////////////////
///////////////////////////////////////////
#define _ENABLE_SERIAL_
#define _ENABLE_LEDS_
#define _ENABLE_DS3231_
#define _ENABLE_INTERRUPTS_
#define _ENABLE_HOMEKIT_
#define _ENABLE_ALS_
// #define _ENABLE_TEST_MODE_ //Turn on all LEDs white
// Hardware
#define BUTTON_PIN        D3
#define SWITCH_DISABLE_HOMEKIT D5      // A on silkscreen
#define LED_PIN           D6           // Neopixel control
#define SWITCH_DISABLE_AUTOCOLOR  D7   // B on silkscreen
#define LIGHT_SENSOR_PIN  A0
// Application settings
#define FALSE             0
#define TRUE              1
#define SERIAL_SPEED      115200
#define REFRESH_TIME_STD  1000        // Refresh time for standard mode (Time\Temperature) [ms]
#define REFRESH_TIME_FAST 200         // Fast refresh time for time setting mode [ms]
#define MENU_TIME         5000        // How long will the menu time will stay active (temp, hour, min) [ms]
#define MENU_TIME_STD     MENU_TIME/REFRESH_TIME_STD
#define MENU_TIME_FAST    MENU_TIME/REFRESH_TIME_FAST
// LEDs
#define LED_COUNT         125         // How many NeoPixels are attached to the Arduino?
#define RED               0x00FF0000
#define GREEN             0x0000FF00
#define BLUE              0x000000FF
#define WHITE             0x00FFFFFF
#define MIN_BRIGHTNESS    1           // Min LED brightness for dark environments
#define MAX_BRIGHTNESS    40          // Max LED brightness for well lit environments
#define AUTO_COLOR_STEP_SIZE 48000/60 // From Red (0) to Blue (~48000) in 60min

////////////////////////////////////////////////////
///////////////// Global variables /////////////////
////////////////////////////////////////////////////
static float BRIGHTNESS_GAIN = (float)(MAX_BRIGHTNESS - MIN_BRIGHTNESS)/100;
static float BRIGHTNESS_OFFSET = MIN_BRIGHTNESS;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ400);
enum states {S_TIME, S_TEMPERATURE, S_SET_HOUR, S_SET_MINUTE};
enum buttonStates {PRESSED, RELEASED};
uint8_t operatingMode = S_TIME;
uint8_t menuTimer = 0;
bool shortPressEvent = FALSE;
DateTime now; // Uninitialized :( 
unsigned long refreshTime = REFRESH_TIME_STD; // How often to refresh while in time mode.
uint8_t hour, hour_to_display, minutes, temperature, minutes_range, dots = 0;
float brightness = 0.0;
uint8_t current_sat = 0;
uint16_t current_hue = 12*182;
uint32_t rgb_colors = WHITE;
bool homekitInitialized = FALSE;

#ifdef _ENABLE_DS3231_
RTC_DS3231 rtc;
#endif // _ENABLE_DS3231_

#ifdef _ENABLE_INTERRUPTS_
const int SHORT_PRESS_TIME = 500;
unsigned long timePressed, timeReleased = 0;
long pressDuration = 0;
#endif // _ENABLE_INTERRUPTS_

#ifdef _ENABLE_HOMEKIT_
bool received_sat = false;
bool received_hue = false;
extern "C" homekit_server_config_t accessory_config;
extern "C" homekit_characteristic_t cha_on;
extern "C" homekit_characteristic_t cha_bright;
extern "C" homekit_characteristic_t cha_sat;
extern "C" homekit_characteristic_t cha_hue;
#endif // _ENABLE_HOMEKIT_


/////////////////////////////////////////////
///////////////// Functions /////////////////
/////////////////////////////////////////////
void setup() {

  #ifdef _ENABLE_LEDS_
  strip.begin();                        // Initiazlize NeoPixel strip object
  strip.setBrightness(MIN_BRIGHTNESS);  // Set BRIGHTNESS (max = 255)
  strip.clear();
  strip.show();
  delay(1000);
  strip.clear();
  strip.show();
  delay(1000);
  #endif // _ENABLE_LEDS_

  pinMode(SWITCH_DISABLE_HOMEKIT, INPUT_PULLUP);
  pinMode(SWITCH_DISABLE_AUTOCOLOR, INPUT_PULLUP);

  #ifdef _ENABLE_SERIAL_
  Serial.begin(SERIAL_SPEED);
  #endif // _ENABLE_SERIAL_

  #ifdef _ENABLE_INTERRUPTS_
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_ISR, CHANGE);
  #endif // _ENABLE_INTERRUPTS_

  #ifdef _ENABLE_ALS_
  analogReference(EXTERNAL);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  #endif // _ENABLE_ALS_

  #ifdef _ENABLE_DS3231_
  while(!rtc.begin()){
    Serial.println("Couldn't find RTC. Retrying...");
    Serial.flush();
    delay(1000); // Stuck here forever...
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(2017, 10, 30, 10, 25, 0));
  }
  #endif // _ENABLE_DS3231_

  // #ifdef _ENABLE_LEDS_
  // strip.begin();                        // Initiazlize NeoPixel strip object
  // strip.setBrightness(MIN_BRIGHTNESS);  // Set BRIGHTNESS (max = 255)
  // strip.clear();
  // strip.show();
  // #endif // _ENABLE_LEDS_
}


void loop() {
  int16_t ambient_light = 0;

  #ifdef _ENABLE_HOMEKIT_
  if (homekitInitialized == TRUE) {my_homekit_loop();}
  #endif // _ENABLE_HOMEKIT_
  
  strip.clear();
  refreshTime = REFRESH_TIME_STD;
  
  switch (operatingMode){
    case S_TIME:
      #ifdef _ENABLE_TEST_MODE_
      Serial.println("Test Mode");
      strip.fill(WHITE, 0, 126);
      strip.setBrightness(10);
      strip.show();
      delay(refreshTime);
      return;
      #endif // _ENABLE_TEST_MODE_

      // Set LED brightness based on ambient light
      ambient_light = getAmbientLight();
      brightness = (float) (ambient_light * BRIGHTNESS_GAIN + BRIGHTNESS_OFFSET);
      strip.setBrightness((uint8_t)brightness);

      #ifdef _ENABLE_DS3231_
      now = rtc.now(); // Get time
      #else
      now = DateTime(2017, 10, 30, 10, 25, 0);
      #endif // _ENABLE_DS3231_
      hour = (now.hour()>12 ? (now.hour()-12) : (now.hour())); // Convert 24-h format into 12-h format
      Serial.print("Time mode: ");
      Serial.print(hour, DEC);
      Serial.print(':');
      Serial.println(now.minute(), DEC);
      #ifdef _ENABLE_ALS_
      Serial.print("Ambient Light: ");
      Serial.println(ambient_light, DEC);
      Serial.print("Brightness: ");
      Serial.println(brightness, DEC);
      #endif // _ENABLE_ALS_

      // Set hours
      hour_to_display = hour;
      if (now.minute() > 39){ 
        hour_to_display = hour + 1;   // Past hour:39 we should print hour+1 "meno venti", "meno un quarto" and so on...
        if (hour_to_display > 12){ hour_to_display = 1; }
      }
      if (hour_to_display == 0){ hour_to_display = 12; }             // Midnight is 0 but should display string #12
      
      // Set minutes (words)
      minutes_range = (uint8_t)floor(now.minute()/5);
      
      // Set minutes (dots)
      dots = now.minute() - minutes_range * 5;
      break;

    case S_TEMPERATURE:
      #ifdef _ENABLE_DS3231_
      temperature = (uint8_t)(rtc.getTemperature()+0.5); // Get temperature (and convert in integer)
      #else
      temperature = 25;
      #endif // _ENABLE_DS3231_
      Serial.print("Temperature mode: ");
      Serial.println(temperature);
      menuTimer++; // Back to time mode once timer expires
      if (menuTimer == MENU_TIME_STD){
        operatingMode = S_TIME;
      }
      break;

    case S_SET_HOUR:
      Serial.print("Setting hour: ");
      Serial.println(hour, DEC);
      refreshTime = REFRESH_TIME_FAST;
      if (shortPressEvent == TRUE){
        shortPressEvent = FALSE; // Clear event
        hour = (hour < 23 ? (hour+1) : (0)); // Increment hours
        menuTimer = 0; // Restart timer
      }
      menuTimer++; // Moving to minutes set mode once timer expires
      if (menuTimer == MENU_TIME_FAST){
        operatingMode = S_SET_MINUTE;
        menuTimer = 0;
        #ifdef _ENABLE_DS3231_
        now = rtc.now(); // Get time
        #endif // _ENABLE_DS3231_
        minutes = now.minute();
      }
      break;

    case S_SET_MINUTE:
      Serial.print("Setting minutes: ");
      Serial.println(minutes, DEC);
      refreshTime = REFRESH_TIME_FAST;
      if (shortPressEvent == TRUE){
        shortPressEvent = FALSE; // Clear event
        minutes = (minutes < 59 ? (minutes+1) : (0)); // Increment minutes
        menuTimer = 0; // Restart timer
      }
      menuTimer++; // Back to time mode once timer expires
      if (menuTimer == MENU_TIME_FAST){
        #ifdef _ENABLE_DS3231_
        rtc.adjust(DateTime(2017, 10, 30, hour, minutes, 0));
        #endif // _ENABLE_DS3231_
        operatingMode = S_TIME;
      }
      break;

    default:
      break;
  }
  updateFrame();
  strip.show();
  if (homekitInitialized == FALSE){setupHomekit();} // Setup HomeKit the first time
  delay(refreshTime);
}


void updateStrip(uint8_t *message, uint8_t len, uint8_t row, uint32_t color){
  for(int i=0; i<len; i++) {
    strip.setPixelColor(message[i] + row * 11, color); 
  }
}


void updateColor()
{
  rgb_colors = strip.gamma32(strip.ColorHSV(current_hue, current_sat));
  // Serial.print("New color: "); Serial.println(rgb_colors, HEX);
  Serial.print("Current hue "); Serial.println(current_hue);
  Serial.print("Current sat "); Serial.println(current_sat);
  // updateFrame();
  // strip.show();
}


void updateFrame(void)
{
   switch (operatingMode){
    case S_TIME:
      if (digitalRead(SWITCH_DISABLE_AUTOCOLOR) == FALSE) // If switch is set to have color to change dynamically
      {
        current_hue = (uint16_t)(now.minute() * AUTO_COLOR_STEP_SIZE); // Increase color
        current_sat = 255; // Set saturation to max
        updateColor(); // Update rgb_colors
      } // Else keep the color the same (either default or from HomeKit)
      if (hour_to_display != 1){ updateStrip(HOURS[0], sizeof(HOURS[0]), NO_ROW, rgb_colors); } // Unless it is 1 print "Sono le ore"
      updateStrip(HOURS[hour_to_display], sizeof(HOURS[hour_to_display]), NO_ROW, rgb_colors);
      updateStrip(MINUTES[minutes_range], sizeof(MINUTES[minutes_range]), NO_ROW, rgb_colors);
      updateStrip(DOTS[dots], sizeof(DOTS[dots]), NO_ROW, rgb_colors);
    break;
    case S_TEMPERATURE:
      updateStrip(NUMBERS[temperature/10], sizeof(NUMBERS[temperature/10]), 6, RED);
      updateStrip(NUMBERS[temperature-(temperature/10)*10], sizeof(NUMBERS[temperature-(temperature/10)*10]), 0, RED);
      break;

    case S_SET_HOUR:
      updateStrip(ORE, sizeof(ORE), NO_ROW, BLUE);
      updateStrip(NUMBERS[hour/10], sizeof(NUMBERS[hour/10]), 6, GREEN);
      updateStrip(NUMBERS[hour-(hour/10)*10], sizeof(NUMBERS[hour-(hour/10)*10]), 0, GREEN);
      break;

    case S_SET_MINUTE:
      updateStrip(MINUTI, sizeof(MINUTI), NO_ROW, BLUE);
      updateStrip(NUMBERS[minutes/10], sizeof(NUMBERS[minutes/10]), 6, GREEN);
      updateStrip(NUMBERS[minutes-(minutes/10)*10], sizeof(NUMBERS[minutes-(minutes/10)*10]), 0, GREEN);
      break;

    default:
      break;
   }
}


#ifdef _ENABLE_INTERRUPTS_
void IRAM_ATTR button_ISR() {
  if(digitalRead(BUTTON_PIN) == PRESSED){ // On press...
    timePressed = millis(); // No action on press. Simply record the time.
    Serial.print("timePressed: ");
    Serial.println(timePressed);
  } else { // On release...
    timeReleased = millis();
    Serial.print("timeReleased: ");
    Serial.println(timeReleased);
    if((timeReleased - timePressed) > SHORT_PRESS_TIME){
      // LONG PRESS - Only effective while in TIME mode
      Serial.println("Long press.");
      if (operatingMode == S_TIME){
        operatingMode = S_SET_HOUR;
        menuTimer = 0;
      }
    } else { 
      // SHORT PRESS
      Serial.println("Short press.");
      if((operatingMode == S_TIME) || (operatingMode == S_TEMPERATURE)){
        operatingMode = S_TEMPERATURE;
        menuTimer = 0;
      } else {
        shortPressEvent = TRUE;
      }
    }
  }
  return;
}
#endif // _ENABLE_INTERRUPTS


int16_t getAmbientLight()
{
  // ADC reading from ambient light goes almost from 0 to full scale (1023) which can easily be converted into a percentage dividing by 10.
  #ifdef _ENABLE_ALS_
  return(analogRead(LIGHT_SENSOR_PIN)/10);
  #else
  return 100;
  #endif // _ENABLE_ALS_
}

#ifdef _ENABLE_HOMEKIT_
void set_on(const homekit_value_t v) {
    bool on = v.bool_value;
    cha_on.value.bool_value = on; //sync the value
}

void set_hue(const homekit_value_t v) {
    // Serial.println("set_hue ");
    float hue = v.float_value;
    cha_hue.value.float_value = hue; //sync the value
    current_hue = (uint16_t)(hue * 182); // Convert 0~360 to 0~65535
    // Serial.println(current_hue);
    received_hue = true;
    updateColor();
}

void set_sat(const homekit_value_t v) {
    // Serial.print("set_sat ");
    float sat = v.float_value;
    cha_sat.value.float_value = sat; //sync the value
    current_sat = (uint8_t)(sat * 2.55); // Convert 0~100 to 0~255
    received_sat = true;
    // Serial.println(current_sat);
    updateColor();
}

void set_bright(const homekit_value_t v) {
    int bright = v.int_value;
    cha_bright.value.int_value = bright; //sync the value
}

void my_homekit_setup() {
  cha_on.setter = set_on;
  cha_bright.setter = set_bright;
  cha_sat.setter = set_sat;
  cha_hue.setter = set_hue;
	arduino_homekit_setup(&accessory_config);
}

void my_homekit_loop() {
	arduino_homekit_loop();
	// const uint32_t t = millis();
	// if (t > next_heap_millis) {
	// 	// show heap info every 5 seconds
	// 	next_heap_millis = t + 5 * 1000;
	// 	LOG_D("Free heap: %d, HomeKit clients: %d",
	// 			ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
	// }
}
#endif // _ENABLE_HOMEKIT_

void setupHomekit(void)
{
  if (digitalRead(SWITCH_DISABLE_HOMEKIT) == FALSE)
  {
    #ifdef _ENABLE_HOMEKIT_
    wifi_connect();
    if (digitalRead(BUTTON_PIN) == PRESSED) // If the push button is pressed at boot HK gets reset
    {
      homekit_storage_reset(); // reset homekit
    }
    my_homekit_setup();
    #endif // _ENABLE_HOMEKIT_
    homekitInitialized = TRUE;
  }
}