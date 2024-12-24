// Switch B enables or disables the lights on the xMas tree
// Switch A toggles between xMas tree and message

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
void updateStrip(uint8_t *message, uint8_t len, int8_t row, uint32_t color);
int16_t getAmbientLight();
void my_homekit_setup();
void my_homekit_loop();
void setupHomekit();
void clearDots();

///////////////////////////////////////////
///////////////// DEFINES /////////////////
///////////////////////////////////////////
#define _ENABLE_SERIAL_
#define _ENABLE_LEDS_
// #define _ENABLE_DS3231_
// #define _ENABLE_INTERRUPTS_
// #define _ENABLE_HOMEKIT_
// #define _ENABLE_ALS_
// #define _ENABLE_TEST_MODE_ //Turn on all LEDs white
// Hardware
#define BUTTON_PIN        D3
#define SWITCH_DUMMY      D5        // A on silkscreen (not sure itis working)
#define LED_PIN           D6        // Neopixel control
#define SWITCH_MODE       D7        // B on silkscreen
#define LIGHT_SENSOR_PIN  A0
// Application settings
#define FALSE             0
#define TRUE              1
#define SERIAL_SPEED      115200
#define REFRESH_TIME_STD  1000        // Refresh time for standard mode (Time\Temperature) [ms]
#define REFRESH_TIME_FAST 400         // Fast refresh time for time setting mode [ms]
#define MENU_TIME         5000        // How long will the menu time will stay active (temp, hour, min) [ms]
#define MENU_TIME_STD     MENU_TIME/REFRESH_TIME_STD
#define MENU_TIME_FAST    MENU_TIME/REFRESH_TIME_FAST
// LEDs
#define LED_COUNT         125         // How many NeoPixels are attached to the Arduino?
#define RED               0x00FF0000
#define GREEN             0x0000FF00
#define BLUE              0x000000FF
#define LIGHT_BLUE        0x00ADD8E6
#define WHITE             0x00FFFFFF
#define BROWN             0x00964B00
#define YELLOW            0x00FFFF00
#define BLACK             0x00000000
#define MIN_BRIGHTNESS    1           // Min LED brightness for dark environments
#define MAX_BRIGHTNESS    40          // Max LED brightness for well lit environments
#define AUTO_COLOR_STEP_SIZE 48000/60 // From Red (0) to Blue (~48000) in 60min
#define SEQUENCES         4

////////////////////////////////////////////////////
///////////////// Global variables /////////////////
////////////////////////////////////////////////////
// static float BRIGHTNESS_GAIN = (float)(MAX_BRIGHTNESS - MIN_BRIGHTNESS)/100;
// static float BRIGHTNESS_OFFSET = MIN_BRIGHTNESS;
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
uint8_t i = 0;
uint8_t j = 1;
uint8_t k = 2;
uint8_t l = 3;
int8_t row = -6;
uint32_t COLORS[] = {
    RED,
    GREEN,
    BLUE,
    WHITE,
    YELLOW
};

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
  strip.setBrightness(MAX_BRIGHTNESS);  // Set BRIGHTNESS (max = 255)
  strip.clear();
  strip.show();
  delay(1000);
  strip.clear();
  strip.show();
  delay(1000);
  #endif // _ENABLE_LEDS_

  pinMode(SWITCH_MODE, INPUT_PULLUP);
  pinMode(SWITCH_DUMMY, INPUT_PULLUP);

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
}


void loop() {

  #ifdef _ENABLE_HOMEKIT_
  if (homekitInitialized == TRUE) {my_homekit_loop();}
  #endif // _ENABLE_HOMEKIT_
  
  strip.clear();

  if (digitalRead(SWITCH_MODE) == TRUE) // xMas tree
  {
    i++; j++; k++; l++;
    if (i > SEQUENCES - 1){i = 0;}
    if (j > SEQUENCES - 1){j = 0;}
    if (k > SEQUENCES - 1){k = 0;}
    if (l > SEQUENCES - 1){l = 0;}
    
    // strip.fill(LIGHT_BLUE);                                          // Sky
    updateStrip(TRUNK, sizeof(TRUNK), NO_ROW, BROWN);                   // Trunk
    updateStrip(TREE, sizeof(TREE), NO_ROW, GREEN);                     // Tree

    updateStrip(LIGHTS[i], sizeof(LIGHTS[i]), NO_ROW, RED);             // Red lights
    updateStrip(LIGHTS[j], sizeof(LIGHTS[j]), NO_ROW, BLUE);            // Blue lights
    updateStrip(LIGHTS[k], sizeof(LIGHTS[k]), NO_ROW, LIGHT_BLUE);      // White lights
    updateStrip(LIGHTS[l], sizeof(LIGHTS[l]), NO_ROW, YELLOW);          // Yellow lights

    refreshTime = REFRESH_TIME_STD;
  }
  else // Message
  {
    #define MESSAGE_LEN 11
    // Running message
    row = row + 2;
    if (row > 8*MESSAGE_LEN)
    {
      row = -6;
      k++;
      if(k>4){k=0;}

    }
    for (i = 0; i < MESSAGE_LEN; i++) {
      updateStrip(MESSAGE[i], sizeof(MESSAGE[i]), row-8*i, COLORS[k]);
    }
    clearDots();
    // Single letter
    // i++;
    // if (i > MESSAGE_LEN - 1){i = 0;}
    // updateStrip(MESSAGE[i], sizeof(MESSAGE[i]), NO_ROW, RED);

    refreshTime = REFRESH_TIME_FAST;
  }

  strip.show();
  delay(refreshTime);
}


void updateStrip(uint8_t *message, uint8_t len, int8_t row, uint32_t color){
  for(int i=0; i<len; i++) {
    strip.setPixelColor(message[i] + row * 11, color); 
  }
}

void clearDots(){
  for(int i=120; i<125; i++) {
    strip.setPixelColor(i, BLACK); 
  }
}


// void updateColor()
// {
//   rgb_colors = strip.gamma32(strip.ColorHSV(current_hue, current_sat));
//   // Serial.print("New color: "); Serial.println(rgb_colors, HEX);
//   Serial.print("Current hue "); Serial.println(current_hue);
//   Serial.print("Current sat "); Serial.println(current_sat);
// }
     
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
  // if (digitalRead(SWITCH_DISABLE_HOMEKIT) == FALSE)
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