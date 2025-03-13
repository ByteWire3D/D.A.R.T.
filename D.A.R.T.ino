#include "Adafruit_VL53L1X.h"
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define IRQ_PIN 4
#define XSHUT_PIN 5

// ST7789 TFT module connections
#define TFT_CS 20   // define chip select pin
#define TFT_DC 5    // define data/command pin
#define TFT_RST 21  // define reset pin, or set to -1 and connect to Arduino RESET pin
#define BNO055_SAMPLERATE_DELAY_MS (100)

#define right_up 1
#define right_down 2
#define left_up 3
#define left_down 4

#define right_buttons 2
#define left_buttons 3

#define NUM_SAMPLES 1000  // Number of readings for averaging

#define DEBOUNCE_DELAY 1000  // 50ms debounce time

enum pageType {
  modemenu,
  distance,
  auto_vh,
  vertical,
  horizon,
  absolute,
  angle,
  live,
  distance_live,
  angle_live,
};

enum pageType current_Page = modemenu;

// Store the last stable states
int lastStableRight = -1;
int lastStableLeft = -1;
unsigned long lastDebounceTimeRight = 0;
unsigned long lastDebounceTimeLeft = 0;

float batt_volt_samples[NUM_SAMPLES] = { 0 };  // Array to store voltage readings
int sample_index = 0;                          // Index for tracking position
float filtered_batt_volt = 0;                  // Filtered voltage valu

bool right_up_wasdown = false;
bool right_down_wasdown = false;
bool left_up_wasdown = false;
bool left_down_wasdown = false;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);


int delay_time_l = 100;
int delay_time_d = 50;

float batt_volt;
float batt_percent;
int batt_per;
int batt_perc = 0;

float roll;
float pitch;
float yaw;

int temprature = 0;
bool update = true;

int menu_pos = 1;
void setup() {
  Serial.begin(115200);

  pinMode(right_buttons, INPUT);
  pinMode(left_buttons, INPUT);
  pinMode(4, INPUT);
  pinMode(9, INPUT_PULLUP);
  tft.init(240, 280);  // Init ST7789 display 240x240 pixel
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  dart_ani();


  float initial_value = (4.11 / 2860) * analogRead(4);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    batt_volt_samples[i] = initial_value;
  }
  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }


  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  Wire.begin();
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}

void loop() {
  switch (current_Page) {
    case modemenu: mode_menu(); break;
    case distance: break;
    case auto_vh: break;
    case vertical: break;
    case horizon: break;
    case absolute: break;
    case angle: angle_diff(); break;
    case live: live_mes(); break;
  }
}
void messure_volt() {
  batt_volt = (4.11 / 2860) * analogRead(4);

  batt_volt_samples[sample_index] = batt_volt;
  sample_index = (sample_index + 1) % NUM_SAMPLES;  // Loop back after filling the array

  // Compute the moving average
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += batt_volt_samples[i];
  }
  filtered_batt_volt = sum / NUM_SAMPLES;

  // Convert to percentage
  float batt_percent = ((filtered_batt_volt - 3.3) / 0.9) * 100;
  int batt_per = int(trunc(batt_percent));

  batt_per = int(trunc(batt_percent));
  if (batt_per != batt_perc) {
    batt_perc = batt_per;
    update = true;
  }
  /*
  Serial.print(batt_volt);
  Serial.print("    |   ");
  Serial.print(batt_percent);
  Serial.print("    |   ");
  Serial.println(batt_perc);
  */
  //Serial.print("   |   voltmes: ");
  //Serial.println(analogRead(4));
}
void mode_menu() {
  static uint32_t lastTime = 0;
  int updateHz = 30;
  int updateTime = 1000 / updateHz;
  if (millis() - lastTime >= updateTime) {
    lastTime = millis();
    messure_volt();
    if (update) {
      drawHeader("Mode menu", ST77XX_WHITE, ST77XX_RED);
      draw_modemenu(menu_pos);
    }
  }
  handle_up_down(3);
  if (Button_IsDown(left_up)) {
    left_up_wasdown = true;
  }
  if (Button_IsDown(left_down)) {
    left_down_wasdown = true;
  }


  if (left_up_wasdown && Button_IsUP(left_up)) {
    left_up_wasdown = false;
    update = true;
    if (menu_pos == 1) {
      current_Page = distance;
      menu_pos = 1;
      tft.fillScreen(ST77XX_BLACK);
    }
    if (menu_pos == 2) {
      current_Page = angle;
      menu_pos = 1;
      tft.fillScreen(ST77XX_BLACK);
    }
    if (menu_pos == 3) {
      current_Page = live;
      menu_pos = 1;
      tft.fillScreen(ST77XX_BLACK);
    }
  }
  if (left_down_wasdown && Button_IsUP(left_down)) {
    left_down_wasdown = false;
    update = true;
    Serial.println("cancel pressed!");
  }
}
void draw_modemenu(int pos) {
  int x = 20;
  int y = 36;
  tft.setCursor(x, y);
  tft.setTextSize(2);
  if (pos == 1) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("> ");
  }
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print("Distance   ");
  y += 24;
  tft.setCursor(x, y);
  tft.setTextSize(2);
  if (pos == 2) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("> ");
  }
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print("Angle   ");
  y += 24;
  tft.setCursor(x, y);
  tft.setTextSize(2);
  if (pos == 3) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("> ");
  }
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print("Live   ");
}
void handle_up_down(int menu_size) {
  if (Button_IsDown(right_up)) {
    right_up_wasdown = true;
  }
  if (Button_IsDown(right_down)) {
    right_down_wasdown = true;
  }

  if (right_up_wasdown && Button_IsUP(right_up)) {
    right_up_wasdown = false;
    menu_pos--;
    if (menu_pos < 1) {
      menu_pos = 1;
    }
    update = true;
  }
  if (right_down_wasdown && Button_IsUP(right_down)) {
    right_down_wasdown = false;
    menu_pos++;
    if (menu_pos > menu_size) {
      menu_pos = menu_size;
    }
    update = true;
  }
}

void angle_diff(){
   static uint32_t lastTime = 0;
  int updateHz = 30;
  int updateTime = 1000 / updateHz;
  if (millis() - lastTime >= updateTime) {
    lastTime = millis();
    messure_volt();
    if (update) {
      drawHeader("Angle menu", ST77XX_WHITE, ST77XX_RED);
    }
  }
  if (Button_IsDown(left_up)) {
    left_up_wasdown = true;
  }
  if (Button_IsDown(left_down)) {
    left_down_wasdown = true;
  }


  if (left_up_wasdown && Button_IsUP(left_up)) {
    left_up_wasdown = false;
    update = true;
  }
  if (left_down_wasdown && Button_IsUP(left_down)) {
    left_down_wasdown = false;
    update = true;
    Serial.println("cancel pressed!");
  }
}
void ver_hor_tot(int mode, int distace) {
}
void live_mes() {
  static uint32_t lastTime = 0;
  int updateHz = 10;
  int updateTime = 1000 / updateHz;
  if (millis() - lastTime >= updateTime) {
    lastTime = millis();
    if (update) {
      drawHeader("Live Measurements", ST77XX_WHITE, ST77XX_RED);
    }
    messure_volt();
    get_draw_live();
  }
  if (Button_IsDown(left_down)) {
    left_down_wasdown = true;
  }

  if (left_down_wasdown && Button_IsUP(left_down)) {
    left_down_wasdown = false;
    update = true;
    current_Page = modemenu;
    menu_pos = 3;
    tft.fillScreen(ST77XX_BLACK);
    Serial.println("cancel pressed!");
  }
}
void get_draw_live() {
  int16_t distance;

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }

    vl53.clearInterrupt();
  }
  sensors_event_t event;
  bno.getEvent(&event);

  pitch = event.orientation.z;
  roll = event.orientation.y;
  yaw = event.orientation.x;

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  int x = 20;
  int y = 36;

  tft.setCursor(x, y);
  tft.print("Distance: ");

  x = 20 + calculateLength("Distance: ");
  tft.setCursor(x, y);
  tft.print(distance);
  tft.print("       ");
  y += 24;
  x = 20;
  tft.setCursor(x, y);
  tft.setTextSize(2);
  tft.print("Pitch: ");
  x = 20 + calculateLength("Pitch: ");
  tft.setCursor(x, y);
  tft.print(pitch);
  tft.print("    ");
  y += 24;
  x = 20;
  tft.setCursor(x, y);
  tft.setTextSize(2);
  tft.print("Roll: ");
  x = 20 + calculateLength("Roll: ");
  tft.setCursor(x, y);
  tft.print(roll);
  tft.print("     ");
  y += 24;
  x = 20;
  tft.setCursor(x, y);
  tft.setTextSize(2);
  tft.print("Yaw: ");
  x = 20 + calculateLength("Yaw: ");
  tft.setCursor(x, y);
  tft.print(yaw);
  tft.print("     ");
}
void draw_options_distance() {
  int x = (280 - calculateLength("all|") - calculateLength("Distance|") - calculateLength("angle")) / 2;
  int y = 220;

  tft.setCursor(x, y);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (menu_pos == 1) {
    tft.fillRect(x, 236, calculateLength("all"), 3, ST77XX_RED);
    x += calculateLength("all|");
    tft.fillRect(x, 236, calculateLength("distance"), 3, ST77XX_BLACK);
    x += calculateLength("Distance|");
    tft.fillRect(x, 236, calculateLength("angle"), 3, ST77XX_BLACK);
  }
  if (menu_pos == 2) {
    tft.fillRect(x, 236, calculateLength("all"), 3, ST77XX_BLACK);
    x += calculateLength("all|");
    tft.fillRect(x, 236, calculateLength("distance"), 3, ST77XX_RED);
    x += calculateLength("Distance|");
    tft.fillRect(x, 236, calculateLength("angle"), 3, ST77XX_BLACK);
  }
  if (menu_pos == 3) {
    tft.fillRect(x, 236, calculateLength("all"), 3, ST77XX_BLACK);
    x += calculateLength("all|");
    tft.fillRect(x, 236, calculateLength("distance"), 3, ST77XX_BLACK);
    x += calculateLength("Distance|");
    tft.fillRect(x, 236, calculateLength("angle"), 3, ST77XX_RED);
  }
  tft.print("All|");
  tft.print("Distance|");
  tft.print("Angle");
}
void drawHeader(const char* headerText, uint16_t color, uint16_t barColor) {
  int headerPos = (280 - calculateLength(headerText)) / 2;  // centers header
  tft.setCursor(headerPos, 2);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.print(headerText);
  tft.fillRoundRect(16, 20, 248, 5, 5, barColor);
  float inv_batt_perc = 100 - batt_perc;
  int x = 246 - (229 * inv_batt_perc / 100);
  int l = 263 - x;
  tft.fillRoundRect(x, 21, l, 3, 3, ST77XX_BLACK);
  //tft.fillRoundRect(x, 21, l, 3, 3, ST77XX_BLACK);
}
int calculateLength(const char* word) {
  int length = 0;
  for (int i = 0; word[i] != '\0'; i++) {
    if (word[i] == ' ') {
      length += 10;
    } else {
      length += 12;
    }
  }
  return length;
}

int calculateLengthSize(char* word, int size) {
  if (size == 1) {
    int length = 0;
    for (int i = 0; word[i] != '\0'; i++) {
      if (word[i] == ' ') {
        length += 6;
      } else {
        length += 8;
      }
    }
    return length;
  }
  if (size == 2) {
    int length = 0;
    for (int i = 0; word[i] != '\0'; i++) {
      if (word[i] == ' ') {
        length += 10;
      } else {
        length += 12;
      }
    }
    return length;
  }
  if (size == 3) {
    int length = 0;
    for (int i = 0; word[i] != '\0'; i++) {
      if (word[i] == ' ') {
        length += 16;
      } else {
        length += 18;
      }
    }
    return length;
  }
  if (size == 4) {
    int length = 0;
    for (int i = 0; word[i] != '\0'; i++) {
      if (word[i] == ' ') {
        length += 20;
      } else {
        length += 24;
      }
    }
    return length;
  }
}
void live_angles() {
  sensors_event_t event;
  bno.getEvent(&event);

  pitch = event.orientation.z;
  roll = event.orientation.y;
  yaw = event.orientation.x;

  Serial.print(F("pitch: "));
  Serial.print(pitch);
  Serial.print(F(" "));
  Serial.print(F("roll: "));
  Serial.print(roll);
  Serial.print(F(" "));
  Serial.print(F("yaw: "));
  Serial.print(yaw);
  Serial.println(F(""));

  /*
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\n");
  */
}

void get_temp() {
  if (bno.getTemp() != temprature) {
    temprature = bno.getTemp();
    update = true;
  }
}
void handle_buttons() {
  if (Button_IsDown(right_up)) {
    right_up_wasdown = true;
  }
  if (Button_IsDown(right_down)) {
    right_down_wasdown = true;
  }
  if (Button_IsDown(left_up)) {
    left_up_wasdown = true;
  }
  if (Button_IsDown(left_down)) {
    left_down_wasdown = true;
  }

  if (right_up_wasdown && Button_IsUP(right_up)) {
    right_up_wasdown = false;
    Serial.println("right up was pressed!");
  }
  if (right_down_wasdown && Button_IsUP(right_down)) {
    right_down_wasdown = false;
    Serial.println("right down was pressed!");
  }
  if (left_up_wasdown && Button_IsUP(left_up)) {
    left_up_wasdown = false;
    Serial.println("left up was pressed!");
  }
  if (left_down_wasdown && Button_IsUP(left_down)) {
    left_down_wasdown = false;
    Serial.println("left down was pressed!");
  }
}

bool Button_IsDown(int button) {
  if (button == right_up) {
    //right up
    if (analogRead(right_buttons) == 4095) {
      return true;
    }
  }
  if (button == right_down) {
    //right down
    if (analogRead(right_buttons) >= 40 && analogRead(right_buttons) <= 100) {
      return true;
    }
  }

  if (button == left_up) {
    //left up
    if (analogRead(left_buttons) == 4095) {
      return true;
    }
  }
  if (button == left_down) {
    //left down
    if (analogRead(left_buttons) <= 50) {
      return true;
    }
  }
  return false;
}

bool Button_IsUP(int button) {
  if (button == right_up) {
    //right up
    if (analogRead(right_buttons) <= 40) {
      return true;
    }
  }
  if (button == right_down) {
    //right down
    if (analogRead(right_buttons) <= 40) {
      return true;
    }
  }
  if (button == left_up) {
    //left up
    if (analogRead(left_buttons) > 50 && analogRead(left_buttons) < 4095) {
      return true;
    }
  }
  if (button == left_down) {
    //left down
    if (analogRead(left_buttons) > 50 && analogRead(left_buttons) < 4095) {
      return true;
    }
  }
  return false;
}

/*
bool Button_IsDown(int button) {
  int rightValue = analogRead(right_buttons);
  int leftValue = analogRead(left_buttons);
  
  unsigned long currentTime = millis();

  // Right Up Button
  if (button == right_up) {
    if (rightValue >= 4000) {
      if ((currentTime - lastDebounceTimeRight) > DEBOUNCE_DELAY) {
        lastStableRight = right_up;
        return true;
      }
    }
  }

  // Right Down Button
  if (button == right_down) {
    if (rightValue >= 2100 && rightValue <= 2250) {
      if ((currentTime - lastDebounceTimeRight) > DEBOUNCE_DELAY) {
        lastStableRight = right_down;
        return true;
      }
    }
  }

  // Left Up Button
  if (button == left_up) {
    if (leftValue >= 4000) {
      if ((currentTime - lastDebounceTimeLeft) > DEBOUNCE_DELAY) {
        lastStableLeft = left_up;
        return true;
      }
    }
  }

  // Left Down Button
  if (button == left_down) {
    if (leftValue <= 250) {
      if ((currentTime - lastDebounceTimeLeft) > DEBOUNCE_DELAY) {
        lastStableLeft = left_down;
        return true;
      }
    }
  }

  return false;
}

bool Button_IsUP(int button) {
  int rightValue = analogRead(right_buttons);
  int leftValue = analogRead(left_buttons);

  unsigned long currentTime = millis();

  // Right Up Released
  if (button == right_up) {
    if (rightValue <= 4000) {
      if ((currentTime - lastDebounceTimeRight) > DEBOUNCE_DELAY) {
        lastStableRight = -1;
        return true;
      }
    }
  }

  // Right Down Released
  if (button == right_down) {
    if (rightValue <= 2000) {
      if ((currentTime - lastDebounceTimeRight) > DEBOUNCE_DELAY) {
        lastStableRight = -1;
        return true;
      }
    }
  }

  // Left Up Released
  if (button == left_up) {
    if (leftValue > 250 && leftValue < 4095) {
      if ((currentTime - lastDebounceTimeLeft) > DEBOUNCE_DELAY) {
        lastStableLeft = -1;
        return true;
      }
    }
  }

  // Left Down Released
  if (button == left_down) {
    if (leftValue > 250 && leftValue < 4095) {
      if ((currentTime - lastDebounceTimeLeft) > DEBOUNCE_DELAY) {
        lastStableLeft = -1;
        return true;
      }
    }
  }

  return false;
}
*/
void dart_ani() {
  tft.setCursor(60, 100);
  tft.setTextSize(4);
  //tft.print("D.A.R.T.");
  tft.setCursor(60, 100);
  tft.print("D");
  delay(delay_time_l);
  tft.setCursor(60, 100);
  tft.print("D.");
  delay(delay_time_d);
  tft.setCursor(60, 100);
  tft.print("D.A");
  delay(delay_time_l);
  tft.setCursor(60, 100);
  tft.print("D.A.");
  delay(delay_time_d);
  tft.setCursor(60, 100);
  tft.print("D.A.R");
  delay(delay_time_l);
  tft.setCursor(60, 100);
  tft.print("D.A.R.");
  delay(delay_time_d);
  tft.setCursor(60, 100);
  tft.print("D.A.R.T");
  delay(delay_time_l);
  tft.setCursor(60, 100);
  tft.print("D.A.R.T");
  delay(delay_time_d);

  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
}
