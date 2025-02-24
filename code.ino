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

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);


int delay_time_l = 100;
int delay_time_d = 50;


void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

  tft.init(240, 280);  // Init ST7789 display 240x240 pixel
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  dart_ani();

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

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
  int16_t distance;

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();
  }

  sensors_event_t event;
  bno.getEvent(&event);

  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));


  Serial.print(analogRead(2));
  Serial.print("   |   ");
  Serial.print(analogRead(3));
  Serial.print("   |   voltmes: ");
  Serial.println(analogRead(4));
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

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
}
