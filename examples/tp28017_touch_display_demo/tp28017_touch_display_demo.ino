/*
  TP28017 resistive touch display demo for Arduino Uno.

  Required libraries from the Arduino Library Manager:
  - Adafruit GFX Library
  - MCUFRIEND_kbv
  - TouchScreen

  The sketch checks the TFT controller ID, fills the display with test colors,
  draws text and shapes, then shows raw and mapped touch coordinates.
*/

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

// Common wiring for 2.4/2.8 inch Arduino Uno resistive TFT shields.
// Change these if your TP28017 module documents different touch pins.
#define YP A3
#define XM A2
#define YM 9
#define XP 8

#define SERIAL_BAUD 115200
#define MINPRESSURE 5
#define MAXPRESSURE 1000
#define TOUCH_POLL_DELAY_MS 5
#define SERIAL_LOG_INTERVAL_MS 60
#define LEFT_EDGE_COMPENSATION_PX 14
#define TOP_EDGE_COMPENSATION_PX 12
#define LEFT_EDGE_COMPENSATION_ZONE_PX 56
#define TOP_EDGE_COMPENSATION_ZONE_PX 44

// Fixed calibration captured from measured corner touches.
#define TS_SWAP_XY true
#define TS_RAW_TOP_LEFT_X 142
#define TS_RAW_TOP_LEFT_Y 103
#define TS_RAW_TOP_RIGHT_X 139
#define TS_RAW_TOP_RIGHT_Y 890
#define TS_RAW_BOTTOM_RIGHT_X 843
#define TS_RAW_BOTTOM_RIGHT_Y 897
#define TS_RAW_BOTTOM_LEFT_X 915
#define TS_RAW_BOTTOM_LEFT_Y 83

#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0

MCUFRIEND_kbv tft;
TouchScreen touch = TouchScreen(XP, YP, XM, YM, 300);

uint16_t screenWidth;
uint16_t screenHeight;
unsigned long lastTouchLogAtMs = 0;

struct RawTouch
{
  int16_t x;
  int16_t y;
  int16_t z;
};

RawTouch readTouch();
bool isPressed(const RawTouch &point);
int16_t estimateTouchX(const RawTouch &point);
int16_t estimateTouchY(const RawTouch &point);
int16_t applyEdgeCompensation(int16_t value, int16_t zone, int16_t maxOffset, int16_t limit);
int16_t mapTouchX(const RawTouch &point);
int16_t mapTouchY(const RawTouch &point);

void showCenteredText(const char *text, int16_t y, uint8_t size, uint16_t color)
{
  int16_t x1;
  int16_t y1;
  uint16_t textWidth;
  uint16_t textHeight;

  tft.setTextSize(size);
  tft.getTextBounds(text, 0, y, &x1, &y1, &textWidth, &textHeight);
  tft.setCursor((screenWidth - textWidth) / 2, y);
  tft.setTextColor(color);
  tft.print(text);
}

void runColorTest()
{
  const uint16_t colors[] = {RED, GREEN, BLUE, CYAN, MAGENTA, YELLOW, WHITE, BLACK};

  for (uint8_t index = 0; index < sizeof(colors) / sizeof(colors[0]); index++) {
    tft.fillScreen(colors[index]);
    delay(350);
  }
}

RawTouch readTouch()
{
  TSPoint point = touch.getPoint();

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  RawTouch raw = {point.x, point.y, point.z};
  return raw;
}

bool isPressed(const RawTouch &point)
{
  return point.z >= MINPRESSURE && point.z <= MAXPRESSURE;
}

int16_t estimateTouchX(const RawTouch &point)
{
  int16_t rawHorizontal = TS_SWAP_XY ? point.y : point.x;
  long rawLeft = (TS_RAW_TOP_LEFT_Y + TS_RAW_BOTTOM_LEFT_Y) / 2;
  long rawRight = (TS_RAW_TOP_RIGHT_Y + TS_RAW_BOTTOM_RIGHT_Y) / 2;
  long mapped = map(rawHorizontal, rawLeft, rawRight, 0, screenWidth - 1);
  return constrain(mapped, 0, screenWidth - 1);
}

int16_t estimateTouchY(const RawTouch &point)
{
  int16_t rawVertical = TS_SWAP_XY ? point.x : point.y;
  long rawTop = (TS_RAW_TOP_LEFT_X + TS_RAW_TOP_RIGHT_X) / 2;
  long rawBottom = (TS_RAW_BOTTOM_LEFT_X + TS_RAW_BOTTOM_RIGHT_X) / 2;
  long mapped = map(rawVertical, rawTop, rawBottom, 0, screenHeight - 1);
  return constrain(mapped, 0, screenHeight - 1);
}

int16_t applyEdgeCompensation(int16_t value, int16_t zone, int16_t maxOffset, int16_t limit)
{
  if (value >= zone || zone <= 0 || maxOffset <= 0) {
    return constrain(value, 0, limit);
  }

  long distanceToZone = zone - value;
  long taperedOffset = (distanceToZone * distanceToZone * maxOffset) / ((long)zone * zone);
  return constrain(value + taperedOffset, 0, limit);
}

int16_t mapTouchX(const RawTouch &point)
{
  int16_t rawHorizontal = TS_SWAP_XY ? point.y : point.x;
  int16_t estimatedY = estimateTouchY(point);
  long rawLeft = map(estimatedY, 0, screenHeight - 1,
                     TS_RAW_TOP_LEFT_Y, TS_RAW_BOTTOM_LEFT_Y);
  long rawRight = map(estimatedY, 0, screenHeight - 1,
                      TS_RAW_TOP_RIGHT_Y, TS_RAW_BOTTOM_RIGHT_Y);
  long mapped = map(rawHorizontal, rawLeft, rawRight, 0, screenWidth - 1);
  return applyEdgeCompensation(mapped, LEFT_EDGE_COMPENSATION_ZONE_PX,
                               LEFT_EDGE_COMPENSATION_PX, screenWidth - 1);
}

int16_t mapTouchY(const RawTouch &point)
{
  int16_t rawVertical = TS_SWAP_XY ? point.x : point.y;
  int16_t estimatedX = estimateTouchX(point);
  long rawTop = map(estimatedX, 0, screenWidth - 1,
                    TS_RAW_TOP_LEFT_X, TS_RAW_TOP_RIGHT_X);
  long rawBottom = map(estimatedX, 0, screenWidth - 1,
                       TS_RAW_BOTTOM_LEFT_X, TS_RAW_BOTTOM_RIGHT_X);
  long mapped = map(rawVertical, rawTop, rawBottom, 0, screenHeight - 1);
  return applyEdgeCompensation(mapped, TOP_EDGE_COMPENSATION_ZONE_PX,
                               TOP_EDGE_COMPENSATION_PX, screenHeight - 1);
}

void drawTestPattern(uint16_t displayId)
{
  tft.fillScreen(BLACK);
  tft.drawRect(0, 0, screenWidth, screenHeight, WHITE);
  tft.drawLine(0, 0, screenWidth - 1, screenHeight - 1, RED);
  tft.drawLine(screenWidth - 1, 0, 0, screenHeight - 1, GREEN);
  tft.fillCircle(screenWidth / 2, screenHeight / 2, 28, BLUE);
  tft.drawCircle(screenWidth / 2, screenHeight / 2, 42, YELLOW);

  showCenteredText("TP28017 TFT Demo", 12, 2, WHITE);

  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.setCursor(8, 40);
  tft.print("LCD ID: 0x");
  tft.print(displayId, HEX);
  tft.setCursor(8, 54);
  tft.print("Touch screen to draw");
  tft.setCursor(8, 68);
  tft.print("Serial Monitor: raw values");
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("TP28017 resistive touch display demo"));

  uint16_t displayId = tft.readID();
  if (displayId == 0xD3D3 || displayId == 0xFFFF || displayId == 0x0000) {
    Serial.println(F("Unknown display ID; using common ILI9341 fallback 0x9341"));
    displayId = 0x9341;
  }

  tft.begin(displayId);
  tft.setRotation(1);
  screenWidth = tft.width();
  screenHeight = tft.height();

  runColorTest();
  drawTestPattern(displayId);
}

void loop()
{
  RawTouch point = readTouch();

  if (!isPressed(point)) {
    return;
  }

  int16_t x = mapTouchX(point);
  int16_t y = mapTouchY(point);

  unsigned long now = millis();
  if (now - lastTouchLogAtMs >= SERIAL_LOG_INTERVAL_MS) {
    lastTouchLogAtMs = now;
    Serial.print(F("raw x="));
    Serial.print(point.x);
    Serial.print(F(" raw y="));
    Serial.print(point.y);
    Serial.print(F(" pressure="));
    Serial.print(point.z);
    Serial.print(F(" mapped x="));
    Serial.print(x);
    Serial.print(F(" mapped y="));
    Serial.println(y);
  }

  tft.fillCircle(x, y, 3, WHITE);
  tft.fillRect(0, screenHeight - 18, screenWidth, 18, BLACK);
  tft.setCursor(4, screenHeight - 14);
  tft.setTextSize(1);
  tft.setTextColor(YELLOW);
  tft.print("X:");
  tft.print(x);
  tft.print(" Y:");
  tft.print(y);
  tft.print(" P:");
  tft.print(point.z);

  delay(TOUCH_POLL_DELAY_MS);
}