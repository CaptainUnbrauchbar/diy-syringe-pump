#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <TMCStepper.h>
#include <stddef.h>
#include <stdio.h>

#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef PGM_P
#define PGM_P const char *
#endif
#ifndef pgm_read_byte
#define pgm_read_byte(address_short) (*(const uint8_t *)(address_short))
#endif

/********************************************************************
	Basic syringe pump firmware for Arduino UNO R4 Minima
********************************************************************/

// Stepper motor and mechanics
#define NOFMICROSTEPS 64
#define NOFSTEPSPER360 200
#define MAXRPM 180
// Manual syringe-loading jog is loop-limited because STEP pulses are
// generated in software while the touchscreen ADC is being polled. Use a
// separate, much coarser microstep setting here so each emitted step moves
// the carriage farther and the jog feels responsive.
#define STARTUP_JOG_MICROSTEPS 4
#define STARTUP_JOG_MAX_RPM 300
#define INVERTDIRECTION true
#define MMPER360 8.0
#define MOTOR_CURRENT_MA 1200

// Syringe setup
#define DEFAULT_FLOW_ML_PER_HOUR 50.00
#define MIN_FLOW_ML_PER_HOUR 1.00
#define MAX_FLOW_ML_PER_HOUR 600.00
#define FLOW_STEP_ML_PER_HOUR 1.00
#define DEFAULT_FLOW_CENTI_ML_PER_HOUR 5000UL
#define MIN_FLOW_CENTI_ML_PER_HOUR 100UL
#define MAX_FLOW_CENTI_ML_PER_HOUR 60000UL
#define DEFAULT_DELIVERY_ML_PER_CM 3.57
#define MIN_DELIVERY_ML_PER_CM 0.01
#define MAX_DELIVERY_ML_PER_CM 999.99
#define DEFAULT_DELIVERY_CENTI_ML_PER_CM 357UL
#define MIN_DELIVERY_CENTI_ML_PER_CM 1UL
#define MAX_DELIVERY_CENTI_ML_PER_CM 99999UL
#define DEFAULT_SYRINGE_CENTI_ML 5000UL
#define MIN_SYRINGE_CENTI_ML 100UL
#define MAX_SYRINGE_CENTI_ML 10000UL
#define DEFAULT_BOLUS_ML 1.00
#define MIN_BOLUS_CENTI_ML 10UL
#define DEFAULT_BOLUS_ENABLED 1
#define DEFAULT_SOUND_ENABLED 1
#define DEFAULT_MAX_BOLUS_PERCENT 10
#define MIN_MAX_BOLUS_PERCENT 1
#define MAX_MAX_BOLUS_PERCENT 50
#define DEFAULT_STARTUP_JOG_SPEED_PERCENT 50
#define MIN_STARTUP_JOG_SPEED_PERCENT 10
#define MAX_STARTUP_JOG_SPEED_PERCENT 100
#define STARTUP_JOG_SPEED_STEP_PERCENT 10
#define BOLUS_DURATION_MS 10000UL
#define BOLUS_MENU_TIMEOUT_MS 5000UL
#define STATUS_DISPLAY_MS 1200UL
#define MOTOR_TEST_STEP_RATE_HZ 10.0

// Arduino UNO R4 Minima + parallel TFT shield pin plan.
// The TFT shield uses D2-D9 and A0-A4. Its SD-card SPI pins are repurposed.
// TMC2209 UART config replaces the separate DIR and ENABLE pins, but STEP
// pulses are still required; the TMC2209 cannot generate motion from UART only.
#define STEP_PIN 10
// UNO R4 Minima has hardware Serial1 on D0/D1. Move the backward endstop to D11.
#define ENDSTOP_PIN_FORWARD A5
#define ENDSTOP_PIN_BACKWARD 11
#define BUZZER_PIN 13

#define TFT_TOUCH_YP A3
#define TFT_TOUCH_XM A2
#define TFT_TOUCH_YM 9
#define TFT_TOUCH_XP 8

#define TMC2209_UART_BAUD 250000UL
#define TMC2209_UART_DRIVER_ADDRESS 0
#define TMC2209_UART_R_SENSE 0.11f
#define TMC2209_EXPECTED_VERSION 0x21
#define TMC2209_READBACK_ATTEMPTS 3
#define TMC2209_CURRENT_TOLERANCE_MA 100

#define DEBOUNCE_DELAY_MS 50
#define DISPLAY_UPDATE_MS 250
#define STEP_PULSE_US 15
#define TOUCH_MIN_PRESSURE 40
#define TOUCH_MAX_PRESSURE 1023
#define TOUCH_ADC_MAX 1023
#define TOUCH_ANALOG_SAMPLES 4
#define TOUCH_SETTLE_US 150
#define TOUCH_DEBOUNCE_MS DEBOUNCE_DELAY_MS
#define TOUCH_RAW_MARGIN 48
#define TOUCH_STABLE_RAW_DELTA 24
#define TOUCH_STABLE_PRESSURE_DELTA 80
// Bottom action bar. Bigger than before for easier targeting but small enough
// that existing body layouts still fit above it (240 - 64 = 176 px content).
#define TOUCH_BUTTON_BAR_HEIGHT 64
#define ACTION_BAR_HEIGHT TOUCH_BUTTON_BAR_HEIGHT
#define ACTION_HEADER_HEIGHT 28
#define ACTION_TITLE_BACK_WIDTH 56
#define ACTION_PRESS_HOLD_INITIAL_MS 380
#define ACTION_PRESS_HOLD_REPEAT_MS 90
#define MAX_ACTION_SLOTS 5
#define LEFT_EDGE_COMPENSATION_PX 14
#define TOP_EDGE_COMPENSATION_PX 12
#define LEFT_EDGE_COMPENSATION_ZONE_PX 56
#define TOP_EDGE_COMPENSATION_ZONE_PX 44

#define TS_SWAP_XY true
#define TS_RAW_TOP_LEFT_X 142
#define TS_RAW_TOP_LEFT_Y 103
#define TS_RAW_TOP_RIGHT_X 139
#define TS_RAW_TOP_RIGHT_Y 890
#define TS_RAW_BOTTOM_RIGHT_X 843
#define TS_RAW_BOTTOM_RIGHT_Y 897
#define TS_RAW_BOTTOM_LEFT_X 915
#define TS_RAW_BOTTOM_LEFT_Y 83

#define TFT_BLACK 0x0000
#define TFT_WHITE 0xFFFF
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_BLUE 0x001F
#define TFT_CYAN 0x07FF
#define TFT_YELLOW 0xFFE0
#define TFT_DARKGREY 0x7BEF
#define TFT_NAVY 0x000F

// Cached fast pin access: on the Renesas UNO R4 core the register and mask
// widths differ from classic AVR boards, so use UNO R4-sized types here.
typedef volatile uint16_t *FastPortReg;
typedef uint16_t FastPortMask;

class UnoParallelTft : public Adafruit_GFX
{
public:
	UnoParallelTft() : Adafruit_GFX(240, 320)
	{
	}

	uint16_t readID()
	{
		return 0x9341;
	}

	void begin(uint16_t displayId = 0x9341)
	{
		(void)displayId;
		pinMode(LCD_RD_PIN, OUTPUT);
		pinMode(LCD_WR_PIN, OUTPUT);
		pinMode(LCD_CD_PIN, OUTPUT);
		pinMode(LCD_CS_PIN, OUTPUT);
		pinMode(LCD_RESET_PIN, OUTPUT);
		for (uint8_t index = 0; index < 8; index++)
			pinMode(dataPin(index), OUTPUT);

		// Cache port output register addresses and bitmasks for every TFT
		// pin so the hot pixel path can use direct register bit twiddling
		// instead of the (slow) Arduino digitalWrite() function.
		cachePin(LCD_RD_PIN, rdPortReg_, rdPortMask_);
		cachePin(LCD_WR_PIN, wrPortReg_, wrPortMask_);
		cachePin(LCD_CD_PIN, cdPortReg_, cdPortMask_);
		cachePin(LCD_CS_PIN, csPortReg_, csPortMask_);
		cachePin(LCD_RESET_PIN, rstPortReg_, rstPortMask_);
		for (uint8_t index = 0; index < 8; index++)
			cachePin(dataPin(index), dataPortReg_[index], dataPortMask_[index]);

		fastHigh(rdPortReg_, rdPortMask_);
		fastHigh(wrPortReg_, wrPortMask_);
		fastHigh(csPortReg_, csPortMask_);
		fastHigh(rstPortReg_, rstPortMask_);
		delay(5);
		fastLow(rstPortReg_, rstPortMask_);
		delay(20);
		fastHigh(rstPortReg_, rstPortMask_);
		delay(150);

		writeCommand(0x01);
		delay(150);
		writeCommand(0x28);
		writeCommandData(0xCF, (const uint8_t[]){0x00, 0xC1, 0x30}, 3);
		writeCommandData(0xED, (const uint8_t[]){0x64, 0x03, 0x12, 0x81}, 4);
		writeCommandData(0xE8, (const uint8_t[]){0x85, 0x00, 0x78}, 3);
		writeCommandData(0xCB, (const uint8_t[]){0x39, 0x2C, 0x00, 0x34, 0x02}, 5);
		writeCommandData(0xF7, (const uint8_t[]){0x20}, 1);
		writeCommandData(0xEA, (const uint8_t[]){0x00, 0x00}, 2);
		writeCommandData(0xC0, (const uint8_t[]){0x23}, 1);
		writeCommandData(0xC1, (const uint8_t[]){0x10}, 1);
		writeCommandData(0xC5, (const uint8_t[]){0x3E, 0x28}, 2);
		writeCommandData(0xC7, (const uint8_t[]){0x86}, 1);
		writeCommandData(0x3A, (const uint8_t[]){0x55}, 1);
		writeCommandData(0xB1, (const uint8_t[]){0x00, 0x18}, 2);
		writeCommandData(0xB6, (const uint8_t[]){0x08, 0x82, 0x27}, 3);
		writeCommandData(0xF2, (const uint8_t[]){0x00}, 1);
		writeCommandData(0x26, (const uint8_t[]){0x01}, 1);
		writeCommandData(0xE0, (const uint8_t[]){0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15);
		writeCommandData(0xE1, (const uint8_t[]){0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15);
		setRotation(0);
		writeCommand(0x11);
		delay(120);
		writeCommand(0x29);
		delay(20);
	}

	void drawPixel(int16_t x, int16_t y, uint16_t color) override
	{
		if (x < 0 || y < 0 || x >= width() || y >= height())
			return;

		beginWrite();
		setAddressWindow(x, y, x, y);
		writeData16(color);
		endWrite();
	}

	void fillRect(int16_t x, int16_t y, int16_t widthValue, int16_t heightValue, uint16_t color) override
	{
		if (widthValue <= 0 || heightValue <= 0 || x >= width() || y >= height())
			return;
		if (x < 0)
		{
			widthValue += x;
			x = 0;
		}
		if (y < 0)
		{
			heightValue += y;
			y = 0;
		}
		if (x + widthValue > width())
			widthValue = width() - x;
		if (y + heightValue > height())
			heightValue = height() - y;
		if (widthValue <= 0 || heightValue <= 0)
			return;

		beginWrite();
		setAddressWindow(x, y, x + widthValue - 1, y + heightValue - 1);
		// CD is already left HIGH at the end of setAddressWindow() so every
		// subsequent byte is treated as pixel data without re-driving CD.
		uint32_t pixelCount = (uint32_t)widthValue * (uint32_t)heightValue;
		uint8_t hi = (uint8_t)(color >> 8);
		uint8_t lo = (uint8_t)(color & 0xFF);
		if (hi == lo)
		{
			// Solid fill where both color bytes are identical (e.g. BLACK,
			// WHITE, DARKGREY). Drive the data pins once, then just pulse
			// WR twice per pixel - no further data-pin updates required.
			driveDataPins(hi);
			uint32_t pulses = pixelCount << 1;
			while (pulses--)
				pulseWr();
		}
		else
		{
			while (pixelCount--)
			{
				driveDataPins(hi);
				pulseWr();
				driveDataPins(lo);
				pulseWr();
			}
		}
		endWrite();
	}

	void fillScreen(uint16_t color) override
	{
		fillRect(0, 0, width(), height(), color);
	}

	void drawFastHLine(int16_t x, int16_t y, int16_t widthValue, uint16_t color) override
	{
		fillRect(x, y, widthValue, 1, color);
	}

	void drawFastVLine(int16_t x, int16_t y, int16_t heightValue, uint16_t color) override
	{
		fillRect(x, y, 1, heightValue, color);
	}

	void invalidateDataBus()
	{
		dataValueValid_ = false;
	}

	void setRotation(uint8_t rotation) override
	{
		rotation &= 3;
		Adafruit_GFX::setRotation(rotation);
		uint8_t madctl = 0x48;
		switch (rotation)
		{
		case 1:
			madctl = 0x28;
			break;
		case 2:
			madctl = 0x88;
			break;
		case 3:
			madctl = 0xE8;
			break;
		default:
			madctl = 0x48;
			break;
		}
		writeCommandData(0x36, &madctl, 1);
	}

private:
	static const uint8_t LCD_RD_PIN = A0;
	static const uint8_t LCD_WR_PIN = A1;
	static const uint8_t LCD_CD_PIN = A2;
	static const uint8_t LCD_CS_PIN = A3;
	static const uint8_t LCD_RESET_PIN = A4;

	uint8_t dataPin(uint8_t index)
	{
		static const uint8_t pins[] = {8, 9, 2, 3, 4, 5, 6, 7};
		return pins[index];
	}

	static inline void fastHigh(FastPortReg reg, FastPortMask mask)
	{
		*reg |= mask;
	}

	static inline void fastLow(FastPortReg reg, FastPortMask mask)
	{
		*reg &= (FastPortMask)~mask;
	}

	static void cachePin(uint8_t pin, FastPortReg &regOut, FastPortMask &maskOut)
	{
		regOut = (FastPortReg)portOutputRegister(digitalPinToPort(pin));
		maskOut = (FastPortMask)digitalPinToBitMask(pin);
	}

	inline void pulseWr()
	{
		fastLow(wrPortReg_, wrPortMask_);
		fastHigh(wrPortReg_, wrPortMask_);
	}

	inline void driveDataPins(uint8_t value)
	{
		// Only touch data pins whose value actually changes. Most UI fills
		// alternate between two byte values (e.g. 0x00/0x0F for NAVY), so this
		// avoids redundant port writes on every pixel while preserving the
		// generic pin mapping for both AVR Uno and UNO R4.
		uint8_t changed = dataValueValid_ ? (uint8_t)(value ^ lastDataValue_) : 0xFF;
		if (!changed)
			return;
		if (changed & 0x01) { if (value & 0x01) fastHigh(dataPortReg_[0], dataPortMask_[0]); else fastLow(dataPortReg_[0], dataPortMask_[0]); }
		if (changed & 0x02) { if (value & 0x02) fastHigh(dataPortReg_[1], dataPortMask_[1]); else fastLow(dataPortReg_[1], dataPortMask_[1]); }
		if (changed & 0x04) { if (value & 0x04) fastHigh(dataPortReg_[2], dataPortMask_[2]); else fastLow(dataPortReg_[2], dataPortMask_[2]); }
		if (changed & 0x08) { if (value & 0x08) fastHigh(dataPortReg_[3], dataPortMask_[3]); else fastLow(dataPortReg_[3], dataPortMask_[3]); }
		if (changed & 0x10) { if (value & 0x10) fastHigh(dataPortReg_[4], dataPortMask_[4]); else fastLow(dataPortReg_[4], dataPortMask_[4]); }
		if (changed & 0x20) { if (value & 0x20) fastHigh(dataPortReg_[5], dataPortMask_[5]); else fastLow(dataPortReg_[5], dataPortMask_[5]); }
		if (changed & 0x40) { if (value & 0x40) fastHigh(dataPortReg_[6], dataPortMask_[6]); else fastLow(dataPortReg_[6], dataPortMask_[6]); }
		if (changed & 0x80) { if (value & 0x80) fastHigh(dataPortReg_[7], dataPortMask_[7]); else fastLow(dataPortReg_[7], dataPortMask_[7]); }
		lastDataValue_ = value;
		dataValueValid_ = true;
	}

	void beginWrite()
	{
		fastLow(csPortReg_, csPortMask_);
	}

	void endWrite()
	{
		fastHigh(csPortReg_, csPortMask_);
	}

	void writeCommand(uint8_t command)
	{
		beginWrite();
		writeCommandByte(command);
		endWrite();
	}

	void writeCommandData(uint8_t command, const uint8_t *data, uint8_t dataLength)
	{
		beginWrite();
		writeCommandByte(command);
		while (dataLength--)
			writeData8(*data++);
		endWrite();
	}

	void writeCommandByte(uint8_t command)
	{
		fastLow(cdPortReg_, cdPortMask_);
		write8(command);
	}

	void writeData8(uint8_t data)
	{
		fastHigh(cdPortReg_, cdPortMask_);
		write8(data);
	}

	void writeData16(uint16_t data)
	{
		writeData8((uint8_t)(data >> 8));
		writeData8((uint8_t)(data & 0xFF));
	}

	void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
	{
		writeCommandByte(0x2A);
		writeData16(x0);
		writeData16(x1);
		writeCommandByte(0x2B);
		writeData16(y0);
		writeData16(y1);
		writeCommandByte(0x2C);
		fastHigh(cdPortReg_, cdPortMask_);
	}

	void write8(uint8_t value)
	{
		driveDataPins(value);
		pulseWr();
	}

	FastPortReg dataPortReg_[8];
	FastPortMask dataPortMask_[8];
	FastPortReg rdPortReg_;
	FastPortMask rdPortMask_;
	FastPortReg wrPortReg_;
	FastPortMask wrPortMask_;
	FastPortReg cdPortReg_;
	FastPortMask cdPortMask_;
	FastPortReg csPortReg_;
	FastPortMask csPortMask_;
	FastPortReg rstPortReg_;
	FastPortMask rstPortMask_;
	uint8_t lastDataValue_ = 0;
	bool dataValueValid_ = false;
};

#define SETTINGS_EEPROM_ADDRESS 0
#define SETTINGS_MAGIC 0x5046
#define SETTINGS_VERSION 9
#define SETTINGS_MENU_COUNT 14
#define SETTINGS_MENU_DELIVERY 0
#define SETTINGS_MENU_SYRINGE 1
#define SETTINGS_MENU_BOLUS 2
#define SETTINGS_MENU_SOUND 3
#define SETTINGS_MENU_MAX_BOLUS 4
#define SETTINGS_MENU_MOTOR_INVERT 5
#define SETTINGS_MENU_STARTUP_SPEED 6
#define SETTINGS_MENU_TEST_MODE 7
#define SETTINGS_MENU_PRESSURE 8
#define SETTINGS_MENU_PRESSURE_SCALE 9
#define SETTINGS_MENU_PRESSURE_ALARM 10
#define SETTINGS_MENU_PRESSURE_ZERO 11
#define SETTINGS_MENU_MOTOR_TEST 12
#define SETTINGS_MENU_SUPPORT 13
#define PRESSURE_SCALE_MIN 1
#define PRESSURE_SCALE_MAX 20
#define PRESSURE_SCALE_DEFAULT 10
#define PRESSURE_SCALE_REFERENCE 10
#define PRESSURE_ALARM_MIN 1
#define PRESSURE_ALARM_MAX 10
#define PRESSURE_ALARM_DEFAULT 7
#define PRESSURE_EMPTY_BAR_SG 14
#define PRESSURE_FULL_BAR_SG 0
#define PRESSURE_BASELINE_DEFAULT PRESSURE_EMPTY_BAR_SG
#define PRESSURE_SG_MASK 0x01FE
#define PRESSURE_SG_MAX 510
// Observed useful SG range for this pump: SG=14 is the empty bar reference,
// SG=0 is the full bar point. Scale 10 maps exactly across that 14-unit span;
// scales 11..20 compress the span further for extra sensitivity.
#define PRESSURE_ALARM_SUSTAIN_TICKS 3
// Exponential moving average shift used to smooth SG_RESULT readings.
// alpha = 1 / (1 << shift); larger shift = more smoothing.
#define PRESSURE_SAMPLE_FILTER_SHIFT 3
// Number of SG samples collected after pump start to derive an automatic
// unloaded baseline when the user has not run a manual zero calibration.
#define PRESSURE_AUTO_BASELINE_SAMPLES 16
// Number of samples averaged by the manual calibratePressureBaseline() routine.
#define PRESSURE_MANUAL_BASELINE_SAMPLES 16
// Spacing in ms between the manual baseline samples.
#define PRESSURE_MANUAL_BASELINE_SAMPLE_MS 60
#define SETTINGS_SUPPORT_TIMEOUT_MS 10000UL
#define STARTUP_SPLASH_MS 1500UL
#define MAIN_MENU_COUNT 3
#define MAIN_MENU_START_INFUSION 0
#define MAIN_MENU_LOAD_SYRINGE 1
#define MAIN_MENU_SETTINGS 2
#define START_MENU_COUNT 2
#define START_MENU_VOLUME_TIME 0
#define START_MENU_RATE 1

#if SETTINGS_MENU_COUNT != 14
#error SETTINGS_MENU_COUNT must match the explicit SETTINGS_MENU_* indices.
#endif

#if MAIN_MENU_COUNT != 3
#error MAIN_MENU_COUNT must match the explicit MAIN_MENU_* indices.
#endif

#if START_MENU_COUNT != 2
#error START_MENU_COUNT must match the explicit START_MENU_* indices.
#endif

const char PROMPT_INFO_LINE_1[] PROGMEM = "Jetzt Spritze einlegen";
const char PROMPT_RATE[] PROGMEM = "Infusionsrate";
const char PROMPT_VOLUME[] PROGMEM = "Zielvolumen";
const char PROMPT_TIME[] PROGMEM = "Zielzeit";
const char PROMPT_DELIVERY[] PROGMEM = "Fluessigkeit (ml/cm)";
const char PROMPT_SYRINGE[] PROGMEM = "Eingelegte Spritze";
const char SUPPORT_DISPLAY_LINE_1[] = "You can solve";
const char SUPPORT_DISPLAY_LINE_2[] = "it yourself! :)";

struct PersistedSettings
{
	uint16_t magic;
	uint8_t version;
	uint8_t motorDirectionInverted;
	uint32_t deliveryCentiMlPerCm;
	uint32_t syringeCentiMl;
	uint8_t bolusEnabled;
	uint8_t soundEnabled;
	uint8_t maxBolusPercent;
	uint8_t startupJogSpeedPercent;
	uint8_t testModeEnabled;
	uint8_t pressureMonitorEnabled;
	uint8_t pressureScale;
	uint8_t pressureAlarmLevel;
	uint16_t pressureBaseline;
	uint16_t crc;
};

struct PersistedSettingsVersion8
{
	uint16_t magic;
	uint8_t version;
	uint8_t motorDirectionInverted;
	uint32_t deliveryCentiMlPerCm;
	uint32_t syringeCentiMl;
	uint8_t bolusEnabled;
	uint8_t soundEnabled;
	uint8_t maxBolusPercent;
	uint8_t startupJogSpeedPercent;
	uint8_t testModeEnabled;
	uint16_t crc;
};

struct PersistedSettingsVersion7
{
	uint16_t magic;
	uint8_t version;
	uint8_t motorDirectionInverted;
	uint32_t deliveryCentiMlPerCm;
	uint32_t syringeCentiMl;
	uint8_t bolusEnabled;
	uint8_t soundEnabled;
	uint8_t maxBolusPercent;
	uint8_t startupJogSpeedPercent;
	uint16_t crc;
};

struct PersistedSettingsVersion6
{
	uint16_t magic;
	uint8_t version;
	uint8_t motorDirectionInverted;
	uint32_t deliveryCentiMlPerCm;
	uint32_t syringeCentiMl;
	uint8_t bolusEnabled;
	uint8_t soundEnabled;
	uint8_t maxBolusPercent;
	uint16_t crc;
};

UnoParallelTft tft;
uint16_t tftWidth = 0;
uint16_t tftHeight = 0;
const void *lastScreenFrameKey = NULL;

// Action bar (bottom touch button bar) is now context-sensitive. Each screen
// declares the slots it needs via setActionBar(). Slots are diff-rendered to
// avoid the previous full-bar repaint on every screen change.
enum ActionSlotMode : uint8_t
{
	ACTION_MODE_RELEASE = 0, // fires on touch-up (slid-off cancels)
	ACTION_MODE_REPEAT = 1,  // fires on touch-down + auto-repeats while held
	ACTION_MODE_HOLD = 2     // fires every tick while held (motor jog)
};

struct ActionSlot
{
	const char *glyph;
	const char *sublabel;
	uint16_t fillColor;
	uint16_t textColor;
	uint8_t action;      // Button enum value
	uint8_t mode;        // ActionSlotMode
};

struct ActionBar
{
	uint8_t count;
	ActionSlot slots[MAX_ACTION_SLOTS];
};

ActionBar currentActionBar = {0, {}};
ActionBar renderedActionBar = {0xFF, {}}; // sentinel: force first render
int8_t pressedSlot = -1;                  // visual press state
int8_t renderedPressedSlot = -1;
bool pressedTitleBack = false;
uint32_t pressStartMs = 0;
uint32_t lastRepeatMs = 0;
bool currentScreenHasBack = false;
bool renderedHasBack = false;

void initializeDisplay()
{
	uint16_t displayId = tft.readID();
	if (displayId == 0xD3D3 || displayId == 0xFFFF || displayId == 0x0000)
		displayId = 0x9341;
	tft.begin(displayId);
	tft.setRotation(1);
	tftWidth = tft.width();
	tftHeight = tft.height();
}

void printProgmem(PGM_P text)
{
	char character;
	while ((character = pgm_read_byte(text++)) != '\0')
		tft.write((uint8_t)character);
}

// ----- Action bar rendering ---------------------------------------------
static bool actionSlotsEqual(const ActionSlot &a, const ActionSlot &b)
{
	return a.glyph == b.glyph && a.sublabel == b.sublabel && a.fillColor == b.fillColor && a.textColor == b.textColor && a.action == b.action && a.mode == b.mode;
}

static void actionSlotGeometry(uint8_t i, int16_t &left, int16_t &width)
{
	uint8_t n = currentActionBar.count;
	if (n == 0)
	{
		left = 0;
		width = tftWidth;
		return;
	}
	int16_t slotW = tftWidth / n;
	left = (int16_t)i * slotW;
	width = (i == n - 1) ? (tftWidth - left) : slotW;
}

void drawActionSlot(uint8_t i, bool pressed)
{
	if (i >= currentActionBar.count)
		return;
	const ActionSlot &s = currentActionBar.slots[i];
	int16_t left, width;
	actionSlotGeometry(i, left, width);
	int16_t top = tftHeight - ACTION_BAR_HEIGHT;
	uint16_t fill = pressed ? TFT_YELLOW : s.fillColor;
	uint16_t fg = pressed ? TFT_BLACK : s.textColor;
	tft.fillRect(left, top, width, ACTION_BAR_HEIGHT, fill);
	tft.drawRect(left, top, width, ACTION_BAR_HEIGHT, pressed ? TFT_WHITE : TFT_WHITE);
	if (pressed)
	{
		// Extra inner outline to make the pressed state pop visually.
		tft.drawRect(left + 1, top + 1, width - 2, ACTION_BAR_HEIGHT - 2, TFT_BLACK);
	}
	if (s.glyph)
	{
		uint8_t size = 3;
		int16_t textW = (int16_t)strlen(s.glyph) * 6 * size;
		int16_t textY = top + (s.sublabel ? 6 : (ACTION_BAR_HEIGHT - 8 * size) / 2);
		tft.setTextSize(size);
		tft.setTextColor(fg, fill);
		tft.setCursor(left + (width - textW) / 2, textY);
		tft.print(s.glyph);
	}
	if (s.sublabel)
	{
		uint8_t size = 1;
		int16_t textW = (int16_t)strlen(s.sublabel) * 6 * size;
		tft.setTextSize(size);
		tft.setTextColor(fg, fill);
		tft.setCursor(left + (width - textW) / 2, top + ACTION_BAR_HEIGHT - 12);
		tft.print(s.sublabel);
	}
}

void drawTitleBack(bool present, bool pressed)
{
	if (!present)
		return;
	uint16_t fill = pressed ? TFT_YELLOW : TFT_NAVY;
	uint16_t fg = pressed ? TFT_BLACK : TFT_WHITE;
	tft.fillRect(0, 0, ACTION_TITLE_BACK_WIDTH, ACTION_HEADER_HEIGHT, fill);
	tft.setTextSize(2);
	tft.setTextColor(fg, fill);
	tft.setCursor(8, 7);
	tft.print('<');
}

void renderActionBar(bool force)
{
	int16_t top = tftHeight - ACTION_BAR_HEIGHT;
	if (currentActionBar.count == 0)
	{
		if (force || renderedActionBar.count != 0)
		{
			tft.fillRect(0, top, tftWidth, ACTION_BAR_HEIGHT, TFT_BLACK);
			renderedActionBar.count = 0;
			renderedPressedSlot = -1;
		}
		return;
	}

	bool layoutChanged = force || renderedActionBar.count != currentActionBar.count;
	if (layoutChanged)
		tft.fillRect(0, top, tftWidth, ACTION_BAR_HEIGHT, TFT_BLACK);

	for (uint8_t i = 0; i < currentActionBar.count; i++)
	{
		bool slotChanged = layoutChanged || !actionSlotsEqual(currentActionBar.slots[i], renderedActionBar.slots[i]);
		bool pressedChanged = (renderedPressedSlot == (int8_t)i) != (pressedSlot == (int8_t)i);
		if (slotChanged || pressedChanged)
			drawActionSlot(i, pressedSlot == (int8_t)i);
	}
	renderedActionBar = currentActionBar;
	renderedPressedSlot = pressedSlot;
}

void setActionBar(const ActionBar &bar, bool hasBack)
{
	// If layout changes mid-press, cancel the press to avoid stale state.
	if (pressedSlot >= 0)
	{
		bool changed = bar.count != currentActionBar.count;
		if (!changed && pressedSlot < (int8_t)bar.count)
			changed = !actionSlotsEqual(bar.slots[pressedSlot], currentActionBar.slots[pressedSlot]);
		if (changed)
			pressedSlot = -1;
	}
	if (pressedTitleBack && !hasBack)
		pressedTitleBack = false;
	currentActionBar = bar;
	currentScreenHasBack = hasBack;
	renderActionBar(false);
	if (hasBack != renderedHasBack)
	{
		drawTitleBack(hasBack, pressedTitleBack);
		if (!hasBack)
			tft.fillRect(0, 0, ACTION_TITLE_BACK_WIDTH, ACTION_HEADER_HEIGHT, TFT_NAVY);
		renderedHasBack = hasBack;
	}
}

// Bar layout helpers ------------------------------------------------------
static ActionBar makeBar2(ActionSlot a, ActionSlot b)
{
	ActionBar bar = {2, {a, b}};
	return bar;
}
static ActionBar makeBar3(ActionSlot a, ActionSlot b, ActionSlot c)
{
	ActionBar bar = {3, {a, b, c}};
	return bar;
}
static ActionBar makeBar4(ActionSlot a, ActionSlot b, ActionSlot c, ActionSlot d)
{
	ActionBar bar = {4, {a, b, c, d}};
	return bar;
}
static ActionBar makeBar5(ActionSlot a, ActionSlot b, ActionSlot c, ActionSlot d, ActionSlot e)
{
	ActionBar bar = {5, {a, b, c, d, e}};
	return bar;
}
static ActionBar makeBar1(ActionSlot a)
{
	ActionBar bar = {1, {a}};
	return bar;
}
static ActionBar makeBarEmpty()
{
	ActionBar bar = {0, {}};
	return bar;
}

// ----- Screen frame helpers ---------------------------------------------
// Returns true the first time a screen frame is drawn (i.e. when the title
// has changed since the last call). Callers can use this to know whether
// a full redraw of body content is needed; when false the chrome (controls
// bar + header bar) is already on-screen and content can be partially
// refreshed without flicker.
bool beginScreen(const __FlashStringHelper *title, bool hasBack = false)
{
	if (lastScreenFrameKey != title)
	{
		// Clear only the body strip (below the header). The header row is
		// painted with TFT_NAVY immediately after, so clearing it to black
		// first would be wasted work and contributes to the visible
		// scan-line during screen transitions.
		tft.fillRect(0, ACTION_HEADER_HEIGHT, tftWidth,
			tftHeight - ACTION_BAR_HEIGHT - ACTION_HEADER_HEIGHT, TFT_BLACK);
		tft.fillRect(0, 0, tftWidth, ACTION_HEADER_HEIGHT, TFT_NAVY);
		tft.setTextSize(2);
		tft.setTextColor(TFT_WHITE, TFT_NAVY);
		tft.setCursor(hasBack ? (ACTION_TITLE_BACK_WIDTH + 8) : 8, 7);
		tft.print(title);
		lastScreenFrameKey = title;
		renderedHasBack = false; // force back glyph re-paint after header redraw
		return true;
	}
	return false;
}

bool beginScreenP(PGM_P title, bool hasBack = false)
{
	if (lastScreenFrameKey != title)
	{
		tft.fillRect(0, ACTION_HEADER_HEIGHT, tftWidth,
			tftHeight - ACTION_BAR_HEIGHT - ACTION_HEADER_HEIGHT, TFT_BLACK);
		tft.fillRect(0, 0, tftWidth, ACTION_HEADER_HEIGHT, TFT_NAVY);
		tft.setTextSize(2);
		tft.setTextColor(TFT_WHITE, TFT_NAVY);
		tft.setCursor(hasBack ? (ACTION_TITLE_BACK_WIDTH + 8) : 8, 7);
		printProgmem(title);
		lastScreenFrameKey = title;
		renderedHasBack = false;
		return true;
	}
	return false;
}

// Chromeless screen variant for the startup splash. Just clears the display
// and tracks the frame key so a re-entry does not blink. frameKey must be a
// stable pointer (typically the address of a function or PROGMEM string).
bool beginCleanScreen(const void *frameKey)
{
	if (lastScreenFrameKey != frameKey)
	{
		tft.fillScreen(TFT_BLACK);
		lastScreenFrameKey = frameKey;
		renderedHasBack = false;
		renderedActionBar.count = 0xFF; // force redraw next time bar is set
		return true;
	}
	return false;
}

void printAt(int16_t x, int16_t y, uint8_t size, uint16_t color, const __FlashStringHelper *text)
{
	tft.setTextSize(size);
	tft.setTextColor(color, TFT_BLACK);
	tft.setCursor(x, y);
	tft.print(text);
}

void printAtText(int16_t x, int16_t y, uint8_t size, uint16_t color, const char *text)
{
	tft.setTextSize(size);
	tft.setTextColor(color, TFT_BLACK);
	tft.setCursor(x, y);
	tft.print(text);
}

void beginMenuItem(uint8_t row, bool selected)
{
	int16_t y = 38 + row * 34;
	uint16_t background = selected ? TFT_DARKGREY : TFT_BLACK;
	tft.fillRect(8, y, tftWidth - 16, 30, background);
	tft.drawRect(8, y, tftWidth - 16, 30, selected ? TFT_YELLOW : TFT_DARKGREY);
	tft.setTextSize(2);
	tft.setTextColor(selected ? TFT_YELLOW : TFT_WHITE, background);
	tft.setCursor(16, y + 8);
}

void drawMenuItem(uint8_t row, bool selected, const __FlashStringHelper *label)
{
	beginMenuItem(row, selected);
	tft.print(label);
}

// Internal diff-based renderer used by drawValueScreen / drawValueScreenP.
// Repaints only the character cells whose digit actually changed and only
// the underline rect that moved, eliminating the full-line flicker that
// used to happen on every cursor or digit change.
static void renderValueScreenBody(bool fullFrame, const char *value, uint8_t cursorColumn, bool cursorVisible)
{
	static char lastValue[24];
	static uint8_t lastCursorColumn = 0xFF;
	static bool lastCursorVisible = false;
	static bool cacheValid = false;

	if (fullFrame || !cacheValid)
	{
		// First paint on this screen frame: wipe the body regions, draw
		// the static hint once, and invalidate the per-character cache so
		// every glyph is forced to repaint below.
		tft.fillRect(18, 78, tftWidth - 36, 24, TFT_BLACK);
		tft.fillRect(18, 106, tftWidth - 36, 4, TFT_BLACK);
		tft.fillRect(18, 138, tftWidth - 36, 10, TFT_BLACK);
		printAt(18, 138, 1, TFT_CYAN, F("< > Stelle   +/- Wert   OK"));
		for (uint8_t i = 0; i < sizeof(lastValue); i++)
			lastValue[i] = '\0';
		lastCursorColumn = 0xFF;
		lastCursorVisible = false;
		cacheValid = true;
	}

	// Per-character diff. Size-3 glyphs occupy an 18x24 cell, and rendering
	// with setTextColor(fg, bg) atomically overwrites the entire cell, so
	// we can update a single digit without any visible flicker and without
	// touching its unchanged neighbours.
	uint8_t maxCols = (uint8_t)((tftWidth - 36) / 18);
	if (maxCols > (uint8_t)(sizeof(lastValue) - 1))
		maxCols = (uint8_t)(sizeof(lastValue) - 1);
	uint8_t valueLen = (uint8_t)strlen(value);
	tft.setTextSize(3);
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	for (uint8_t c = 0; c < maxCols; c++)
	{
		char newCh = c < valueLen ? value[c] : ' ';
		if (lastValue[c] != newCh)
		{
			int16_t x = 18 + (int16_t)c * 18;
			tft.setCursor(x, 78);
			tft.write((uint8_t)newCh);
			lastValue[c] = newCh;
		}
	}

	// Underline cursor diff: clear only the old rect and draw the new one.
	if (cursorVisible != lastCursorVisible || cursorColumn != lastCursorColumn)
	{
		if (lastCursorVisible && lastCursorColumn < maxCols)
		{
			int16_t oldX = 18 + (int16_t)lastCursorColumn * 18;
			tft.fillRect(oldX, 106, 16, 3, TFT_BLACK);
		}
		if (cursorVisible)
		{
			int16_t newX = 18 + (int16_t)cursorColumn * 18;
			tft.fillRect(newX, 106, 16, 3, TFT_YELLOW);
		}
		lastCursorVisible = cursorVisible;
		lastCursorColumn = cursorColumn;
	}
}

void drawValueScreen(const __FlashStringHelper *title, const char *value, uint8_t cursorColumn, bool cursorVisible, bool hasBack = false)
{
	bool fullFrame = beginScreen(title, hasBack);
	renderValueScreenBody(fullFrame, value, cursorColumn, cursorVisible);
}

void drawValueScreenP(PGM_P title, const char *value, uint8_t cursorColumn, bool cursorVisible, bool hasBack = false)
{
	bool fullFrame = beginScreenP(title, hasBack);
	renderValueScreenBody(fullFrame, value, cursorColumn, cursorVisible);
}

HardwareSerial &tmc2209Serial = Serial1;
TMC2209Stepper tmc2209Driver(&tmc2209Serial, TMC2209_UART_R_SENSE, TMC2209_UART_DRIVER_ADDRESS);

enum Button
{
	BUTTON_RIGHT,
	BUTTON_UP,
	BUTTON_DOWN,
	BUTTON_LEFT,
	BUTTON_SELECT,
	BUTTON_BACK,
	BUTTON_NONE
};

struct RawTouch
{
	int16_t x;
	int16_t y;
	int16_t z;
};

enum PumpStopReason
{
	STOP_MANUAL,
	STOP_ENDSTOP,
	STOP_TIMER_ERROR,
	STOP_TARGET_VOLUME
};

enum UiScreen
{
	SCREEN_STARTUP_SPLASH,
	SCREEN_STARTUP_INFO,
	SCREEN_MAIN_MENU,
	SCREEN_PUMP_RUNNING,
	SCREEN_START_MENU,
	SCREEN_SETTINGS_MENU,
	SCREEN_EDIT_RATE,
	SCREEN_EDIT_VOLUME,
	SCREEN_EDIT_TIME,
	SCREEN_EDIT_DELIVERY,
	SCREEN_EDIT_SYRINGE,
	SCREEN_EDIT_MAX_BOLUS,
	SCREEN_EDIT_STARTUP_MOTOR_SPEED,
	SCREEN_SETTINGS_SUPPORT,
	SCREEN_SUPPORT_INFO,
	SCREEN_EDIT_BOLUS,
	SCREEN_CONFIRM_BOLUS,
	SCREEN_CONFIRM_INFUSION
};

volatile int32_t stepCounter = 0;
volatile bool stepPulseHigh = false;
volatile bool motorClockRunning = false;
volatile uint32_t motorClockHighPulseUs = STEP_PULSE_US;
volatile uint32_t motorClockLowPulseUs = 0;
volatile uint32_t motorClockNextTransitionMicros = 0;
volatile bool endstopInterruptLatched = false;
volatile bool motorClockDirectionForward = true;
volatile bool motorClockStopOnAnyEndstop = true;

float flowMlPerHour = DEFAULT_FLOW_ML_PER_HOUR;
float baseInfusionFlowMlPerHour = DEFAULT_FLOW_ML_PER_HOUR;
float targetVolumeMl = 50.00;
uint32_t targetTimeSeconds = 0;
bool targetVolumeEnabled = false;
float deliveryMlPerCm = DEFAULT_DELIVERY_ML_PER_CM;
uint32_t syringeCentiMl = DEFAULT_SYRINGE_CENTI_ML;
bool motorDirectionInverted = INVERTDIRECTION;
bool bolusEnabled = DEFAULT_BOLUS_ENABLED == 1;
bool soundEnabled = DEFAULT_SOUND_ENABLED == 1;
bool testModeEnabled = false;
bool pressureMonitorEnabled = true;
uint8_t pressureScale = PRESSURE_SCALE_DEFAULT;
uint8_t pressureAlarmLevel = PRESSURE_ALARM_DEFAULT;
uint16_t pressureBaseline = PRESSURE_BASELINE_DEFAULT;
uint16_t pressureCurrentSg = PRESSURE_SG_MAX;
uint8_t pressureCurrentBarPercent = 0;
uint8_t pressureHighTicks = 0;
uint16_t pressureFilteredSg = 0;
bool pressureFilteredSgValid = false;
uint32_t pressureAutoBaselineSum = 0;
uint8_t pressureAutoBaselineCount = 0;
bool pressureAutoBaselineDone = false;
uint32_t pressureLastAlarmBeepMillis = 0;
uint8_t maxBolusPercent = DEFAULT_MAX_BOLUS_PERCENT;
uint8_t startupJogSpeedPercent = DEFAULT_STARTUP_JOG_SPEED_PERCENT;
bool bolusActive = false;
bool settingsResetToDefaults = false;
uint32_t bolusStartMillis = 0;
uint32_t bolusDurationMillis = BOLUS_DURATION_MS;
uint32_t pendingBolusDurationMillis = BOLUS_DURATION_MS;
float pendingBolusFlowMlPerHour = 0.0;
uint32_t lastBolusInputMillis = 0;
uint32_t editBolusCentiMl = 100;
bool pumpRunning = false;
bool pendingInfusionTargetVolumeEnabled = false;
uint32_t pumpStartMillis = 0;
uint32_t lastDisplayMillis = 0;
uint32_t startupScreenStartMillis = 0;
bool startupInfoFromMainMenu = false;
UiScreen currentScreen = SCREEN_STARTUP_SPLASH;
uint8_t mainMenuIndex = 0;
uint8_t startMenuIndex = 0;
uint8_t settingsMenuIndex = 0;
uint32_t editRateCentiMlPerHour = DEFAULT_FLOW_CENTI_ML_PER_HOUR;
uint32_t editVolumeCentiMl = 5000;
uint32_t editTimeSeconds = 0;
uint32_t editDeliveryCentiMlPerCm = 1000;
uint32_t editSyringeCentiMl = DEFAULT_SYRINGE_CENTI_ML;
uint8_t editMaxBolusPercent = DEFAULT_MAX_BOLUS_PERCENT;
uint8_t editStartupJogSpeedPercent = DEFAULT_STARTUP_JOG_SPEED_PERCENT;
uint8_t editCursorIndex = 0;
uint32_t settingsSupportStartMillis = 0;
bool motorTestRunning = false;
bool motorTestDirectionForward = true;
bool startupJogRunning = false;
bool startupJogDirectionForward = true;
bool tmc2209UartConfigured = false;
bool tmc2209UartVerified = false;
bool tmc2209UartLibraryPresent = true;
bool tmc2209WriteOnlyMode = false;
bool tmc2209Ms1High = false;
bool tmc2209Ms2High = false;
uint8_t tmc2209IfcCount = 0;
uint8_t tmc2209Version = 0;
uint8_t tmc2209ConnectionStatus = 0xFF;
uint8_t tmc2209DetectedAddress = 0xFF;
uint16_t tmc2209ReadbackMicrosteps = 0;
uint16_t tmc2209ReadbackCurrentMa = 0;
bool statusActive = false;
bool alarmActive = false;
bool statusRequiresAcknowledge = false;
uint32_t statusUntilMillis = 0;
UiScreen statusReturnScreen = SCREEN_MAIN_MENU;

struct BuzzerBeep
{
	uint16_t onMs;
	uint16_t offMs;
};

const uint8_t BUZZER_QUEUE_CAPACITY = 8;
BuzzerBeep buzzerQueue[BUZZER_QUEUE_CAPACITY];
uint8_t buzzerQueueHead = 0;
uint8_t buzzerQueueTail = 0;
uint8_t buzzerQueueCount = 0;
bool buzzerOutputActive = false;
bool buzzerOverflowPending = false;
uint16_t buzzerCurrentOffMs = 0;
uint32_t buzzerNextTransitionMillis = 0;

Button readButton();
Button readButtonPress();
RawTouch readTouch();
bool isPressed(const RawTouch &point);
int16_t readTouchAxisX();
int16_t readTouchAxisY();
int16_t readTouchPressure();
int16_t averageAnalogRead(uint8_t pin);
void restoreTouchSharedPins();
int16_t estimateTouchX(const RawTouch &point);
int16_t estimateTouchY(const RawTouch &point);
int16_t applyEdgeCompensation(int16_t value, int16_t zone, int16_t maxOffset, int16_t limit);
int16_t mapTouchX(const RawTouch &point);
int16_t mapTouchY(const RawTouch &point);
Button buttonFromTouch(int16_t x, int16_t y);
void showSplash();
void showInfoScreen();
void updateStartupScreens(Button button);
void finishStartupInfo();
void showCurrentScreen();
void showMainMenu();
void showStartMenu();
void showSettingsMenu();
void beginStartupInfoFromMainMenu();
void beginRateEditor();
void beginVolumeEditor();
void beginTimeEditor();
void beginDeliveryEditor();
void beginSyringeEditor();
void beginMaxBolusEditor();
void beginStartupMotorSpeedEditor();
void beginSettingsSupportScreen();
void beginSupportInfoScreen();
void beginBolusEditor();
void beginInfusionConfirm(bool volumeTargetEnabled);
void showRateEditor();
void showVolumeEditor();
void showTimeEditor();
void showDeliveryEditor();
void showSyringeEditor();
void showMaxBolusEditor();
void showStartupMotorSpeedEditor();
void showSettingsSupportScreen();
void showSupportInfoScreen();
void showBolusEditor();
void showBolusConfirm();
void showInfusionConfirm();
void showIdleScreen();
void showPumpScreen();
void showStatus(const __FlashStringHelper *line1, const __FlashStringHelper *line2);
void showAcknowledgedStatus(const __FlashStringHelper *line1, const __FlashStringHelper *line2);
void showAlarm(const __FlashStringHelper *line1, const __FlashStringHelper *line2);
void clearAlarm();
void acknowledgeStatus();
void updateStatusScreen();
void printSettingsMenuItem(uint8_t itemIndex, bool selected);
void updateSettingsSupportScreen();
void handleUiButton(Button button);
void handlePumpButton(Button button);
void handleRateEditorButton(Button button);
void handleVolumeEditorButton(Button button);
void handleTimeEditorButton(Button button);
void handleDeliveryEditorButton(Button button);
void handleSyringeEditorButton(Button button);
void handleMaxBolusEditorButton(Button button);
void handleStartupMotorSpeedEditorButton(Button button);
void handleBolusEditorButton(Button button);
void handleBolusConfirmButton(Button button);
void handleInfusionConfirmButton(Button button);
void moveEditCursor(int8_t delta);
void changeRateDigit(int8_t delta);
void changeVolumeDigit(int8_t delta);
void changeTimeDigit(int8_t delta);
void changeDeliveryDigit(int8_t delta);
void changeSyringeDigit(int8_t delta);
void changeMaxBolusPercent(int8_t delta);
void changeStartupMotorSpeedPercent(int8_t delta);
void changeBolusDigit(int8_t delta);
uint16_t rateDigitFactor();
uint16_t volumeDigitFactor();
uint32_t timeDigitFactor();
uint16_t deliveryDigitFactor();
uint16_t bolusDigitFactor();
uint8_t editCursorColumn();
uint32_t maxBolusCentiMl();
uint32_t maxVolumeCentiMl();
float syringeVolumeMl();
bool calculateVolumeTimeFlow(float *calculatedFlow);
bool startVolumeTimeInfusion();
bool prepareBolusPlan();
bool calculateBolusPlan(float *plannedFlowMlPerHour, uint32_t *plannedDurationMillis);
bool startBolus();
void updateBolusState();
bool applyFlowRate(float nextFlowMlPerHour);
bool changeRunningInfusionRate(int8_t deltaSteps);
void beep(uint16_t delayOn = 15, uint16_t delayOff = 80, bool force = false);
void updateBuzzer();
void cancelBuzzerBeeps();
void serviceWatchdog();
void serviceMotorClock();
void watchdogDelay(uint16_t durationMs);
void serviceWait(uint16_t durationMs);
void setFlow(float nextFlow);
void setupTmc2209Uart();
bool verifyTmc2209Readback();
bool tmc2209CurrentReadbackOk(uint16_t currentMa);
void updateStartupMotorJog(Button heldButton);
bool startStartupMotorJog(bool forward);
void stopStartupMotorJog();
bool startMotorSelfTest();
void stopMotorSelfTest();
void setMotorDirection(bool forward);
bool canStartPump();
bool forwardEndstopActive();
bool backwardEndstopActive();
bool anyEndstopActive();
bool endstopActiveForDirection(bool forward);
void enableMotorDriver();
void disableMotorDriver();
void configureEndstopInterrupts();
void stopMotorClock();
void setStepPinLow();
void setStepPinHigh();
bool startPump();
void stopPump(PumpStopReason reason);
void enableTimer1Interrupt();
void enableTimer1InterruptPreservePhase();
bool timerIsRunning();
bool timerCanRepresentStepRate(float stepRateHz);
bool configureTimer1(float stepRateHz);
bool configureTimer1PreservePhase(float stepRateHz);
bool configureTimer1Internal(float stepRateHz, bool preservePhase);
void disableTimer1();
float maxStepRateHz();
float stepsPerMl();
float stepRateHzForFlow(float mlPerHour);
float pumpedMlFromSteps(int32_t steps);
int32_t atomicStepCounter();
void formatElapsedTime(uint32_t elapsedSeconds, char *buffer, size_t bufferSize);
void loadPersistentSettings();
void savePersistentSettings();
bool validDeliveryCenti(uint32_t value);
bool validSyringeCenti(uint32_t value);
bool validMotorDirectionInverted(uint8_t value);
bool validBolusEnabled(uint8_t value);
bool validSoundEnabled(uint8_t value);
bool validTestModeEnabled(uint8_t value);
bool validPressureMonitorEnabled(uint8_t value);
bool validPressureScale(uint8_t value);
bool validPressureAlarmLevel(uint8_t value);
bool validPressureBaseline(uint16_t value);
bool validMaxBolusPercent(uint8_t value);
bool validStartupJogSpeedPercent(uint8_t value);
void applyDeliveryCenti(uint32_t value);
void applySyringeCenti(uint32_t value);
void applyMotorDirectionInverted(uint8_t value);
void applyBolusEnabled(uint8_t value);
void applySoundEnabled(uint8_t value);
void applyTestModeEnabled(uint8_t value);
void applyPressureMonitorEnabled(uint8_t value);
void applyPressureScale(uint8_t value);
void applyPressureAlarmLevel(uint8_t value);
void applyPressureBaseline(uint16_t value);
void applyMaxBolusPercent(uint8_t value);
void applyStartupJogSpeedPercent(uint8_t value);
uint16_t settingsCrc16Bytes(const uint8_t *bytes, uint8_t byteCount);
uint16_t settingsCrc16(const PersistedSettings *settings);
bool settingsCrcMatches(const PersistedSettings *settings);
bool settingsVersion8CrcMatches(const PersistedSettingsVersion8 *settings);
bool settingsVersion7CrcMatches(const PersistedSettingsVersion7 *settings);
bool settingsVersion6CrcMatches(const PersistedSettingsVersion6 *settings);
uint8_t readEepromByte(uint16_t address);
void waitForEepromReady();
void updateEepromByte(uint16_t address, uint8_t value);
void readPersistentSettingsVersion6(PersistedSettingsVersion6 *settings);
void readPersistentSettingsVersion7(PersistedSettingsVersion7 *settings);
void readPersistentSettingsVersion8(PersistedSettingsVersion8 *settings);
void readPersistentSettings(PersistedSettings *settings);
void writePersistentSettings(const PersistedSettings *settings);
uint16_t readPressureSg();
uint8_t pressurePercentFromSg(uint16_t sg);
void calibratePressureBaseline();

void setup()
{
	initializeDisplay();

	pinMode(STEP_PIN, OUTPUT);
	pinMode(ENDSTOP_PIN_FORWARD, INPUT_PULLDOWN);
	pinMode(ENDSTOP_PIN_BACKWARD, INPUT_PULLDOWN);
	pinMode(BUZZER_PIN, OUTPUT);

	digitalWrite(STEP_PIN, LOW);
	digitalWrite(BUZZER_PIN, LOW);
	restoreTouchSharedPins();
	configureEndstopInterrupts();
	setupTmc2209Uart();
	disableMotorDriver();

	loadPersistentSettings();

	startupScreenStartMillis = millis();
	currentScreen = SCREEN_STARTUP_SPLASH;
	showSplash();
	beep();
}

void loop()
{
	serviceWatchdog();
	serviceMotorClock();
	updateBuzzer();
	Button button = readButtonPress();
	if (currentScreen == SCREEN_STARTUP_SPLASH || currentScreen == SCREEN_STARTUP_INFO)
	{
		updateStartupScreens(button);
		return;
	}

	updateStatusScreen();

	if (alarmActive)
	{
		if (button == BUTTON_SELECT)
			clearAlarm();
		return;
	}

	if (!pumpRunning)
	{
		if (statusActive)
		{
			if (button == BUTTON_SELECT)
				acknowledgeStatus();
			return;
		}

		updateSettingsSupportScreen();
		handleUiButton(button);
		return;
	}

	if (!statusActive)
		handlePumpButton(button);
	else if (button == BUTTON_LEFT)
		stopPump(STOP_MANUAL);

	updateBolusState();

	if (!pumpRunning)
		return;

	if (!timerIsRunning())
	{
		if (endstopInterruptLatched || anyEndstopActive())
			stopPump(STOP_ENDSTOP);
		else
			stopPump(STOP_TIMER_ERROR);
		return;
	}

	// Manual bolus delivery counts toward the target volume. This avoids
	// delivering more than the selected total volume during volume/time runs.
	if (targetVolumeEnabled && pumpedMlFromSteps(atomicStepCounter()) >= targetVolumeMl)
	{
		stopPump(STOP_TARGET_VOLUME);
		return;
	}

	if (millis() - lastDisplayMillis >= DISPLAY_UPDATE_MS)
	{
		if (statusActive)
			lastDisplayMillis = millis();
		else if (currentScreen == SCREEN_EDIT_BOLUS)
			showBolusEditor();
		else if (currentScreen == SCREEN_CONFIRM_BOLUS)
			showBolusConfirm();
		else
			showPumpScreen();
	}
}

RawTouch readTouch()
{
	RawTouch raw = {readTouchAxisX(), readTouchAxisY(), readTouchPressure()};
	if (raw.z >= TOUCH_MIN_PRESSURE && raw.z <= TOUCH_MAX_PRESSURE)
	{
		RawTouch confirm = {readTouchAxisX(), readTouchAxisY(), readTouchPressure()};
		if (abs(raw.x - confirm.x) <= TOUCH_STABLE_RAW_DELTA &&
			abs(raw.y - confirm.y) <= TOUCH_STABLE_RAW_DELTA &&
			abs(raw.z - confirm.z) <= TOUCH_STABLE_PRESSURE_DELTA)
		{
			raw.x = (int16_t)((raw.x + confirm.x) / 2);
			raw.y = (int16_t)((raw.y + confirm.y) / 2);
			raw.z = (int16_t)((raw.z + confirm.z) / 2);
		}
		else
		{
			raw.x = 0;
			raw.y = 0;
			raw.z = 0;
		}
	}
	restoreTouchSharedPins();
	return raw;
}

bool isPressed(const RawTouch &point)
{
	int16_t rawMinX = min(TS_RAW_TOP_LEFT_X, TS_RAW_TOP_RIGHT_X) - TOUCH_RAW_MARGIN;
	int16_t rawMaxX = max(TS_RAW_BOTTOM_LEFT_X, TS_RAW_BOTTOM_RIGHT_X) + TOUCH_RAW_MARGIN;
	int16_t rawMinY = min(TS_RAW_TOP_LEFT_Y, TS_RAW_BOTTOM_LEFT_Y) - TOUCH_RAW_MARGIN;
	int16_t rawMaxY = max(TS_RAW_TOP_RIGHT_Y, TS_RAW_BOTTOM_RIGHT_Y) + TOUCH_RAW_MARGIN;
	return point.z >= TOUCH_MIN_PRESSURE && point.z <= TOUCH_MAX_PRESSURE &&
		point.x >= rawMinX && point.x <= rawMaxX &&
		point.y >= rawMinY && point.y <= rawMaxY;
}

int16_t readTouchAxisX()
{
	pinMode(TFT_TOUCH_YP, INPUT);
	pinMode(TFT_TOUCH_YM, INPUT);
	pinMode(TFT_TOUCH_XP, OUTPUT);
	pinMode(TFT_TOUCH_XM, OUTPUT);
	digitalWrite(TFT_TOUCH_XP, HIGH);
	digitalWrite(TFT_TOUCH_XM, LOW);
	delayMicroseconds(TOUCH_SETTLE_US);
	return TOUCH_ADC_MAX - averageAnalogRead(TFT_TOUCH_YP);
}

int16_t readTouchAxisY()
{
	pinMode(TFT_TOUCH_XP, INPUT);
	pinMode(TFT_TOUCH_XM, INPUT);
	pinMode(TFT_TOUCH_YP, OUTPUT);
	pinMode(TFT_TOUCH_YM, OUTPUT);
	digitalWrite(TFT_TOUCH_YP, HIGH);
	digitalWrite(TFT_TOUCH_YM, LOW);
	delayMicroseconds(TOUCH_SETTLE_US);
	return TOUCH_ADC_MAX - averageAnalogRead(TFT_TOUCH_XM);
}

int16_t readTouchPressure()
{
	pinMode(TFT_TOUCH_XP, OUTPUT);
	pinMode(TFT_TOUCH_YM, OUTPUT);
	pinMode(TFT_TOUCH_XM, INPUT);
	pinMode(TFT_TOUCH_YP, INPUT);
	digitalWrite(TFT_TOUCH_XP, LOW);
	digitalWrite(TFT_TOUCH_YM, HIGH);
	delayMicroseconds(TOUCH_SETTLE_US);
	int16_t z1 = averageAnalogRead(TFT_TOUCH_XM);
	int16_t z2 = averageAnalogRead(TFT_TOUCH_YP);
	int16_t pressure = TOUCH_ADC_MAX - (z2 - z1);
	return constrain(pressure, 0, TOUCH_ADC_MAX);
}

int16_t averageAnalogRead(uint8_t pin)
{
	uint32_t total = 0;
	for (uint8_t sample = 0; sample < TOUCH_ANALOG_SAMPLES; sample++)
	{
		serviceMotorClock();
		total += analogRead(pin);
		serviceMotorClock();
	}
	return (int16_t)(total / TOUCH_ANALOG_SAMPLES);
}

void restoreTouchSharedPins()
{
	pinMode(TFT_TOUCH_XP, OUTPUT);
	pinMode(TFT_TOUCH_YM, OUTPUT);
	pinMode(TFT_TOUCH_XM, OUTPUT);
	pinMode(TFT_TOUCH_YP, OUTPUT);
	digitalWrite(TFT_TOUCH_XP, LOW);
	digitalWrite(TFT_TOUCH_YM, LOW);
	digitalWrite(TFT_TOUCH_XM, HIGH);
	digitalWrite(TFT_TOUCH_YP, HIGH);
	tft.invalidateDataBus();
}

int16_t estimateTouchX(const RawTouch &point)
{
	int16_t rawHorizontal = TS_SWAP_XY ? point.y : point.x;
	long rawLeft = (TS_RAW_TOP_LEFT_Y + TS_RAW_BOTTOM_LEFT_Y) / 2;
	long rawRight = (TS_RAW_TOP_RIGHT_Y + TS_RAW_BOTTOM_RIGHT_Y) / 2;
	long mapped = map(rawHorizontal, rawLeft, rawRight, 0, tftWidth - 1);
	return constrain(mapped, 0, tftWidth - 1);
}

int16_t estimateTouchY(const RawTouch &point)
{
	int16_t rawVertical = TS_SWAP_XY ? point.x : point.y;
	long rawTop = (TS_RAW_TOP_LEFT_X + TS_RAW_TOP_RIGHT_X) / 2;
	long rawBottom = (TS_RAW_BOTTOM_LEFT_X + TS_RAW_BOTTOM_RIGHT_X) / 2;
	long mapped = map(rawVertical, rawTop, rawBottom, 0, tftHeight - 1);
	return constrain(mapped, 0, tftHeight - 1);
}

int16_t applyEdgeCompensation(int16_t value, int16_t zone, int16_t maxOffset, int16_t limit)
{
	if (value >= zone || zone <= 0 || maxOffset <= 0)
		return constrain(value, 0, limit);

	long distanceToZone = zone - value;
	long taperedOffset = (distanceToZone * distanceToZone * maxOffset) / ((long)zone * zone);
	return constrain(value + taperedOffset, 0, limit);
}

int16_t mapTouchX(const RawTouch &point)
{
	int16_t rawHorizontal = TS_SWAP_XY ? point.y : point.x;
	int16_t estimatedY = estimateTouchY(point);
	long rawLeft = map(estimatedY, 0, tftHeight - 1, TS_RAW_TOP_LEFT_Y, TS_RAW_BOTTOM_LEFT_Y);
	long rawRight = map(estimatedY, 0, tftHeight - 1, TS_RAW_TOP_RIGHT_Y, TS_RAW_BOTTOM_RIGHT_Y);
	long mapped = map(rawHorizontal, rawLeft, rawRight, 0, tftWidth - 1);
	return applyEdgeCompensation(mapped, LEFT_EDGE_COMPENSATION_ZONE_PX,
		LEFT_EDGE_COMPENSATION_PX, tftWidth - 1);
}

int16_t mapTouchY(const RawTouch &point)
{
	int16_t rawVertical = TS_SWAP_XY ? point.x : point.y;
	int16_t estimatedX = estimateTouchX(point);
	long rawTop = map(estimatedX, 0, tftWidth - 1, TS_RAW_TOP_LEFT_X, TS_RAW_TOP_RIGHT_X);
	long rawBottom = map(estimatedX, 0, tftWidth - 1, TS_RAW_BOTTOM_LEFT_X, TS_RAW_BOTTOM_RIGHT_X);
	long mapped = map(rawVertical, rawTop, rawBottom, 0, tftHeight - 1);
	return applyEdgeCompensation(mapped, TOP_EDGE_COMPENSATION_ZONE_PX,
		TOP_EDGE_COMPENSATION_PX, tftHeight - 1);
}

Button buttonFromTouch(int16_t x, int16_t y)
{
	// Title-bar BACK zone (only honoured if the current screen advertises a
	// back action).
	if (currentScreenHasBack && y < ACTION_HEADER_HEIGHT && x < ACTION_TITLE_BACK_WIDTH)
		return BUTTON_BACK;

	if (currentActionBar.count == 0)
		return BUTTON_NONE;
	if (y < (int16_t)tftHeight - ACTION_BAR_HEIGHT)
		return BUTTON_NONE;

	uint8_t n = currentActionBar.count;
	int16_t slotW = tftWidth / n;
	int16_t slot = x / slotW;
	if (slot < 0)
		slot = 0;
	if (slot >= (int16_t)n)
		slot = n - 1;
	return (Button)currentActionBar.slots[slot].action;
}

// Maps a touch coordinate to the action-bar slot index, or -1 if not on the
// bar. Used by the touch state machine for visual feedback.
static int8_t slotIndexFromTouch(int16_t x, int16_t y)
{
	if (currentActionBar.count == 0)
		return -1;
	if (y < (int16_t)tftHeight - ACTION_BAR_HEIGHT)
		return -1;
	uint8_t n = currentActionBar.count;
	int16_t slotW = tftWidth / n;
	int16_t slot = x / slotW;
	if (slot < 0)
		slot = 0;
	if (slot >= (int16_t)n)
		slot = n - 1;
	return (int8_t)slot;
}

Button readButton()
{
	RawTouch point = readTouch();
	if (!isPressed(point))
		return BUTTON_NONE;
	return buttonFromTouch(mapTouchX(point), mapTouchY(point));
}

// Drives the action-bar press state machine and emits a Button event for the
// current loop tick.
//  * RELEASE mode: emits on touch-up while finger is still inside the slot.
//  * REPEAT mode: emits on touch-down, then repeats every
//    ACTION_PRESS_HOLD_REPEAT_MS after ACTION_PRESS_HOLD_INITIAL_MS held.
//  * HOLD mode: emits every call while finger is on the slot (motor jog).
// Also handles the optional title-bar BACK touch zone (RELEASE semantics,
// emitting BUTTON_LEFT).
Button readButtonPress()
{
	RawTouch point = readTouch();
	bool touched = isPressed(point);
	int16_t tx = 0, ty = 0;
	int8_t slotNow = -1;
	bool backNow = false;
	if (touched)
	{
		tx = mapTouchX(point);
		ty = mapTouchY(point);
		if (currentScreenHasBack && ty < ACTION_HEADER_HEIGHT && tx < ACTION_TITLE_BACK_WIDTH)
			backNow = true;
		else
			slotNow = slotIndexFromTouch(tx, ty);
	}

	uint32_t nowMs = millis();

	// --- Title BACK state machine -----------------------------------
	if (pressedTitleBack)
	{
		if (!touched)
		{
			drawTitleBack(true, false);
			pressedTitleBack = false;
			return BUTTON_BACK;
		}
		if (!backNow)
		{
			drawTitleBack(true, false);
			pressedTitleBack = false;
		}
		return BUTTON_NONE;
	}

	// --- Action-bar slot state machine ------------------------------
	if (pressedSlot < 0)
	{
		if (slotNow >= 0)
		{
			pressedSlot = slotNow;
			pressStartMs = nowMs;
			lastRepeatMs = nowMs;
			drawActionSlot(pressedSlot, true);
			renderedPressedSlot = pressedSlot;
			const ActionSlot &s = currentActionBar.slots[pressedSlot];
			if (s.mode == ACTION_MODE_REPEAT || s.mode == ACTION_MODE_HOLD)
				return (Button)s.action;
			return BUTTON_NONE;
		}
		if (backNow)
		{
			pressedTitleBack = true;
			pressStartMs = nowMs;
			drawTitleBack(true, true);
		}
		return BUTTON_NONE;
	}

	// pressedSlot >= 0
	const ActionSlot &s = currentActionBar.slots[pressedSlot];
	if (touched && slotNow == pressedSlot)
	{
		if (s.mode == ACTION_MODE_HOLD)
			return (Button)s.action;
		if (s.mode == ACTION_MODE_REPEAT)
		{
			uint32_t held = nowMs - pressStartMs;
			if (held >= ACTION_PRESS_HOLD_INITIAL_MS && nowMs - lastRepeatMs >= ACTION_PRESS_HOLD_REPEAT_MS)
			{
				lastRepeatMs = nowMs;
				return (Button)s.action;
			}
		}
		return BUTTON_NONE;
	}

	// Finger left slot (moved off or lifted).
	bool fireOnRelease = !touched && s.mode == ACTION_MODE_RELEASE;
	uint8_t releasedAction = s.action;
	drawActionSlot(pressedSlot, false);
	pressedSlot = -1;
	renderedPressedSlot = -1;

	if (fireOnRelease)
		return (Button)releasedAction;

	// If the user slid onto a different slot, start a fresh press on it.
	if (touched && slotNow >= 0)
	{
		pressedSlot = slotNow;
		pressStartMs = nowMs;
		lastRepeatMs = nowMs;
		drawActionSlot(pressedSlot, true);
		renderedPressedSlot = pressedSlot;
		const ActionSlot &s2 = currentActionBar.slots[pressedSlot];
		if (s2.mode == ACTION_MODE_REPEAT || s2.mode == ACTION_MODE_HOLD)
			return (Button)s2.action;
	}
	return BUTTON_NONE;
}

// ----- Per-screen action-bar layouts ------------------------------------
// These are tiny helpers that build the ActionBar struct each call. Slots
// reuse the existing Button enum so the existing handler dispatch keeps
// working unchanged.
static ActionBar menuActionBar()
{
	ActionSlot up = {"^", "UP", TFT_NAVY, TFT_WHITE, BUTTON_UP, ACTION_MODE_REPEAT};
	ActionSlot dn = {"v", "DN", TFT_NAVY, TFT_WHITE, BUTTON_DOWN, ACTION_MODE_REPEAT};
	ActionSlot ok = {"OK", NULL, TFT_GREEN, TFT_WHITE, BUTTON_SELECT, ACTION_MODE_RELEASE};
	return makeBar3(up, dn, ok);
}
static ActionBar valueEditorActionBar()
{
	ActionSlot left = {"<", NULL, TFT_NAVY, TFT_WHITE, BUTTON_LEFT, ACTION_MODE_REPEAT};
	ActionSlot minus = {"-", NULL, TFT_NAVY, TFT_WHITE, BUTTON_DOWN, ACTION_MODE_REPEAT};
	ActionSlot ok = {"OK", NULL, TFT_GREEN, TFT_WHITE, BUTTON_SELECT, ACTION_MODE_RELEASE};
	ActionSlot plus = {"+", NULL, TFT_NAVY, TFT_WHITE, BUTTON_UP, ACTION_MODE_REPEAT};
	ActionSlot right = {">", NULL, TFT_NAVY, TFT_WHITE, BUTTON_RIGHT, ACTION_MODE_REPEAT};
	return makeBar5(left, minus, ok, plus, right);
}
static ActionBar plusMinusActionBar()
{
	// For editors that only step a percentage (max bolus, startup speed).
	ActionSlot minus = {"-", NULL, TFT_NAVY, TFT_WHITE, BUTTON_DOWN, ACTION_MODE_REPEAT};
	ActionSlot ok = {"OK", NULL, TFT_GREEN, TFT_WHITE, BUTTON_SELECT, ACTION_MODE_RELEASE};
	ActionSlot plus = {"+", NULL, TFT_NAVY, TFT_WHITE, BUTTON_UP, ACTION_MODE_REPEAT};
	return makeBar3(minus, plus, ok);
}
static ActionBar confirmActionBar()
{
	ActionSlot abort = {"X", "ABBR", TFT_RED, TFT_WHITE, BUTTON_LEFT, ACTION_MODE_RELEASE};
	ActionSlot go = {"OK", "START", TFT_GREEN, TFT_WHITE, BUTTON_SELECT, ACTION_MODE_RELEASE};
	return makeBar2(abort, go);
}
static ActionBar pumpActionBar(bool withBolus)
{
	ActionSlot stop = {"STOP", NULL, TFT_RED, TFT_WHITE, BUTTON_LEFT, ACTION_MODE_RELEASE};
	ActionSlot slower = {"-", "RATE", TFT_NAVY, TFT_WHITE, BUTTON_DOWN, ACTION_MODE_REPEAT};
	ActionSlot faster = {"+", "RATE", TFT_NAVY, TFT_WHITE, BUTTON_UP, ACTION_MODE_REPEAT};
	if (!withBolus)
		return makeBar3(stop, slower, faster);
	ActionSlot bolus = {"+", "BOLUS", TFT_NAVY, TFT_WHITE, BUTTON_RIGHT, ACTION_MODE_RELEASE};
	return makeBar4(stop, slower, faster, bolus);
}
static ActionBar ackActionBar(uint16_t color)
{
	ActionSlot ok = {"OK", NULL, color, TFT_WHITE, BUTTON_SELECT, ACTION_MODE_RELEASE};
	return makeBar1(ok);
}
static ActionBar motortestActionBar(bool running)
{
	ActionSlot zur = {"v", "ZUR", TFT_NAVY, TFT_WHITE, BUTTON_DOWN, ACTION_MODE_RELEASE};
	ActionSlot toggle = {running ? "STP" : "GO", NULL, running ? TFT_RED : TFT_GREEN, TFT_WHITE, BUTTON_SELECT, ACTION_MODE_RELEASE};
	ActionSlot vor = {"^", "VOR", TFT_NAVY, TFT_WHITE, BUTTON_UP, ACTION_MODE_RELEASE};
	return makeBar3(zur, toggle, vor);
}
static ActionBar insertSyringeActionBar()
{
	ActionSlot jl = {"<", "MOTOR", TFT_NAVY, TFT_WHITE, BUTTON_LEFT, ACTION_MODE_HOLD};
	ActionSlot jr = {">", "MOTOR", TFT_NAVY, TFT_WHITE, BUTTON_RIGHT, ACTION_MODE_HOLD};
	ActionSlot ok = {"OK", "WEITER", TFT_GREEN, TFT_WHITE, BUTTON_SELECT, ACTION_MODE_RELEASE};
	return makeBar3(jl, jr, ok);
}

void showSplash()
{
	// Startup splash: no controls bar and no header bar - just the centered
	// product wordmark.
	if (!beginCleanScreen((const void *)showSplash))
		return;
	printAt(24, 68, 3, TFT_WHITE, F("DIY Syringe Pump"));
	printAt(60, 120, 2, TFT_CYAN, F("A Flo Project"));
	setActionBar(makeBarEmpty(), false);
}

void showInfoScreen()
{
	currentScreen = SCREEN_STARTUP_INFO;
	char syringeLine[17];
	uint16_t whole = syringeCentiMl / 100;
	uint8_t fraction = syringeCentiMl % 100;
	if (whole >= 100)
		snprintf(syringeLine, sizeof(syringeLine), "BBRAUN %03u.%02u ml", (unsigned int)whole, (unsigned int)fraction);
	else
		snprintf(syringeLine, sizeof(syringeLine), "BBRAUN %02u.%02u ml", (unsigned int)whole, (unsigned int)fraction);
	bool full = beginScreenP(PROMPT_INFO_LINE_1);
	if (full)
		printAt(20, 130, 1, TFT_CYAN, F("OK weiter   <  > Motor jog"));
	// Only repaint the dynamic value line to avoid full-screen flicker when
	// the user jogs the motor and the syringe volume hint refreshes.
	tft.fillRect(30, 74, tftWidth - 60, 24, TFT_BLACK);
	printAtText(30, 74, 3, TFT_WHITE, syringeLine);
	setActionBar(insertSyringeActionBar(), false);
}

void updateStartupScreens(Button button)
{
	if (currentScreen == SCREEN_STARTUP_SPLASH)
	{
		if (button == BUTTON_SELECT || millis() - startupScreenStartMillis >= STARTUP_SPLASH_MS)
		{
			beep(8, 30);
			showInfoScreen();
		}
		return;
	}

	if (currentScreen == SCREEN_STARTUP_INFO && button == BUTTON_SELECT)
	{
		stopStartupMotorJog();
		if (startupInfoFromMainMenu)
		{
			startupInfoFromMainMenu = false;
			currentScreen = SCREEN_MAIN_MENU;
			beep(8, 30);
			showMainMenu();
			return;
		}
		finishStartupInfo();
		return;
	}

	if (currentScreen == SCREEN_STARTUP_INFO)
		updateStartupMotorJog(readButton());
}

void finishStartupInfo()
{
	beep(8, 30);
	currentScreen = SCREEN_MAIN_MENU;
	showMainMenu();
	if (!tmc2209UartVerified)
	{
		showAcknowledgedStatus(F("TMC2209 Fehler"), F("UART/MS pruefen"));
		beep(60, 120);
	}
	else if (settingsResetToDefaults)
	{
		showStatus(F("Settings reset"), F("Defaults aktiv"));
		beep(60, 120);
	}
}

void showCurrentScreen()
{
	if (currentScreen == SCREEN_MAIN_MENU)
		showMainMenu();
	else if (currentScreen == SCREEN_START_MENU)
		showStartMenu();
	else if (currentScreen == SCREEN_PUMP_RUNNING)
		showPumpScreen();
	else if (currentScreen == SCREEN_STARTUP_SPLASH)
		showSplash();
	else if (currentScreen == SCREEN_STARTUP_INFO)
		showInfoScreen();
	else if (currentScreen == SCREEN_EDIT_RATE)
		showRateEditor();
	else if (currentScreen == SCREEN_EDIT_VOLUME)
		showVolumeEditor();
	else if (currentScreen == SCREEN_EDIT_TIME)
		showTimeEditor();
	else if (currentScreen == SCREEN_EDIT_DELIVERY)
		showDeliveryEditor();
	else if (currentScreen == SCREEN_EDIT_SYRINGE)
		showSyringeEditor();
	else if (currentScreen == SCREEN_EDIT_MAX_BOLUS)
		showMaxBolusEditor();
	else if (currentScreen == SCREEN_EDIT_STARTUP_MOTOR_SPEED)
		showStartupMotorSpeedEditor();
	else if (currentScreen == SCREEN_SETTINGS_SUPPORT)
		showSettingsSupportScreen();
	else if (currentScreen == SCREEN_SUPPORT_INFO)
		showSupportInfoScreen();
	else if (currentScreen == SCREEN_EDIT_BOLUS)
		showBolusEditor();
	else if (currentScreen == SCREEN_CONFIRM_BOLUS)
		showBolusConfirm();
	else if (currentScreen == SCREEN_CONFIRM_INFUSION)
		showInfusionConfirm();
	else
		showSettingsMenu();
}

static const __FlashStringHelper *mainMenuLabel(uint8_t index)
{
	switch (index)
	{
	case MAIN_MENU_START_INFUSION:
		return F("Infusion starten");
	case MAIN_MENU_LOAD_SYRINGE:
		return F("Neu einlegen");
	case MAIN_MENU_SETTINGS:
		return F("Einstellungen");
	}
	return F("?");
}

static const __FlashStringHelper *startMenuLabel(uint8_t index)
{
	switch (index)
	{
	case START_MENU_VOLUME_TIME:
		return F("Volumen & Zeit");
	case START_MENU_RATE:
		return F("Infusionsrate");
	}
	return F("?");
}

void showMainMenu()
{
	bool full = beginScreen(F("Navigation"));
	static uint8_t lastIndex = 0xFF;
	if (full || lastIndex >= MAIN_MENU_COUNT)
	{
		for (uint8_t i = 0; i < MAIN_MENU_COUNT; i++)
			drawMenuItem(i, mainMenuIndex == i, mainMenuLabel(i));
	}
	else if (lastIndex != mainMenuIndex)
	{
		drawMenuItem(lastIndex, false, mainMenuLabel(lastIndex));
		drawMenuItem(mainMenuIndex, true, mainMenuLabel(mainMenuIndex));
	}
	else
	{
		drawMenuItem(mainMenuIndex, true, mainMenuLabel(mainMenuIndex));
	}
	lastIndex = mainMenuIndex;
	setActionBar(menuActionBar(), false);
}

void showStartMenu()
{
	bool full = beginScreen(F("Programmwahl"), true);
	static uint8_t lastIndex = 0xFF;
	if (full || lastIndex >= START_MENU_COUNT)
	{
		for (uint8_t i = 0; i < START_MENU_COUNT; i++)
			drawMenuItem(i, startMenuIndex == i, startMenuLabel(i));
	}
	else if (lastIndex != startMenuIndex)
	{
		drawMenuItem(lastIndex, false, startMenuLabel(lastIndex));
		drawMenuItem(startMenuIndex, true, startMenuLabel(startMenuIndex));
	}
	else
	{
		drawMenuItem(startMenuIndex, true, startMenuLabel(startMenuIndex));
	}
	lastIndex = startMenuIndex;
	setActionBar(menuActionBar(), true);
}

void showSettingsMenu()
{
	bool full = beginScreen(F("Einstellungen"), true);
	uint8_t firstIndex = 0;
	if (settingsMenuIndex > 1)
		firstIndex = settingsMenuIndex - 1;
	if (firstIndex > SETTINGS_MENU_COUNT - 4)
		firstIndex = SETTINGS_MENU_COUNT - 4;

	static uint8_t lastIndex = 0xFF;
	static uint8_t lastFirstIndex = 0xFF;
	if (full || lastIndex >= SETTINGS_MENU_COUNT || lastFirstIndex != firstIndex)
	{
		for (uint8_t row = 0; row < 4; row++)
			printSettingsMenuItem(firstIndex + row, settingsMenuIndex == firstIndex + row);
	}
	else if (lastIndex != settingsMenuIndex)
	{
		printSettingsMenuItem(lastIndex, false);
		printSettingsMenuItem(settingsMenuIndex, true);
	}
	else
	{
		// No selection change: redraw only the current row so toggles like
		// Bolus AN/AUS, Ton EIN/AUS, Motor inv etc. update without flicker.
		printSettingsMenuItem(settingsMenuIndex, true);
	}
	lastIndex = settingsMenuIndex;
	lastFirstIndex = firstIndex;
	setActionBar(menuActionBar(), true);
}

void printSettingsMenuItem(uint8_t itemIndex, bool selected)
{
	uint8_t row = itemIndex;
	if (settingsMenuIndex > 1)
		row = itemIndex - settingsMenuIndex + 1;
	if (settingsMenuIndex > SETTINGS_MENU_COUNT - 3)
		row = itemIndex - (SETTINGS_MENU_COUNT - 4);
	beginMenuItem(row, selected);
	switch (itemIndex)
	{
	case SETTINGS_MENU_DELIVERY:
		tft.print(F("Abgabemenge"));
		break;
	case SETTINGS_MENU_SYRINGE:
		tft.print(F("Spritze ml"));
		break;
	case SETTINGS_MENU_BOLUS:
		tft.print(bolusEnabled ? F("Bolus: AN") : F("Bolus: AUS"));
		break;
	case SETTINGS_MENU_SOUND:
		tft.print(soundEnabled ? F("Ton: EIN") : F("Ton: AUS"));
		break;
	case SETTINGS_MENU_MAX_BOLUS:
		tft.print(F("Max Bolus %"));
		break;
	case SETTINGS_MENU_MOTOR_INVERT:
		tft.print(motorDirectionInverted ? F("Motor inv: AN") : F("Motor inv: AUS"));
		break;
	case SETTINGS_MENU_STARTUP_SPEED:
	{
		char valueLine[17];
		snprintf(valueLine, sizeof(valueLine), "Motorspeed:%03u%%", (unsigned int)startupJogSpeedPercent);
		tft.print(valueLine);
		break;
	}
	case SETTINGS_MENU_TEST_MODE:
		tft.print(testModeEnabled ? F("Test Mode: AN") : F("Test Mode: AUS"));
		break;
	case SETTINGS_MENU_PRESSURE:
		tft.print(pressureMonitorEnabled ? F("Druck: AN") : F("Druck: AUS"));
		break;
	case SETTINGS_MENU_PRESSURE_SCALE:
	{
		char valueLine[17];
		snprintf(valueLine, sizeof(valueLine), "Druck-Skala:%02u", (unsigned int)pressureScale);
		tft.print(valueLine);
		break;
	}
	case SETTINGS_MENU_PRESSURE_ALARM:
	{
		char valueLine[17];
		snprintf(valueLine, sizeof(valueLine), "Druck-Alarm:%02u", (unsigned int)pressureAlarmLevel);
		tft.print(valueLine);
		break;
	}
	case SETTINGS_MENU_PRESSURE_ZERO:
		tft.print(F("Druck nullen"));
		break;
	case SETTINGS_MENU_MOTOR_TEST:
		tft.print(F("Motortest"));
		break;
	case SETTINGS_MENU_SUPPORT:
		tft.print(F("Support"));
		break;
	default:
		tft.print(F("?"));
		break;
	}
}

void beginRateEditor()
{
	float centiRate = flowMlPerHour * 100.0 + 0.5;
	if (centiRate < (float)MIN_FLOW_CENTI_ML_PER_HOUR)
		centiRate = (float)MIN_FLOW_CENTI_ML_PER_HOUR;
	if (centiRate > (float)MAX_FLOW_CENTI_ML_PER_HOUR)
		centiRate = (float)MAX_FLOW_CENTI_ML_PER_HOUR;

	editRateCentiMlPerHour = (uint32_t)centiRate;
	editCursorIndex = 0;
	currentScreen = SCREEN_EDIT_RATE;
	showRateEditor();
}

void beginStartupInfoFromMainMenu()
{
	showInfoScreen();
	startupInfoFromMainMenu = true;
}

void beginVolumeEditor()
{
	float centiVolume = targetVolumeMl * 100.0 + 0.5;
	if (centiVolume < 1.0)
		centiVolume = 1.0;
	if (centiVolume > (float)maxVolumeCentiMl())
		centiVolume = (float)maxVolumeCentiMl();

	editVolumeCentiMl = (uint32_t)centiVolume;
	editCursorIndex = 0;
	currentScreen = SCREEN_EDIT_VOLUME;
	showVolumeEditor();
}

void beginTimeEditor()
{
	editTimeSeconds = targetTimeSeconds;
	editCursorIndex = 0;
	currentScreen = SCREEN_EDIT_TIME;
	showTimeEditor();
}

void beginDeliveryEditor()
{
	float centiDelivery = deliveryMlPerCm * 100.0 + 0.5;
	if (centiDelivery < (float)MIN_DELIVERY_CENTI_ML_PER_CM)
		centiDelivery = (float)MIN_DELIVERY_CENTI_ML_PER_CM;
	if (centiDelivery > (float)MAX_DELIVERY_CENTI_ML_PER_CM)
		centiDelivery = (float)MAX_DELIVERY_CENTI_ML_PER_CM;

	editDeliveryCentiMlPerCm = (uint32_t)centiDelivery;
	editCursorIndex = 0;
	currentScreen = SCREEN_EDIT_DELIVERY;
	showDeliveryEditor();
}

void beginSyringeEditor()
{
	editSyringeCentiMl = syringeCentiMl;
	if (!validSyringeCenti(editSyringeCentiMl))
		editSyringeCentiMl = DEFAULT_SYRINGE_CENTI_ML;

	editCursorIndex = 0;
	currentScreen = SCREEN_EDIT_SYRINGE;
	showSyringeEditor();
}

void beginMaxBolusEditor()
{
	editMaxBolusPercent = maxBolusPercent;
	if (!validMaxBolusPercent(editMaxBolusPercent))
		editMaxBolusPercent = DEFAULT_MAX_BOLUS_PERCENT;

	currentScreen = SCREEN_EDIT_MAX_BOLUS;
	showMaxBolusEditor();
}

void beginStartupMotorSpeedEditor()
{
	editStartupJogSpeedPercent = startupJogSpeedPercent;
	if (!validStartupJogSpeedPercent(editStartupJogSpeedPercent))
		editStartupJogSpeedPercent = DEFAULT_STARTUP_JOG_SPEED_PERCENT;

	currentScreen = SCREEN_EDIT_STARTUP_MOTOR_SPEED;
	showStartupMotorSpeedEditor();
}

void beginSettingsSupportScreen()
{
	settingsSupportStartMillis = millis();
	motorTestDirectionForward = true;
	currentScreen = SCREEN_SETTINGS_SUPPORT;
	showSettingsSupportScreen();
}

void beginSupportInfoScreen()
{
	currentScreen = SCREEN_SUPPORT_INFO;
	showSupportInfoScreen();
}

void beginBolusEditor()
{
	if (!bolusEnabled)
	{
		showStatus(F("Bolus AUS"), F("Einstellung"));
		return;
	}

	if (editBolusCentiMl < MIN_BOLUS_CENTI_ML || editBolusCentiMl > maxBolusCentiMl())
		editBolusCentiMl = 100;

	if (editBolusCentiMl < MIN_BOLUS_CENTI_ML)
		editBolusCentiMl = MIN_BOLUS_CENTI_ML;
	if (editBolusCentiMl > maxBolusCentiMl())
		editBolusCentiMl = maxBolusCentiMl();

	editCursorIndex = 0;
	lastBolusInputMillis = millis();
	currentScreen = SCREEN_EDIT_BOLUS;
	showBolusEditor();
}

void beginInfusionConfirm(bool volumeTargetEnabled)
{
	pendingInfusionTargetVolumeEnabled = volumeTargetEnabled;
	currentScreen = SCREEN_CONFIRM_INFUSION;
	showInfusionConfirm();
}

void showRateEditor()
{
	uint16_t whole = editRateCentiMlPerHour / 100;
	uint8_t fraction = editRateCentiMlPerHour % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u.%02u ml/h", (unsigned int)whole, (unsigned int)fraction);
	drawValueScreenP(PROMPT_RATE, valueLine, editCursorColumn(), true, true);
	setActionBar(valueEditorActionBar(), true);
}

void showVolumeEditor()
{
	uint16_t whole = editVolumeCentiMl / 100;
	uint8_t fraction = editVolumeCentiMl % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u.%02u ml", (unsigned int)whole, (unsigned int)fraction);
	drawValueScreenP(PROMPT_VOLUME, valueLine, editCursorColumn(), true, true);
	setActionBar(valueEditorActionBar(), true);
}

void showTimeEditor()
{
	uint8_t hours = editTimeSeconds / 3600;
	uint8_t minutes = (editTimeSeconds % 3600) / 60;
	uint8_t seconds = editTimeSeconds % 60;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%02u:%02u:%02u H/M/S", (unsigned int)hours, (unsigned int)minutes, (unsigned int)seconds);
	drawValueScreenP(PROMPT_TIME, valueLine, editCursorColumn(), true, true);
	setActionBar(valueEditorActionBar(), true);
}

void showDeliveryEditor()
{
	uint16_t whole = editDeliveryCentiMlPerCm / 100;
	uint8_t fraction = editDeliveryCentiMlPerCm % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u.%02u ml/cm", (unsigned int)whole, (unsigned int)fraction);
	drawValueScreenP(PROMPT_DELIVERY, valueLine, editCursorColumn(), true, true);
	setActionBar(valueEditorActionBar(), true);
}

void showSyringeEditor()
{
	uint16_t whole = editSyringeCentiMl / 100;
	uint8_t fraction = editSyringeCentiMl % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u.%02u ml", (unsigned int)whole, (unsigned int)fraction);
	drawValueScreenP(PROMPT_SYRINGE, valueLine, editCursorColumn(), true, true);
	setActionBar(valueEditorActionBar(), true);
}

void showMaxBolusEditor()
{
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%02u %% Spritze", (unsigned int)editMaxBolusPercent);
	beginScreen(F("Max Bolus"), true);
	printAtText(38, 78, 3, TFT_WHITE, valueLine);
	printAt(34, 138, 1, TFT_CYAN, F("- / + anpassen   OK speichern"));
	setActionBar(plusMinusActionBar(), true);
}

void showStartupMotorSpeedEditor()
{
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u %%", (unsigned int)editStartupJogSpeedPercent);
	beginScreen(F("Motorspeed"), true);
	printAtText(88, 78, 3, TFT_WHITE, valueLine);
	printAt(34, 138, 1, TFT_CYAN, F("- / + anpassen   OK speichern"));
	setActionBar(plusMinusActionBar(), true);
}

void showSettingsSupportScreen()
{
	char line1[17];
	snprintf(line1, sizeof(line1), "Test %s %s",
		motorTestDirectionForward ? "VOR" : "ZUR",
		motorTestRunning ? "AN" : "AUS");
	beginScreen(F("Motortest"), true);
	printAtText(36, 72, 3, TFT_WHITE, line1);
	printAt(28, 128, 1, TFT_CYAN, F("GO/STP   ^ Vor   v Zur   < Zurueck"));
	setActionBar(motortestActionBar(motorTestRunning), true);
}

void showSupportInfoScreen()
{
	beginScreen(F("Support"), true);
	printAtText(40, 72, 3, TFT_WHITE, SUPPORT_DISPLAY_LINE_1);
	printAtText(40, 112, 3, TFT_CYAN, SUPPORT_DISPLAY_LINE_2);
	setActionBar(ackActionBar(TFT_GREEN), false);
}

void showBolusEditor()
{
	uint8_t whole = editBolusCentiMl / 100;
	uint8_t fraction = editBolusCentiMl % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "Bolus ml: %02u.%02u", (unsigned int)whole, (unsigned int)fraction);
	drawValueScreen(F("Manueller Bolus"), valueLine, editCursorColumn(), true, true);
	setActionBar(valueEditorActionBar(), true);
	lastDisplayMillis = millis();
}

void showBolusConfirm()
{
	char flowLine[17];
	char flowText[8];
	char timeLine[17];
	dtostrf(pendingBolusFlowMlPerHour, 6, 1, flowText);
	snprintf(flowLine, sizeof(flowLine), "BOL %s ml/h", flowText);
	uint32_t seconds = (pendingBolusDurationMillis + 999UL) / 1000UL;
	snprintf(timeLine, sizeof(timeLine), "Zeit %4lus", (unsigned long)seconds);
	beginScreen(F("Bolus"));
	printAtText(34, 66, 3, TFT_WHITE, flowLine);
	printAtText(58, 112, 3, TFT_CYAN, timeLine);
	setActionBar(confirmActionBar(), false);
	lastDisplayMillis = millis();
}

void showInfusionConfirm()
{
	char rateText[8];
	char syringeLine[32];
	char rateLine[54];
	uint16_t syringeWholeMl = syringeCentiMl / 100UL;
	uint32_t centiFlow = (uint32_t)(flowMlPerHour * 100.0 + 0.5);
	snprintf(rateText, sizeof(rateText), "%u.%02u", (unsigned int)(centiFlow / 100UL), (unsigned int)(centiFlow % 100UL));
	uint32_t durationSeconds = pendingInfusionTargetVolumeEnabled
		? targetTimeSeconds
		: (flowMlPerHour > 0.0 ? (uint32_t)(((float)syringeCentiMl / 100.0) * 3600.0 / flowMlPerHour + 0.5) : 0UL);
	uint32_t durationMinutes = (durationSeconds + 30UL) / 60UL;

	snprintf(syringeLine, sizeof(syringeLine), "Eingelegte Spritze: %uml", (unsigned int)syringeWholeMl);
	snprintf(rateLine, sizeof(rateLine), "Infusionsrate: %s ml/h, Dauer: %lu min", rateText, (unsigned long)durationMinutes);

	beginScreen(F("Abgabe starten?"));
	tft.fillRect(12, 56, tftWidth - 24, 42, TFT_BLACK);
	printAtText(12, 58, 1, TFT_WHITE, syringeLine);
	printAtText(12, 78, 1, TFT_WHITE, rateLine);
	setActionBar(confirmActionBar(), false);
	lastDisplayMillis = millis();
}

void showIdleScreen()
{
	showMainMenu();
}

void showPumpScreen()
{
	// Cached per-character state so subsequent ticks only repaint the
	// digit/character cells that actually changed. Painting with
	// setTextColor(fg, bg) atomically overwrites the previous glyph cell,
	// so we never need to clear the body region between ticks - that full
	// clear was the cause of the visible flicker in the middle of the
	// screen on every update.
	static char lastFlowText[8] = {0};
	static char lastLineText[24] = {0};
	static bool pumpCacheValid = false;
	// Pressure bar cache (diff render). 0xFFFF == cache invalid.
	static uint16_t lastPressureFillPx = 0xFFFF;
	static uint16_t lastPressureColor = 0;
	static uint16_t lastPressureSgText = 0xFFFF;
	static bool lastPressureBarVisible = false;

	bool fullFrame = beginScreen(F("Infusion laeuft"));
	if (fullFrame || !pumpCacheValid)
	{
		// First paint on this screen frame: clear the body region exactly
		// once and draw the static "Rate ml/h" label. Invalidate the
		// character caches so every glyph is forced to repaint below.
		tft.fillRect(16, 52, tftWidth - 32, 116, TFT_BLACK);
		printAt(16, 52, 2, TFT_CYAN, F("Rate ml/h"));
		for (uint8_t i = 0; i < sizeof(lastFlowText); i++)
			lastFlowText[i] = '\0';
		for (uint8_t i = 0; i < sizeof(lastLineText); i++)
			lastLineText[i] = '\0';
		lastPressureFillPx = 0xFFFF;
		lastPressureColor = 0;
		lastPressureSgText = 0xFFFF;
		lastPressureBarVisible = false;
		pumpCacheValid = true;
	}

	lastDisplayMillis = millis();

	char flowText[8];
	uint32_t centiFlow = (uint32_t)(flowMlPerHour * 100.0 + 0.5);
	snprintf(flowText, sizeof(flowText), "%03u.%02u", (unsigned int)(centiFlow / 100UL), (unsigned int)(centiFlow % 100UL));

	uint32_t elapsedSeconds = (millis() - pumpStartMillis) / 1000;
	float pumpedMl = pumpedMlFromSteps(atomicStepCounter());
	char elapsedText[8];
	char volumeText[8];
	char lineText[24];
	if (targetVolumeEnabled)
	{
		float remainingMl = targetVolumeMl - pumpedMl;
		if (remainingMl < 0.0)
			remainingMl = 0.0;

		uint32_t remainingSeconds = flowMlPerHour > 0.0 ? (uint32_t)((remainingMl * 3600.0 / flowMlPerHour) + 0.5) : 0;
		formatElapsedTime(remainingSeconds, elapsedText, sizeof(elapsedText));
		dtostrf(remainingMl, 1, 2, volumeText);
		snprintf(lineText, sizeof(lineText), "Rest %s ml %s", volumeText, elapsedText);
	}
	else
	{
		uint32_t pumpedCentiMl = pumpedMl <= 0.0 ? 0UL : (uint32_t)(pumpedMl * 100.0 + 0.5);
		uint16_t pumpedWholeMl = pumpedCentiMl / 100UL;
		uint8_t pumpedFractionMl = pumpedCentiMl % 100UL;
		formatElapsedTime(elapsedSeconds, elapsedText, sizeof(elapsedText));
		if (pumpedWholeMl < 100U)
			snprintf(volumeText, sizeof(volumeText), "%02u.%02u", (unsigned int)pumpedWholeMl, (unsigned int)pumpedFractionMl);
		else
			snprintf(volumeText, sizeof(volumeText), "%03u.%02u", (unsigned int)pumpedWholeMl, (unsigned int)pumpedFractionMl);
		snprintf(lineText, sizeof(lineText), "%s  %s ml", elapsedText, volumeText);
	}

	// Per-character diff for the flow rate "XXX.XX" at text size 4 (24px
	// wide cells). Drawing the same cell with setTextColor(fg, bg) repaints
	// the background, so unchanged digits are simply skipped.
	const uint8_t flowColumns = 6; // "NNN.NN"
	tft.setTextSize(4);
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	for (uint8_t c = 0; c < flowColumns; c++)
	{
		char newCh = c < (uint8_t)strlen(flowText) ? flowText[c] : ' ';
		if (lastFlowText[c] != newCh)
		{
			tft.setCursor(16 + (int16_t)c * 24, 78);
			tft.write((uint8_t)newCh);
			lastFlowText[c] = newCh;
		}
	}

	// Per-character diff for the status line at text size 2 (12px cells).
	// Pad with spaces past the new length so old trailing characters from
	// a longer previous line (e.g. mode switch) are erased cleanly.
	const uint8_t lineColumns = 23; // fits within tftWidth - 32 at size 2
	uint8_t lineLen = (uint8_t)strlen(lineText);
	if (lineLen > lineColumns)
		lineLen = lineColumns;
	tft.setTextSize(2);
	tft.setTextColor(TFT_YELLOW, TFT_BLACK);
	for (uint8_t c = 0; c < lineColumns; c++)
	{
		char newCh = c < lineLen ? lineText[c] : ' ';
		if (lastLineText[c] != newCh)
		{
			tft.setCursor(16 + (int16_t)c * 12, 124);
			tft.write((uint8_t)newCh);
			lastLineText[c] = newCh;
		}
	}

	// --- Pressure bar (StallGuard4 load indicator) -----------------------
	// Bar geometry: x=16, y=148, full body width, 18 px high. Inner fill
	// area is inset by 1 px so the dark-grey border stays intact between
	// frames and we only need to repaint the changed portion of the fill.
	const int16_t pressureBarX = 16;
	const int16_t pressureBarY = 148;
	const int16_t pressureBarW = (int16_t)(tftWidth - 32);
	const int16_t pressureBarH = 18;
	const int16_t pressureFillX = pressureBarX + 1;
	const int16_t pressureFillY = pressureBarY + 1;
	const int16_t pressureFillW = pressureBarW - 2;
	const int16_t pressureFillH = pressureBarH - 2;
	const int16_t pressureSgTextX = (int16_t)(tftWidth - 76);
	const int16_t pressureSgTextY = pressureBarY - 8;

	if (pressureMonitorEnabled)
	{
		uint16_t sg = readPressureSg();
		pressureCurrentSg = sg;
		uint8_t percent = pressurePercentFromSg(sg);
		pressureCurrentBarPercent = percent;

		// Choose fill color from the visual scale level. Yellow above
		// ~60% and red once the user-configured alarm level (1..10) is
		// reached.
		uint8_t alarmPercent = (uint8_t)((uint16_t)pressureAlarmLevel * 10U);
		uint16_t fillColor = TFT_GREEN;
		if (percent >= alarmPercent)
			fillColor = TFT_RED;
		else if (percent >= 60)
			fillColor = TFT_YELLOW;

		int16_t fillPx = (int16_t)(((int32_t)pressureFillW * (int32_t)percent) / 100);
		if (fillPx < 0)
			fillPx = 0;
		if (fillPx > pressureFillW)
			fillPx = pressureFillW;

		bool needBorder = !lastPressureBarVisible;
		if (needBorder)
			tft.drawRect(pressureBarX, pressureBarY, pressureBarW, pressureBarH, TFT_DARKGREY);

		if (needBorder || lastPressureColor != fillColor)
		{
			// Color changed or first paint -> repaint entire filled
			// portion and clear the unfilled remainder.
			if (fillPx > 0)
				tft.fillRect(pressureFillX, pressureFillY, fillPx, pressureFillH, fillColor);
			if (fillPx < pressureFillW)
				tft.fillRect(pressureFillX + fillPx, pressureFillY,
					pressureFillW - fillPx, pressureFillH, TFT_BLACK);
		}
		else if ((uint16_t)fillPx != lastPressureFillPx)
		{
			// Same color, only the fill length changed -> paint only the
			// delta segment so the bar grows/shrinks without flicker.
			if ((uint16_t)fillPx > lastPressureFillPx)
			{
				tft.fillRect(pressureFillX + (int16_t)lastPressureFillPx, pressureFillY,
					fillPx - (int16_t)lastPressureFillPx, pressureFillH, fillColor);
			}
			else
			{
				tft.fillRect(pressureFillX + fillPx, pressureFillY,
					(int16_t)lastPressureFillPx - fillPx, pressureFillH, TFT_BLACK);
			}
		}

		lastPressureFillPx = (uint16_t)fillPx;
		lastPressureColor = fillColor;
		lastPressureBarVisible = true;
		if (needBorder || lastPressureSgText != sg)
		{
			char sgText[9];
			snprintf(sgText, sizeof(sgText), "SG:%3u", (unsigned int)sg);
			tft.fillRect(pressureSgTextX, pressureSgTextY, 60, 8, TFT_BLACK);
			tft.setTextSize(1);
			tft.setTextColor(TFT_WHITE, TFT_BLACK);
			tft.setCursor(pressureSgTextX, pressureSgTextY);
			tft.print(sgText);
			lastPressureSgText = sg;
		}

		// Sustained-high alarm: only beep after several ticks above the
		// threshold so a single noisy SG sample does not produce a false
		// alarm. Re-arm by dropping below threshold for one tick.
		if (percent >= alarmPercent)
		{
			if (pressureHighTicks < 0xFF)
				pressureHighTicks++;
			if (pressureHighTicks >= PRESSURE_ALARM_SUSTAIN_TICKS)
			{
				uint32_t now = millis();
				if (now - pressureLastAlarmBeepMillis >= 1000UL)
				{
					beep(60, 120);
					pressureLastAlarmBeepMillis = now;
				}
			}
		}
		else
		{
			pressureHighTicks = 0;
		}
	}
	else if (lastPressureBarVisible)
	{
		// Monitor was just disabled (or cache invalidated) -> erase any
		// previously drawn bar so the area stays clean.
		tft.fillRect(pressureBarX, pressureBarY, pressureBarW, pressureBarH, TFT_BLACK);
		tft.fillRect(pressureSgTextX, pressureSgTextY, 60, 8, TFT_BLACK);
		lastPressureBarVisible = false;
		lastPressureFillPx = 0xFFFF;
		lastPressureColor = 0;
		lastPressureSgText = 0xFFFF;
		pressureHighTicks = 0;
	}

	setActionBar(pumpActionBar(bolusEnabled), false);
}

void showStatus(const __FlashStringHelper *line1, const __FlashStringHelper *line2)
{
	uint32_t now = millis();
	uint32_t nextStatusUntil = now + STATUS_DISPLAY_MS;
	if (statusActive && !alarmActive && (int32_t)(statusUntilMillis - now) > 0)
	{
		uint32_t extendedStatusUntil = statusUntilMillis + STATUS_DISPLAY_MS;
		if ((int32_t)(extendedStatusUntil - nextStatusUntil) > 0)
			nextStatusUntil = extendedStatusUntil;
	}

	alarmActive = false;
	cancelBuzzerBeeps();
	statusReturnScreen = currentScreen;
	statusActive = true;
	statusRequiresAcknowledge = false;
	statusUntilMillis = nextStatusUntil;
	digitalWrite(BUZZER_PIN, LOW);
	beginScreen(F("Status"));
	printAt(20, 70, 3, TFT_WHITE, line1);
	printAt(20, 116, 2, TFT_CYAN, line2);
	setActionBar(ackActionBar(TFT_GREEN), false);
	lastDisplayMillis = millis();
}

void showAcknowledgedStatus(const __FlashStringHelper *line1, const __FlashStringHelper *line2)
{
	alarmActive = false;
	cancelBuzzerBeeps();
	statusReturnScreen = currentScreen;
	statusActive = true;
	statusRequiresAcknowledge = true;
	statusUntilMillis = 0;
	digitalWrite(BUZZER_PIN, LOW);
	beginScreen(F("Meldung"));
	printAt(20, 70, 3, TFT_WHITE, line1);
	printAt(20, 116, 2, TFT_CYAN, line2);
	setActionBar(ackActionBar(TFT_GREEN), false);
	lastDisplayMillis = millis();
	beep(200, 100, true);
	beep(200, 0, true);
}

void showAlarm(const __FlashStringHelper *line1, const __FlashStringHelper *line2)
{
	cancelBuzzerBeeps();
	alarmActive = false;
	statusActive = true;
	statusRequiresAcknowledge = true;
	statusReturnScreen = SCREEN_MAIN_MENU;
	digitalWrite(BUZZER_PIN, LOW);
	beginScreen(F("ALARM"));
	tft.fillRect(0, 28, tftWidth, 154, TFT_RED);
	tft.setTextSize(3);
	tft.setTextColor(TFT_WHITE, TFT_RED);
	tft.setCursor(20, 70);
	tft.print(line1);
	tft.setTextSize(2);
	tft.setCursor(20, 116);
	tft.print(line2);
	setActionBar(ackActionBar(TFT_YELLOW), false);
	lastDisplayMillis = millis();
	beep(300, 0, true);
	alarmActive = true;
}

void clearAlarm()
{
	alarmActive = false;
	statusActive = false;
	statusRequiresAcknowledge = false;
	cancelBuzzerBeeps();
	digitalWrite(BUZZER_PIN, LOW);
	currentScreen = SCREEN_MAIN_MENU;
	beep(8, 30);
	showMainMenu();
}

void acknowledgeStatus()
{
	if (!statusActive || alarmActive)
		return;

	statusActive = false;
	statusRequiresAcknowledge = false;
	currentScreen = statusReturnScreen;
	beep(8, 30);
	showCurrentScreen();
}

void updateStatusScreen()
{
	if (alarmActive)
		return;

	if (statusRequiresAcknowledge)
		return;

	if (!statusActive || (int32_t)(millis() - statusUntilMillis) < 0)
		return;

	statusActive = false;
	statusRequiresAcknowledge = false;
	currentScreen = statusReturnScreen;
	showCurrentScreen();
}

void updateSettingsSupportScreen()
{
	if (currentScreen != SCREEN_SETTINGS_SUPPORT)
		return;

	if (!motorTestRunning)
		return;

	if (!timerIsRunning())
	{
		stopMotorSelfTest();
		if (endstopInterruptLatched || anyEndstopActive())
			showStatus(F("Test gestoppt"), F("Endstop aktiv"));
		else
			showStatus(F("Test gestoppt"), F("Timerfehler"));
	}
}
void handleUiButton(Button button)
{
	if (button == BUTTON_NONE)
	{
		if (currentScreen == SCREEN_SETTINGS_SUPPORT)
			updateSettingsSupportScreen();
		return;
	}

	if (currentScreen == SCREEN_SETTINGS_SUPPORT)
	{
		if (button == BUTTON_LEFT || button == BUTTON_BACK)
		{
			stopMotorSelfTest();
			currentScreen = SCREEN_SETTINGS_MENU;
			beep(8, 30);
			showSettingsMenu();
		}
		else if (button == BUTTON_SELECT)
		{
			if (motorTestRunning)
				stopMotorSelfTest();
			else
				startMotorSelfTest();
			beep(8, 30);
			showSettingsSupportScreen();
		}
		else if (button == BUTTON_UP || button == BUTTON_DOWN)
		{
			bool nextDirectionForward = button == BUTTON_UP;
			if (motorTestDirectionForward != nextDirectionForward)
			{
				motorTestDirectionForward = nextDirectionForward;
				if (motorTestRunning)
				{
					stopMotorSelfTest();
					startMotorSelfTest();
				}
				beep(8, 30);
				showSettingsSupportScreen();
			}
		}
		return;
	}

	if (currentScreen == SCREEN_SUPPORT_INFO)
	{
		if (button == BUTTON_LEFT || button == BUTTON_RIGHT || button == BUTTON_SELECT)
		{
			currentScreen = SCREEN_SETTINGS_MENU;
			beep(8, 30);
			showSettingsMenu();
		}
		return;
	}

	if (currentScreen == SCREEN_EDIT_RATE)
	{
		handleRateEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_EDIT_VOLUME)
	{
		handleVolumeEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_EDIT_TIME)
	{
		handleTimeEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_EDIT_DELIVERY)
	{
		handleDeliveryEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_EDIT_SYRINGE)
	{
		handleSyringeEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_EDIT_MAX_BOLUS)
	{
		handleMaxBolusEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_EDIT_STARTUP_MOTOR_SPEED)
	{
		handleStartupMotorSpeedEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_CONFIRM_BOLUS)
	{
		handleBolusConfirmButton(button);
		return;
	}

	if (currentScreen == SCREEN_CONFIRM_INFUSION)
	{
		handleInfusionConfirmButton(button);
		return;
	}

	if (currentScreen == SCREEN_MAIN_MENU)
	{
		if (button == BUTTON_UP || button == BUTTON_DOWN)
		{
			if (button == BUTTON_UP)
				mainMenuIndex = mainMenuIndex == 0 ? MAIN_MENU_COUNT - 1 : mainMenuIndex - 1;
			else
				mainMenuIndex = mainMenuIndex >= MAIN_MENU_COUNT - 1 ? 0 : mainMenuIndex + 1;
			beep(8, 30);
			showMainMenu();
		}
		else if (button == BUTTON_RIGHT || button == BUTTON_SELECT)
		{
			beep(8, 30);
			if (mainMenuIndex == MAIN_MENU_START_INFUSION)
			{
				currentScreen = SCREEN_START_MENU;
				showStartMenu();
			}
			else if (mainMenuIndex == MAIN_MENU_LOAD_SYRINGE)
				beginStartupInfoFromMainMenu();
			else if (mainMenuIndex == MAIN_MENU_SETTINGS)
			{
				currentScreen = SCREEN_SETTINGS_MENU;
				settingsMenuIndex = 0;
				showSettingsMenu();
			}
		}
		else if (button == BUTTON_LEFT)
		{
			beep(8, 30);
		}
		return;
	}

	if (currentScreen == SCREEN_START_MENU)
	{
		if (button == BUTTON_UP || button == BUTTON_DOWN)
		{
			if (button == BUTTON_UP)
				startMenuIndex = startMenuIndex == 0 ? START_MENU_COUNT - 1 : startMenuIndex - 1;
			else
				startMenuIndex = startMenuIndex >= START_MENU_COUNT - 1 ? 0 : startMenuIndex + 1;
			beep(8, 30);
			showStartMenu();
		}
		else if (button == BUTTON_LEFT || button == BUTTON_BACK)
		{
			currentScreen = SCREEN_MAIN_MENU;
			beep(8, 30);
			showMainMenu();
		}
		else if (button == BUTTON_RIGHT || button == BUTTON_SELECT)
		{
			beep(8, 30);
			if (startMenuIndex == START_MENU_VOLUME_TIME)
				beginVolumeEditor();
			else if (startMenuIndex == START_MENU_RATE)
				beginRateEditor();
		}
		return;
	}

	if (currentScreen == SCREEN_SETTINGS_MENU)
	{
		if (button == BUTTON_UP || button == BUTTON_DOWN)
		{
			if (button == BUTTON_UP)
				settingsMenuIndex = settingsMenuIndex == 0 ? SETTINGS_MENU_COUNT - 1 : settingsMenuIndex - 1;
			else
				settingsMenuIndex = settingsMenuIndex >= SETTINGS_MENU_COUNT - 1 ? 0 : settingsMenuIndex + 1;
			beep(8, 30);
			showSettingsMenu();
		}
		else if (button == BUTTON_LEFT || button == BUTTON_BACK)
		{
			currentScreen = SCREEN_MAIN_MENU;
			beep(8, 30);
			showMainMenu();
		}
		else if (button == BUTTON_RIGHT || button == BUTTON_SELECT)
		{
			beep(8, 30);
			if (settingsMenuIndex == SETTINGS_MENU_DELIVERY)
				beginDeliveryEditor();
			else if (settingsMenuIndex == SETTINGS_MENU_SYRINGE)
				beginSyringeEditor();
			else if (settingsMenuIndex == SETTINGS_MENU_BOLUS)
			{
				bolusEnabled = !bolusEnabled;
				savePersistentSettings();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_SOUND)
			{
				soundEnabled = !soundEnabled;
				savePersistentSettings();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_MAX_BOLUS)
				beginMaxBolusEditor();
			else if (settingsMenuIndex == SETTINGS_MENU_MOTOR_INVERT)
			{
				motorDirectionInverted = !motorDirectionInverted;
				savePersistentSettings();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_STARTUP_SPEED)
				beginStartupMotorSpeedEditor();
			else if (settingsMenuIndex == SETTINGS_MENU_TEST_MODE)
			{
				testModeEnabled = !testModeEnabled;
				savePersistentSettings();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_PRESSURE)
			{
				pressureMonitorEnabled = !pressureMonitorEnabled;
				savePersistentSettings();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_PRESSURE_SCALE)
			{
				pressureScale = pressureScale >= PRESSURE_SCALE_MAX
					? PRESSURE_SCALE_MIN
					: (uint8_t)(pressureScale + 1);
				savePersistentSettings();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_PRESSURE_ALARM)
			{
				pressureAlarmLevel = pressureAlarmLevel >= PRESSURE_ALARM_MAX
					? PRESSURE_ALARM_MIN
					: (uint8_t)(pressureAlarmLevel + 1);
				savePersistentSettings();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_PRESSURE_ZERO)
			{
				calibratePressureBaseline();
				showSettingsMenu();
			}
			else if (settingsMenuIndex == SETTINGS_MENU_MOTOR_TEST)
				beginSettingsSupportScreen();
			else if (settingsMenuIndex == SETTINGS_MENU_SUPPORT)
				beginSupportInfoScreen();
		}
		return;
	}

	if (button == BUTTON_LEFT)
	{
		currentScreen = SCREEN_MAIN_MENU;
		beep(8, 30);
		showMainMenu();
	}
	else if (button == BUTTON_RIGHT || button == BUTTON_SELECT)
	{
		beep(8, 30);
		beginDeliveryEditor();
	}
}

void handlePumpButton(Button button)
{
	if (button == BUTTON_NONE)
		return;

	if (currentScreen == SCREEN_EDIT_BOLUS)
	{
		handleBolusEditorButton(button);
		return;
	}

	if (currentScreen == SCREEN_CONFIRM_BOLUS)
	{
		handleBolusConfirmButton(button);
		return;
	}

	if (currentScreen == SCREEN_CONFIRM_INFUSION)
	{
		handleInfusionConfirmButton(button);
		return;
	}

	if (button == BUTTON_LEFT)
		stopPump(STOP_MANUAL);
	else if (button == BUTTON_UP)
		changeRunningInfusionRate(1);
	else if (button == BUTTON_DOWN)
		changeRunningInfusionRate(-1);
	else if (button == BUTTON_RIGHT)
	{
		if (bolusEnabled)
			beginBolusEditor();
		else
			showStatus(F("Bolus AUS"), F("Einstellung"));
	}
}

void handleRateEditorButton(Button button)
{
	if (button == BUTTON_LEFT)
		moveEditCursor(-1);
	else if (button == BUTTON_RIGHT)
		moveEditCursor(1);
	else if (button == BUTTON_UP)
		changeRateDigit(1);
	else if (button == BUTTON_DOWN)
		changeRateDigit(-1);
	else if (button == BUTTON_SELECT)
	{
		flowMlPerHour = (float)editRateCentiMlPerHour / 100.0;
		beginInfusionConfirm(false);
	}
	else if (button == BUTTON_BACK)
	{
		currentScreen = SCREEN_START_MENU;
		beep(8, 30);
		showStartMenu();
	}
}

void handleVolumeEditorButton(Button button)
{
	if (button == BUTTON_LEFT)
		moveEditCursor(-1);
	else if (button == BUTTON_RIGHT)
		moveEditCursor(1);
	else if (button == BUTTON_UP)
		changeVolumeDigit(1);
	else if (button == BUTTON_DOWN)
		changeVolumeDigit(-1);
	else if (button == BUTTON_SELECT)
	{
		targetVolumeMl = (float)editVolumeCentiMl / 100.0;
		beginTimeEditor();
	}
	else if (button == BUTTON_BACK)
	{
		currentScreen = SCREEN_START_MENU;
		beep(8, 30);
		showStartMenu();
	}
}

void handleTimeEditorButton(Button button)
{
	if (button == BUTTON_LEFT)
		moveEditCursor(-1);
	else if (button == BUTTON_RIGHT)
		moveEditCursor(1);
	else if (button == BUTTON_UP)
		changeTimeDigit(1);
	else if (button == BUTTON_DOWN)
		changeTimeDigit(-1);
	else if (button == BUTTON_SELECT)
	{
		float calculatedFlow;
		targetTimeSeconds = editTimeSeconds;
		if (calculateVolumeTimeFlow(&calculatedFlow))
		{
			flowMlPerHour = calculatedFlow;
			beginInfusionConfirm(true);
		}
	}
	else if (button == BUTTON_BACK)
	{
		currentScreen = SCREEN_EDIT_VOLUME;
		beep(8, 30);
		showVolumeEditor();
	}
}

void handleDeliveryEditorButton(Button button)
{
	if (button == BUTTON_LEFT)
		moveEditCursor(-1);
	else if (button == BUTTON_RIGHT)
		moveEditCursor(1);
	else if (button == BUTTON_UP)
		changeDeliveryDigit(1);
	else if (button == BUTTON_DOWN)
		changeDeliveryDigit(-1);
	else if (button == BUTTON_SELECT)
	{
		applyDeliveryCenti(editDeliveryCentiMlPerCm);
		savePersistentSettings();
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
	else if (button == BUTTON_BACK)
	{
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
}

void handleSyringeEditorButton(Button button)
{
	if (button == BUTTON_LEFT)
		moveEditCursor(-1);
	else if (button == BUTTON_RIGHT)
		moveEditCursor(1);
	else if (button == BUTTON_UP)
		changeSyringeDigit(1);
	else if (button == BUTTON_DOWN)
		changeSyringeDigit(-1);
	else if (button == BUTTON_SELECT)
	{
		applySyringeCenti(editSyringeCentiMl);
		savePersistentSettings();
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
	else if (button == BUTTON_BACK)
	{
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
}

void handleMaxBolusEditorButton(Button button)
{
	if (button == BUTTON_UP)
		changeMaxBolusPercent(1);
	else if (button == BUTTON_DOWN)
		changeMaxBolusPercent(-1);
	else if (button == BUTTON_LEFT || button == BUTTON_BACK)
	{
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
	else if (button == BUTTON_SELECT || button == BUTTON_RIGHT)
	{
		applyMaxBolusPercent(editMaxBolusPercent);
		savePersistentSettings();
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
}

void handleStartupMotorSpeedEditorButton(Button button)
{
	if (button == BUTTON_UP)
		changeStartupMotorSpeedPercent(1);
	else if (button == BUTTON_DOWN)
		changeStartupMotorSpeedPercent(-1);
	else if (button == BUTTON_LEFT || button == BUTTON_BACK)
	{
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
	else if (button == BUTTON_SELECT || button == BUTTON_RIGHT)
	{
		applyStartupJogSpeedPercent(editStartupJogSpeedPercent);
		savePersistentSettings();
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
}

void handleBolusEditorButton(Button button)
{
	lastBolusInputMillis = millis();

	if (button == BUTTON_LEFT)
		moveEditCursor(-1);
	else if (button == BUTTON_RIGHT)
		moveEditCursor(1);
	else if (button == BUTTON_UP)
		changeBolusDigit(1);
	else if (button == BUTTON_DOWN)
		changeBolusDigit(-1);
	else if (button == BUTTON_SELECT)
	{
		if (prepareBolusPlan())
		{
			currentScreen = SCREEN_CONFIRM_BOLUS;
			lastBolusInputMillis = millis();
			beep(8, 30);
			showBolusConfirm();
		}
	}
	else if (button == BUTTON_BACK)
	{
		currentScreen = SCREEN_PUMP_RUNNING;
		beep(8, 30);
		showPumpScreen();
	}
}

void handleBolusConfirmButton(Button button)
{
	// Jede Eingabe verlaengert den Inaktivitaets-Timeout, damit der
	// Bestaetigungs-Screen nicht direkt vor dem SELECT verschwindet.
	if (button != BUTTON_NONE)
		lastBolusInputMillis = millis();

	if (button == BUTTON_LEFT)
	{
		currentScreen = SCREEN_EDIT_BOLUS;
		beep(8, 30);
		showBolusEditor();
	}
	else if (button == BUTTON_SELECT || button == BUTTON_RIGHT)
	{
		startBolus();
	}
}

void handleInfusionConfirmButton(Button button)
{
	if (button == BUTTON_LEFT)
	{
		currentScreen = pendingInfusionTargetVolumeEnabled ? SCREEN_EDIT_TIME : SCREEN_EDIT_RATE;
		beep(8, 30);
		showCurrentScreen();
	}
	else if (button == BUTTON_SELECT || button == BUTTON_RIGHT)
	{
		beep(8, 30);
		if (pendingInfusionTargetVolumeEnabled)
			startVolumeTimeInfusion();
		else
		{
			targetVolumeEnabled = false;
			startPump();
		}
	}
}

void moveEditCursor(int8_t delta)
{
	uint8_t maxIndex = currentScreen == SCREEN_EDIT_TIME ? 5 : 4;
	if (currentScreen == SCREEN_EDIT_BOLUS)
		maxIndex = 3;

	if (delta < 0)
		editCursorIndex = editCursorIndex == 0 ? maxIndex : editCursorIndex - 1;
	else
		editCursorIndex = editCursorIndex == maxIndex ? 0 : editCursorIndex + 1;

	beep(8, 30);
	showCurrentScreen();
}

void changeRateDigit(int8_t delta)
{
	int32_t nextRate = (int32_t)editRateCentiMlPerHour + ((int32_t)delta * (int32_t)rateDigitFactor());
	const int32_t minRate = (int32_t)MIN_FLOW_CENTI_ML_PER_HOUR;
	const int32_t maxRate = (int32_t)MAX_FLOW_CENTI_ML_PER_HOUR;

	if (nextRate < minRate)
		nextRate = minRate;
	if (nextRate > maxRate)
		nextRate = maxRate;

	editRateCentiMlPerHour = (uint32_t)nextRate;
	beep(8, 30);
	showRateEditor();
}

void changeVolumeDigit(int8_t delta)
{
	int32_t nextVolume = (int32_t)editVolumeCentiMl + ((int32_t)delta * (int32_t)volumeDigitFactor());
	uint32_t maxVolume = maxVolumeCentiMl();

	if (nextVolume < 1)
		nextVolume = 1;
	if ((uint32_t)nextVolume > maxVolume)
		nextVolume = (int32_t)maxVolume;

	editVolumeCentiMl = (uint32_t)nextVolume;
	beep(8, 30);
	showVolumeEditor();
}

void changeTimeDigit(int8_t delta)
{
	int32_t nextTime = (int32_t)editTimeSeconds + ((int32_t)delta * (int32_t)timeDigitFactor());

	if (nextTime < 0)
		nextTime = 0;
	if (nextTime > 359999)
		nextTime = 359999;

	editTimeSeconds = (uint32_t)nextTime;
	beep(8, 30);
	showTimeEditor();
}

void changeDeliveryDigit(int8_t delta)
{
	int32_t nextDelivery = (int32_t)editDeliveryCentiMlPerCm + ((int32_t)delta * (int32_t)deliveryDigitFactor());
	const int32_t minDelivery = (int32_t)MIN_DELIVERY_CENTI_ML_PER_CM;
	const int32_t maxDelivery = (int32_t)MAX_DELIVERY_CENTI_ML_PER_CM;

	if (nextDelivery < minDelivery)
		nextDelivery = minDelivery;
	if (nextDelivery > maxDelivery)
		nextDelivery = maxDelivery;

	editDeliveryCentiMlPerCm = (uint32_t)nextDelivery;
	beep(8, 30);
	showDeliveryEditor();
}

void changeSyringeDigit(int8_t delta)
{
	int32_t nextSyringe = (int32_t)editSyringeCentiMl + ((int32_t)delta * (int32_t)deliveryDigitFactor());

	if (nextSyringe < (int32_t)MIN_SYRINGE_CENTI_ML)
		nextSyringe = (int32_t)MIN_SYRINGE_CENTI_ML;
	if (nextSyringe > (int32_t)MAX_SYRINGE_CENTI_ML)
		nextSyringe = (int32_t)MAX_SYRINGE_CENTI_ML;

	editSyringeCentiMl = (uint32_t)nextSyringe;
	beep(8, 30);
	showSyringeEditor();
}

void changeMaxBolusPercent(int8_t delta)
{
	int16_t nextPercent = (int16_t)editMaxBolusPercent + delta;
	if (nextPercent < MIN_MAX_BOLUS_PERCENT)
		nextPercent = MIN_MAX_BOLUS_PERCENT;
	if (nextPercent > MAX_MAX_BOLUS_PERCENT)
		nextPercent = MAX_MAX_BOLUS_PERCENT;

	editMaxBolusPercent = (uint8_t)nextPercent;
	beep(8, 30);
	showMaxBolusEditor();
}

void changeStartupMotorSpeedPercent(int8_t delta)
{
	int16_t nextPercent = (int16_t)editStartupJogSpeedPercent + ((int16_t)delta * STARTUP_JOG_SPEED_STEP_PERCENT);
	if (nextPercent < MIN_STARTUP_JOG_SPEED_PERCENT)
		nextPercent = MIN_STARTUP_JOG_SPEED_PERCENT;
	if (nextPercent > MAX_STARTUP_JOG_SPEED_PERCENT)
		nextPercent = MAX_STARTUP_JOG_SPEED_PERCENT;

	editStartupJogSpeedPercent = (uint8_t)nextPercent;
	beep(8, 30);
	showStartupMotorSpeedEditor();
}

void changeBolusDigit(int8_t delta)
{
	int32_t nextBolus = (int32_t)editBolusCentiMl + ((int32_t)delta * (int32_t)bolusDigitFactor());
	uint32_t maxBolus = maxBolusCentiMl();

	if (nextBolus < (int32_t)MIN_BOLUS_CENTI_ML)
		nextBolus = (int32_t)MIN_BOLUS_CENTI_ML;
	if ((uint32_t)nextBolus > maxBolus)
		nextBolus = maxBolus;

	editBolusCentiMl = (uint32_t)nextBolus;
	beep(8, 30);
	showBolusEditor();
}

uint16_t rateDigitFactor()
{
	return deliveryDigitFactor();
}

uint16_t volumeDigitFactor()
{
	return deliveryDigitFactor();
}

uint32_t timeDigitFactor()
{
	switch (editCursorIndex)
	{
	case 0:
		return 36000;
	case 1:
		return 3600;
	case 2:
		return 600;
	case 3:
		return 60;
	case 4:
		return 10;
	default:
		return 1;
	}
}

uint16_t deliveryDigitFactor()
{
	switch (editCursorIndex)
	{
	case 0:
		return 10000;
	case 1:
		return 1000;
	case 2:
		return 100;
	case 3:
		return 10;
	default:
		return 1;
	}
}

uint16_t bolusDigitFactor()
{
	switch (editCursorIndex)
	{
	case 0:
		return 1000;
	case 1:
		return 100;
	case 2:
		return 10;
	default:
		return 1;
	}
}

uint8_t editCursorColumn()
{
	if (currentScreen == SCREEN_EDIT_TIME)
	{
		switch (editCursorIndex)
		{
		case 0:
			return 0;
		case 1:
			return 1;
		case 2:
			return 3;
		case 3:
			return 4;
		case 4:
			return 6;
		default:
			return 7;
		}
	}

	if (currentScreen == SCREEN_EDIT_BOLUS)
	{
		switch (editCursorIndex)
		{
		case 0:
			return 10;
		case 1:
			return 11;
		case 2:
			return 13;
		default:
			return 14;
		}
	}

	switch (editCursorIndex)
	{
	case 0:
		return 0;
	case 1:
		return 1;
	case 2:
		return 2;
	case 3:
		return 4;
	default:
		return 5;
	}
}

uint32_t maxBolusCentiMl()
{
	uint32_t limit = (syringeCentiMl * (uint32_t)maxBolusPercent + 50UL) / 100UL;
	return limit < MIN_BOLUS_CENTI_ML ? MIN_BOLUS_CENTI_ML : limit;
}

uint32_t maxVolumeCentiMl()
{
	return syringeCentiMl;
}

float syringeVolumeMl()
{
	return (float)syringeCentiMl / 100.0;
}

bool calculateVolumeTimeFlow(float *calculatedFlow)
{
	if (targetTimeSeconds == 0 || targetVolumeMl <= 0.0)
	{
		showStatus(F("Ungueltige Zeit"), F("oder Volumen"));
		return false;
	}

	if (targetVolumeMl > syringeVolumeMl())
	{
		showStatus(F("Vol > Spritze"), F("Setting pruefen"));
		return false;
	}

	float nextFlow = targetVolumeMl * 3600.0 / (float)targetTimeSeconds;
	if (nextFlow < MIN_FLOW_ML_PER_HOUR || nextFlow > MAX_FLOW_ML_PER_HOUR)
	{
		showStatus(F("Rate ungueltig"), F("Zeit/Vol pruefen"));
		return false;
	}

	*calculatedFlow = nextFlow;
	return true;
}

bool startVolumeTimeInfusion()
{
	float calculatedFlow;
	if (!calculateVolumeTimeFlow(&calculatedFlow))
		return false;

	flowMlPerHour = calculatedFlow;
	targetVolumeEnabled = true;
	if (startPump())
		return true;

	targetVolumeEnabled = false;
	return false;
}

bool prepareBolusPlan()
{
	if (!bolusEnabled)
	{
		showStatus(F("Bolus AUS"), F("Einstellung"));
		return false;
	}

	return calculateBolusPlan(&pendingBolusFlowMlPerHour, &pendingBolusDurationMillis);
}

bool calculateBolusPlan(float *plannedFlowMlPerHour, uint32_t *plannedDurationMillis)
{
	if (!bolusEnabled)
	{
		showStatus(F("Bolus AUS"), F("Einstellung"));
		return false;
	}

	if (!pumpRunning || bolusActive)
	{
		showStatus(F("Bolus blockiert"), F("Bitte warten"));
		return false;
	}

	float availableFlowMlPerHour = MAX_FLOW_ML_PER_HOUR - baseInfusionFlowMlPerHour;
	if (availableFlowMlPerHour <= 0.0)
	{
		showStatus(F("Bolus blockiert"), F("Rate Maximum"));
		return false;
	}

	float bolusMl = (float)editBolusCentiMl / 100.0;
	uint32_t durationMillis = BOLUS_DURATION_MS;
	float bolusFlowMlPerHour = bolusMl * 3600000.0 / (float)durationMillis;
	if (baseInfusionFlowMlPerHour + bolusFlowMlPerHour > MAX_FLOW_ML_PER_HOUR)
	{
		durationMillis = (uint32_t)((bolusMl * 3600000.0 / availableFlowMlPerHour) + 0.999);
		if (durationMillis < BOLUS_DURATION_MS)
			durationMillis = BOLUS_DURATION_MS;
		bolusFlowMlPerHour = bolusMl * 3600000.0 / (float)durationMillis;
	}

	float totalFlow = baseInfusionFlowMlPerHour + bolusFlowMlPerHour;
	if (totalFlow > MAX_FLOW_ML_PER_HOUR)
		totalFlow = MAX_FLOW_ML_PER_HOUR;

	float requestedStepRateHz = stepRateHzForFlow(totalFlow);
	if (requestedStepRateHz > maxStepRateHz())
	{
		showStatus(F("Bolus zu schnell"), F("Motor Limit"));
		return false;
	}

	if (!timerCanRepresentStepRate(requestedStepRateHz))
	{
		showStatus(F("Bolus zu langsam"), F("Timer Limit"));
		return false;
	}

	*plannedFlowMlPerHour = totalFlow;
	*plannedDurationMillis = durationMillis;
	return true;
}

bool startBolus()
{
	float plannedFlowMlPerHour;
	uint32_t plannedDurationMillis;
	if (!calculateBolusPlan(&plannedFlowMlPerHour, &plannedDurationMillis))
		return false;

	if (!applyFlowRate(plannedFlowMlPerHour))
		return false;

	bolusDurationMillis = plannedDurationMillis;
	bolusActive = true;
	bolusStartMillis = millis();
	currentScreen = SCREEN_PUMP_RUNNING;
	beep(20, 40);
	showPumpScreen();
	return true;
}

void updateBolusState()
{
	if ((currentScreen == SCREEN_EDIT_BOLUS || currentScreen == SCREEN_CONFIRM_BOLUS) && millis() - lastBolusInputMillis >= BOLUS_MENU_TIMEOUT_MS)
	{
		currentScreen = SCREEN_PUMP_RUNNING;
		showPumpScreen();
	}

	if (bolusActive && millis() - bolusStartMillis >= bolusDurationMillis)
	{
		bolusActive = false;
		if (!applyFlowRate(baseInfusionFlowMlPerHour))
		{
			// Basisrate konnte nicht wiederhergestellt werden -> Pumpe darf
			// nicht auf Bolusrate weiterlaufen. Sicherer Stop mit Alarm.
			stopPump(STOP_TIMER_ERROR);
			return;
		}
		if (!statusActive)
			showPumpScreen();
	}
}

bool applyFlowRate(float nextFlowMlPerHour)
{
	if (nextFlowMlPerHour < MIN_FLOW_ML_PER_HOUR || nextFlowMlPerHour > MAX_FLOW_ML_PER_HOUR)
	{
		showStatus(F("Rate ungueltig"), F("Wert pruefen"));
		return false;
	}

	float requestedStepRateHz = stepRateHzForFlow(nextFlowMlPerHour);
	if (requestedStepRateHz > maxStepRateHz())
	{
		showStatus(F("Rate zu schnell"), F("Motor Limit"));
		return false;
	}

	if (!timerCanRepresentStepRate(requestedStepRateHz))
	{
		showStatus(F("Rate zu langsam"), F("Timer Limit"));
		return false;
	}

	float previousFlow = flowMlPerHour;
	flowMlPerHour = nextFlowMlPerHour;

	if (!configureTimer1PreservePhase(requestedStepRateHz))
	{
		flowMlPerHour = previousFlow;
		configureTimer1(stepRateHzForFlow(flowMlPerHour));
		enableTimer1Interrupt();
		showStatus(F("Timer setup err"), F("Check flow rate "));
		return false;
	}

	enableTimer1InterruptPreservePhase();
	return true;
}

bool changeRunningInfusionRate(int8_t deltaSteps)
{
	if (!pumpRunning || deltaSteps == 0)
		return false;

	float bolusFlowMlPerHour = bolusActive ? flowMlPerHour - baseInfusionFlowMlPerHour : 0.0;
	if (bolusFlowMlPerHour < 0.0)
		bolusFlowMlPerHour = 0.0;

	float nextBaseFlowMlPerHour = baseInfusionFlowMlPerHour + ((float)deltaSteps * FLOW_STEP_ML_PER_HOUR);
	if (nextBaseFlowMlPerHour < MIN_FLOW_ML_PER_HOUR)
		nextBaseFlowMlPerHour = MIN_FLOW_ML_PER_HOUR;

	float maxBaseFlowMlPerHour = MAX_FLOW_ML_PER_HOUR - bolusFlowMlPerHour;
	if (maxBaseFlowMlPerHour < MIN_FLOW_ML_PER_HOUR)
		maxBaseFlowMlPerHour = MIN_FLOW_ML_PER_HOUR;
	if (nextBaseFlowMlPerHour > maxBaseFlowMlPerHour)
		nextBaseFlowMlPerHour = maxBaseFlowMlPerHour;

	if (nextBaseFlowMlPerHour == baseInfusionFlowMlPerHour)
	{
		beep(8, 30);
		showPumpScreen();
		return true;
	}

	float nextTotalFlowMlPerHour = nextBaseFlowMlPerHour + bolusFlowMlPerHour;
	if (!applyFlowRate(nextTotalFlowMlPerHour))
		return false;

	baseInfusionFlowMlPerHour = nextBaseFlowMlPerHour;
	beep(8, 30);
	showPumpScreen();
	return true;
}

void beep(uint16_t delayOn, uint16_t delayOff, bool force)
{
	if (!force && !soundEnabled)
		return;

	if (alarmActive)
		return;

	if (buzzerQueueCount >= BUZZER_QUEUE_CAPACITY)
	{
		buzzerOverflowPending = true;
		return;
	}

	buzzerQueue[buzzerQueueTail].onMs = delayOn;
	buzzerQueue[buzzerQueueTail].offMs = delayOff;
	buzzerQueueTail = (buzzerQueueTail + 1) % BUZZER_QUEUE_CAPACITY;
	buzzerQueueCount++;
	updateBuzzer();
}

void updateBuzzer()
{
	uint32_t nowMillis = millis();
	if (buzzerOutputActive)
	{
		if ((int32_t)(nowMillis - buzzerNextTransitionMillis) >= 0)
		{
			digitalWrite(BUZZER_PIN, LOW);
			buzzerOutputActive = false;
			buzzerNextTransitionMillis = nowMillis + buzzerCurrentOffMs;
		}
		return;
	}

	if (buzzerOverflowPending && buzzerQueueCount < BUZZER_QUEUE_CAPACITY)
	{
		buzzerQueue[buzzerQueueTail].onMs = 8;
		buzzerQueue[buzzerQueueTail].offMs = 30;
		buzzerQueueTail = (buzzerQueueTail + 1) % BUZZER_QUEUE_CAPACITY;
		buzzerQueueCount++;
		buzzerOverflowPending = false;
	}

	if ((int32_t)(nowMillis - buzzerNextTransitionMillis) < 0 || buzzerQueueCount == 0)
		return;

	BuzzerBeep currentBeep = buzzerQueue[buzzerQueueHead];
	buzzerQueueHead = (buzzerQueueHead + 1) % BUZZER_QUEUE_CAPACITY;
	buzzerQueueCount--;
	buzzerCurrentOffMs = currentBeep.offMs;
	buzzerOutputActive = true;
	buzzerNextTransitionMillis = nowMillis + currentBeep.onMs;
	digitalWrite(BUZZER_PIN, HIGH);
}

void cancelBuzzerBeeps()
{
	buzzerQueueHead = 0;
	buzzerQueueTail = 0;
	buzzerQueueCount = 0;
	buzzerOutputActive = false;
	buzzerOverflowPending = false;
	buzzerCurrentOffMs = 0;
	buzzerNextTransitionMillis = 0;
}

void serviceWatchdog()
{
}

void watchdogDelay(uint16_t durationMs)
{
	serviceWait(durationMs);
}

void serviceWait(uint16_t durationMs)
{
	uint32_t startMillis = millis();
	while (millis() - startMillis < durationMs)
	{
		serviceWatchdog();
		serviceMotorClock();
		updateBuzzer();
		uint16_t remaining = durationMs - (uint16_t)(millis() - startMillis);
		delay(remaining > 5 ? 5 : remaining);
	}
}

void setStepPinLow()
{
	digitalWrite(STEP_PIN, LOW);
}

void setStepPinHigh()
{
	digitalWrite(STEP_PIN, HIGH);
}

void serviceMotorClock()
{
	if (!motorClockRunning)
		return;

	uint32_t nowMicros = micros();
	if ((int32_t)(nowMicros - motorClockNextTransitionMicros) < 0)
		return;

	if (motorClockStopOnAnyEndstop ? anyEndstopActive() : endstopActiveForDirection(motorClockDirectionForward))
	{
		endstopInterruptLatched = true;
		stopMotorClock();
		return;
	}

	if (stepPulseHigh)
	{
		setStepPinLow();
		stepPulseHigh = false;
		stepCounter++;
		motorClockNextTransitionMicros = nowMicros + motorClockLowPulseUs;
	}
	else
	{
		setStepPinHigh();
		stepPulseHigh = true;
		motorClockNextTransitionMicros = nowMicros + motorClockHighPulseUs;
	}
}

void setFlow(float nextFlow)
{
	if (nextFlow < MIN_FLOW_ML_PER_HOUR)
		nextFlow = MIN_FLOW_ML_PER_HOUR;
	if (nextFlow > MAX_FLOW_ML_PER_HOUR)
		nextFlow = MAX_FLOW_ML_PER_HOUR;

	flowMlPerHour = nextFlow;
	beep(8, 30);
	showIdleScreen();
}

void setupTmc2209Uart()
{
	tmc2209UartConfigured = false;
	tmc2209UartVerified = false;
	tmc2209WriteOnlyMode = false;
	tmc2209Ms1High = false;
	tmc2209Ms2High = false;
	tmc2209IfcCount = 0;
	tmc2209Version = 0;
	tmc2209ConnectionStatus = 0xFF;
	tmc2209DetectedAddress = TMC2209_UART_DRIVER_ADDRESS;
	tmc2209ReadbackMicrosteps = 0;
	tmc2209ReadbackCurrentMa = 0;

	tmc2209Serial.begin(TMC2209_UART_BAUD);
	tmc2209Driver.begin();
	tmc2209Driver.toff(4);
	tmc2209Driver.pdn_disable(1);
	tmc2209Driver.en_spreadCycle(0);  // Force StealthChop; required for StallGuard4
	tmc2209Driver.mstep_reg_select(1);
	tmc2209Driver.rms_current(MOTOR_CURRENT_MA);
	tmc2209Driver.microsteps(NOFMICROSTEPS);
	tmc2209Driver.intpol(1);

	// CoolStep is intentionally disabled. Datasheet §12.3 warns that dynamic
	// current scaling driven by CoolStep destabilises SG_RESULT, and CoolStep
	// itself is silently inactive once IRUN drops below 10. For a load-sensing
	// use case we want a constant current envelope so the SG reading reflects
	// mechanical load and nothing else.
	tmc2209Driver.semin(0);
	tmc2209Driver.semax(0);
	tmc2209Driver.sedn(0);

	// Force StealthChop at all velocities so SG4 is always the active stall
	// detector (SpreadCycle uses SG2-style coil-current sensing instead).
	tmc2209Driver.TPWMTHRS(0);

	// We poll SG_RESULT directly via UART and do not use the DIAG stall pin,
	// so the on-chip stall threshold is parked at 0 to prevent spurious DIAG
	// activity. TCOOLTHRS is left at maximum so SG_RESULT updates at every
	// step rate the pump can produce.
	tmc2209Driver.SGTHRS(0);
	tmc2209Driver.TCOOLTHRS(0xFFFFF);

	tmc2209UartConfigured = true;
	tmc2209UartVerified = verifyTmc2209Readback();
}

bool verifyTmc2209Readback()
{
	for (uint8_t attempt = 0; attempt < TMC2209_READBACK_ATTEMPTS; attempt++)
	{
		tmc2209ConnectionStatus = tmc2209Driver.test_connection();
		if (tmc2209ConnectionStatus == 0)
		{
			tmc2209Version = tmc2209Driver.version();
			tmc2209ReadbackMicrosteps = tmc2209Driver.microsteps();
			tmc2209ReadbackCurrentMa = tmc2209Driver.rms_current();
			tmc2209IfcCount = attempt + 1;
			if (tmc2209Version == TMC2209_EXPECTED_VERSION &&
				tmc2209ReadbackMicrosteps == NOFMICROSTEPS &&
				tmc2209CurrentReadbackOk(tmc2209ReadbackCurrentMa))
			{
				return true;
			}
		}
		serviceWait(15);
	}

	return false;
}

bool tmc2209CurrentReadbackOk(uint16_t currentMa)
{
	uint16_t minCurrentMa = MOTOR_CURRENT_MA > TMC2209_CURRENT_TOLERANCE_MA ? MOTOR_CURRENT_MA - TMC2209_CURRENT_TOLERANCE_MA : 1;
	uint16_t maxCurrentMa = MOTOR_CURRENT_MA + TMC2209_CURRENT_TOLERANCE_MA;
	return currentMa >= minCurrentMa && currentMa <= maxCurrentMa;
}

void updateStartupMotorJog(Button heldButton)
{
	if (heldButton != BUTTON_LEFT && heldButton != BUTTON_RIGHT)
	{
		stopStartupMotorJog();
		return;
	}

	if (startupJogRunning && !timerIsRunning())
	{
		stopStartupMotorJog();
		return;
	}

	bool nextDirectionForward = heldButton == BUTTON_RIGHT;
	if (startupJogRunning && startupJogDirectionForward == nextDirectionForward)
		return;

	stopStartupMotorJog();
	startStartupMotorJog(nextDirectionForward);
}

bool startStartupMotorJog(bool forward)
{
	if (!testModeEnabled && !tmc2209UartVerified)
		return false;

	if (!testModeEnabled && endstopActiveForDirection(forward))
		return false;

	noInterrupts();
	stepCounter = 0;
	stepPulseHigh = false;
	endstopInterruptLatched = false;
	interrupts();
	setStepPinLow();

	setMotorDirection(forward);

	float jogSpeedFactor = (float)startupJogSpeedPercent / 100.0;
	float startupJogMaxStepRateHz =
		(float)STARTUP_JOG_MAX_RPM * (float)NOFSTEPSPER360 * (float)STARTUP_JOG_MICROSTEPS / 60.0;
	if (!configureTimer1(startupJogMaxStepRateHz * jogSpeedFactor))
		return false;

	if (!testModeEnabled && tmc2209UartConfigured)
		tmc2209Driver.microsteps(STARTUP_JOG_MICROSTEPS);

	motorClockDirectionForward = forward;
	motorClockStopOnAnyEndstop = false;
	enableMotorDriver();
	enableTimer1Interrupt();
	startupJogRunning = true;
	startupJogDirectionForward = forward;
	return true;
}

void stopStartupMotorJog()
{
	if (!startupJogRunning)
		return;

	disableTimer1();
	if (!testModeEnabled && tmc2209UartConfigured)
		tmc2209Driver.microsteps(NOFMICROSTEPS);
	disableMotorDriver();
	startupJogRunning = false;
}

bool startMotorSelfTest()
{
	if (!testModeEnabled && !tmc2209UartVerified)
	{
		showStatus(F("Test blockiert"), F("TMC UART fehlt"));
		beep(250, 80, true);
		return false;
	}

	if (!testModeEnabled && endstopActiveForDirection(motorTestDirectionForward))
	{
		showStatus(F("Test blockiert"), F("Endstop aktiv"));
		return false;
	}

	noInterrupts();
	stepCounter = 0;
	stepPulseHigh = false;
	endstopInterruptLatched = false;
	interrupts();
	setStepPinLow();

	setMotorDirection(motorTestDirectionForward);

	if (!configureTimer1(MOTOR_TEST_STEP_RATE_HZ))
	{
		showStatus(F("Test blockiert"), F("Timerfehler"));
		return false;
	}

	motorClockDirectionForward = motorTestDirectionForward;
	motorClockStopOnAnyEndstop = false;
	enableMotorDriver();
	enableTimer1Interrupt();
	motorTestRunning = true;
	settingsSupportStartMillis = millis();
	return true;
}

void stopMotorSelfTest()
{
	disableTimer1();
	disableMotorDriver();
	motorTestRunning = false;
}

bool canStartPump()
{
	if (!testModeEnabled && !tmc2209UartVerified)
	{
		showStatus(F("TMC2209 Fehler"), F("UART lesen fail"));
		beep(250, 80, true);
		return false;
	}

	if (forwardEndstopActive())
	{
		showStatus(F("Forward blocked"), F("Endstop aktiv  "));
		beep(250, 80, true);
		return false;
	}

	if (backwardEndstopActive())
	{
		showStatus(F("Backward blocked"), F("Endstop aktiv  "));
		beep(250, 80, true);
		return false;
	}

	return true;
}

bool forwardEndstopActive()
{
	if (testModeEnabled)
		return false;

	return digitalRead(ENDSTOP_PIN_FORWARD) == HIGH;
}

bool backwardEndstopActive()
{
	if (testModeEnabled)
		return false;

	return digitalRead(ENDSTOP_PIN_BACKWARD) == HIGH;
}

bool anyEndstopActive()
{
	return forwardEndstopActive() || backwardEndstopActive();
}

bool endstopActiveForDirection(bool forward)
{
	// Jog/Test-"forward" entspricht mechanisch der Rueckzugsrichtung.
	return forward ? backwardEndstopActive() : forwardEndstopActive();
}

void setMotorDirection(bool forward)
{
	bool previousDirHigh = forward
		? (motorDirectionInverted ? LOW : HIGH)
		: (motorDirectionInverted ? HIGH : LOW);
	tmc2209Driver.shaft(previousDirHigh == HIGH);
}

void enableMotorDriver()
{
	if (tmc2209UartConfigured)
		tmc2209Driver.toff(3);
}

void disableMotorDriver()
{
	if (tmc2209UartConfigured)
		tmc2209Driver.toff(0);
	setStepPinLow();
}

void configureEndstopInterrupts()
{
	endstopInterruptLatched = false;
}

void stopMotorClock()
{
	// On UNO R4 this is called from the software motor clock service.
	motorClockRunning = false;
	setStepPinLow();
}

bool startPump()
{
	if (!canStartPump())
		return false;

	if (flowMlPerHour < MIN_FLOW_ML_PER_HOUR || flowMlPerHour > MAX_FLOW_ML_PER_HOUR)
	{
		showStatus(F("Rate ungueltig"), F("Wert pruefen"));
		beep(250, 80, true);
		return false;
	}

	float requestedStepRateHz = stepRateHzForFlow(flowMlPerHour);
	if (requestedStepRateHz > maxStepRateHz())
	{
		showStatus(F("Rate zu schnell"), F("Motor Limit"));
		beep(250, 80, true);
		return false;
	}

	if (!timerCanRepresentStepRate(requestedStepRateHz))
	{
		showStatus(F("Rate zu langsam"), F("Timer Limit"));
		beep(250, 80, true);
		return false;
	}

	setMotorDirection(false);
	motorClockDirectionForward = false;
	motorClockStopOnAnyEndstop = true;

	noInterrupts();
	stepCounter = 0;
	stepPulseHigh = false;
	endstopInterruptLatched = false;
	interrupts();
	setStepPinLow();

	if (!configureTimer1(requestedStepRateHz))
	{
		showStatus(F("Timer setup err"), F("Check flow rate "));
		beep(250, 80, true);
		return false;
	}

	baseInfusionFlowMlPerHour = flowMlPerHour;
	bolusActive = false;
	// Reset pressure smoothing & auto-baseline state so each new run gets
	// a fresh reference (unless the user has a persisted manual baseline).
	pressureFilteredSgValid = false;
	pressureAutoBaselineSum = 0;
	pressureAutoBaselineCount = 0;
	pressureAutoBaselineDone = (pressureBaseline != 0);
	pressureHighTicks = 0;
	enableMotorDriver();
	enableTimer1Interrupt();
	pumpStartMillis = millis();
	lastDisplayMillis = 0;
	pumpRunning = true;
	currentScreen = SCREEN_PUMP_RUNNING;
	beep();
	showPumpScreen();
	return true;
}

void stopPump(PumpStopReason reason)
{
	bool expectedForwardEndstop = reason == STOP_ENDSTOP && !targetVolumeEnabled && forwardEndstopActive() && !backwardEndstopActive();

	flowMlPerHour = bolusActive ? baseInfusionFlowMlPerHour : flowMlPerHour;
	baseInfusionFlowMlPerHour = flowMlPerHour;

	disableTimer1();
	disableMotorDriver();
	pumpRunning = false;
	if (reason == STOP_MANUAL || reason == STOP_TARGET_VOLUME || expectedForwardEndstop)
		targetVolumeEnabled = false;
	bolusActive = false;
	currentScreen = SCREEN_MAIN_MENU;

	if (reason == STOP_MANUAL)
		showStatus(F("Perfusor STOP   "), F("Manueller Stopp    "));
	else if (expectedForwardEndstop)
		showAcknowledgedStatus(F("Ziel erreicht"), F("Infusion STOP"));
	else if (reason == STOP_ENDSTOP)
		showAlarm(F("ALARM ENDSTOP  "), F("SELECT quittiert"));
	else if (reason == STOP_TARGET_VOLUME)
		showAcknowledgedStatus(F("Ziel erreicht"), F("Infusion STOP"));
	else
		showAlarm(F("ALARM TIMEOUT    "), F("SELECT quittiert"));
}

void formatElapsedTime(uint32_t elapsedSeconds, char *buffer, size_t bufferSize)
{
	uint32_t hours = elapsedSeconds / 3600UL;
	uint8_t minutes = (elapsedSeconds / 60UL) % 60UL;
	uint8_t seconds = elapsedSeconds % 60UL;

	if (hours > 9)
		snprintf(buffer, bufferSize, "9:59:59");
	else
		snprintf(buffer, bufferSize, "%lu:%02u:%02u", (unsigned long)hours, (unsigned int)minutes, (unsigned int)seconds);
}

void enableTimer1Interrupt()
{
	noInterrupts();
	stepPulseHigh = false;
	setStepPinLow();
	motorClockNextTransitionMicros = micros() + motorClockLowPulseUs;
	motorClockRunning = true;
	interrupts();
}

void enableTimer1InterruptPreservePhase()
{
	noInterrupts();
	if (!motorClockRunning)
		motorClockNextTransitionMicros = micros() + (stepPulseHigh ? motorClockHighPulseUs : motorClockLowPulseUs);
	motorClockRunning = true;
	interrupts();
}

bool timerIsRunning()
{
	noInterrupts();
	bool running = motorClockRunning;
	interrupts();
	return running;
}

bool timerCanRepresentStepRate(float stepRateHz)
{
	if (stepRateHz <= 0.0)
		return false;

	float totalPulseUsFloat = 1000000.0 / stepRateHz;
	if (totalPulseUsFloat > 4294967295.0)
		return false;

	uint32_t totalPulseUs = (uint32_t)(totalPulseUsFloat + 0.5);
	return totalPulseUs > STEP_PULSE_US + 1UL;
}

bool configureTimer1(float stepRateHz)
{
	return configureTimer1Internal(stepRateHz, false);
}

bool configureTimer1PreservePhase(float stepRateHz)
{
	return configureTimer1Internal(stepRateHz, true);
}

bool configureTimer1Internal(float stepRateHz, bool preservePhase)
{
	if (!timerCanRepresentStepRate(stepRateHz))
		return false;

	uint32_t totalPulseUs = (uint32_t)(1000000.0 / stepRateHz + 0.5);
	if (totalPulseUs <= STEP_PULSE_US + 1UL)
		return false;

	noInterrupts();
	motorClockHighPulseUs = STEP_PULSE_US;
	motorClockLowPulseUs = totalPulseUs - STEP_PULSE_US;
	if (!preservePhase || !motorClockRunning)
	{
		stepPulseHigh = false;
		setStepPinLow();
		motorClockNextTransitionMicros = micros() + motorClockLowPulseUs;
		motorClockRunning = false;
	}
	interrupts();
	return true;
}

void disableTimer1()
{
	noInterrupts();
	motorClockRunning = false;
	stepPulseHigh = false;
	setStepPinLow();
	interrupts();
}

float maxStepRateHz()
{
	return (float)MAXRPM * (float)NOFSTEPSPER360 * (float)NOFMICROSTEPS / 60.0;
}

float stepsPerMl()
{
	float effectiveDeliveryMlPerCm = deliveryMlPerCm > 0.0 ? deliveryMlPerCm : DEFAULT_DELIVERY_ML_PER_CM;
	float stepsPerCm = (float)NOFSTEPSPER360 * (float)NOFMICROSTEPS * (10.0 / MMPER360);
	return stepsPerCm / effectiveDeliveryMlPerCm;
}

float stepRateHzForFlow(float mlPerHour)
{
	return mlPerHour * stepsPerMl() / 3600.0;
}

float pumpedMlFromSteps(int32_t steps)
{
	return (float)steps / stepsPerMl();
}

int32_t atomicStepCounter()
{
	int32_t steps;
	noInterrupts();
	steps = stepCounter;
	interrupts();
	return steps;
}

void loadPersistentSettings()
{
	PersistedSettings settings;
	readPersistentSettings(&settings);
	uint32_t deliveryCenti = DEFAULT_DELIVERY_CENTI_ML_PER_CM;
	uint32_t persistedSyringeCenti = DEFAULT_SYRINGE_CENTI_ML;
	uint8_t motorInverted = INVERTDIRECTION ? 1 : 0;
	uint8_t persistedBolusEnabled = DEFAULT_BOLUS_ENABLED;
	uint8_t persistedSoundEnabled = DEFAULT_SOUND_ENABLED;
	uint8_t persistedTestModeEnabled = 0;
	uint8_t persistedPressureMonitorEnabled = 1;
	uint8_t persistedPressureScale = PRESSURE_SCALE_DEFAULT;
	uint8_t persistedPressureAlarmLevel = PRESSURE_ALARM_DEFAULT;
	uint16_t persistedPressureBaseline = PRESSURE_BASELINE_DEFAULT;
	uint8_t persistedMaxBolusPercent = DEFAULT_MAX_BOLUS_PERCENT;
	uint8_t persistedStartupJogSpeedPercent = DEFAULT_STARTUP_JOG_SPEED_PERCENT;
	bool shouldSave = true;
	bool settingsWereResetToDefaults = true;

	if (settings.magic == SETTINGS_MAGIC && settings.version == SETTINGS_VERSION)
	{
		if (validDeliveryCenti(settings.deliveryCentiMlPerCm) &&
			validSyringeCenti(settings.syringeCentiMl) &&
			validMotorDirectionInverted(settings.motorDirectionInverted) &&
			validBolusEnabled(settings.bolusEnabled) &&
			validSoundEnabled(settings.soundEnabled) &&
			validTestModeEnabled(settings.testModeEnabled) &&
			validPressureMonitorEnabled(settings.pressureMonitorEnabled) &&
			validPressureScale(settings.pressureScale) &&
			validPressureAlarmLevel(settings.pressureAlarmLevel) &&
			validPressureBaseline(settings.pressureBaseline) &&
			validMaxBolusPercent(settings.maxBolusPercent) &&
			validStartupJogSpeedPercent(settings.startupJogSpeedPercent) &&
			settingsCrcMatches(&settings))
		{
			deliveryCenti = settings.deliveryCentiMlPerCm;
			persistedSyringeCenti = settings.syringeCentiMl;
			motorInverted = settings.motorDirectionInverted;
			persistedBolusEnabled = settings.bolusEnabled;
			persistedSoundEnabled = settings.soundEnabled;
			persistedTestModeEnabled = settings.testModeEnabled;
			persistedPressureMonitorEnabled = settings.pressureMonitorEnabled;
			persistedPressureScale = settings.pressureScale;
			persistedPressureAlarmLevel = settings.pressureAlarmLevel;
			persistedPressureBaseline = settings.pressureBaseline;
			persistedMaxBolusPercent = settings.maxBolusPercent;
			persistedStartupJogSpeedPercent = settings.startupJogSpeedPercent;
			shouldSave = false;
		}
	}
	else if (settings.magic == SETTINGS_MAGIC && settings.version == 8)
	{
		PersistedSettingsVersion8 previousSettings;
		readPersistentSettingsVersion8(&previousSettings);
		if (validDeliveryCenti(previousSettings.deliveryCentiMlPerCm) &&
			validSyringeCenti(previousSettings.syringeCentiMl) &&
			validMotorDirectionInverted(previousSettings.motorDirectionInverted) &&
			validBolusEnabled(previousSettings.bolusEnabled) &&
			validSoundEnabled(previousSettings.soundEnabled) &&
			validTestModeEnabled(previousSettings.testModeEnabled) &&
			validMaxBolusPercent(previousSettings.maxBolusPercent) &&
			validStartupJogSpeedPercent(previousSettings.startupJogSpeedPercent) &&
			settingsVersion8CrcMatches(&previousSettings))
		{
			deliveryCenti = previousSettings.deliveryCentiMlPerCm;
			persistedSyringeCenti = previousSettings.syringeCentiMl;
			motorInverted = previousSettings.motorDirectionInverted;
			persistedBolusEnabled = previousSettings.bolusEnabled;
			persistedSoundEnabled = previousSettings.soundEnabled;
			persistedTestModeEnabled = previousSettings.testModeEnabled;
			persistedMaxBolusPercent = previousSettings.maxBolusPercent;
			persistedStartupJogSpeedPercent = previousSettings.startupJogSpeedPercent;
			settingsWereResetToDefaults = false;
		}
	}
	else if (settings.magic == SETTINGS_MAGIC && settings.version == 7)
	{
		PersistedSettingsVersion7 previousSettings;
		readPersistentSettingsVersion7(&previousSettings);
		if (validDeliveryCenti(previousSettings.deliveryCentiMlPerCm) &&
			validSyringeCenti(previousSettings.syringeCentiMl) &&
			validMotorDirectionInverted(previousSettings.motorDirectionInverted) &&
			validBolusEnabled(previousSettings.bolusEnabled) &&
			validSoundEnabled(previousSettings.soundEnabled) &&
			validMaxBolusPercent(previousSettings.maxBolusPercent) &&
			validStartupJogSpeedPercent(previousSettings.startupJogSpeedPercent) &&
			settingsVersion7CrcMatches(&previousSettings))
		{
			deliveryCenti = previousSettings.deliveryCentiMlPerCm;
			persistedSyringeCenti = previousSettings.syringeCentiMl;
			motorInverted = previousSettings.motorDirectionInverted;
			persistedBolusEnabled = previousSettings.bolusEnabled;
			persistedSoundEnabled = previousSettings.soundEnabled;
			persistedMaxBolusPercent = previousSettings.maxBolusPercent;
			persistedStartupJogSpeedPercent = previousSettings.startupJogSpeedPercent;
			settingsWereResetToDefaults = false;
		}
	}
	else if (settings.magic == SETTINGS_MAGIC && settings.version == 6)
	{
		PersistedSettingsVersion6 previousSettings;
		readPersistentSettingsVersion6(&previousSettings);
		if (validDeliveryCenti(previousSettings.deliveryCentiMlPerCm) &&
			validSyringeCenti(previousSettings.syringeCentiMl) &&
			validMotorDirectionInverted(previousSettings.motorDirectionInverted) &&
			validBolusEnabled(previousSettings.bolusEnabled) &&
			validSoundEnabled(previousSettings.soundEnabled) &&
			validMaxBolusPercent(previousSettings.maxBolusPercent) &&
			settingsVersion6CrcMatches(&previousSettings))
		{
			deliveryCenti = previousSettings.deliveryCentiMlPerCm;
			persistedSyringeCenti = previousSettings.syringeCentiMl;
			motorInverted = previousSettings.motorDirectionInverted;
			persistedBolusEnabled = previousSettings.bolusEnabled;
			persistedSoundEnabled = previousSettings.soundEnabled;
			persistedMaxBolusPercent = previousSettings.maxBolusPercent;
			persistedStartupJogSpeedPercent = DEFAULT_STARTUP_JOG_SPEED_PERCENT;
			settingsWereResetToDefaults = false;
		}
	}

	applyDeliveryCenti(deliveryCenti);
	applySyringeCenti(persistedSyringeCenti);
	applyMotorDirectionInverted(motorInverted);
	applyBolusEnabled(persistedBolusEnabled);
	applySoundEnabled(persistedSoundEnabled);
	applyTestModeEnabled(persistedTestModeEnabled);
	applyPressureMonitorEnabled(persistedPressureMonitorEnabled);
	applyPressureScale(persistedPressureScale);
	applyPressureAlarmLevel(persistedPressureAlarmLevel);
	applyPressureBaseline(persistedPressureBaseline);
	applyMaxBolusPercent(persistedMaxBolusPercent);
	applyStartupJogSpeedPercent(persistedStartupJogSpeedPercent);

	if (shouldSave)
	{
		settingsResetToDefaults = settingsWereResetToDefaults;
		savePersistentSettings();
	}
}

void savePersistentSettings()
{
	if (!validDeliveryCenti(editDeliveryCentiMlPerCm) || !validSyringeCenti(syringeCentiMl) ||
		!validBolusEnabled(bolusEnabled ? 1 : 0) || !validSoundEnabled(soundEnabled ? 1 : 0) ||
		!validTestModeEnabled(testModeEnabled ? 1 : 0) ||
		!validPressureMonitorEnabled(pressureMonitorEnabled ? 1 : 0) ||
		!validPressureScale(pressureScale) ||
		!validPressureAlarmLevel(pressureAlarmLevel) ||
		!validPressureBaseline(pressureBaseline) ||
		!validMaxBolusPercent(maxBolusPercent) || !validStartupJogSpeedPercent(startupJogSpeedPercent))
		return;

	PersistedSettings previousSettings;
	readPersistentSettings(&previousSettings);

	if (previousSettings.magic == SETTINGS_MAGIC &&
		previousSettings.version == SETTINGS_VERSION &&
		validDeliveryCenti(previousSettings.deliveryCentiMlPerCm) &&
		validSyringeCenti(previousSettings.syringeCentiMl) &&
		validMotorDirectionInverted(previousSettings.motorDirectionInverted) &&
		validBolusEnabled(previousSettings.bolusEnabled) &&
		validSoundEnabled(previousSettings.soundEnabled) &&
		validTestModeEnabled(previousSettings.testModeEnabled) &&
		validPressureMonitorEnabled(previousSettings.pressureMonitorEnabled) &&
		validPressureScale(previousSettings.pressureScale) &&
		validPressureAlarmLevel(previousSettings.pressureAlarmLevel) &&
		validPressureBaseline(previousSettings.pressureBaseline) &&
		validMaxBolusPercent(previousSettings.maxBolusPercent) &&
		validStartupJogSpeedPercent(previousSettings.startupJogSpeedPercent) &&
		settingsCrcMatches(&previousSettings) &&
		previousSettings.deliveryCentiMlPerCm == editDeliveryCentiMlPerCm &&
		previousSettings.syringeCentiMl == syringeCentiMl &&
		previousSettings.bolusEnabled == (bolusEnabled ? 1 : 0) &&
		previousSettings.soundEnabled == (soundEnabled ? 1 : 0) &&
		previousSettings.testModeEnabled == (testModeEnabled ? 1 : 0) &&
		previousSettings.pressureMonitorEnabled == (pressureMonitorEnabled ? 1 : 0) &&
		previousSettings.pressureScale == pressureScale &&
		previousSettings.pressureAlarmLevel == pressureAlarmLevel &&
		previousSettings.pressureBaseline == pressureBaseline &&
		previousSettings.maxBolusPercent == maxBolusPercent &&
		previousSettings.startupJogSpeedPercent == startupJogSpeedPercent &&
		previousSettings.motorDirectionInverted == (motorDirectionInverted ? 1 : 0))
	{
		return;
	}

	PersistedSettings settings;
	settings.magic = SETTINGS_MAGIC;
	settings.version = SETTINGS_VERSION;
	settings.motorDirectionInverted = motorDirectionInverted ? 1 : 0;
	settings.deliveryCentiMlPerCm = editDeliveryCentiMlPerCm;
	settings.syringeCentiMl = syringeCentiMl;
	settings.bolusEnabled = bolusEnabled ? 1 : 0;
	settings.soundEnabled = soundEnabled ? 1 : 0;
	settings.testModeEnabled = testModeEnabled ? 1 : 0;
	settings.maxBolusPercent = maxBolusPercent;
	settings.startupJogSpeedPercent = startupJogSpeedPercent;
	settings.pressureMonitorEnabled = pressureMonitorEnabled ? 1 : 0;
	settings.pressureScale = pressureScale;
	settings.pressureAlarmLevel = pressureAlarmLevel;
	settings.pressureBaseline = pressureBaseline;
	settings.crc = settingsCrc16(&settings);
	writePersistentSettings(&settings);
}

bool validDeliveryCenti(uint32_t value)
{
	return value >= MIN_DELIVERY_CENTI_ML_PER_CM && value <= MAX_DELIVERY_CENTI_ML_PER_CM;
}

bool validSyringeCenti(uint32_t value)
{
	return value >= MIN_SYRINGE_CENTI_ML && value <= MAX_SYRINGE_CENTI_ML;
}

bool validMotorDirectionInverted(uint8_t value)
{
	return value == 0 || value == 1;
}

bool validBolusEnabled(uint8_t value)
{
	return value == 0 || value == 1;
}

bool validSoundEnabled(uint8_t value)
{
	return value == 0 || value == 1;
}

bool validTestModeEnabled(uint8_t value)
{
	return value == 0 || value == 1;
}

bool validPressureMonitorEnabled(uint8_t value)
{
	return value == 0 || value == 1;
}

bool validPressureScale(uint8_t value)
{
	return value >= PRESSURE_SCALE_MIN && value <= PRESSURE_SCALE_MAX;
}

bool validPressureAlarmLevel(uint8_t value)
{
	return value >= PRESSURE_ALARM_MIN && value <= PRESSURE_ALARM_MAX;
}

bool validPressureBaseline(uint16_t value)
{
	return value <= PRESSURE_SG_MAX && (value & 0x0001U) == 0;
}

bool validMaxBolusPercent(uint8_t value)
{
	return value >= MIN_MAX_BOLUS_PERCENT && value <= MAX_MAX_BOLUS_PERCENT;
}

bool validStartupJogSpeedPercent(uint8_t value)
{
	return value >= MIN_STARTUP_JOG_SPEED_PERCENT && value <= MAX_STARTUP_JOG_SPEED_PERCENT;
}

void applyDeliveryCenti(uint32_t value)
{
	if (!validDeliveryCenti(value))
		value = DEFAULT_DELIVERY_CENTI_ML_PER_CM;

	editDeliveryCentiMlPerCm = value;
	deliveryMlPerCm = (float)value / 100.0;
}

void applySyringeCenti(uint32_t value)
{
	if (!validSyringeCenti(value))
		value = DEFAULT_SYRINGE_CENTI_ML;

	syringeCentiMl = value;
	editSyringeCentiMl = value;

	if (targetVolumeMl > syringeVolumeMl())
		targetVolumeMl = syringeVolumeMl();
	if (editVolumeCentiMl > maxVolumeCentiMl())
		editVolumeCentiMl = maxVolumeCentiMl();
	if (editBolusCentiMl > maxBolusCentiMl())
		editBolusCentiMl = maxBolusCentiMl();
	if (editBolusCentiMl < MIN_BOLUS_CENTI_ML)
		editBolusCentiMl = MIN_BOLUS_CENTI_ML;
}

void applyBolusEnabled(uint8_t value)
{
	if (!validBolusEnabled(value))
		value = DEFAULT_BOLUS_ENABLED;

	bolusEnabled = value == 1;
}

void applySoundEnabled(uint8_t value)
{
	if (!validSoundEnabled(value))
		value = DEFAULT_SOUND_ENABLED;

	soundEnabled = value == 1;
}

void applyTestModeEnabled(uint8_t value)
{
	if (!validTestModeEnabled(value))
		value = 0;

	testModeEnabled = value == 1;
}

void applyPressureMonitorEnabled(uint8_t value)
{
	if (!validPressureMonitorEnabled(value))
		value = 1;

	pressureMonitorEnabled = value == 1;
}

void applyPressureScale(uint8_t value)
{
	if (!validPressureScale(value))
		value = PRESSURE_SCALE_DEFAULT;

	pressureScale = value;
}

void applyPressureAlarmLevel(uint8_t value)
{
	if (!validPressureAlarmLevel(value))
		value = PRESSURE_ALARM_DEFAULT;

	pressureAlarmLevel = value;
}

void applyPressureBaseline(uint16_t value)
{
	if (!validPressureBaseline(value))
		value = PRESSURE_BASELINE_DEFAULT;

	pressureBaseline = value;
}

uint16_t readPressureSg()
{
	// Without a verified UART link to the driver we cannot read SG_RESULT.
	// Also skip the read in test mode (no real driver attached) and when
	// the motor is not stepping, since StallGuard4 needs continuous step
	// pulses to produce a meaningful value.
	if (!tmc2209UartConfigured || !tmc2209UartVerified || testModeEnabled || !pumpRunning)
	{
		pressureFilteredSgValid = false;
		pressureAutoBaselineSum = 0;
		pressureAutoBaselineCount = 0;
		pressureAutoBaselineDone = false;
		return PRESSURE_SG_MAX;
	}

	// Datasheet: SG_RESULT is exposed as a 10-bit register, but bits 9 and 0
	// are fixed to 0 on the TMC2209. Keep only the documented payload bits.
	uint16_t raw = tmc2209Driver.SG_RESULT() & PRESSURE_SG_MASK;

	// Exponential moving average to smooth out the SG noise. SG_RESULT only
	// refreshes once per full step, so at pump speeds (sub-1 RPM) successive
	// reads can return identical values for a while followed by a sudden
	// jump. The EWMA hides the staircase and improves bar stability.
	if (!pressureFilteredSgValid)
	{
		pressureFilteredSg = raw;
		pressureFilteredSgValid = true;
	}
	else
	{
		int16_t delta = (int16_t)raw - (int16_t)pressureFilteredSg;
		pressureFilteredSg = (uint16_t)((int16_t)pressureFilteredSg + (delta >> PRESSURE_SAMPLE_FILTER_SHIFT));
	}

	// Auto-baseline: if the user has never run a manual zero calibration,
	// average the first N unloaded samples after pump start and adopt that
	// as the reference. Without this the bar can never read 0% because the
	// old theoretical fallback reference of 510 is unrealistically high.
	if (!pressureAutoBaselineDone && pressureBaseline == 0)
	{
		if (raw > 0)
		{
			pressureAutoBaselineSum += raw;
			pressureAutoBaselineCount++;
			if (pressureAutoBaselineCount >= PRESSURE_AUTO_BASELINE_SAMPLES)
			{
				uint16_t avg = (uint16_t)(pressureAutoBaselineSum / pressureAutoBaselineCount);
				if (avg > PRESSURE_EMPTY_BAR_SG)
					avg = PRESSURE_EMPTY_BAR_SG;
				pressureBaseline = avg & PRESSURE_SG_MASK;
				pressureAutoBaselineDone = true;
			}
		}
	}

	return pressureFilteredSg & PRESSURE_SG_MASK;
}

uint8_t pressurePercentFromSg(uint16_t sg)
{
	// Higher load = lower SG_RESULT. Compute load relative to the calibrated
	// baseline (free-running SG value) and map to a 0..100 percentage
	// according to the user-selected sensitivity scale.
	sg &= PRESSURE_SG_MASK;
	uint16_t reference = pressureBaseline == 0
		? PRESSURE_EMPTY_BAR_SG
		: (pressureBaseline & PRESSURE_SG_MASK);
	if (reference > PRESSURE_EMPTY_BAR_SG)
		reference = PRESSURE_EMPTY_BAR_SG;
	if (reference <= PRESSURE_FULL_BAR_SG)
		return sg <= PRESSURE_FULL_BAR_SG ? 100 : 0;
	if (sg >= reference)
		return 0;
	uint16_t load = (uint16_t)(reference - sg);

	uint8_t scale = pressureScale;
	if (scale < PRESSURE_SCALE_MIN)
		scale = PRESSURE_SCALE_MIN;
	if (scale > PRESSURE_SCALE_MAX)
		scale = PRESSURE_SCALE_MAX;
	uint16_t emptyToFullUnits = reference - PRESSURE_FULL_BAR_SG;
	uint16_t fullScaleUnits = emptyToFullUnits;
	if (scale < PRESSURE_SCALE_REFERENCE)
	{
		fullScaleUnits = (uint16_t)(emptyToFullUnits *
			(uint16_t)((PRESSURE_SCALE_REFERENCE + 1) - scale));
	}
	else if (scale > PRESSURE_SCALE_REFERENCE)
	{
		uint8_t extraSensitivity = scale - PRESSURE_SCALE_REFERENCE;
		fullScaleUnits = emptyToFullUnits > extraSensitivity
			? (uint16_t)(emptyToFullUnits - extraSensitivity)
			: 1;
	}
	if (fullScaleUnits == 0)
		fullScaleUnits = 1;
	uint32_t percent = ((uint32_t)load * 100UL) / (uint32_t)fullScaleUnits;
	if (percent > 100UL)
		percent = 100UL;
	return (uint8_t)percent;
}

void calibratePressureBaseline()
{
	if (!tmc2209UartConfigured || !tmc2209UartVerified || testModeEnabled)
	{
		showStatus(F("Treiber n. bereit"), F(""));
		return;
	}
	// SG_RESULT is only meaningful while the motor is actually stepping.
	// Refuse to calibrate from an idle motor; the user must start the pump
	// (or a self-test jog) first so we sample real running conditions.
	if (!pumpRunning)
	{
		showStatus(F("Pumpe nicht aktiv"), F("Erst starten"));
		return;
	}

	uint32_t sum = 0;
	uint8_t valid = 0;
	for (uint8_t i = 0; i < PRESSURE_MANUAL_BASELINE_SAMPLES; i++)
	{
		uint16_t s = tmc2209Driver.SG_RESULT() & PRESSURE_SG_MASK;
		if (s > 0)
		{
			sum += s;
			valid++;
		}
		serviceWait(PRESSURE_MANUAL_BASELINE_SAMPLE_MS);
	}
	if (valid == 0)
	{
		showStatus(F("Kein SG-Signal"), F(""));
		return;
	}
	uint16_t baseline = (uint16_t)((sum / valid) & PRESSURE_SG_MASK);
	if (baseline > PRESSURE_EMPTY_BAR_SG)
		baseline = PRESSURE_EMPTY_BAR_SG;
	pressureBaseline = baseline;
	pressureAutoBaselineDone = true;
	pressureAutoBaselineSum = 0;
	pressureAutoBaselineCount = 0;
	savePersistentSettings();
	showStatus(F("Druck genullt"), F(""));
}

void applyMaxBolusPercent(uint8_t value)
{
	if (!validMaxBolusPercent(value))
		value = DEFAULT_MAX_BOLUS_PERCENT;

	maxBolusPercent = value;
	editMaxBolusPercent = value;
	if (editBolusCentiMl > maxBolusCentiMl())
		editBolusCentiMl = maxBolusCentiMl();
	if (editBolusCentiMl < MIN_BOLUS_CENTI_ML)
		editBolusCentiMl = MIN_BOLUS_CENTI_ML;
}

void applyStartupJogSpeedPercent(uint8_t value)
{
	if (!validStartupJogSpeedPercent(value))
		value = DEFAULT_STARTUP_JOG_SPEED_PERCENT;

	startupJogSpeedPercent = value;
	editStartupJogSpeedPercent = value;
}

void applyMotorDirectionInverted(uint8_t value)
{
	if (!validMotorDirectionInverted(value))
		value = INVERTDIRECTION ? 1 : 0;

	motorDirectionInverted = value == 1;
}

uint16_t settingsCrc16Bytes(const uint8_t *bytes, uint8_t byteCount)
{
	uint16_t crc = 0xFFFF;

	for (uint8_t index = 0; index < byteCount; index++)
	{
		crc ^= (uint16_t)bytes[index] << 8;
		for (uint8_t bit = 0; bit < 8; bit++)
		{
			if (crc & 0x8000)
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			else
				crc = (uint16_t)(crc << 1);
		}
	}

	return crc;
}

uint16_t settingsCrc16(const PersistedSettings *settings)
{
	return settingsCrc16Bytes((const uint8_t *)settings, offsetof(PersistedSettings, crc));
}

bool settingsCrcMatches(const PersistedSettings *settings)
{
	return settings->crc == settingsCrc16(settings);
}

bool settingsVersion7CrcMatches(const PersistedSettingsVersion7 *settings)
{
	return settings->crc == settingsCrc16Bytes((const uint8_t *)settings, offsetof(PersistedSettingsVersion7, crc));
}

bool settingsVersion8CrcMatches(const PersistedSettingsVersion8 *settings)
{
	return settings->crc == settingsCrc16Bytes((const uint8_t *)settings, offsetof(PersistedSettingsVersion8, crc));
}

bool settingsVersion6CrcMatches(const PersistedSettingsVersion6 *settings)
{
	return settings->crc == settingsCrc16Bytes((const uint8_t *)settings, offsetof(PersistedSettingsVersion6, crc));
}

uint8_t readEepromByte(uint16_t address)
{
	return EEPROM.read(address);
}

void waitForEepromReady()
{
}

void updateEepromByte(uint16_t address, uint8_t value)
{
	EEPROM.update(address, value);
}

void readPersistentSettings(PersistedSettings *settings)
{
	uint8_t *bytes = (uint8_t *)settings;
	for (uint8_t index = 0; index < sizeof(PersistedSettings); index++)
		bytes[index] = readEepromByte(SETTINGS_EEPROM_ADDRESS + index);
}

void readPersistentSettingsVersion6(PersistedSettingsVersion6 *settings)
{
	uint8_t *bytes = (uint8_t *)settings;
	for (uint8_t index = 0; index < sizeof(PersistedSettingsVersion6); index++)
		bytes[index] = readEepromByte(SETTINGS_EEPROM_ADDRESS + index);
}

void readPersistentSettingsVersion7(PersistedSettingsVersion7 *settings)
{
	uint8_t *bytes = (uint8_t *)settings;
	for (uint8_t index = 0; index < sizeof(PersistedSettingsVersion7); index++)
		bytes[index] = readEepromByte(SETTINGS_EEPROM_ADDRESS + index);
}

void readPersistentSettingsVersion8(PersistedSettingsVersion8 *settings)
{
	uint8_t *bytes = (uint8_t *)settings;
	for (uint8_t index = 0; index < sizeof(PersistedSettingsVersion8); index++)
		bytes[index] = readEepromByte(SETTINGS_EEPROM_ADDRESS + index);
}

void writePersistentSettings(const PersistedSettings *settings)
{
	const uint8_t *bytes = (const uint8_t *)settings;
	for (uint8_t index = 0; index < sizeof(PersistedSettings); index++)
		updateEepromByte(SETTINGS_EEPROM_ADDRESS + index, bytes[index]);
}

