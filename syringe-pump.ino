#include <Arduino.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <stddef.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#if defined(ARDUINO_AVR_UNO) && !defined(__AVR_ATmega328P__)
#include <avr/iom328p.h>
#endif

#ifndef PD2
#define PD2 2
#endif

#ifndef PB4
#define PB4 4
#endif

#ifndef PC1
#define PC1 1
#endif

#ifndef PC2
#define PC2 2
#endif

/********************************************************************
  Basic syringe pump firmware for Arduino Uno
********************************************************************/

// Stepper motor and mechanics
#define NOFMICROSTEPS 32
#define NOFSTEPSPER360 200
#define MAXRPM 120
#define INVERTDIRECTION true
#define MMPER360 8.0
#define MOTOR_CURRENT_MA 700

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
#define STARTUP_JOG_SPEED_STEP_PERCENT 5
#define BOLUS_DURATION_MS 10000UL
#define BOLUS_MENU_TIMEOUT_MS 5000UL
#define STATUS_DISPLAY_MS 1200UL
#define MOTOR_TEST_STEP_RATE_HZ 10.0

// Arduino Pins
#define STEP_PIN 2
#define DIRECTION_PIN 3
#define ENABLE_PIN 12
#define TMC2209_UART_RX_PIN A3
#define TMC2209_UART_TX_PIN A4
#define ENDSTOP_PIN_FORWARD A2
#define ENDSTOP_PIN_BACKWARD A1
#define BUZZER_PIN 11
#define BUTTONS_PIN A0
#define LCD_BACKLIGHT_PIN 10

#define TMC2209_UART_BAUD 19200UL
#define TMC2209_UART_DRIVER_ADDRESS 0
#define TMC2209_UART_R_SENSE 0.11f
#define TMC2209_EXPECTED_VERSION 0x21
#define TMC2209_READBACK_ATTEMPTS 3
#define TMC2209_CURRENT_TOLERANCE_MA 100

#define STEP_PORT PORTD
#define STEP_MASK 0x04
#define ENABLE_PORT PORTB
#define ENABLE_MASK 0x10
#define ENDSTOP_PIN_REGISTER PINC
#define ENDSTOP_FORWARD_MASK 0x04
#define ENDSTOP_BACKWARD_MASK 0x02
#define ENDSTOP_MASK (ENDSTOP_FORWARD_MASK | ENDSTOP_BACKWARD_MASK)

#define DEBOUNCE_DELAY_MS 50
#define DISPLAY_UPDATE_MS 250
#define SCROLL_INTERVAL_MS 330
#define LCD_COLUMNS 16
#define SCROLL_GAP_COLUMNS 3
#define STEP_PULSE_US 15

#define SETTINGS_EEPROM_ADDRESS 0
#define SETTINGS_MAGIC 0x5046
#define SETTINGS_VERSION 7
#define SETTINGS_MENU_COUNT 9
#define SETTINGS_MENU_DELIVERY 0
#define SETTINGS_MENU_SYRINGE 1
#define SETTINGS_MENU_BOLUS 2
#define SETTINGS_MENU_SOUND 3
#define SETTINGS_MENU_MAX_BOLUS 4
#define SETTINGS_MENU_MOTOR_INVERT 5
#define SETTINGS_MENU_STARTUP_SPEED 6
#define SETTINGS_MENU_MOTOR_TEST 7
#define SETTINGS_MENU_SUPPORT 8
#define SETTINGS_SUPPORT_TIMEOUT_MS 10000UL
#define STARTUP_SPLASH_MS 1200UL
#define MAIN_MENU_COUNT 3
#define MAIN_MENU_START_INFUSION 0
#define MAIN_MENU_LOAD_SYRINGE 1
#define MAIN_MENU_SETTINGS 2
#define START_MENU_COUNT 2
#define START_MENU_VOLUME_TIME 0
#define START_MENU_RATE 1

#if SETTINGS_MENU_COUNT != 9
#error SETTINGS_MENU_COUNT must match the explicit SETTINGS_MENU_* indices.
#endif

#if MAIN_MENU_COUNT != 3
#error MAIN_MENU_COUNT must match the explicit MAIN_MENU_* indices.
#endif

#if START_MENU_COUNT != 2
#error START_MENU_COUNT must match the explicit START_MENU_* indices.
#endif

#if defined(EEPE)
#define EEPROM_WRITE_ENABLE_BIT EEPE
#else
#define EEPROM_WRITE_ENABLE_BIT EEWE
#endif

#if defined(EEMPE)
#define EEPROM_MASTER_WRITE_ENABLE_BIT EEMPE
#else
#define EEPROM_MASTER_WRITE_ENABLE_BIT EEMWE
#endif

const char PROMPT_INFO_LINE_1[] PROGMEM = "Insert Syringe";
const char PROMPT_RATE[] PROGMEM = "Enter Infusion Rate ml/h";
const char PROMPT_VOLUME[] PROGMEM = "Target Volume";
const char PROMPT_TIME[] PROGMEM = "Target Time";
const char PROMPT_DELIVERY[] PROGMEM = "Fluid Amount ml/cm";
const char PROMPT_SYRINGE[] PROGMEM = "Syringe ml";
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

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

SoftwareSerial tmc2209Serial(TMC2209_UART_RX_PIN, TMC2209_UART_TX_PIN);
TMC2209Stepper tmc2209Driver(&tmc2209Serial, TMC2209_UART_R_SENSE, TMC2209_UART_DRIVER_ADDRESS);

enum Button
{
	BUTTON_RIGHT,
	BUTTON_UP,
	BUTTON_DOWN,
	BUTTON_LEFT,
	BUTTON_SELECT,
	BUTTON_NONE
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
volatile uint16_t timer1HighPulseOcr = 0;
volatile uint16_t timer1LowPulseOcr = 0;
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
PGM_P scrollPrompt = NULL;
const char *scrollPromptRam = NULL;
uint8_t scrollPromptRow = 0;
uint8_t scrollOffset = 0;
uint32_t lastScrollMillis = 0;
char infusionConfirmDetail[65];

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
void showSplash();
void showInfoScreen();
void updateStartupScreens(Button button);
void finishStartupInfo();
void showCurrentScreen();
void showMainMenu();
void printMainMenuItem(uint8_t itemIndex, bool selected);
void showStartMenu();
void printStartMenuItem(uint8_t itemIndex, bool selected);
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
void clearScrollPrompt();
void useScrollPrompt(PGM_P prompt, uint8_t row);
void useRamScrollPrompt(const char *prompt, uint8_t row);
void printFlashWindow(PGM_P text, uint8_t offset);
void printRamWindow(const char *text, uint8_t offset);
void redrawScrollPromptRow();
void printSettingsMenuItem(uint8_t itemIndex, bool selected);
void updateScrollPrompt();
void updateSettingsSupportScreen();
uint8_t scrollCycleLength(PGM_P text);
uint8_t ramScrollCycleLength(const char *text);
uint8_t nextScrollOffset(PGM_P text, uint8_t offset);
uint8_t nextRamScrollOffset(const char *text, uint8_t offset);
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
uint16_t timerPrescalerForIndex(uint8_t index);
uint8_t timerClockBitsForIndex(uint8_t index);
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
void beep(uint16_t delayOn = 15, uint16_t delayOff = 80, bool force = false);
void updateBuzzer();
void cancelBuzzerBeeps();
void serviceWatchdog();
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
bool canStartPump();
bool forwardEndstopActive();
bool backwardEndstopActive();
bool anyEndstopActive();
bool endstopActiveForDirection(bool forward);
void enableMotorDriver();
void disableMotorDriver();
void configureEndstopInterrupts();
void stopMotorClockFromIsr();
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
bool validMaxBolusPercent(uint8_t value);
bool validStartupJogSpeedPercent(uint8_t value);
void applyDeliveryCenti(uint32_t value);
void applySyringeCenti(uint32_t value);
void applyMotorDirectionInverted(uint8_t value);
void applyBolusEnabled(uint8_t value);
void applySoundEnabled(uint8_t value);
void applyMaxBolusPercent(uint8_t value);
void applyStartupJogSpeedPercent(uint8_t value);
uint16_t settingsCrc16Bytes(const uint8_t *bytes, uint8_t byteCount);
uint16_t settingsCrc16(const PersistedSettings *settings);
bool settingsCrcMatches(const PersistedSettings *settings);
bool settingsVersion6CrcMatches(const PersistedSettingsVersion6 *settings);
uint8_t readEepromByte(uint16_t address);
void waitForEepromReady();
void updateEepromByte(uint16_t address, uint8_t value);
void readPersistentSettingsVersion6(PersistedSettingsVersion6 *settings);
void readPersistentSettings(PersistedSettings *settings);
void writePersistentSettings(const PersistedSettings *settings);

void setup()
{
	MCUSR = 0;
	wdt_disable();

	lcd.begin(16, 2);

	pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
	digitalWrite(LCD_BACKLIGHT_PIN, HIGH);

	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIRECTION_PIN, OUTPUT);
	pinMode(ENABLE_PIN, OUTPUT);
	pinMode(ENDSTOP_PIN_FORWARD, INPUT_PULLUP);
	pinMode(ENDSTOP_PIN_BACKWARD, INPUT_PULLUP);
	pinMode(BUZZER_PIN, OUTPUT);

	disableMotorDriver();
	digitalWrite(STEP_PIN, LOW);
	digitalWrite(BUZZER_PIN, LOW);
	configureEndstopInterrupts();
	setupTmc2209Uart();

	wdt_enable(WDTO_1S);
	loadPersistentSettings();

	startupScreenStartMillis = millis();
	currentScreen = SCREEN_STARTUP_SPLASH;
	showSplash();
	beep();
}

void loop()
{
	serviceWatchdog();
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
		updateScrollPrompt();
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

Button buttonFromAdcReading(uint16_t reading)
{
	if (reading < 52)
		return BUTTON_RIGHT;
	if (reading < 184)
		return BUTTON_UP;
	if (reading < 336)
		return BUTTON_DOWN;
	if (reading < 524)
		return BUTTON_LEFT;
	if (reading < 828)
		return BUTTON_SELECT;
	return BUTTON_NONE;
}

Button readButton()
{
	const uint8_t sampleCount = 5;
	const uint8_t requiredMajority = 3;
	uint16_t samples[sampleCount];
	uint8_t buttonCounts[BUTTON_NONE + 1] = {0};

	for (uint8_t sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++)
	{
		samples[sampleIndex] = analogRead(BUTTONS_PIN);
		buttonCounts[buttonFromAdcReading(samples[sampleIndex])]++;
	}

	for (uint8_t buttonIndex = 0; buttonIndex <= BUTTON_NONE; buttonIndex++)
	{
		if (buttonCounts[buttonIndex] >= requiredMajority)
			return static_cast<Button>(buttonIndex);
	}

	for (uint8_t sampleIndex = 1; sampleIndex < sampleCount; sampleIndex++)
	{
		uint16_t sample = samples[sampleIndex];
		uint8_t insertIndex = sampleIndex;
		while (insertIndex > 0 && samples[insertIndex - 1] > sample)
		{
			samples[insertIndex] = samples[insertIndex - 1];
			insertIndex--;
		}
		samples[insertIndex] = sample;
	}

	return buttonFromAdcReading(samples[sampleCount / 2]);
}

Button readButtonPress()
{
	static Button lastReading = BUTTON_NONE;
	static Button reportedButton = BUTTON_NONE;
	static uint32_t changedAt = 0;

	Button currentReading = readButton();
	if (currentReading != lastReading)
	{
		lastReading = currentReading;
		changedAt = millis();
		reportedButton = BUTTON_NONE;
		return BUTTON_NONE;
	}

	if (currentReading == BUTTON_NONE)
	{
		reportedButton = BUTTON_NONE;
		return BUTTON_NONE;
	}

	if (reportedButton == BUTTON_NONE && millis() - changedAt >= DEBOUNCE_DELAY_MS)
	{
		reportedButton = currentReading;
		return currentReading;
	}

	return BUTTON_NONE;
}

void showSplash()
{
	lcd.noCursor();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("  DIY Syringe Pump  "));
	lcd.setCursor(0, 1);
	lcd.print(F(" A Flo Project"));
}

void showInfoScreen()
{
	currentScreen = SCREEN_STARTUP_INFO;
	lcd.noCursor();
	clearScrollPrompt();
	char syringeLine[17];
	uint16_t whole = syringeCentiMl / 100;
	uint8_t fraction = syringeCentiMl % 100;
	if (whole >= 100)
		snprintf(syringeLine, sizeof(syringeLine), "Aktuell%03u.%02u ml", (unsigned int)whole, (unsigned int)fraction);
	else
		snprintf(syringeLine, sizeof(syringeLine), "Aktuell %02u.%02u ml", (unsigned int)whole, (unsigned int)fraction);
	lcd.clear();
	lcd.setCursor(0, 0);
	printFlashWindow(PROMPT_INFO_LINE_1, 0);
	lcd.setCursor(0, 1);
	lcd.print(syringeLine);
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

void showMainMenu()
{
	clearScrollPrompt();
	lcd.noCursor();
	lcd.clear();
	uint8_t firstIndex = mainMenuIndex >= MAIN_MENU_COUNT - 1 ? MAIN_MENU_COUNT - 2 : mainMenuIndex;

	lcd.setCursor(0, 0);
	printMainMenuItem(firstIndex, mainMenuIndex == firstIndex);
	lcd.setCursor(0, 1);
	printMainMenuItem(firstIndex + 1, mainMenuIndex == firstIndex + 1);
}

void printMainMenuItem(uint8_t itemIndex, bool selected)
{
	char prefix = selected ? '>' : ' ';
	switch (itemIndex)
	{
	case MAIN_MENU_START_INFUSION:
		lcd.print(prefix);
		lcd.print(F("Start Infusion"));
		break;
	case MAIN_MENU_LOAD_SYRINGE:
		lcd.print(prefix);
		lcd.print(F("Neu einlegen"));
		break;
	case MAIN_MENU_SETTINGS:
		lcd.print(prefix);
		lcd.print(F("Einstellungen"));
		break;
	default:
		lcd.print(prefix);
		lcd.print(F("?"));
		break;
	}
}

void showStartMenu()
{
	clearScrollPrompt();
	lcd.noCursor();
	lcd.clear();
	uint8_t firstIndex = startMenuIndex >= START_MENU_COUNT - 1 ? START_MENU_COUNT - 2 : startMenuIndex;

	lcd.setCursor(0, 0);
	printStartMenuItem(firstIndex, startMenuIndex == firstIndex);
	lcd.setCursor(0, 1);
	printStartMenuItem(firstIndex + 1, startMenuIndex == firstIndex + 1);
}

void printStartMenuItem(uint8_t itemIndex, bool selected)
{
	char prefix = selected ? '>' : ' ';
	switch (itemIndex)
	{
	case START_MENU_VOLUME_TIME:
		lcd.print(prefix);
		lcd.print(F("Volumen & Zeit"));
		break;
	case START_MENU_RATE:
		lcd.print(prefix);
		lcd.print(F("Infusionsrate"));
		break;
	default:
		lcd.print(prefix);
		lcd.print(F("?"));
		break;
	}
}

void showSettingsMenu()
{
	clearScrollPrompt();
	lcd.noCursor();
	lcd.clear();
	uint8_t firstIndex = settingsMenuIndex >= SETTINGS_MENU_COUNT - 1 ? SETTINGS_MENU_COUNT - 2 : settingsMenuIndex;

	lcd.setCursor(0, 0);
	printSettingsMenuItem(firstIndex, settingsMenuIndex == firstIndex);
	lcd.setCursor(0, 1);
	printSettingsMenuItem(firstIndex + 1, settingsMenuIndex == firstIndex + 1);
}

void printSettingsMenuItem(uint8_t itemIndex, bool selected)
{
	char prefix = selected ? '>' : ' ';
	switch (itemIndex)
	{
	case SETTINGS_MENU_DELIVERY:
		lcd.print(prefix);
		lcd.print(F("Abgabemenge"));
		break;
	case SETTINGS_MENU_SYRINGE:
		lcd.print(prefix);
		lcd.print(F("Spritze ml"));
		break;
	case SETTINGS_MENU_BOLUS:
		lcd.print(prefix);
		lcd.print(bolusEnabled ? F("Bolus: AN") : F("Bolus: AUS"));
		break;
	case SETTINGS_MENU_SOUND:
		lcd.print(prefix);
		lcd.print(soundEnabled ? F("Ton: EIN") : F("Ton: AUS"));
		break;
	case SETTINGS_MENU_MAX_BOLUS:
		lcd.print(prefix);
		lcd.print(F("Max Bolus %"));
		break;
	case SETTINGS_MENU_MOTOR_INVERT:
		lcd.print(prefix);
		lcd.print(motorDirectionInverted ? F("Motor inv: AN") : F("Motor inv: AUS"));
		break;
	case SETTINGS_MENU_STARTUP_SPEED:
	{
		char valueLine[17];
		snprintf(valueLine, sizeof(valueLine), "%cMotorspeed:%03u%%", prefix, (unsigned int)startupJogSpeedPercent);
		lcd.print(valueLine);
		break;
	}
	case SETTINGS_MENU_MOTOR_TEST:
		lcd.print(prefix);
		lcd.print(F("Motortest"));
		break;
	case SETTINGS_MENU_SUPPORT:
		lcd.print(prefix);
		lcd.print(F("Support"));
		break;
	default:
		lcd.print(prefix);
		lcd.print(F("?"));
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
	lcd.noCursor();
	showMaxBolusEditor();
}

void beginStartupMotorSpeedEditor()
{
	editStartupJogSpeedPercent = startupJogSpeedPercent;
	if (!validStartupJogSpeedPercent(editStartupJogSpeedPercent))
		editStartupJogSpeedPercent = DEFAULT_STARTUP_JOG_SPEED_PERCENT;

	currentScreen = SCREEN_EDIT_STARTUP_MOTOR_SPEED;
	lcd.noCursor();
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

	lcd.clear();
	useScrollPrompt(PROMPT_RATE, 0);
	lcd.setCursor(0, 0);
	printFlashWindow(PROMPT_RATE, scrollOffset);
	lcd.setCursor(0, 1);
	lcd.print(valueLine);

	lcd.setCursor(editCursorColumn(), 1);
	lcd.cursor();
}

void showVolumeEditor()
{
	uint16_t whole = editVolumeCentiMl / 100;
	uint8_t fraction = editVolumeCentiMl % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u.%02u ml", (unsigned int)whole, (unsigned int)fraction);

	lcd.clear();
	useScrollPrompt(PROMPT_VOLUME, 0);
	lcd.setCursor(0, 0);
	printFlashWindow(PROMPT_VOLUME, scrollOffset);
	lcd.setCursor(0, 1);
	lcd.print(valueLine);

	lcd.setCursor(editCursorColumn(), 1);
	lcd.cursor();
}

void showTimeEditor()
{
	uint8_t hours = editTimeSeconds / 3600;
	uint8_t minutes = (editTimeSeconds % 3600) / 60;
	uint8_t seconds = editTimeSeconds % 60;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%02u:%02u:%02u H/M/S", (unsigned int)hours, (unsigned int)minutes, (unsigned int)seconds);

	lcd.clear();
	useScrollPrompt(PROMPT_TIME, 0);
	lcd.setCursor(0, 0);
	printFlashWindow(PROMPT_TIME, scrollOffset);
	lcd.setCursor(0, 1);
	lcd.print(valueLine);

	lcd.setCursor(editCursorColumn(), 1);
	lcd.cursor();
}

void showDeliveryEditor()
{
	uint16_t whole = editDeliveryCentiMlPerCm / 100;
	uint8_t fraction = editDeliveryCentiMlPerCm % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u.%02u ml/cm", (unsigned int)whole, (unsigned int)fraction);

	lcd.clear();
	useScrollPrompt(PROMPT_DELIVERY, 0);
	lcd.setCursor(0, 0);
	printFlashWindow(PROMPT_DELIVERY, scrollOffset);
	lcd.setCursor(0, 1);
	lcd.print(valueLine);

	lcd.setCursor(editCursorColumn(), 1);
	lcd.cursor();
}

void showSyringeEditor()
{
	uint16_t whole = editSyringeCentiMl / 100;
	uint8_t fraction = editSyringeCentiMl % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u.%02u ml", (unsigned int)whole, (unsigned int)fraction);

	lcd.clear();
	clearScrollPrompt();
	lcd.setCursor(0, 0);
	lcd.print(F("Spritze ml"));
	lcd.setCursor(0, 1);
	lcd.print(valueLine);

	lcd.setCursor(editCursorColumn(), 1);
	lcd.cursor();
}

void showMaxBolusEditor()
{
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%02u %% Spritze", (unsigned int)editMaxBolusPercent);

	lcd.clear();
	clearScrollPrompt();
	lcd.noCursor();
	lcd.setCursor(0, 0);
	lcd.print(F("Max Bolus"));
	lcd.setCursor(0, 1);
	lcd.print(valueLine);
}

void showStartupMotorSpeedEditor()
{
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "%03u %%", (unsigned int)editStartupJogSpeedPercent);

	lcd.clear();
	clearScrollPrompt();
	lcd.noCursor();
	lcd.setCursor(0, 0);
	lcd.print(F("Motorspeed"));
	lcd.setCursor(0, 1);
	lcd.print(valueLine);
}

void showSettingsSupportScreen()
{
	clearScrollPrompt();
	lcd.noCursor();
	lcd.clear();
	char line1[17];
	snprintf(line1, sizeof(line1), "Test %s %s",
		motorTestDirectionForward ? "VOR" : "ZUR",
		motorTestRunning ? "AN" : "AUS");
	lcd.setCursor(0, 0);
	lcd.print(line1);
	lcd.setCursor(0, 1);
	lcd.print(F("SEL U/D L RAUS"));
}

void showSupportInfoScreen()
{
	clearScrollPrompt();
	lcd.noCursor();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(SUPPORT_DISPLAY_LINE_1);
	lcd.setCursor(0, 1);
	lcd.print(SUPPORT_DISPLAY_LINE_2);
}

void showBolusEditor()
{
	uint8_t whole = editBolusCentiMl / 100;
	uint8_t fraction = editBolusCentiMl % 100;
	char valueLine[17];
	snprintf(valueLine, sizeof(valueLine), "Bolus ml: %02u.%02u", (unsigned int)whole, (unsigned int)fraction);

	lcd.clear();
	clearScrollPrompt();
	lcd.setCursor(0, 0);
	lcd.print(F("Manueller Bolus"));
	lcd.setCursor(0, 1);
	lcd.print(valueLine);

	lcd.setCursor(editCursorColumn(), 1);
	lcd.cursor();
	lastDisplayMillis = millis();
}

void showBolusConfirm()
{
	lcd.noCursor();
	lcd.clear();
	char flowLine[17];
	char flowText[8];
	char timeLine[17];
	dtostrf(pendingBolusFlowMlPerHour, 6, 1, flowText);
	snprintf(flowLine, sizeof(flowLine), "BOL %s ml/h", flowText);
	uint32_t seconds = (pendingBolusDurationMillis + 999UL) / 1000UL;
	snprintf(timeLine, sizeof(timeLine), "Zeit %4lus", (unsigned long)seconds);
	lcd.setCursor(0, 0);
	lcd.print(flowLine);
	lcd.setCursor(0, 1);
	lcd.print(timeLine);
	lastDisplayMillis = millis();
}

void showInfusionConfirm()
{
	char rateText[8];
	uint16_t syringeWholeMl = syringeCentiMl / 100UL;
	uint32_t centiFlow = (uint32_t)(flowMlPerHour * 100.0 + 0.5);
	snprintf(rateText, sizeof(rateText), "%u.%02u", (unsigned int)(centiFlow / 100UL), (unsigned int)(centiFlow % 100UL));

	if (pendingInfusionTargetVolumeEnabled)
	{
		uint32_t durationMinutes = (targetTimeSeconds + 30UL) / 60UL;
		snprintf(infusionConfirmDetail, sizeof(infusionConfirmDetail), "Spritze: %uml, Rate: %s ml/h, Dauer: %lumin",
			(unsigned int)syringeWholeMl, rateText, (unsigned long)durationMinutes);
	}
	else
	{
		snprintf(infusionConfirmDetail, sizeof(infusionConfirmDetail), "Spritze: %uml, Rate: %s ml/h, Bis Ende",
			(unsigned int)syringeWholeMl, rateText);
	}

	lcd.clear();
	clearScrollPrompt();
	lcd.noCursor();
	lcd.setCursor(0, 0);
	lcd.print(F("Abgabe starten?"));
	useRamScrollPrompt(infusionConfirmDetail, 1);
	lcd.setCursor(0, 1);
	printRamWindow(infusionConfirmDetail, scrollOffset);
	lastDisplayMillis = millis();
}

void showIdleScreen()
{
	showMainMenu();
}

void showPumpScreen()
{
	clearScrollPrompt();
	lcd.noCursor();
	lastDisplayMillis = millis();

	char flowText[8];
	uint32_t centiFlow = (uint32_t)(flowMlPerHour * 100.0 + 0.5);
	snprintf(flowText, sizeof(flowText), "%03u.%02u", (unsigned int)(centiFlow / 100UL), (unsigned int)(centiFlow % 100UL));

	lcd.setCursor(0, 0);
	lcd.print(F("INF "));
	lcd.print(flowText);
	lcd.print(F(" ml/h "));

	uint32_t elapsedSeconds = (millis() - pumpStartMillis) / 1000;
	float pumpedMl = pumpedMlFromSteps(atomicStepCounter());
	char elapsedText[8];
	char volumeText[8];
	char lineText[17];
	if (targetVolumeEnabled)
	{
		float remainingMl = targetVolumeMl - pumpedMl;
		if (remainingMl < 0.0)
			remainingMl = 0.0;

		uint32_t remainingSeconds = flowMlPerHour > 0.0 ? (uint32_t)((remainingMl * 3600.0 / flowMlPerHour) + 0.5) : 0;
		formatElapsedTime(remainingSeconds, elapsedText, sizeof(elapsedText));
		dtostrf(remainingMl, 1, 2, volumeText);
		snprintf(lineText, sizeof(lineText), "R %s %s", volumeText, elapsedText);
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
		snprintf(lineText, sizeof(lineText), "%s %sml", elapsedText, volumeText);
	}
	for (uint8_t index = strlen(lineText); index < LCD_COLUMNS; index++)
		lineText[index] = ' ';
	lineText[LCD_COLUMNS] = '\0';

	lcd.setCursor(0, 1);
	lcd.print(lineText);
}

void showStatus(const __FlashStringHelper *line1, const __FlashStringHelper *line2)
{
	clearScrollPrompt();
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
	lcd.noCursor();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(line1);
	lcd.setCursor(0, 1);
	lcd.print(line2);
	lastDisplayMillis = millis();
}

void showAcknowledgedStatus(const __FlashStringHelper *line1, const __FlashStringHelper *line2)
{
	clearScrollPrompt();
	alarmActive = false;
	cancelBuzzerBeeps();
	statusReturnScreen = currentScreen;
	statusActive = true;
	statusRequiresAcknowledge = true;
	statusUntilMillis = 0;
	digitalWrite(BUZZER_PIN, LOW);
	lcd.noCursor();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(line1);
	lcd.setCursor(0, 1);
	lcd.print(line2);
	lastDisplayMillis = millis();
	beep(200, 100, true);
	beep(200, 0, true);
}

void showAlarm(const __FlashStringHelper *line1, const __FlashStringHelper *line2)
{
	clearScrollPrompt();
	cancelBuzzerBeeps();
	alarmActive = false;
	statusActive = true;
	statusRequiresAcknowledge = true;
	statusReturnScreen = SCREEN_MAIN_MENU;
	digitalWrite(BUZZER_PIN, LOW);
	lcd.noCursor();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(line1);
	lcd.setCursor(0, 1);
	lcd.print(line2);
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

void clearScrollPrompt()
{
	scrollPrompt = NULL;
	scrollPromptRam = NULL;
	scrollOffset = 0;
}

void useScrollPrompt(PGM_P prompt, uint8_t row)
{
	if (scrollPrompt != prompt || scrollPromptRam != NULL)
	{
		scrollOffset = 0;
		lastScrollMillis = millis();
	}

	scrollPrompt = prompt;
	scrollPromptRam = NULL;
	scrollPromptRow = row;
}

void useRamScrollPrompt(const char *prompt, uint8_t row)
{
	if (scrollPromptRam != prompt || scrollPrompt != NULL)
	{
		scrollOffset = 0;
		lastScrollMillis = millis();
	}

	scrollPrompt = NULL;
	scrollPromptRam = prompt;
	scrollPromptRow = row;
}

void printFlashWindow(PGM_P text, uint8_t offset)
{
	uint8_t textLength = strlen_P(text);
	uint8_t cycleLength = scrollCycleLength(text);

	for (uint8_t column = 0; column < LCD_COLUMNS; column++)
	{
		if (textLength <= LCD_COLUMNS)
		{
			char character = column < textLength ? pgm_read_byte(text + column) : '\0';
			lcd.write((uint8_t)(character == '\0' ? ' ' : character));
		}
		else
		{
			uint8_t position = (offset + column) % cycleLength;
			char character = position < textLength ? (char)pgm_read_byte(text + position) : ' ';
			lcd.write((uint8_t)character);
		}
	}
}

void printRamWindow(const char *text, uint8_t offset)
{
	uint8_t textLength = strlen(text);
	uint8_t cycleLength = ramScrollCycleLength(text);

	for (uint8_t column = 0; column < LCD_COLUMNS; column++)
	{
		if (textLength <= LCD_COLUMNS)
		{
			char character = column < textLength ? text[column] : '\0';
			lcd.write((uint8_t)(character == '\0' ? ' ' : character));
		}
		else
		{
			uint8_t position = (offset + column) % cycleLength;
			char character = position < textLength ? text[position] : ' ';
			lcd.write((uint8_t)character);
		}
	}
}

void redrawScrollPromptRow()
{
	if (scrollPrompt == NULL && scrollPromptRam == NULL)
		return;

	lcd.noCursor();
	lcd.setCursor(0, scrollPromptRow);
	if (scrollPrompt != NULL)
		printFlashWindow(scrollPrompt, scrollOffset);
	else
		printRamWindow(scrollPromptRam, scrollOffset);

	if (currentScreen != SCREEN_CONFIRM_INFUSION)
	{
		lcd.setCursor(editCursorColumn(), 1);
		lcd.cursor();
	}
}

void updateScrollPrompt()
{
	if (scrollPrompt == NULL && scrollPromptRam == NULL)
		return;

	uint8_t cycleLength = scrollPrompt != NULL ? scrollCycleLength(scrollPrompt) : ramScrollCycleLength(scrollPromptRam);
	if (cycleLength == 0 || millis() - lastScrollMillis < SCROLL_INTERVAL_MS)
		return;

	lastScrollMillis = millis();
	scrollOffset = scrollPrompt != NULL ? nextScrollOffset(scrollPrompt, scrollOffset) : nextRamScrollOffset(scrollPromptRam, scrollOffset);
	redrawScrollPromptRow();
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

uint8_t scrollCycleLength(PGM_P text)
{
	uint8_t length = strlen_P(text);
	return length > LCD_COLUMNS ? length + SCROLL_GAP_COLUMNS : 0;
}

uint8_t ramScrollCycleLength(const char *text)
{
	uint8_t length = strlen(text);
	return length > LCD_COLUMNS ? length + SCROLL_GAP_COLUMNS : 0;
}

uint8_t nextScrollOffset(PGM_P text, uint8_t offset)
{
	uint8_t cycleLength = scrollCycleLength(text);
	if (cycleLength == 0)
		return 0;

	uint8_t nextOffset = offset + 1;
	return nextOffset >= cycleLength ? 0 : nextOffset;
}

uint8_t nextRamScrollOffset(const char *text, uint8_t offset)
{
	uint8_t cycleLength = ramScrollCycleLength(text);
	if (cycleLength == 0)
		return 0;

	uint8_t nextOffset = offset + 1;
	return nextOffset >= cycleLength ? 0 : nextOffset;
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
		if (button == BUTTON_LEFT)
		{
			stopMotorSelfTest();
			currentScreen = SCREEN_MAIN_MENU;
			beep(8, 30);
			showMainMenu();
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
		else if (button == BUTTON_LEFT)
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
		else if (button == BUTTON_LEFT)
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
		currentScreen = SCREEN_MAIN_MENU;
		beep(8, 30);
		showMainMenu();
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
		currentScreen = SCREEN_MAIN_MENU;
		beep(8, 30);
		showMainMenu();
	}
}

void handleMaxBolusEditorButton(Button button)
{
	if (button == BUTTON_UP)
		changeMaxBolusPercent(1);
	else if (button == BUTTON_DOWN)
		changeMaxBolusPercent(-1);
	else if (button == BUTTON_LEFT)
	{
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
	else if (button == BUTTON_SELECT || button == BUTTON_RIGHT)
	{
		applyMaxBolusPercent(editMaxBolusPercent);
		savePersistentSettings();
		currentScreen = SCREEN_MAIN_MENU;
		beep(8, 30);
		showMainMenu();
	}
}

void handleStartupMotorSpeedEditorButton(Button button)
{
	if (button == BUTTON_UP)
		changeStartupMotorSpeedPercent(1);
	else if (button == BUTTON_DOWN)
		changeStartupMotorSpeedPercent(-1);
	else if (button == BUTTON_LEFT)
	{
		currentScreen = SCREEN_SETTINGS_MENU;
		beep(8, 30);
		showSettingsMenu();
	}
	else if (button == BUTTON_SELECT || button == BUTTON_RIGHT)
	{
		applyStartupJogSpeedPercent(editStartupJogSpeedPercent);
		savePersistentSettings();
		currentScreen = SCREEN_MAIN_MENU;
		beep(8, 30);
		showMainMenu();
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
	wdt_reset();
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
		updateBuzzer();
		uint16_t remaining = durationMs - (uint16_t)(millis() - startMillis);
		delay(remaining > 5 ? 5 : remaining);
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
	tmc2209Serial.listen();
	tmc2209Driver.begin();
	tmc2209Driver.pdn_disable(true);
	tmc2209Driver.mstep_reg_select(true);
	tmc2209Driver.microsteps(NOFMICROSTEPS);
	tmc2209Driver.intpol(true);
	tmc2209Driver.rms_current(MOTOR_CURRENT_MA);

	tmc2209UartConfigured = true;
	tmc2209UartVerified = verifyTmc2209Readback();
}

bool verifyTmc2209Readback()
{
	for (uint8_t attempt = 0; attempt < TMC2209_READBACK_ATTEMPTS; attempt++)
	{
		tmc2209Serial.listen();
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
	if (!tmc2209UartVerified)
		return false;

	if (endstopActiveForDirection(forward))
		return false;

	noInterrupts();
	stepCounter = 0;
	stepPulseHigh = false;
	endstopInterruptLatched = false;
	interrupts();
	STEP_PORT &= ~STEP_MASK;

	digitalWrite(DIRECTION_PIN,
		forward
			? (motorDirectionInverted ? LOW : HIGH)
			: (motorDirectionInverted ? HIGH : LOW));

	float jogSpeedFactor = (float)startupJogSpeedPercent / 100.0;
	if (!configureTimer1(maxStepRateHz() * jogSpeedFactor))
		return false;

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
	disableMotorDriver();
	startupJogRunning = false;
}

bool startMotorSelfTest()
{
	if (!tmc2209UartVerified)
	{
		showStatus(F("Test blockiert"), F("TMC UART fehlt"));
		beep(250, 80, true);
		return false;
	}

	if (endstopActiveForDirection(motorTestDirectionForward))
	{
		showStatus(F("Test blockiert"), F("Endstop aktiv"));
		return false;
	}

	noInterrupts();
	stepCounter = 0;
	stepPulseHigh = false;
	endstopInterruptLatched = false;
	interrupts();
	STEP_PORT &= ~STEP_MASK;

	digitalWrite(DIRECTION_PIN,
		motorTestDirectionForward
			? (motorDirectionInverted ? LOW : HIGH)
			: (motorDirectionInverted ? HIGH : LOW));

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
	if (!tmc2209UartVerified)
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
	return (ENDSTOP_PIN_REGISTER & ENDSTOP_FORWARD_MASK) != 0;
}

bool backwardEndstopActive()
{
	return (ENDSTOP_PIN_REGISTER & ENDSTOP_BACKWARD_MASK) != 0;
}

bool anyEndstopActive()
{
	return (ENDSTOP_PIN_REGISTER & ENDSTOP_MASK) != 0;
}

bool endstopActiveForDirection(bool forward)
{
	// Jog/Test-"forward" entspricht mechanisch der Rueckzugsrichtung.
	return forward ? backwardEndstopActive() : forwardEndstopActive();
}

void enableMotorDriver()
{
	digitalWrite(ENABLE_PIN, LOW);
}

void disableMotorDriver()
{
	digitalWrite(ENABLE_PIN, HIGH);
	STEP_PORT &= ~STEP_MASK;
}

void configureEndstopInterrupts()
{
	endstopInterruptLatched = false;
}

void stopMotorClockFromIsr()
{
	// ISR-only: AVR enters ISRs with global interrupts disabled, and this code
	// does not enable nested interrupts. Keep this cutoff direct for endstop latency.
	TIMSK1 &= ~(1 << OCIE1A);
	STEP_PORT &= ~STEP_MASK;
	ENABLE_PORT |= ENABLE_MASK;
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

	digitalWrite(DIRECTION_PIN, motorDirectionInverted ? HIGH : LOW);
	motorClockDirectionForward = false;
	motorClockStopOnAnyEndstop = true;

	noInterrupts();
	stepCounter = 0;
	stepPulseHigh = false;
	endstopInterruptLatched = false;
	interrupts();
	STEP_PORT &= ~STEP_MASK;

	if (!configureTimer1(requestedStepRateHz))
	{
		showStatus(F("Timer setup err"), F("Check flow rate "));
		beep(250, 80, true);
		return false;
	}

	baseInfusionFlowMlPerHour = flowMlPerHour;
	bolusActive = false;
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
		showStatus(F("Pump stopped   "), F("Manual stop    "));
	else if (expectedForwardEndstop)
		showAcknowledgedStatus(F("Ziel erreicht"), F("Infusion stop"));
	else if (reason == STOP_ENDSTOP)
		showAlarm(F("ALARM Endstop  "), F("SELECT quittiert"));
	else if (reason == STOP_TARGET_VOLUME)
		showAcknowledgedStatus(F("Ziel erreicht"), F("Infusion stop"));
	else
		showAlarm(F("ALARM Timer    "), F("SELECT quittiert"));
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
	TCNT1 = 0;
	OCR1A = timer1LowPulseOcr;
	TIFR1 |= (1 << OCF1A);
	TIMSK1 |= (1 << OCIE1A);
	interrupts();
}

void enableTimer1InterruptPreservePhase()
{
	// Schaltet OCIE1A scharf, ohne TCNT1/OCR1A anzufassen.
	// Wird nach configureTimer1PreservePhase() benutzt, damit der
	// dort proportional skalierte Zaehlerstand erhalten bleibt.
	noInterrupts();
	TIFR1 |= (1 << OCF1A);
	TIMSK1 |= (1 << OCIE1A);
	interrupts();
}

bool timerIsRunning()
{
	noInterrupts();
	bool running = (TIMSK1 & (1 << OCIE1A)) != 0;
	interrupts();
	return running;
}

bool timerCanRepresentStepRate(float stepRateHz)
{
	if (stepRateHz <= 0.0)
		return false;

	for (uint8_t prescalerIndex = 0; prescalerIndex < 5; prescalerIndex++)
	{
		uint16_t prescaler = timerPrescalerForIndex(prescalerIndex);
		float totalTicksFloat = (float)F_CPU / ((float)prescaler * stepRateHz);
		if (totalTicksFloat < 1.0 || totalTicksFloat > 65536.5)
			continue;

		uint32_t totalTicks = (uint32_t)(totalTicksFloat + 0.5);
		uint32_t highTicks = (((uint32_t)(F_CPU / 1000000UL) * STEP_PULSE_US) + prescaler - 1) / prescaler;
		if (highTicks < 1)
			highTicks = 1;

		if (totalTicks > highTicks && totalTicks <= 65536UL)
		{
			uint32_t lowTicks = totalTicks - highTicks;
			if (lowTicks >= 1 && lowTicks <= 65536UL && highTicks <= 65536UL)
				return true;
		}
	}

	return false;
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

	bool preserveLowPhase = false;
	uint16_t previousCounter = 0;
	uint16_t previousCompare = 0;
	if (preservePhase)
	{
		noInterrupts();
		preserveLowPhase = (TIMSK1 & (1 << OCIE1A)) != 0 && !stepPulseHigh;
		previousCounter = TCNT1;
		previousCompare = OCR1A;
		interrupts();
	}

	for (uint8_t prescalerIndex = 0; prescalerIndex < 5; prescalerIndex++)
	{
		uint16_t prescaler = timerPrescalerForIndex(prescalerIndex);
		float totalTicksFloat = (float)F_CPU / ((float)prescaler * stepRateHz);
		if (totalTicksFloat < 1.0 || totalTicksFloat > 65536.5)
			continue;

		uint32_t totalTicks = (uint32_t)(totalTicksFloat + 0.5);
		uint32_t highTicks = (((uint32_t)(F_CPU / 1000000UL) * STEP_PULSE_US) + prescaler - 1) / prescaler;
		if (highTicks < 1)
			highTicks = 1;

		if (totalTicks > highTicks && totalTicks <= 65536UL)
		{
			uint32_t lowTicks = totalTicks - highTicks;
			if (lowTicks < 1 || lowTicks > 65536UL || highTicks > 65536UL)
				continue;

			uint16_t initialCounter = 0;
			if (preserveLowPhase && previousCompare > 0)
			{
				uint32_t previousTicks = (uint32_t)previousCompare + 1UL;
				uint32_t boundedCounter = previousCounter;
				if (boundedCounter >= previousTicks)
					boundedCounter = previousTicks - 1UL;

				uint32_t scaledCounter = (boundedCounter * lowTicks) / previousTicks;
				if (scaledCounter >= lowTicks)
					scaledCounter = lowTicks - 1UL;
				initialCounter = (uint16_t)scaledCounter;
			}

			noInterrupts();
			TCCR1A = 0;
			TCCR1B = 0;
			stepPulseHigh = false;
			timer1HighPulseOcr = (uint16_t)highTicks - 1;
			timer1LowPulseOcr = (uint16_t)lowTicks - 1;
			OCR1A = timer1LowPulseOcr;
			TCNT1 = initialCounter;
			STEP_PORT &= ~STEP_MASK;
			TIFR1 |= (1 << OCF1A);
			TIMSK1 &= ~(1 << OCIE1A);
			TCCR1B = (1 << WGM12) | timerClockBitsForIndex(prescalerIndex);
			interrupts();
			return true;
		}
	}

	return false;
}

uint16_t timerPrescalerForIndex(uint8_t index)
{
	switch (index)
	{
	case 0:
		return 1;
	case 1:
		return 8;
	case 2:
		return 64;
	case 3:
		return 256;
	default:
		return 1024;
	}
}

uint8_t timerClockBitsForIndex(uint8_t index)
{
	switch (index)
	{
	case 0:
		return (1 << CS10);
	case 1:
		return (1 << CS11);
	case 2:
		return (1 << CS11) | (1 << CS10);
	case 3:
		return (1 << CS12);
	default:
		return (1 << CS12) | (1 << CS10);
	}
}

void disableTimer1()
{
	noInterrupts();
	TIMSK1 &= ~(1 << OCIE1A);
	TCCR1A = 0;
	TCCR1B = 0;
	stepPulseHigh = false;
	STEP_PORT &= ~STEP_MASK;
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
			validMaxBolusPercent(settings.maxBolusPercent) &&
			validStartupJogSpeedPercent(settings.startupJogSpeedPercent) &&
			settingsCrcMatches(&settings))
		{
			deliveryCenti = settings.deliveryCentiMlPerCm;
			persistedSyringeCenti = settings.syringeCentiMl;
			motorInverted = settings.motorDirectionInverted;
			persistedBolusEnabled = settings.bolusEnabled;
			persistedSoundEnabled = settings.soundEnabled;
			persistedMaxBolusPercent = settings.maxBolusPercent;
			persistedStartupJogSpeedPercent = settings.startupJogSpeedPercent;
			shouldSave = false;
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
		validMaxBolusPercent(previousSettings.maxBolusPercent) &&
		validStartupJogSpeedPercent(previousSettings.startupJogSpeedPercent) &&
		settingsCrcMatches(&previousSettings) &&
		previousSettings.deliveryCentiMlPerCm == editDeliveryCentiMlPerCm &&
		previousSettings.syringeCentiMl == syringeCentiMl &&
		previousSettings.bolusEnabled == (bolusEnabled ? 1 : 0) &&
		previousSettings.soundEnabled == (soundEnabled ? 1 : 0) &&
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
	settings.maxBolusPercent = maxBolusPercent;
	settings.startupJogSpeedPercent = startupJogSpeedPercent;
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

bool settingsVersion6CrcMatches(const PersistedSettingsVersion6 *settings)
{
	return settings->crc == settingsCrc16Bytes((const uint8_t *)settings, offsetof(PersistedSettingsVersion6, crc));
}

uint8_t readEepromByte(uint16_t address)
{
	waitForEepromReady();

	EEAR = address;
	EECR |= (1 << EERE);
	return EEDR;
}

void waitForEepromReady()
{
	while (EECR & (1 << EEPROM_WRITE_ENABLE_BIT))
		serviceWatchdog();
}

void updateEepromByte(uint16_t address, uint8_t value)
{
	if (readEepromByte(address) == value)
		return;

	waitForEepromReady();

	uint8_t savedStatus = SREG;
	cli();
	EEAR = address;
	EEDR = value;
	EECR |= (1 << EEPROM_MASTER_WRITE_ENABLE_BIT);
	EECR |= (1 << EEPROM_WRITE_ENABLE_BIT);
	SREG = savedStatus;
	waitForEepromReady();
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

void writePersistentSettings(const PersistedSettings *settings)
{
	const uint8_t *bytes = (const uint8_t *)settings;
	for (uint8_t index = 0; index < sizeof(PersistedSettings); index++)
		updateEepromByte(SETTINGS_EEPROM_ADDRESS + index, bytes[index]);
}

ISR(TIMER1_COMPA_vect)
{
	if (motorClockStopOnAnyEndstop ? anyEndstopActive() : endstopActiveForDirection(motorClockDirectionForward))
	{
		endstopInterruptLatched = true;
		stopMotorClockFromIsr();
		return;
	}

	if (stepPulseHigh)
	{
		STEP_PORT &= ~STEP_MASK;
		stepPulseHigh = false;
		stepCounter++;
		OCR1A = timer1LowPulseOcr;
	}
	else
	{
		STEP_PORT |= STEP_MASK;
		stepPulseHigh = true;
		OCR1A = timer1HighPulseOcr;
	}
}
