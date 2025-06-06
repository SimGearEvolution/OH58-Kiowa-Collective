#include <Joystick_ESP32S2.h>
#include <SPI.h>
#include <BleGamepad.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Preferences.h>
#include <NimBLEServer.h>
#include <ESP32SPISlave.h>
#define DEBUG_MODE
#define STRUCT
Preferences Settings;
#define RW_MODE false
#define RO_MODE true
#define BLE


BleGamepad bleGamepad("OH58 Kiowa Collective", "Simulator", 100);

const char *ssid = "OH58 Kiowa Collective Update";
const char *password = "";
WebServer server(80);
bool otaMode = false;
unsigned long menuTriggerStartTime = 0;
const unsigned long TRIGGER_DURATION = 5000;  // 5 seconds in milliseconds
unsigned long ct;

extern "C" bool tud_mounted(void);
extern "C" bool tud_connected(void);

// BLE Gamepad Configuration
#define NUM_BUTTONS 32
#define NUM_HAT_SWITCHES 0

// SPI Configuration
SPIClass *hspi = NULL;
#define HSPI_MISO 11
#define HSPI_SCLK 12
#define HSPI_SS 13
#define CS 10

//Slave SPI
ESP32SPISlave *fspi = NULL;
#define FSPI_SDO 36  //To MISO
#define FSPI_SCLK 37
#define FSPI_SS 35

static constexpr size_t BUFFER_SIZE = 8;
static constexpr size_t BUFFER_SIZE1 = 4;
uint8_t rx_buf[BUFFER_SIZE1];
uint8_t res = 0;    // 1 byte
uint8_t id = 0x02;  // 1 byte

#define JETTISON 48

// ADC Configuration
#define CONVERSIONS_PER_PIN 500
uint8_t adc_pins[] = { 1 };
uint8_t adc_pins_count = 1;
uint16_t simMin = 0;
uint16_t simMax = 4096;
uint16_t x;
adc_continuous_data_t *result = NULL;
bool isUSBConnected = false;
bool isBLEInitialized = false;
bool wasBLEInitialized = false;
bool bleManualMode = false;
bool usbManualMode = false;
bool fspiManualMode = false;
bool bleMode = false;
bool usbMode = true;
bool fspiMode = true;
bool autoMode = true;
bool hspiIni = false;
bool fspiIni = false;
bool usbIni = false;
bool usbwasIni = false;
bool MenuTriggered = false;
uint32_t rawStates;
uint16_t throttle;
uint32_t ButtonStates = 0;
uint32_t ShiftedButtonStates =0;
uint32_t changedButtons = 0;
const unsigned long CALIBRATION_TRIGGER_DURATION = 10000;  // 5 seconds in milliseconds
static const int SPI_CLOCK = 30000000;                     // 30 MHz SPI clock
bool calibrationMode = false;
unsigned long calibrationStartTime = 0;
unsigned long lastThrottleChangeTime = 0;
unsigned long frameCount;
unsigned long tsample;
unsigned long lastMillis;
uint16_t calibratedMin;
uint16_t calibratedMax;
volatile bool CSactivated = false;  // Atomic flag
void IRAM_ATTR handleCSrising() {
  CSactivated = true;  // Marks end of transaction
}
volatile bool adc_conversion_done = false;
void ARDUINO_ISR_ATTR adcComplete() {
  adc_conversion_done = true;
}
volatile bool JETTISON_CHANGE = false;
void IRAM_ATTR isr() {
  JETTISON_CHANGE = true;
}

//-------------------------------------------------------------------------------------------------
void setlimits() {
  Settings.begin("Throttle", RW_MODE);
  size_t whatsLeft = Settings.freeEntries();
  Serial.printf("There are: %u spaces available in table.\n", whatsLeft);
  if (!Settings.isKey("bleManualMode")) {
    Settings.putBool("bleManualMode", bleManualMode);
  }
  if (!Settings.isKey("bleMode")) {
    Settings.putBool("bleMode", bleMode);
  }
  if (!Settings.isKey("usbManualMode")) {
    Settings.putBool("usbManualMode", usbManualMode);
  }
  if (!Settings.isKey("usbMode")) {
    Settings.putBool("usbMode", usbMode);
  }
  if (!Settings.isKey("fspiManualMode")) {
    Settings.putBool("fspiManualMode", fspiManualMode);
  }
  if (!Settings.isKey("fspiMode")) {
    Settings.putBool("fspiMode", fspiMode);
  }
  if (!Settings.isKey("autoMode")) {
    Settings.putBool("autoMode", autoMode);
  }
  if (!Settings.isKey("minP")) {
    Settings.putUShort("minP", 65535);
  }
  if (!Settings.isKey("maxP")) {
    Settings.putUShort("maxP", 0);
  }
  calibratedMin = Settings.getUShort("minP");
  calibratedMax = Settings.getUShort("maxP");
  calibratedMin = Settings.getUShort("minP");
  calibratedMax = Settings.getUShort("maxP");
  bleManualMode = Settings.getBool("bleManualMode");
  bleMode = Settings.getBool("bleMode");
  usbManualMode = Settings.getBool("usbManualMode");
  usbMode = Settings.getBool("usbMode");
  fspiManualMode = Settings.getBool("fspiManualMode");
  fspiMode = Settings.getBool("fspiMode");
  autoMode = Settings.getBool("autoMode");
  Serial.print("CalibrationAvailable:");
  Serial.printf("\t Min: %u", calibratedMin);
  Serial.printf("\t Max: %u\n", calibratedMax);
  Serial.printf("Modes:");
  Serial.printf("\t bleManualMode: %u", bleManualMode);
  Serial.printf("\t bleMode: %u", bleMode);
  Serial.printf("\t usbManualMode: %u", usbManualMode);
  Serial.printf("\t usbMode: %u", usbMode);
  Serial.printf("\t fspiManualMode: %u", fspiManualMode);
  Serial.printf("\t fspiMode: %u", fspiMode);
  Serial.printf("\t autoMode: %u \n", autoMode);
  Settings.end();
}
//-------------------------------------------------------------------------------------------------
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
                   JOYSTICK_TYPE_GAMEPAD, NUM_BUTTONS,
                   0,  // No hat switches
                   false, false, false, false, false, false,
                   false, true, false, false, false);
void setup() {
  Serial.begin(921600);
  pinMode(JETTISON, INPUT_PULLUP);
  attachInterrupt(JETTISON, isr, CHANGE);
  setlimits();
  setupADC();
  startHSPI();
  if (usbMode) {
    setupUSB();
  }

  if (fspiManualMode) {
    startFSPI();
  }
  if (bleManualMode) {
    setupBLEGamepad();
  }
}

void startHSPI() {
  hspi = new SPIClass(HSPI);
  hspi->begin(HSPI_SCLK, HSPI_MISO, -1, HSPI_SS);
  hspi->setSSInvert(true);
  hspi->setHwCs(true);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, LOW);
  hspi->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE2));
  hspiIni = true;
  Serial.printf("\t hspiIni: %u\n", hspiIni);
}

void startFSPI() {
  fspi = new ESP32SPISlave();
  pinMode(FSPI_SS, INPUT_PULLUP);
  attachInterrupt(FSPI_SS, handleCSrising, RISING);
  fspi->setDataMode(SPI_MODE1);
  fspi->begin(FSPI, FSPI_SCLK, FSPI_SDO, -1, FSPI_SS);
  fspiIni = true;
  Serial.printf("\t fspiIni: %u\n", fspiIni);
}

#ifdef STRUCT
/*
uint32_t swap32AndReverseBits(uint32_t val) {
  // First swap the bytes
  //uint32_t swapped = ((val & 0xFF) << 24) | ((val & 0xFF00) << 8) | ((val & 0xFF0000) >> 8) | ((val & 0xFF000000) >> 24);

  uint32_t result = 0;
  for (int i = 0; i < 32; i++) {
    if (val & (1UL << i))
      result |= (1UL << (31 - i));
  }
  return result;
}
 */
// 16-bit byte swap
uint16_t swap16(uint16_t val) {
  return (val << 8) | (val >> 8);
}
#pragma pack(push, 1)
typedef struct {
  uint32_t Buttons;
  uint16_t Throttle;
  uint8_t _res = 0;
  uint8_t id = 0x07;
} Controller_Data;
#pragma pack(pop)
Controller_Data ctrl_data;
#endif

bool lastStableState = false;
bool currentState = false;
bool isTUDmounted = false;
unsigned long debounceStart = 0;
const unsigned long debounceDelay = 500;  // Stabilization period

void checkUSBConnection() {
  currentState = tud_mounted();

  if (currentState != lastStableState) {
    // State changed, start debounce timer
    if (debounceStart == 0) {
      debounceStart = millis();
    } else if (millis() - debounceStart >= debounceDelay) {
      // State stabilized, update stable state
      lastStableState = currentState;
      debounceStart = 0;

      if (currentState) {
        isTUDmounted = true;
        Serial.println("USB mounted");
      } else {
        isTUDmounted = false;
        Serial.println("USB unmounted");
      }
    }
  } else {

    debounceStart = 0;
  }
}

//-------------------------------------------------------------------------------------------------
void loop() {
  ct = millis();
  static unsigned long check;
  static unsigned long time;
  bool inputsChanged = 0;
  ++frameCount;

  if (ct - time >= 1) {
    time = ct;
    inputsChanged = processButtons() || checkADCConversion();
    if (autoMode) {
      checkUSBConnection();
      if (!isTUDmounted) {
        if (!isBLEInitialized && bleMode && !wasBLEInitialized) {
          setupBLEGamepad();
          Serial.println("BLE ON");
        } else if (!isBLEInitialized && bleMode && wasBLEInitialized) {
          Serial.println("Restarting");
          ESP.restart();
        }
        if (!fspiIni && fspiMode) {
          startFSPI();
          Serial.println("FSPI ON");
        }
      } else if (isTUDmounted) {
        if (isBLEInitialized && bleMode && !bleManualMode) {
          bleGamepad.end();
          Serial.println("BLE OFF");
          isBLEInitialized = false;
        }
        if (fspiIni && !fspiManualMode) {
          fspi->end();
          fspiIni = false;
        }
      }
    }
  }

  if (inputsChanged) {
    if (fspiIni) {
      ctrl_data.Buttons = ShiftedButtonStates;           
      ctrl_data.Throttle = swap16(map(throttle, simMin, simMax, 0, 65535));  // Reverse 2 bytes
      ctrl_data._res;
      ctrl_data.id;
    }

    if (bleGamepad.isConnected()) {
      bleGamepad.setButtons32(ShiftedButtonStates);  // Update all buttons at once, >> 1 if your buttons start with one
      bleGamepad.setThrottle(throttle);
      bleGamepad.sendReport();
    }


    if (usbMode && isTUDmounted) {
      Joystick.setButtons32(ShiftedButtonStates);  // Update all buttons at once, >> 1 if your buttons start with one
      Joystick.setThrottle(throttle);
      Joystick.sendState();
    }
  }

  if (CSactivated && fspiIni) {
    CSactivated = false;
    fspi->queue((uint8_t *)&ctrl_data, NULL, sizeof(Controller_Data));
    fspi->trigger();
  }

  if (MenuTriggered) {
    menuMode();
  }

  if (otaMode) {
    server.handleClient();
    ElegantOTA.loop();
  }

  if (ct - lastMillis >= 1000) {
    MenuTrigger();
    // if (autoMode) {
    //  if (!wasBLEInitialized && !isBLEInitialized && !isUSBConnected && bleMode) {
    //    setupBLEGamepad();
    //  }
    //  }

#ifdef DEBUG_MODE
    Serial.printf("\t CPU:%u", frameCount);
    //Serial.print("  buttons: ");
    //Serial.print(ShiftedButtonStates, BIN);
    Serial.printf("\t adcHZ:%u\n", tsample);

    tsample = 0;
    frameCount = 0;
#endif
    lastMillis = ct;
  }
}
//-------------------------------------------------------------------------------------------------
void setupBLEGamepad() {
  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  bleGamepadConfig.setButtonCount(NUM_BUTTONS);
  bleGamepadConfig.setHatSwitchCount(NUM_HAT_SWITCHES);
  bleGamepadConfig.setWhichAxes(false, false, false, false, false, false, false, false);
  bleGamepadConfig.setWhichSimulationControls(false, true, false, false, false);
  bleGamepadConfig.setVid(0x1209);
  bleGamepadConfig.setPid(0x0c58);
  bleGamepadConfig.setSimulationMin(simMin);
  bleGamepadConfig.setSimulationMax(simMax);
  bleGamepad.begin(&bleGamepadConfig);
  isBLEInitialized = true;
  wasBLEInitialized = true;
  Serial.printf("\t isBLEInitialized %u\n", isBLEInitialized);
}
//-------------------------------------------------------------------------------------------------
void setupUSB() {
  USB.VID(0x1209);
  USB.PID(0x0c58);
  USB.productName("OH58 Kiowa Collective");
  USB.manufacturerName("UltimateEvolutionControls");
  Joystick.setThrottleRange(simMin, simMax);
  Joystick.begin(false);
  USB.begin();
  usbIni = true;
  usbwasIni = true;
  Serial.printf("\t usbIni: %u\n", usbIni);
}
//-------------------------------------------------------------------------------------------------
void MenuTrigger() {
  if ((ButtonStates & ((1UL << 22) | (1UL << 23))) == ((1UL << 22) | (1UL << 23))) {
    if (!menuTriggerStartTime) {

      menuTriggerStartTime = millis();
    } else if (millis() - menuTriggerStartTime >= TRIGGER_DURATION) {
      Serial.println("Menu ON");
      MenuTriggered = true;
    }
  } else {
    menuTriggerStartTime = 0;
  }
}

void menuMode() {
  static bool saved;
  static bool tempbleMode = bleMode;
  static bool tempusbMode = usbMode;
  static bool tempautoMode = autoMode;
  if ((ButtonStates & (1UL << 17)) && !otaMode) {
    startOTAMode();
  }

  if ((ButtonStates & (1UL << 18)) && !calibrationMode) {
    calibrationMode = true;
    calibrationStartTime = millis();
    lastThrottleChangeTime = millis();
    Serial.println("Calibration mode activated");
  }
  if (calibrationMode) {
    performCalibration();
  }

  if ((ButtonStates & (1UL << 6)) && bleManualMode) {
    bleManualMode = false;
    Serial.println("bleManualMode: false");
  }

  if ((ButtonStates & (1UL << 5)) && !bleManualMode) {

    bleManualMode = true;
    Serial.println("bleManualMode: true");
  }

  if ((ButtonStates & (1UL << 13)) && tempbleMode) {
    tempbleMode = false;
    Serial.println("tempbleMode: false");
  }
  if ((ButtonStates & (1UL << 14)) && !tempbleMode) {
    tempbleMode = true;
    Serial.println("tempbleMode: true");
  }

  if ((ButtonStates & (1UL << 8)) && usbManualMode) {

    usbManualMode = false;
    Serial.println("usbManualMode: false");
  }
  if (ButtonStates & (1UL << 7) && !usbManualMode) {

    usbManualMode = true;
    Serial.println("usbManualMode: true");
  }

  if ((ButtonStates & (1UL << 15)) && tempusbMode) {
    tempusbMode = false;
    Serial.println("tempusbMode: false");
  }
  if ((ButtonStates & (1UL << 16)) && !tempusbMode) {
    tempusbMode = true;
    Serial.println("tempusbMode: true");
  }

  if ((ButtonStates & (1UL << 4)) && fspiManualMode) {

    fspiManualMode = false;
    Serial.println("fspiManualMode: false");
  }
  if ((ButtonStates & (1UL << 3)) && !fspiManualMode) {

    fspiManualMode = true;
    Serial.println("fspiManualMode: true");
  }
  if ((ButtonStates & (1UL << 12)) && fspiMode) {
    fspiMode = false;
    Serial.println("fspiMode: false");
  }
  if ((ButtonStates & (1UL << 11)) && !fspiMode) {
    fspiMode = true;
    Serial.println("fspiMode: true");
  }

  if ((ButtonStates & (1UL << 10)) && tempautoMode) {
    tempautoMode = false;
    Serial.println("tempautoMode: false");
  }
  if ((ButtonStates & (1UL << 9)) && !tempautoMode) {
    tempautoMode = true;
    Serial.println("tempautoMode: true");
  }

  if (ButtonStates & (1UL << 1)) {  //menuEnd
    setlimits();
    MenuTriggered = false;
  }

  if ((ButtonStates & (1UL << 2)) && !saved) {  //saveSettings
    saved = true;
    Settings.begin("Throttle", RW_MODE);
    size_t whatsLeft = Settings.freeEntries();
    Serial.printf("There are: %u spaces available in table.\n", whatsLeft);
    if (bleManualMode != Settings.getBool("bleManualMode")) {
      Settings.putBool("bleManualMode", bleManualMode);
    }
    if (bleMode != Settings.getBool("bleMode")) {
      Settings.putBool("bleMode", tempbleMode);
    }
    if (usbManualMode != Settings.getBool("usbManualMode")) {
      Settings.putBool("usbManualMode", usbManualMode);
    }
    if (usbMode != Settings.getBool("usbMode")) {
      Settings.putBool("usbMode", tempusbMode);
    }
    if (fspiManualMode != Settings.getBool("fspiManualMode")) {
      Settings.putBool("fspiManualMode", fspiManualMode);
    }
    if (fspiMode != Settings.getBool("fspiMode")) {
      Settings.putBool("fspiMode", fspiMode);
    }
    if (autoMode != Settings.getBool("autoMode")) {
      Settings.putBool("autoMode", tempautoMode);
    }
    Serial.printf("Settings Saved \n");
    whatsLeft = Settings.freeEntries();
    Serial.printf("There are: %u spaces available in table.\n", whatsLeft);
    Settings.end();
  }
  if (!(ButtonStates & (1UL << 2)) && saved) {
    saved = false;
  }
}
void startOTAMode() {
  otaMode = true;
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP address: ");
  Serial.println(IP);

  server.on("/", []() {
    server.send(200, "text/plain", "OH58 Kiowa Collective OTA Update Server");
  });

  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("OTA update server started");
}
void performCalibration() {
  static bool i;
  if (calibrationMode && !i) {
    i = true;
    calibratedMin = 32767;
    calibratedMax = 0;
  }

  // Update min and max values
  if (x < calibratedMin) {
    calibratedMin = x;
    lastThrottleChangeTime = millis();
  }
  if (x > calibratedMax) {
    calibratedMax = x;
    lastThrottleChangeTime = millis();
  }
  if (ct - lastMillis >= 100) {
    Serial.print("lastThrottleChangeTime:");
    Serial.print(lastThrottleChangeTime);
    Serial.print(" \t ADC:");
    Serial.print(x);
    Serial.print(" \t New Min:");
    Serial.print(calibratedMin);
    Serial.print(" \t New Max:");
    Serial.println(calibratedMax);
  }
  // If throttle hasn't changed for 5 seconds, save and exit calibration
  if (millis() - lastThrottleChangeTime >= 5000) {
    Settings.begin("Throttle", RW_MODE);
    Serial.print("Old Min: ");
    Serial.println(Settings.getUShort("minP"));
    Serial.print("Old Max: ");
    Serial.println(Settings.getUShort("maxP"));
    Settings.putUShort("minP", calibratedMin);
    Settings.putUShort("maxP", calibratedMax);
    Settings.end();
    Serial.println("Calibration saved");
    Serial.print("New Min: ");
    Serial.println(calibratedMin);
    Serial.print("New Max: ");
    Serial.println(calibratedMax);
    Serial.println("Calibration mode deactivated");
    calibrationMode = false;
    i = 0;
  }
}

bool processButtons() {
  rawStates = hspi->transfer32(0);
  static uint32_t lastButtonStates1;
  static uint32_t lastButtonStates2;
  uint32_t update = rawStates ^ lastButtonStates1;
  lastButtonStates1 = rawStates;



  if (update || JETTISON_CHANGE) {
    JETTISON_CHANGE = 0;
    if (update & (1UL << 8)) {  // center of second switch
      ButtonStates &= ~(1 << 3 | 1 << 4 | 1 << 12 | 1 << 11);
      ButtonStates |= ((rawStates >> 12 & rawStates >> 11 & 1UL) << 3);
      ButtonStates |= ((rawStates >> 12 & rawStates >> 9 & 1UL) << 4);
      ButtonStates |= ((rawStates >> 9 & rawStates >> 10 & 1UL) << 12);
      ButtonStates |= ((rawStates >> 10 & rawStates >> 11 & 1UL) << 11);
    } else if (update & (1UL << 0)) {
      ButtonStates &= ~(1 << 1 | 1 << 2 | 1 << 9 | 1 << 10);
      ButtonStates |= ((rawStates >> 4 & 1UL) << 1);
      ButtonStates |= ((rawStates >> 2 & 1UL) << 2);
      ButtonStates |= ((rawStates >> 3 & 1UL) << 9);
      ButtonStates |= ((rawStates >> 1 & 1UL) << 10);
    } else if (update & (1UL << 21)) {  // center of third switch
      ButtonStates &= ~(1 << 5 | 1 << 6 | 1 << 13 | 1 << 14);
      ButtonStates |= ((rawStates >> 29 & 1UL) << 5);
      ButtonStates |= ((rawStates >> 31 & 1UL) << 6);
      ButtonStates |= ((rawStates >> 30 & 1UL) << 13);
      ButtonStates |= ((rawStates >> 28 & 1UL) << 14);
    } else if (update & (1UL << 16)) {  // center of fourth switch
      ButtonStates &= ~(1 << 26 | 1 << 18 | 1 << 25 | 1 << 17);
      ButtonStates |= ((rawStates >> 20 & 1UL) << 17);
      ButtonStates |= ((rawStates >> 18 & 1UL) << 18);
      ButtonStates |= ((rawStates >> 19 & 1UL) << 25);
      ButtonStates |= ((rawStates >> 17 & 1UL) << 26);
    } else if (update & (1UL << 24)) {  // center of fifth switch
      ButtonStates &= ~(1 << 7 | 1 << 8 | 1 << 15 | 1 << 16);
      ButtonStates |= ((rawStates >> 5 & 1UL) << 7);
      ButtonStates |= ((rawStates >> 27 & 1UL) << 8);
      ButtonStates |= ((rawStates >> 25 & 1UL) << 15);
      ButtonStates |= ((rawStates >> 26 & 1UL) << 16);
    }
    ButtonStates &= ~(1 << 30 | 1 << 29 | 1 << 0 | 1 << 31 | 1 << 24 | 1 << 23 | 1 << 22);
    ButtonStates |= ((rawStates >> 6 & 1UL) << 30);
    ButtonStates |= ((rawStates >> 7 & 1UL) << 29);
    ButtonStates |= ((rawStates >> 13 & 1UL) << 0);
    ButtonStates |= ((rawStates >> 14 & 1UL) << 31);
    ButtonStates |= ((rawStates >> 15 & 1UL) << 24);
    ButtonStates |= ((rawStates >> 23 & 1UL) << 23);
    if (!digitalRead(JETTISON)) {
      ButtonStates |= (1UL << 22);
    }
  }
  ShiftedButtonStates = ButtonStates >>1;
  changedButtons = ButtonStates ^ lastButtonStates2;
  lastButtonStates2 = ButtonStates;
  return changedButtons != 0;
}

void setupADC() {
  analogContinuousSetAtten(ADC_11db);
  analogContinuousSetWidth(12);
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 60000, &adcComplete);
  analogContinuousStart();
}

bool checkADCConversion() {
  static uint16_t previousAdcValue;
  const uint16_t HYSTERESIS = 3;      // Adjust based on your noise level
  const uint16_t DEADZONE_LOW = 10;   // Lower dead zone margin
  const uint16_t DEADZONE_HIGH = 30;  // Upper dead zone margin

  if (adc_conversion_done) {
    tsample++;
    adc_conversion_done = false;

    if (analogContinuousRead(&result, 0)) {
      x = result[0].avg_read_raw;

      // 1. Apply dead zones and constrain to calibrated range
      uint16_t raw_x = constrain(x,
                                 calibratedMin + DEADZONE_LOW,
                                 calibratedMax - DEADZONE_HIGH);

      // 2. Check for significant change or boundary conditions
      bool shouldUpdate = (abs(raw_x - previousAdcValue) > HYSTERESIS) || (raw_x <= calibratedMin + DEADZONE_LOW) || (raw_x >= calibratedMax - DEADZONE_HIGH);

      if (shouldUpdate) {
        // 3. Map to final throttle range (prevents reverse mapping)
        throttle = map(raw_x,
                       calibratedMin + DEADZONE_LOW,
                       calibratedMax - DEADZONE_HIGH,
                       simMin,
                       simMax);

        // 4. Update tracking value only when changes are committed
        previousAdcValue = x;
      }
    }
    return true;
  }
  return false;
}
