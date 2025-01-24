#include <Joystick_ESP32S2.h>
#include <SPI.h>
#include <BleGamepad.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Preferences.h>
#include <NimBLEServer.h>

Preferences Settings;
#define RW_MODE false
#define RO_MODE true

BleGamepad bleGamepad("OH58 Kiowa Collective", "DrSimgear", 100);

const char* ssid = "OH58 Kiowa Collective Update";
const char* password = "";
WebServer server(80);
bool otaMode = false;
unsigned long otaTriggerStartTime = 0;
const unsigned long OTA_TRIGGER_DURATION = 5000; // 5 seconds in milliseconds
unsigned long ct;

bool isUSBConnected = false;
bool isBLEInitialized = false;
extern "C" bool tud_mounted(void);
extern "C" bool tud_connected(void);
extern "C" bool tud_mounted(void);


// BLE Gamepad Configuration
#define NUM_BUTTONS 32
#define NUM_HAT_SWITCHES 0

// SPI Configuration
#define HSPI_MOSI 11
#define HSPI_SCLK 12
#define CS 10
#define LOAD_PIN 13

static const int SPI_CLOCK = 30000000;  // 30 MHz SPI clock
SPIClass *hspi = NULL;

// ADC Configuration
#define CONVERSIONS_PER_PIN 500
uint8_t adc_pins[] = {1};
uint8_t adc_pins_count = 1;

volatile bool GPIO0_PIN_CHANGE = false;
volatile bool adc_conversion_done = false;
adc_continuous_data_t *result = NULL;


void ARDUINO_ISR_ATTR adcComplete() {
    adc_conversion_done = true;
 }
void IRAM_ATTR isr() {
	GPIO0_PIN_CHANGE = true;
 }    

uint32_t rawStates;
uint16_t throttle;
#define GPIO0_BUTTON 22
#define GPIO0_PIN 0
uint16_t x;
uint32_t currentButtonStates = 0;
uint32_t changedButtons = 0;
const unsigned long CALIBRATION_TRIGGER_DURATION = 5000; // 5 seconds in milliseconds


bool calibrationMode = false;
unsigned long calibrationStartTime = 0;
unsigned long lastThrottleChangeTime = 0;
unsigned long frameCount;
unsigned long tsample;
unsigned long lastMillis;
uint16_t calibratedMin;
uint16_t calibratedMax;

void setlimits(){
 Settings.begin("Throttle", RO_MODE);
 size_t whatsLeft = Settings.freeEntries();    
  Serial.printf("There are: %u spaces available in table.\n", whatsLeft);
 bool calmax = Settings.isKey("maxP");
 bool calmin = Settings.isKey("minP");
 PreferenceType Typemax = Settings.getType("maxP");
 PreferenceType Typemin = Settings.getType("minP");
  Serial.print("CalibrationStatus Min:"); Serial.print(calmin); Serial.printf(" Type: %u", Typemin); Serial.printf("\t Max: %u", calmax); Serial.printf(" Type: %u \n",Typemax);
 if(!calmin || !calmax){
  Settings.end();
  Serial.println("CalbirationFALSE");  
  Settings.begin("Throttle", RW_MODE);
  //Settings.clear();
  Settings.putUShort("minP", 65535);
  Settings.putUShort("maxP", 0);
  calibratedMin =  Settings.getUShort("minP");
  calibratedMax =  Settings.getUShort("maxP");
  Settings.end();
 }else{
  uint16_t error = 1337;
  calibratedMin = Settings.getUShort("minP", error);
  calibratedMax =  Settings.getUShort("maxP", error);
  Serial.print("CalibrationAvailable");
  Serial.print("\t Min:");
  Serial.print(calibratedMin);
  Serial.print("\t MAX:");
  Serial.println(calibratedMax);
  Settings.end();
}
}



void setup() {
    Serial.begin(115200);
    
    pinMode(GPIO0_PIN, INPUT_PULLUP);
    attachInterrupt(GPIO0_PIN, isr, CHANGE);

    hspi = new SPIClass(HSPI);
    hspi->begin(HSPI_SCLK, HSPI_MOSI);
    pinMode(CS, OUTPUT);
    pinMode(LOAD_PIN, OUTPUT);
    hspi->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE2));
    setupADC();
    setupUSB();
    setlimits();
    
    }


void loop() {
    ct = millis();
    static unsigned long check;
    
    ++frameCount;

   
      bool inputsChanged = processButtons() || checkADCConversion();
        if (inputsChanged) {
        updateGamepads();
        }
        
        if (ct - check >= 500) {
          check = ct;
          if (tud_mounted() && !isUSBConnected) {
            isUSBConnected = true;
            Serial.println("USB mounted");
              if (isBLEInitialized){
              ESP.restart();
              }
          }else if(!tud_mounted() && isUSBConnected){
            isUSBConnected = false;
            Serial.println("USB unmounted");
              
          }    
        }
        

        if (ct - lastMillis >= 1000) {
          
         checkCalibrationTrigger();
         checkOTATrigger();

        if (otaMode) {
        server.handleClient();
        ElegantOTA.loop();
        } 

        if (calibrationMode) {
             performCalibration();
          }
        if(!isBLEInitialized && !tud_mounted() && !isUSBConnected){
              setupBLEGamepad();
          }   
         Serial.print("ADC:"); Serial.print(x); Serial.print("\t Cal:"); Serial.print(throttle); Serial.printf("\t CPU:%u", frameCount); Serial.printf("\t adcHZ:%u\n",tsample);
            tsample = 0;
            frameCount = 0;
            lastMillis = ct;
            }
            
    }


void setupADC() {
    analogContinuousSetAtten(ADC_11db);
    analogContinuousSetWidth(12);
    analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 60000, &adcComplete);
    analogContinuousStart();}

void setupBLEGamepad() {
    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setAutoReport(false);
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleGamepadConfig.setButtonCount(NUM_BUTTONS);
    bleGamepadConfig.setHatSwitchCount(NUM_HAT_SWITCHES);
    bleGamepadConfig.setWhichAxes(false, false, false, false, false, false, false, false);
    bleGamepadConfig.setWhichSimulationControls(false, true, false, false, false);
    bleGamepadConfig.setVid(0xEB58);
    bleGamepadConfig.setPid(0x1209);
    bleGamepadConfig.setSimulationMin(0);
    bleGamepadConfig.setSimulationMax(65535);
    bleGamepad.begin(&bleGamepadConfig);
    isBLEInitialized = true;}

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_GAMEPAD, NUM_BUTTONS,
  0, // No hat switches
  false, false, false, false, false, false,
  false, true, false, false, false);

void setupUSB() {
    USB.VID(0xEB58);
    USB.PID(0x1209);
    USB.productName("OH58 Kiowa Collective");
    USB.manufacturerName("DrSimgear");
    Joystick.setThrottleRange(0, 65535);
    Joystick.begin(false);
    USB.begin();
    }

void checkOTATrigger() {
    if ((currentButtonStates & ((1UL << 3) | (1UL << 28))) == ((1UL << 3) | (1UL << 28))) {
        if (!otaTriggerStartTime) {
            otaTriggerStartTime = ct;
        } else if (ct - otaTriggerStartTime >= OTA_TRIGGER_DURATION) {
            Serial.println("OTA update mode triggered");
            startOTAMode();
             if (isBLEInitialized){
       
         }
        } else {
            otaTriggerStartTime = 0;
        }
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
    Serial.println("OTA update server started");}

void checkCalibrationTrigger() {
    static unsigned long triggerStartTime = 0;
    if ((currentButtonStates & ((1UL << 22) | (1UL << 23))) == ((1UL << 22) | (1UL << 23))) {
        if (!triggerStartTime) {
            triggerStartTime = millis();
        } else if ((millis() - triggerStartTime >= CALIBRATION_TRIGGER_DURATION)) {
            calibrationMode = true;
            calibrationStartTime = millis();
            lastThrottleChangeTime = millis();
          Serial.println("Calibration mode activated");
        }
    } else {
        triggerStartTime = 0;
    }

    }

void performCalibration() {
 static bool i;
  if (calibrationMode && !i){
    i=true;
    calibratedMin= 32767;
    calibratedMax= 0;
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
        Serial.print("lastThrottleChangeTime:");Serial.print(lastThrottleChangeTime);
        Serial.print(" \t ADC:"); Serial.print(x);Serial.print(" \t New Min:"); Serial.print(calibratedMin); 
        Serial.print(" \t New Max:"); Serial.println(calibratedMax);
        }
        // If throttle hasn't changed for 5 seconds, save and exit calibration
        if (millis() - lastThrottleChangeTime >= 5000) {
        Settings.begin("Throttle", false); 
        Serial.print("Old Min: "); Serial.println(Settings.getUShort("minP"));
        Serial.print("Old Max: "); Serial.println(Settings.getUShort("maxP"));
        Settings.putUShort("minP", calibratedMin);
        Settings.putUShort("maxP", calibratedMax);
        Settings.end();
        Serial.println("Calibration saved");
        Serial.print("New Min: "); Serial.println(calibratedMin);
        Serial.print("New Max: "); Serial.println(calibratedMax);
        
        Serial.println("Calibration mode deactivated");
        calibrationMode = false;
        i = 0;
        }
    }  

uint32_t readShiftRegisters() {
    
    digitalWrite(LOAD_PIN, LOW);
    digitalWrite(LOAD_PIN, HIGH);
    digitalWrite(CS, LOW);
    
    rawStates = 0;
    for (int i = 0; i < 4; ++i) {
        rawStates = (rawStates << 8) | hspi->transfer(0);
    }
    
    digitalWrite(CS, HIGH);
    //hspi->endTransaction();

    return rawStates;}

bool processButtons() {
rawStates = readShiftRegisters();
static uint32_t lastButtonStates1;
static uint32_t lastButtonStates2;
uint32_t update = rawStates ^ lastButtonStates1;
lastButtonStates1 = rawStates;



  if (update||GPIO0_PIN_CHANGE){
    GPIO0_PIN_CHANGE = 0;
    if (update & (1UL << 8)) {  // center of second switch
      currentButtonStates &= ~(1 << 3 | 1 << 4 | 1 << 12 | 1 << 11);
      currentButtonStates |= ((rawStates >> 12 & rawStates >> 11 & 1UL) << 3);
      currentButtonStates |= ((rawStates >> 12 & rawStates >> 9  & 1UL) << 4);
      currentButtonStates |= ((rawStates >> 9  & rawStates >> 10 & 1UL) << 12);
      currentButtonStates |= ((rawStates >> 10 & rawStates >> 11 & 1UL) << 11);
  } else if (update & (1UL << 0)) { 
      currentButtonStates &= ~(1 << 1 | 1 << 2 | 1 << 9 | 1 << 10);
      currentButtonStates |= ((rawStates >> 4 & 1UL) << 1);
      currentButtonStates |= ((rawStates >> 2 & 1UL) << 2);
      currentButtonStates |= ((rawStates >> 3 & 1UL) << 9);
      currentButtonStates |= ((rawStates >> 1 & 1UL) << 10);
  } else if (update & (1UL << 21)) {  // center of third switch
      currentButtonStates &= ~(1 << 5 | 1 << 6 | 1 << 13 | 1 << 14);
      currentButtonStates |= ((rawStates >> 29 & 1UL) << 5);
      currentButtonStates |= ((rawStates >> 31 & 1UL) << 6);
      currentButtonStates |= ((rawStates >> 30 & 1UL) << 13);
      currentButtonStates |= ((rawStates >> 28 & 1UL) << 14);
  } else if (update & (1UL << 16)) {  // center of fourth switch
      currentButtonStates &= ~(1 << 20 | 1 << 18 | 1 << 19 | 1 << 17);
      currentButtonStates |= ((rawStates >> 20 & 1UL) << 17);
      currentButtonStates |= ((rawStates >> 18 & 1UL) << 18);
      currentButtonStates |= ((rawStates >> 19 & 1UL) << 25);
      currentButtonStates |= ((rawStates >> 17 & 1UL) << 26);
  } else if (update & (1UL << 24)) {  // center of fifth switch
      currentButtonStates &= ~(1 << 7 | 1 << 8 | 1 << 15 | 1 << 16);
      currentButtonStates |= ((rawStates >> 5 & 1UL) << 7);
      currentButtonStates |= ((rawStates >> 27 & 1UL) << 8);
      currentButtonStates |= ((rawStates >> 25 & 1UL) << 15);
      currentButtonStates |= ((rawStates >> 26 & 1UL) << 16);
  }
      currentButtonStates &= ~(1 << 30 | 1 << 29 | 1 << 0 | 1 << 31 | 1 << 24 | 1 << 23| 1 << 22);
      currentButtonStates |= ((rawStates >> 6 & 1UL) << 30);
      currentButtonStates |= ((rawStates >> 7 & 1UL) << 29);
      currentButtonStates |= ((rawStates >> 13 & 1UL) << 0);
      currentButtonStates |= ((rawStates >> 14 & 1UL) << 31);
      currentButtonStates |= ((rawStates >> 15 & 1UL) << 24);
      currentButtonStates |= ((rawStates >> 23 & 1UL) << 23);
    if (!digitalRead(GPIO0_PIN)) {
       currentButtonStates |= (1UL << GPIO0_BUTTON);
    }
    } 

  changedButtons = currentButtonStates ^ lastButtonStates2;
    lastButtonStates2 = currentButtonStates;
    return changedButtons != 0;
}


bool checkADCConversion() { 
  static uint16_t previousAdcValue;
    if (adc_conversion_done) {
      tsample++;   
        adc_conversion_done = false;
        if (analogContinuousRead(&result, 0)) { 
        x = result[0].avg_read_raw;
        if (abs(x - previousAdcValue) > 2) { //hysterisys
         int y = map(x, calibratedMin+5, calibratedMax-10, 65535, 0);
         throttle = constrain(y, 0, 65535);
        }
        previousAdcValue = x;   
        }
        return true;
    }
    return false;
    }

void updateGamepads() {
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        if (changedButtons & (1UL << i)) {
            if (currentButtonStates & (1UL << i)) {
                if (isUSBConnected) {
                    Joystick.pressButton(i - 1);
                } else if (bleGamepad.isConnected()) {
                    bleGamepad.press(i);
                }
            } else {
                if (isUSBConnected) {
                    Joystick.releaseButton(i - 1);
                } else if (bleGamepad.isConnected()) {
                    bleGamepad.release(i);
                }
            }
        }
    }
    
    if (isUSBConnected) {
      Joystick.setThrottle(throttle);
        Joystick.sendState();
    } 
    else if (bleGamepad.isConnected()) {
      bleGamepad.setThrottle(throttle);
        bleGamepad.sendReport();
    }
}
