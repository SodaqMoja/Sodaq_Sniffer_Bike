/*
Copyright (c) 2018, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include <Wire.h>
#include "RTCTimer.h"
#include "RTCZero.h"
#include "Sodaq_wdt.h"
#include "Config.h"
#include "BootMenu.h"
#include "ublox.h"
#include "MyTime.h"
#include "Sodaq_LSM303AGR.h"
#include "PM3015.h"
#include "Adafruit_BME680.h"
#include "LedColor.h"
#include "Enums.h"
#include "Sodaq_nbIOT.h"
#include "UdpMessage.h"
#include "GpsRecord.h"
#include "SensorData.h"
#include "BatteryState.h"

// #define DEBUG

#define PROJECT_NAME "SODAQ - Sniffer Bike"
#define VERSION "1.0.0"
#define STARTUP_DELAY 5000

#define DEFAULT_SENSORS_SAMPLE_INTERVAL 10 // 10 seconds
#define UDP_SEND_LED_DURATION 2000 // keep led on and green for 2 seconds on udp send
#define SENSORS_ON_LED_BLINK_INTERVAL 500

#define GPS_TIME_VALIDITY 0b00000011 // date and time (but not fully resolved)
#define GPS_FIX_FLAGS 0b00000001 // just gnssFixOK
#define GPS_COMM_CHECK_TIMEOUT 3 // seconds

#define MAX_RTC_EPOCH_OFFSET 25

#define ADC_AREF 3.3f
#define BATVOLT_R1 4.7f
#define BATVOLT_R2 10.0f

#define BATTERY_LOW_WARNING_THRESHOLD 3700 // mV
#define BATTERY_TOO_LOW_THRESHOLD 3500 // mV

#define DEBUG_STREAM SerialUSB
#define CONSOLE_STREAM SerialUSB

#define CONSOLE_BAUD 115200
#define DEBUG_BAUD 115200 // only used when CONSOLE is different than debug, otherwise console baud is used only

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

// macro to do compile time sanity checks / assertions
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#define consolePrint(x) CONSOLE_STREAM.print(x)
#define consolePrintln(x) CONSOLE_STREAM.println(x)

#define debugPrint(x) if (params.getIsDebugOn()) { DEBUG_STREAM.print(x); }
#define debugPrintln(x) if (params.getIsDebugOn()) { DEBUG_STREAM.println(x); }

RTCZero rtc;
RTCTimer timer;
UBlox ublox;
Time time;
Sodaq_LSM303AGR accelerometer;
PM3015 pm3015;
Sodaq_nbIOT nbiot;
Adafruit_BME680 bme;

#define DEFAULT_APN "cdp.iot.t-mobile.nl"
#define DEFAULT_CDP "172.27.131.100"
#define DEFAULT_TARGET_IP "172.27.131.100"
#define DEFAULT_TARGET_PORT 15683
#define DEFAULT_FORCE_OPERATOR "20416"
#define DEFAULT_BAND 8

#define MODEM_TX_ENABLE_PIN SARA_TX_ENABLE
#define MODEM_ON_OFF_PIN SARA_ENABLE
#define MODEM_RESET SARA_RESET
#define MODEM_STREAM Serial1
#define PM_SENSOR_STREAM Serial
#define PM_SENSOR_RX_PIN (PIN_SERIAL_RX)
#define PM_SENSOR_TX_PIN (PIN_SERIAL_TX)

#define BOOST_CONVERTER_PIN 3

UdpMessage pendingSamplesUdpMessage;
size_t currentSampleRecordIndex;

GpsRecord pendingGpsRecord;
bool isPendingGpsRecordNew; // this is set to true only when pendingReportDataRecord is written by the delegate

volatile bool minuteFlag = false;
volatile bool isOnTheMoveActivated = false;
volatile uint32_t lastOnTheMoveActivationTimestamp = 0;
volatile bool updateOnTheMoveTimestampFlag = false;
static uint32_t lastSampleTime = 0;
static bool isImeiInitialized = false;
static bool isCcidInitialized = false;
static char cachedImei[16];
static char cachedCcid[32];
static uint8_t lastResetCause = 0;
static bool isGpsInitialized = false;
static bool isRtcInitialized = false;
static bool isDeviceInitialized = false;
static bool isOnTheMoveInitialized = false;
static int64_t rtcEpochDelta = 0; // set in setNow() and used in getGpsFix() for correcting time in loop
static bool socketsLeftOpen = false;
static bool lastOnTheMoveActivationStatus = false;
static BatteryState batteryState = BatteryState::BatteryNormal;
static uint32_t lastUdpSendTime = 0;
static bool areSensorsOn = false;
static uint32_t lastSensorBlink = 0;
static bool blinkOn = false;

void setup();
void loop();

uint32_t getNow();
void setNow(uint32_t now);
void handleBootUpCommands();
void initRtc();
void accelerometerInt1Handler();
void rtcAlarmHandler();
void initRtcTimer();
void resetRtcTimerEvents();
void initSleep();
bool initGps();
void initOnTheMove();
void systemSleep();
void setGpsActive(bool on);
void setAccelerometerTempSensorActive(bool on);
bool isCurrentTimeOfDayWithin(uint32_t daySecondsFrom, uint32_t daySecondsTo);
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt);
bool getGpsFix();
uint16_t getBatteryVoltage();
uint8_t getPackedBatteryVoltage();
void onConfigReset(void);
void setupBOD33();
bool setModemActive(bool on);
void initModem();

static void printCpuResetCause(Stream& stream);
static void printBootUpMessage(Stream& stream);

/**
 * Setup method
 */
void setup()
{
    lastResetCause = PM->RCAUSE.reg;

    // In case of reset (this is probably unnecessary)
    sodaq_wdt_disable();

    // Setup the BOD33
    setupBOD33();

    sodaq_wdt_enable(WDT_PERIOD_8X);
    sodaq_wdt_reset();

    CONSOLE_STREAM.begin(CONSOLE_BAUD);
    if ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM) {
        DEBUG_STREAM.begin(DEBUG_BAUD);
    }

    sodaq_wdt_safe_delay(STARTUP_DELAY);
    printBootUpMessage(CONSOLE_STREAM);

    initSleep();
    initRtc();

    Wire.begin();
    sodaq_wdt_safe_delay(200);

    // init params
    params.setConfigResetCallback(onConfigReset);
    params.read();

    // if allowed, init early for faster initial fix
    isGpsInitialized = initGps();

    // disable the watchdog only for the boot menu
    sodaq_wdt_disable();
    handleBootUpCommands();
    sodaq_wdt_enable(WDT_PERIOD_8X);

    // init and turn on the sensors
    initBMESensor();
    pinMode(BOOST_CONVERTER_PIN, OUTPUT);
    PM_SENSOR_STREAM.begin(pm3015.getDefaultBaudrate());
    pm3015.init(PM_SENSOR_STREAM);

    // make sure the debug option is honored
    if (params.getIsDebugOn() && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) {
        DEBUG_STREAM.begin(DEBUG_BAUD);
    }

    // if gps initialization failed earlier, retry
    if (!isGpsInitialized) {
        isGpsInitialized = initGps();
    }

    initModem();

    accelerometer.disableMagnetometer();
    pinMode(MAG_INT, OUTPUT);
    digitalWrite(MAG_INT, LOW);
    if (params.getAccelerationPercentage() > 0) {
        initOnTheMove();

        isOnTheMoveInitialized = true;
    }

    initRtcTimer();

    batteryState = getBatteryState();

    isDeviceInitialized = true;

    consolePrintln("** Boot-up completed successfully!");
    sodaq_wdt_reset();

    // disable the USB if not needed for debugging
    if (!params.getIsDebugOn() || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        consolePrintln("The USB is going to be disabled now.");
        debugPrintln("The USB is going to be disabled now.");

        SerialUSB.flush();
        sodaq_wdt_safe_delay(3000);
        SerialUSB.end();
        USBDevice.detach();
        USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
    }

    // disable the debug stream if it is not disabled by the above
    if (!params.getIsDebugOn() && ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        DEBUG_STREAM.flush();
        DEBUG_STREAM.end();
    }

    // disable the console stream if it is not disabled by the above, 
    // and only if it is different than the debug stream
    if ((long)&CONSOLE_STREAM != (long)&SerialUSB && ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM)) {
        CONSOLE_STREAM.flush();
        CONSOLE_STREAM.end();
    }

    setSensorsActive(false);

    debugPrintln("Getting initial gps fix...");
    getGpsFix();
}

/**
 * Disconnects passed pin
 */
void resetPin(uint8_t pin)
{
    PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg = (uint8_t)(0);
    PORT->Group[g_APinDescription[pin].ulPort].DIRCLR.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin);
    PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin);
}

/**
 * Initializes the BME sensor
 */
void initBMESensor()
{
    if (!bme.begin(0x76)) {
        debugPrintln("ERROR: Could not find BME680 sensor!");
        return;
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    uint32_t endTime = bme.beginReading();
    if (endTime == 0) {
        debugPrintln("Failed to begin reading");
    }
}

/**
 * Gets IMEI from modem
 */
const char* getImei()
{
    if (!isImeiInitialized) {
        char tmpBuffer[16];
        if (nbiot.getIMEI(tmpBuffer, sizeof(tmpBuffer))) {
            strncpy(cachedImei, tmpBuffer, sizeof(cachedImei));
            isImeiInitialized = true;
        }
    }

    return cachedImei;
}

/**
 * Prints UDP message contents
 */
void printUdpMessage(const uint8_t* message, size_t size)
{
    debugPrintln("Sending udp message: ");

    for (size_t i = 0; i < size; i++) {
        debugPrint(static_cast<char>(NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(message[i]))));
        debugPrint(static_cast<char>(NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(message[i]))));
    }

    debugPrintln();
}

/**
 * Sets the state of the sensors
 */
void setSensorsActive(bool active) 
{
    if (active) {
        debugPrintln("Sensors on");
        digitalWrite(BOOST_CONVERTER_PIN, HIGH);
        sodaq_wdt_safe_delay(200);
        PM_SENSOR_STREAM.begin(pm3015.getDefaultBaudrate());
        pm3015.openMeasurement();
    }
    else {
        debugPrintln("Sensors off");
        pm3015.closeMeasurement();
        sodaq_wdt_safe_delay(200);
        PM_SENSOR_STREAM.end();
        resetPin(PM_SENSOR_RX_PIN);
        resetPin(PM_SENSOR_TX_PIN);

        digitalWrite(BOOST_CONVERTER_PIN, LOW);
    }
    
    areSensorsOn = active;
}

/** 
 * Gets data from sensors and puts them in the passed structure
 */
void getSensorData(SensorData& sensorData)
{
    sensorData.time = getNow();

    // get gps data
    if (getGpsFix()) {
        debugPrintln("Gps fix successful");
        sensorData.latitude = pendingGpsRecord.latitude;
        sensorData.longitude = pendingGpsRecord.longitude;
        sensorData.horizontalAccuracy = pendingGpsRecord.horizontalAccuracy;
        sensorData.verticalAccuracy = pendingGpsRecord.verticalAccuracy;
    }
    else {
        debugPrintln("ERROR: GPS fix failed!");
    }

    // get particulate matter sensor data
    uint32_t pm1_0;
    uint32_t pm2_5;
    uint32_t pm10;
    if (pm3015.readMeasurements(&pm1_0, &pm2_5, &pm10)) {
        sensorData.pm1_0 = pm1_0;
        sensorData.pm2_5 = pm2_5;
        sensorData.pm10 = pm10;
    }
    else {
        debugPrintln("ERROR: failed to read measurements from pm3015!");
    }

    // get temp/hum/pressure/voc data
    if (bme.endReading()) {
        sensorData.temp = floorf(bme.temperature * 10);
        sensorData.humidity = static_cast<uint8_t>(bme.humidity);
        sensorData.pressure = static_cast<uint32_t>(bme.pressure);
        sensorData.voc = (bme.gas_resistance / 1000.0);

        // start next measurement
        uint32_t endTime = bme.beginReading();
        if (endTime == 0) {
            debugPrintln("ERROR: failed to begin BME680 reading!");
        }
    }
    else {
        debugPrintln("ERROR: failed to complete BME680 reading!");
    }

    // get battery voltage data
    sensorData.voltage = getPackedBatteryVoltage();

    printSensorData(sensorData);
}

/**
 * Prints contents of sensors readings
 */
void printSensorData(SensorData& sensorData)
{
    debugPrintln("Sample data: ");
    
    debugPrint("Time: ");
    debugPrintln(sensorData.time);
    
    debugPrint("Latitude: ");
    debugPrintln(sensorData.latitude);
    debugPrint("Longitude: ");
    debugPrintln(sensorData.longitude);
    debugPrint("Horizontal accuracy: ");
    debugPrintln(sensorData.horizontalAccuracy);
    debugPrint("Vertical accuracy: ");
    debugPrintln(sensorData.verticalAccuracy);
    
    debugPrint("PM1.0: ");
    debugPrintln(sensorData.pm1_0);
    debugPrint("PM2.5: ");
    debugPrintln(sensorData.pm2_5);
    debugPrint("PM10:");
    debugPrintln(sensorData.pm10);
    
    debugPrint("Temperature: ");
    debugPrintln(sensorData.temp);
    debugPrint("Pressure: ");
    debugPrintln(sensorData.pressure);
    debugPrint("VOC: ");
    debugPrintln(sensorData.voc);
    debugPrint("Humidity: ");
    debugPrintln(sensorData.humidity);
    
    debugPrint("Voltage: ");
    debugPrintln(sensorData.voltage);
}

/**
 * Sends a udp message (tries to turn modem on if not already on)
 */
bool sendUdpMessage(UdpMessage messageToSend)
{
    if (!setModemActive(true)) {
        debugPrintln("ERROR: could not connect to network!");
        return false;
    }

    // check if there were any sockets left open
    // and close them
    if (socketsLeftOpen) {
        socketsLeftOpen = false;

        for (uint8_t i = 0; i < 7; i++) {
            nbiot.closeSocket(i);
        }
    }

    printUdpMessage(messageToSend.raw, sizeof(messageToSend.raw));

    bool success = false;
    uint16_t sourcePortNumber = 1666;
    uint8_t socketId = nbiot.createSocket(sourcePortNumber);
    if (socketId != -1) {
        uint8_t* udpMsg = messageToSend.raw;
        size_t udpMsgSize = sizeof(messageToSend.raw);
        size_t bytesSent = nbiot.socketSend(socketId, params.getTargetIP(), params.getTargetPort(), udpMsg, udpMsgSize);
        
        if (bytesSent == udpMsgSize) {
            success = true;
        }

        if (!nbiot.closeSocket(socketId)) {
            socketsLeftOpen = true;
        }
    }
    else {
        debugPrintln("ERROR: failed to open socket!");
        success = false;
    }

    setModemActive(false);

    return success;
}

/**
 * Main program loop
 */
void loop()
{
    if (sodaq_wdt_flag) {
        // Reset watchdog
        sodaq_wdt_reset();
        sodaq_wdt_flag = false;
    }

    if (updateOnTheMoveTimestampFlag) {
        lastOnTheMoveActivationTimestamp = getNow();
        updateOnTheMoveTimestampFlag = false;
    }

    if (isOnTheMoveActivated) {
        if (batteryState == BatteryState::BatteryTooLow) {
            setLedColor(RED);
            sodaq_wdt_safe_delay(1000);

            isOnTheMoveActivated = false;
            lastOnTheMoveActivationStatus = false;
            setLedColor(NONE);
            setSensorsActive(false);
        }
        else {
            if (lastOnTheMoveActivationStatus == false) {
                setLedColor(BLUE);

                setSensorsActive(true);
                lastOnTheMoveActivationStatus = true;
            }

            uint32_t now = getNow();
            if (now - lastSampleTime > params.getSensorsSampleInterval()) {
                // sample sensors
                SensorData sensorData;
                sensorData.init();
                getSensorData(sensorData);

                // add acquired data to udp message
                pendingSamplesUdpMessage.samples[currentSampleRecordIndex] = sensorData;
                currentSampleRecordIndex++;
                if (currentSampleRecordIndex >= SAMPLE_COUNT) {
                    debugPrintln("Sending udp message...");
                    pendingSamplesUdpMessage.imei = atoll(getImei());
                    if (sendUdpMessage(pendingSamplesUdpMessage)) {
                        setLedColor(GREEN);
                    }
                    else {
                        setLedColor(RED);
                    }
                    lastUdpSendTime = millis();

                    memset(pendingSamplesUdpMessage.raw, 0, sizeof(pendingSamplesUdpMessage.raw));
                    currentSampleRecordIndex = 0;
                }

                lastSampleTime = now;
            }

            if (now - lastOnTheMoveActivationTimestamp > params.getOnTheMoveTimeout() * 60) {
                isOnTheMoveActivated = false;
                lastOnTheMoveActivationStatus = false;

                setLedColor(NONE);
                setSensorsActive(false);
            }
        }
    }

    uint32_t now = millis();
    if (lastUdpSendTime > 0) {
        if (now - lastUdpSendTime > UDP_SEND_LED_DURATION) {
            setLedColor(NONE);
            lastUdpSendTime = 0;
        }
    }
    else if (areSensorsOn) {
        if (now - lastSensorBlink > SENSORS_ON_LED_BLINK_INTERVAL) {
            if (blinkOn) {
                setLedColor(NONE);
            }
            else {
                if (batteryState == BatteryState::LowBatteryWarning) {
                    setLedColor(RED);
                }
                else {
                    setLedColor(BLUE);
                }
            }

            blinkOn = !blinkOn;
            lastSensorBlink = now;
        }
        else {
            // Handled in accelerometer timeout (check a few lines above) 
        }
    }

    if (minuteFlag) {
        batteryState = getBatteryState();

        timer.update(); // handle scheduled events

        minuteFlag = false;
    }

    systemSleep();
}

/**
 * Returns the battery state
 */
BatteryState getBatteryState()
{
    uint16_t batteryVoltage = getBatteryVoltage();

    debugPrint("Checking for low battery, reported battery voltage: ");
    debugPrintln(batteryVoltage);

    if (batteryVoltage > BATTERY_LOW_WARNING_THRESHOLD) {
        return BatteryState::BatteryNormal;
    }
    else if (batteryVoltage > BATTERY_TOO_LOW_THRESHOLD) {
        return BatteryState::LowBatteryWarning;
    }
    else {
        return BatteryState::BatteryTooLow;
    }
}
/**
 * Initializes modem
 */
void initModem()
{
    debugPrintln("Initializing Modem...");

    if (params.getIsDebugOn()) {
        nbiot.setDiag(DEBUG_STREAM);
    }

    pinMode(MODEM_RESET, OUTPUT);
    digitalWrite(MODEM_RESET, HIGH);

    MODEM_STREAM.begin(nbiot.getDefaultBaudrate());
    nbiot.init(MODEM_STREAM, MODEM_ON_OFF_PIN, MODEM_TX_ENABLE_PIN, -1, params.getCid());

    setModemActive(false); // make sure it is off
}

/**
 * Turns modem on and tries to connect to network
 */
bool setModemActive(bool on)
{
    sodaq_wdt_reset();

    // TODO Fix reset mechanism
    if (on) {
        if (!nbiot.isConnected()) {
            if (!nbiot.connect(params._apn, params._cdp, params._forceOperator)) {
                nbiot.off();
                sodaq_wdt_safe_delay(450);
                nbiot.on();
                sodaq_wdt_safe_delay(450);

                // try just one last time
                return nbiot.connect(params._apn, params._cdp, params._forceOperator);
            }
            return true;
        }
        else {
            return true;
        }
    }
    else {
        // do not turn off modem, just return
        return true;
    }
}

/**
 * Initializes the CPU sleep mode.
 */
void initSleep()
{
    // Set the sleep mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

/**
 * Returns the battery voltage
 */
uint16_t getBatteryVoltage()
{
    return (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BAT_VOLT));
}

/**
 * Returns the battery voltage minus 3 volts.
 */
uint8_t getPackedBatteryVoltage()
{
    uint16_t voltage = getBatteryVoltage();
    voltage = (voltage - 3000) / 10;

    return (voltage > 255 ? 255 : (uint8_t)voltage);
}

/**
 * Initializes the GPS and leaves it on if succesful.
 * Returns true if successful.
*/
bool initGps()
{
    pinMode(GPS_ENABLE, OUTPUT);
    pinMode(GPS_TIMEPULSE, INPUT);

    // attempt to turn on and communicate with the GPS
    digitalWrite(GPS_ENABLE, HIGH);
    ublox.enable();
    ublox.flush();

    uint32_t startTime = getNow();
    bool found = false;
    while (!found && (getNow() - startTime <= GPS_COMM_CHECK_TIMEOUT)) {
        sodaq_wdt_reset();

        found = ublox.exists();
    }

    // check for success
    if (found) {
        setGpsActive(true); // properly turn on before returning

        return true;
    }

    consolePrintln("*** GPS not found!");
    debugPrintln("*** GPS not found!");

    // turn off before returning in case of failure
    setGpsActive(false);

    return false;
}

/**
* Initializes the on-the-move functionality (interrupt on acceleration).
*/
void initOnTheMove()
{
    pinMode(ACCEL_INT1, INPUT);
    attachInterrupt(ACCEL_INT1, accelerometerInt1Handler, CHANGE);

    // Configure EIC to use GCLK1 which uses XOSC32K, XOSC32K is already running in standby
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
        GCLK_CLKCTRL_GEN_GCLK1 |
        GCLK_CLKCTRL_CLKEN;

    accelerometer.enableAccelerometer(
        Sodaq_LSM303AGR::LowPowerMode,
        Sodaq_LSM303AGR::HrNormalLowPower10Hz,
        Sodaq_LSM303AGR::XYZ,
        Sodaq_LSM303AGR::Scale8g,
        true);
    sodaq_wdt_safe_delay(100);

    accelerometer.enableInterrupt1(
        Sodaq_LSM303AGR::XHigh | Sodaq_LSM303AGR::XLow | Sodaq_LSM303AGR::YHigh | Sodaq_LSM303AGR::YLow | Sodaq_LSM303AGR::ZHigh | Sodaq_LSM303AGR::ZLow,
        params.getAccelerationPercentage() * 8.0 / 100.0,
        params.getAccelerationDuration(),
        Sodaq_LSM303AGR::MovementRecognition);
}

/**
 * Powers down all devices and puts the system to deep sleep.
 */
void systemSleep()
{
    MODEM_STREAM.flush();

    setModemActive(false);
    setGpsActive(false); // explicitly disable after resetting the pins

    // go to sleep, unless USB is used for debugging
    if (!params.getIsDebugOn() || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
        noInterrupts();
        if (!(sodaq_wdt_flag || minuteFlag || isOnTheMoveActivated)) {
            interrupts();

            __WFI(); // SAMD sleep
        }
        interrupts();
    }
}

/**
 * Setup BOD33
 *
 * Setup BOD the way we want it.
 *  - BOD33USERLEVEL = 0x30 - shutdown at 3.07 Volt
 *  - BOD33_EN = [X] //Enabled
 *  - BOD33_ACTION = 0x01
 *  - BOD33_HYST = [X] //Enabled
 */
void setupBOD33()
{
    SYSCTRL->BOD33.bit.LEVEL = 0x07;    // ~1.7 Volt
    SYSCTRL->BOD33.bit.ACTION = 1;      // Go to Reset
    SYSCTRL->BOD33.bit.ENABLE = 1;      // Enabled
    SYSCTRL->BOD33.bit.HYST = 1;        // Hysteresis on
    while (!SYSCTRL->PCLKSR.bit.B33SRDY) {
        /* Wait for synchronization */
    }
}

/**
 * Returns the current datetime (seconds since unix epoch).
 */
uint32_t getNow()
{
    return rtc.getEpoch();
}

/**
 * Sets the RTC epoch and "rtcEpochDelta".
 */
void setNow(uint32_t newEpoch)
{
    uint32_t currentEpoch = getNow();

    debugPrint("Setting RTC from ");
    debugPrint(currentEpoch);
    debugPrint(" to ");
    debugPrintln(newEpoch);

    rtcEpochDelta = newEpoch - currentEpoch;
    rtc.setEpoch(newEpoch);

    timer.adjust(currentEpoch, newEpoch);

    isRtcInitialized = true;
}

/**
 * Shows and handles the boot up commands.
 */
void handleBootUpCommands()
{
    do {
        showBootMenu(CONSOLE_STREAM);
    } while (!params.checkConfig(CONSOLE_STREAM));

    params.showConfig(&CONSOLE_STREAM);
    params.commit();
}

/**
 * Initializes the RTC.
 */
void initRtc()
{
    rtc.begin();

    // Schedule the wakeup interrupt for every minute
    // Alarm is triggered 1 cycle after match
    rtc.setAlarmSeconds(59);
    rtc.enableAlarm(RTCZero::MATCH_SS); // alarm every minute

    // Attach handler
    rtc.attachInterrupt(rtcAlarmHandler);

    // This sets it to 2000-01-01
    rtc.setEpoch(0);
}

/**
 * Runs every minute by the rtc alarm.
*/
void rtcAlarmHandler()
{
    minuteFlag = true;
}

/**
 * Runs every time acceleration is over the limits
 * set by the user (if enabled).
*/
void accelerometerInt1Handler()
{
    if (digitalRead(ACCEL_INT1)) {
        // debugPrintln("On-the-move is triggered");

        isOnTheMoveActivated = true;
        updateOnTheMoveTimestampFlag = true;
    }
}

/**
 * Initializes the RTC Timer and schedules the default events.
 */
void initRtcTimer()
{
    timer.setNowCallback(getNow); // set how to get the current time
    timer.allowMultipleEvents();

    resetRtcTimerEvents();
}

/**
 * Clears the RTC Timer events and schedules the default events.
 */
void resetRtcTimerEvents()
{
    timer.clearAllEvents();
}

/**
 * Returns true if the current rtc time is within the given times of day (in seconds).
*/
bool isCurrentTimeOfDayWithin(uint32_t daySecondsFrom, uint32_t daySecondsTo)
{
    uint32_t daySecondsCurrent = rtc.getHours() * 60 * 60 + rtc.getMinutes() * 60;

    return (daySecondsCurrent >= daySecondsFrom && daySecondsCurrent < daySecondsTo);
}

/**
 *  GPS delegate method
 *  Checks validity of data, adds valid points to the points list, syncs the RTC
 */
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt)
{
    sodaq_wdt_reset();

    if (!isGpsInitialized) {
        debugPrintln("delegateNavPvt exiting because GPS is not initialized.");

        return;
    }

    // note: db_printf gets enabled/disabled according to the "DEBUG" define (ublox.cpp)
    ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
        NavPvt->year, NavPvt->month, NavPvt->day,
        NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
        NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

    // sync the RTC time
    if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
        uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

        // check if there is an actual offset before setting the RTC
        if (abs((int64_t)getNow() - (int64_t)epoch) > MAX_RTC_EPOCH_OFFSET) {
            setNow(epoch);
        }
    }

    // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
    if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
        pendingGpsRecord.latitude = NavPvt->lat;
        pendingGpsRecord.longitude = NavPvt->lon;
        pendingGpsRecord.satelitesCount = NavPvt->numSV;
        pendingGpsRecord.horizontalAccuracy = NavPvt->hAcc;
        pendingGpsRecord.verticalAccuracy = NavPvt->vAcc;

        isPendingGpsRecordNew = true;
    }
}

/**
 * Tries to get a GPS fix and saves the record
 */
bool getGpsFix()
{
    debugPrintln("Starting getGpsFix...");

    if (!isGpsInitialized) {
        debugPrintln("GPS is not initialized, exiting...");

        return false;
    }

    bool isSuccessful = false;
    setGpsActive(true);

    pendingGpsRecord.satelitesCount = 0; // reset satellites to use them as a quality metric in the loop
    uint32_t startTime = getNow();
    while ((getNow() - startTime <= params.getGpsFixTimeout())
        && (pendingGpsRecord.satelitesCount < params.getGpsMinSatelliteCount()))
    {
        sodaq_wdt_reset();
        uint16_t bytes = ublox.available();

        if (bytes) {
            rtcEpochDelta = 0;
            isPendingGpsRecordNew = false;
            ublox.GetPeriodic(bytes); // calls the delegate method for passing results

            startTime += rtcEpochDelta; // just in case the clock was changed (by the delegate in ublox.GetPeriodic)

            // isPendingReportDataRecordNew guarantees at least a 3d fix or GNSS + dead reckoning combined
            // and is good enough to keep, but the while loop should keep trying until timeout or sat count larger than set
            if (isPendingGpsRecordNew) {
                isSuccessful = true;
            }
        }
    }

    setGpsActive(false); // turn off gps as soon as it is not needed

    return isSuccessful;
}

/**
 * Turns the GPS on or off.
 */
void setGpsActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        digitalWrite(GPS_ENABLE, HIGH);

        ublox.enable();
        ublox.flush();

        sodaq_wdt_safe_delay(100);

        PortConfigurationDDC pcd;

        uint8_t maxRetries = 6;
        int8_t retriesLeft;

        retriesLeft = maxRetries;
        while (!ublox.getPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
            debugPrintln("Retrying ublox.getPortConfigurationDDC(&pcd)...");
            sodaq_wdt_safe_delay(15);
        }
        if (retriesLeft == -1) {
            debugPrintln("ublox.getPortConfigurationDDC(&pcd) failed!");

            return;
        }

        pcd.outProtoMask = 1; // Disable NMEA
        retriesLeft = maxRetries;
        while (!ublox.setPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
            debugPrintln("Retrying ublox.setPortConfigurationDDC(&pcd)...");
            sodaq_wdt_safe_delay(15);
        }
        if (retriesLeft == -1) {
            debugPrintln("ublox.setPortConfigurationDDC(&pcd) failed!");

            return;
        }

        ublox.CfgMsg(UBX_NAV_PVT, 1); // Navigation Position Velocity TimeSolution
        ublox.funcNavPvt = delegateNavPvt;
    }
    else {
        ublox.disable();
        digitalWrite(GPS_ENABLE, LOW);
    }
}

/**
 * Initializes the accelerometer or puts it in power-down mode
 * for the purpose of reading its temperature delta.
*/
void setAccelerometerTempSensorActive(bool on)
{
    // if on-the-move is initialized then the accelerometer is enabled anyway
    if (isOnTheMoveInitialized) {
        return;
    }

    if (on) {
        accelerometer.enableAccelerometer(Sodaq_LSM303AGR::LowPowerMode, Sodaq_LSM303AGR::HrNormalLowPower100Hz, Sodaq_LSM303AGR::XYZ, Sodaq_LSM303AGR::Scale2g, true);
        sodaq_wdt_safe_delay(30); // should be enough for initilization and 2 measurement periods
    }
    else {
        accelerometer.disableAccelerometer();
    }
}

/**
 * Prints the cause of the last reset to the given stream.
 *
 * It uses the PM->RCAUSE register to detect the cause of the last reset.
 */
static void printCpuResetCause(Stream& stream)
{
    stream.print("CPU reset by");

    if (PM->RCAUSE.bit.SYST) {
        stream.print(" Software");
    }

    // Syntax error due to #define WDT in CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/samd21j18a.h
    // if (PM->RCAUSE.bit.WDT) {
    if ((PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0) {
        stream.print(" Watchdog");
    }

    if (PM->RCAUSE.bit.EXT) {
        stream.print(" External");
    }

    if (PM->RCAUSE.bit.BOD33) {
        stream.print(" BOD33");
    }

    if (PM->RCAUSE.bit.BOD12) {
        stream.print(" BOD12");
    }

    if (PM->RCAUSE.bit.POR) {
        stream.print(" Power On Reset");
    }

    stream.print(" [");
    stream.print(PM->RCAUSE.reg);
    stream.println("]");
}

/**
 * Prints a boot-up message that includes project name, version,
 * and Cpu reset cause.
 */
static void printBootUpMessage(Stream& stream)
{
    stream.println("** " PROJECT_NAME " - " VERSION " **");
    stream.println();

    initModem();
    nbiot.on();

    // give modem time to initialize and continue anyway
    uint32_t start = millis();
    while (!nbiot.isAlive() && (millis() - start < 5000)) {
        sodaq_wdt_safe_delay(50);
    }

    stream.print("IMEI: ");
    stream.println(getImei());

    stream.print(" -> ");
    printCpuResetCause(stream);

    stream.println();
}

/**
 * Callback from Config.reset(), used to override default values.
 */
void onConfigReset(void)
{
#ifdef DEFAULT_APN
    // fail if the defined string is larger than what is expected in the config
    BUILD_BUG_ON(sizeof(DEFAULT_APN) > sizeof(params._apn));

    strcpy(params._apn, DEFAULT_APN);
#endif

#ifdef DEFAULT_CDP
    // fail if the defined string is larger than what is expected in the config
    BUILD_BUG_ON(sizeof(DEFAULT_CDP) > sizeof(params._cdp));

    strcpy(params._cdp, DEFAULT_CDP);
#endif

#ifdef DEFAULT_FORCE_OPERATOR
    // fail if the defined string is larger than what is expected in the config
    BUILD_BUG_ON(sizeof(DEFAULT_FORCE_OPERATOR) > sizeof(params._forceOperator));

    strcpy(params._forceOperator, DEFAULT_FORCE_OPERATOR);
#endif

#ifdef DEFAULT_BAND
    params._band = DEFAULT_BAND;
#endif

#ifdef DEFAULT_TARGET_IP
    // fail if the defined string is larger than what is expected in the config
    BUILD_BUG_ON(sizeof(DEFAULT_TARGET_IP) > sizeof(params._targetIP));

    strcpy(params._targetIP, DEFAULT_TARGET_IP);
#endif

#ifdef DEFAULT_TARGET_PORT
    params._targetPort = DEFAULT_TARGET_PORT;
#endif

#ifdef DEBUG
    params._isDebugOn = true;
#endif

#ifdef DEFAULT_SENSORS_SAMPLE_INTERVAL
    params._sensorsSampleInterval = DEFAULT_SENSORS_SAMPLE_INTERVAL;
#endif
}
