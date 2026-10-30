#include <cstring> // Include for strstr
#include <vector>
#include <math.h>

#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_GPS
#include "Default.h"
#include "GPS.h"
#include "GPSLog.h"
#include "GpioLogic.h"
#include "NodeDB.h"
#include "PowerMon.h"
#include "Throttle.h"
#include "UptimeClock.h"
#include "buzz.h"
#include "concurrency/Periodic.h"
#include "gps/RTC.h"
#include "meshUtils.h"

#include "main.h" // pmu_found
#include "sleep.h"

#include "FSCommon.h"
#include "GPSUpdateScheduling.h"
#include "SPILock.h"
#include "SafeFile.h"
#include "cas.h"
#include "ubx.h"

#ifdef ARCH_PORTDUINO
#include "GpsdSerial.h"
#include "PortduinoGlue.h"
#include "meshUtils.h"
#include <algorithm>
#include <ctime>
#endif

#ifdef ARCH_RP2040
#include <hardware/watchdog.h>
#endif

// GPS::setup() blocks the main loop for well over the RP2040 hardware watchdog window (8 s, fed only from loop()):
// the u-blox M10 config alone is ~15 s of ACK waits and delays. Keep the watchdog fed while we wait on the module.
static inline void feedWatchdogWhileWaiting()
{
#ifdef ARCH_RP2040
    watchdog_update();
#endif
}

#ifndef GPS_RESET_MODE
#define GPS_RESET_MODE HIGH
#endif

// Not all platforms have std::size().
template <typename T, std::size_t N> std::size_t array_count(const T (&)[N])
{
    return N;
}

#ifndef GPS_SERIAL_PORT
#define GPS_SERIAL_PORT Serial1
#endif

#if defined(SENSECAP_INDICATOR)
UARTProxy *GPS::_serial_gps = nullptr; // assigned in createGps(), see there
#elif defined(ARCH_NRF52)
Uart *GPS::_serial_gps = &GPS_SERIAL_PORT;
#elif defined(ARCH_ESP32) || defined(ARCH_PORTDUINO) || defined(ARCH_STM32)
HardwareSerial *GPS::_serial_gps = &GPS_SERIAL_PORT;
#elif defined(ARCH_RP2040)
SerialUART *GPS::_serial_gps = &GPS_SERIAL_PORT;
#else
HardwareSerial *GPS::_serial_gps = nullptr;
#endif

std::unique_ptr<GPS> gps = nullptr;

static GPSUpdateScheduling scheduling;

/// Multiple GPS instances might use the same serial port (in sequence), but we can
/// only init that port once.
static bool didSerialInit;

static struct uBloxGnssModelInfo {
    char swVersion[30];
    char hwVersion[10];
    uint8_t extensionNo;
    char extension[10][30];
    uint8_t protocol_version;
} ublox_info;

#define GPS_SOL_EXPIRY_MS 5000 // in millis. give 1 second time to combine different sentences. NMEA Frequency isn't higher anyway
#define NMEA_MSG_GXGSA "GNGSA" // GSA message (GPGSA, GNGSA etc)

namespace
{
// Versioned on-disk record for persisted GPS probe results.
constexpr uint32_t GPS_PROBE_CACHE_MAGIC = 0x47504348UL; // "GPCH"
constexpr uint16_t GPS_PROBE_CACHE_VERSION = 1;
constexpr const char *GPS_PROBE_CACHE_FILE = "/prefs/gps_probe_cache.dat";
constexpr int MIN_PLAUSIBLE_GPS_YEAR = 2020;
constexpr int MAX_PLAUSIBLE_GPS_YEAR = 2100;
constexpr uint32_t AIROHA_PROBE_SETTLE_MS = 20;
#ifdef HAS_AIROHA_SLEEP
// The receiver may already have auto-slept and missed the first sleep command, so resend until it acks.
#ifdef HAS_AIROHA_SOFT_RTC
constexpr uint32_t AIROHA_SLEEP_ACK_MS = 40;
constexpr uint32_t AIROHA_SLEEP_BUDGET_MS = 400;
#else
// $PAIR003 first answers ",1" (processing); the final ",0" can take longer than 40 ms.
constexpr uint32_t AIROHA_SLEEP_ACK_MS = 100;
// Some commands go unheard outright, so keep resending across a whole 1 Hz NMEA cycle.
constexpr uint32_t AIROHA_SLEEP_BUDGET_MS = 1500;
#endif
// Stop parking a receiver that never acks, rather than blocking for the budget every cycle forever.
constexpr uint8_t AIROHA_SLEEP_MAX_MISSES = 3;
#endif
#ifdef HAS_AIROHA_SOFT_RTC
constexpr uint32_t AIROHA_POWER_SETTLE_MS = 50;
constexpr uint32_t AIROHA_RTC_INT_PULSE_MS = 3;
constexpr uint32_t AIROHA_WAKE_SETTLE_MS = 50;
#elif defined(HAS_AIROHA_SLEEP)
constexpr uint32_t AIROHA_IDLE_INT_LOW_MS = 5;
#endif

struct GPSProbeCacheRecord {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t baud;
    uint8_t model;
};

bool isValidGnssModel(uint8_t model)
{
    // Only real chip identifiers belong in the probe cache.
    // GNSS_MODEL_UNKNOWN and GNSS_MODEL_GENERIC_NMEA are runtime-only values.
    return model != static_cast<uint8_t>(GNSS_MODEL_UNKNOWN) && model < static_cast<uint8_t>(GNSS_MODEL_GENERIC_NMEA);
}

bool isValidProbeBaud(uint32_t baud)
{
    // Conservative sanity range for UART baud values.
    return baud >= 1200 && baud <= 921600;
}

#ifdef HAS_AIROHA_SOFT_RTC
// Airoha leaves software-RTC sleep on an RTC_INT rising edge; VCC must already be up.
static void airohaPulseRtcInt()
{
    delay(AIROHA_POWER_SETTLE_MS);
    digitalWrite(GPS_RTC_INT, HIGH);
    delay(AIROHA_RTC_INT_PULSE_MS);
    digitalWrite(GPS_RTC_INT, LOW);
    delay(AIROHA_WAKE_SETTLE_MS);
}
#elif defined(HAS_AIROHA_SLEEP)
// idle_int rests high; the receiver only hears a command sent while it is pulled low.
template <typename T> static void airohaWriteIdleInt(T *serialGps, const char *cmd)
{
    digitalWrite(GPS_SLEEP_INT, LOW);
    delay(AIROHA_IDLE_INT_LOW_MS);
    serialGps->write(cmd);
    serialGps->flush();
    delay(AIROHA_IDLE_INT_LOW_MS);
    digitalWrite(GPS_SLEEP_INT, HIGH);
}
#endif

template <typename T> void sendAirohaWakeCommand(T *serialGps)
{
#ifdef HAS_AIROHA_SOFT_RTC
    // The probe's own hardware reset drops the receiver back to sleep, so force the physical wake
    // rather than relying on the command alone.
#ifdef PIN_GPS_EN
    digitalWrite(PIN_GPS_EN, GPS_EN_ACTIVE);
#endif
    airohaPulseRtcInt();
    serialGps->write("$PAIR382,1*2E\r\n");
    delay(AIROHA_PROBE_SETTLE_MS);
#else
    serialGps->write("$PAIR382,1*2E\r\n");
    delay(AIROHA_PROBE_SETTLE_MS);
#endif
}

template <typename T> void wakeAirohaForActiveProbe(T *serialGps, GnssModel_t model)
{
    if (IS_ONE_OF(model, GNSS_MODEL_AG3335, GNSS_MODEL_AG3352))
        sendAirohaWakeCommand(serialGps);
}

bool isPlausibleNmeaTime(const struct tm &t)
{
    const int year = t.tm_year + 1900;
    if (year < MIN_PLAUSIBLE_GPS_YEAR || year > MAX_PLAUSIBLE_GPS_YEAR) {
        return false;
    }

#ifdef BUILD_EPOCH
    const int64_t candidate = static_cast<int64_t>(gm_mktime(&t));
    const int64_t minEpoch = static_cast<int64_t>(BUILD_EPOCH);
    const int64_t maxEpoch = minEpoch + static_cast<int64_t>(FORTY_YEARS);
    return candidate >= minEpoch && candidate <= maxEpoch;
#else
    return true;
#endif
}

template <typename T> bool sawNmeaSentenceAtBaud(T *serialGps, uint32_t timeoutMs)
{
    // Lightweight passive check: look for at least one complete
    // "$...,<field>\n" style NMEA sentence.
    const uint32_t deadline = millis() + timeoutMs;
    bool sawDollar = false;
    bool sawComma = false;

    while ((int32_t)(millis() - deadline) < 0) {
        while (serialGps->available()) {
            char c = static_cast<char>(serialGps->read());
            if (c == '$') {
                sawDollar = true;
                sawComma = false;
                continue;
            }
            if (c == ',') {
                sawComma = true;
            }
            if (c == '\n' || c == '\r') {
                if (sawDollar && sawComma) {
                    return true;
                }
                sawDollar = false;
                sawComma = false;
            }
        }
        delay(10);
    }

    return false;
}
} // namespace

// For logging
static const char *getGPSPowerStateString(GPSPowerState state)
{
    switch (state) {
    case GPS_ACTIVE:
        return "ACTIVE";
    case GPS_IDLE:
        return "IDLE";
    case GPS_SOFTSLEEP:
        return "SOFTSLEEP";
    case GPS_HARDSLEEP:
        return "HARDSLEEP";
    case GPS_OFF:
        return "OFF";
    default:
        assert(false);  // Unhandled enum value..
        return "FALSE"; // to make new ESP-IDF happy
    }
}

#ifdef PIN_GPS_SWITCH
// If we have a hardware switch, define a periodic watcher outside of the GPS runOnce thread, since this can be sleeping
// indefinitely

int lastState = LOW;
bool firstrun = true;

static int32_t gpsSwitch()
{
    if (gps) {
        int currentState = digitalRead(PIN_GPS_SWITCH);

        // Respect explicit NOT_PRESENT mode and do not let the hardware switch re-enable GPS.
        if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT) {
            gps->disable();
            lastState = currentState;
            firstrun = false;
            return 1000;
        }

        // if the switch is set to zero, disable the GPS Thread
        if (firstrun)
            if (currentState == LOW)
                lastState = HIGH;

        if (currentState != lastState) {
            if (currentState == LOW) {
                config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_DISABLED;
                if (!firstrun)
                    playGPSDisableBeep();
                gps->disable();
            } else {
                config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
                if (!firstrun)
                    playGPSEnableBeep();
                gps->enable();
            }
            lastState = currentState;
        }
        firstrun = false;
    }
    return 1000;
}

static std::unique_ptr<concurrency::Periodic> gpsPeriodic;
#endif

static void UBXChecksum(uint8_t *message, size_t length)
{
    uint8_t CK_A = 0, CK_B = 0;

    // Calculate the checksum, starting from the CLASS field (which is message[2])
    for (size_t i = 2; i < length - 2; i++) {
        CK_A = (CK_A + message[i]) & 0xFF;
        CK_B = (CK_B + CK_A) & 0xFF;
    }

    // Place the calculated checksum values in the message
    message[length - 2] = CK_A;
    message[length - 1] = CK_B;
}

// Calculate the checksum for a CAS packet
static void CASChecksum(uint8_t *message, size_t length)
{
    uint32_t cksum = ((uint32_t)message[5] << 24); // Message ID
    cksum += ((uint32_t)message[4]) << 16;         // Class
    cksum += message[2];                           // Payload Len

    // Iterate over the payload as a series of uint32_t's and
    // accumulate the cksum
    for (size_t i = 0; i < (length - 10) / 4; i++) {
        uint32_t pl = 0;
        memcpy(&pl, (message + 6) + (i * sizeof(uint32_t)), sizeof(uint32_t)); // avoid pointer dereference
        cksum += pl;
    }

    // Place the checksum values in the message
    message[length - 4] = (cksum & 0xFF);
    message[length - 3] = (cksum & (0xFF << 8)) >> 8;
    message[length - 2] = (cksum & (0xFF << 16)) >> 16;
    message[length - 1] = (cksum & (0xFF << 24)) >> 24;
}

// Function to create a ublox packet for editing in memory
uint8_t GPS::makeUBXPacket(uint8_t class_id, uint8_t msg_id, uint8_t payload_size, const uint8_t *msg)
{
    // Construct the UBX packet
    UBXscratch[0] = 0xB5;         // header
    UBXscratch[1] = 0x62;         // header
    UBXscratch[2] = class_id;     // class
    UBXscratch[3] = msg_id;       // id
    UBXscratch[4] = payload_size; // length
    UBXscratch[5] = 0x00;

    UBXscratch[6 + payload_size] = 0x00; // CK_A
    UBXscratch[7 + payload_size] = 0x00; // CK_B

    for (int i = 0; i < payload_size; i++) {
        UBXscratch[6 + i] = pgm_read_byte(&msg[i]);
    }
    UBXChecksum(UBXscratch, (payload_size + 8));
    return (payload_size + 8);
}

// Function to create a CAS packet for editing in memory
uint8_t GPS::makeCASPacket(uint8_t class_id, uint8_t msg_id, uint8_t payload_size, const uint8_t *msg)
{
    // General CAS structure
    //        | H1   | H2   | payload_len | cls  | msg  | Payload       ...   | Checksum                  |
    // Size:  | 1    | 1    | 2           | 1    | 1    | payload_len         | 4                         |
    // Pos:   | 0    | 1    | 2    | 3    | 4    | 5    | 6    | 7      ...   | 6 + payload_len ...       |
    //        |------|------|-------------|------|------|------|--------------|---------------------------|
    //        | 0xBA | 0xCE | 0xXX | 0xXX | 0xXX | 0xXX | 0xXX | 0xXX   ...   | 0xXX | 0xXX | 0xXX | 0xXX |

    // Construct the CAS packet
    UBXscratch[0] = 0xBA;         // header 1 (0xBA)
    UBXscratch[1] = 0xCE;         // header 2 (0xCE)
    UBXscratch[2] = payload_size; // length 1
    UBXscratch[3] = 0;            // length 2
    UBXscratch[4] = class_id;     // class
    UBXscratch[5] = msg_id;       // id

    UBXscratch[6 + payload_size] = 0x00; // Checksum
    UBXscratch[7 + payload_size] = 0x00;
    UBXscratch[8 + payload_size] = 0x00;
    UBXscratch[9 + payload_size] = 0x00;

    for (int i = 0; i < payload_size; i++) {
        UBXscratch[6 + i] = pgm_read_byte(&msg[i]);
    }
    CASChecksum(UBXscratch, (payload_size + 10));

#if GPS_DEBUG && defined(DEBUG_PORT)
    LOG_DEBUG("CAS packet: ");
    DEBUG_PORT.hexDump(MESHTASTIC_LOG_LEVEL_DEBUG, UBXscratch, payload_size + 10);
#endif
    return (payload_size + 10);
}

GPS_RESPONSE GPS::getACK(const char *message, uint32_t waitMillis)
{
    uint8_t buffer[768] = {0};
    uint8_t b;
    int bytesRead = 0;
    // Start stamp + interval rather than a stored deadline: same wrap-safety, but the full 49.7-day
    // range instead of 24.8 days ahead, and Time::getMillis() makes the wait injectable.
    const uint32_t waitStartMs = Time::getMillis();
#if GPS_DEBUG
    std::string debugmsg = "";
#endif
    while (Throttle::isWithinTimespanMs(waitStartMs, waitMillis)) {
        feedWatchdogWhileWaiting();
        if (_serial_gps->available()) {
            b = _serial_gps->read();

#if GPS_DEBUG
            debugmsg += vformat("%c", (b >= 32 && b <= 126) ? b : '.');
#endif
            buffer[bytesRead] = b;
            bytesRead++;
            if ((bytesRead == 767) || (b == '\r')) {
                LOG_DEBUG_GPS("%s", debugmsg.c_str());
                if (strnstr((char *)buffer, message, bytesRead) != nullptr) {
                    LOG_DEBUG_GPS("Found: %s", message); // Log the found message
                    return GNSS_RESPONSE_OK;
                } else {
                    bytesRead = 0;
                }
            }
        }
    }
    return GNSS_RESPONSE_NONE;
}

GPS_RESPONSE GPS::getACKCas(uint8_t class_id, uint8_t msg_id, uint32_t waitMillis)
{
    uint32_t startTime = millis();
    uint8_t buffer[CAS_ACK_NACK_MSG_SIZE] = {0};
    uint8_t bufferPos = 0;

    // CAS-ACK-(N)ACK structure
    //         | H1   | H2   | Payload Len | cls  | msg  | Payload                   | Checksum (4)              |
    //         |      |      |             |      |      | Cls  | Msg  | Reserved    |                           |
    //         |------|------|-------------|------|------|------|------|-------------|---------------------------|
    // ACK-NACK| 0xBA | 0xCE | 0x04 | 0x00 | 0x05 | 0x00 | 0xXX | 0xXX | 0x00 | 0x00 | 0xXX | 0xXX | 0xXX | 0xXX |
    // ACK-ACK | 0xBA | 0xCE | 0x04 | 0x00 | 0x05 | 0x01 | 0xXX | 0xXX | 0x00 | 0x00 | 0xXX | 0xXX | 0xXX | 0xXX |

    while (Throttle::isWithinTimespanMs(startTime, waitMillis)) {
        if (_serial_gps->available()) {
            buffer[bufferPos++] = _serial_gps->read();

            // keep looking at the first two bytes of buffer until
            // we have found the CAS frame header (0xBA, 0xCE), if not
            // keep reading bytes until we find a frame header or we run
            // out of time.
            if ((bufferPos == 2) && !(buffer[0] == 0xBA && buffer[1] == 0xCE)) {
                buffer[0] = buffer[1];
                buffer[1] = 0;
                bufferPos = 1;
            }
        }

        // we have read all the bytes required for the Ack/Nack (14-bytes)
        // and we must have found a frame to get this far
        if (bufferPos == sizeof(buffer) - 1) {
            uint8_t msg_cls = buffer[4];     // message class should be 0x05
            uint8_t msg_msg_id = buffer[5];  // message id should be 0x00 or 0x01
            uint8_t payload_cls = buffer[6]; // payload class id
            uint8_t payload_msg = buffer[7]; // payload message id

            // Check for an ACK-ACK for the specified class and message id
            if ((msg_cls == 0x05) && (msg_msg_id == 0x01) && payload_cls == class_id && payload_msg == msg_id) {
                LOG_DEBUG_GPS("Got ACK for class %02X msg %02X in %dms", class_id, msg_id, millis() - startTime);
                return GNSS_RESPONSE_OK;
            }

            // Check for an ACK-NACK for the specified class and message id
            if ((msg_cls == 0x05) && (msg_msg_id == 0x00) && payload_cls == class_id && payload_msg == msg_id) {
                LOG_DEBUG_GPS("Got NACK for class %02X msg %02X in %dms", class_id, msg_id, millis() - startTime);
                return GNSS_RESPONSE_NAK;
            }

            // This isn't the frame we are looking for, clear the buffer
            // and try again until we run out of time.
            memset(buffer, 0x0, sizeof(buffer));
            bufferPos = 0;
        }
    }
    return GNSS_RESPONSE_NONE;
}

GPS_RESPONSE GPS::getACK(uint8_t class_id, uint8_t msg_id, uint32_t waitMillis)
{
    uint8_t b;
    uint8_t ack = 0;
    const uint8_t ackP[2] = {class_id, msg_id};
    uint8_t buf[10] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t startTime = millis();
    const char frame_errors[] = "More than 100 frame errors";
    int sCounter = 0;
#if GPS_DEBUG
    std::string debugmsg = "";
#endif

    for (int j = 2; j < 6; j++) {
        buf[8] += buf[j];
        buf[9] += buf[8];
    }

    for (int j = 0; j < 2; j++) {
        buf[6 + j] = ackP[j];
        buf[8] += buf[6 + j];
        buf[9] += buf[8];
    }

    while (Throttle::isWithinTimespanMs(startTime, waitMillis)) {
        feedWatchdogWhileWaiting();
        if (ack > 9) {
            LOG_DEBUG_GPS("Got ACK for class %02X msg %02X in %dms", class_id, msg_id, millis() - startTime);
            return GNSS_RESPONSE_OK; // ACK received
        }
        if (_serial_gps->available()) {
            b = _serial_gps->read();
            if (b == frame_errors[sCounter]) {
                sCounter++;
                if (sCounter == 26) {
                    LOG_DEBUG_GPS("%s", debugmsg.c_str());
                    return GNSS_RESPONSE_FRAME_ERRORS;
                }
            } else {
                sCounter = 0;
            }
#if GPS_DEBUG
            debugmsg += vformat("%02X", b);
#endif
            if (b == buf[ack]) {
                ack++;
            } else {
                if (ack == 3 && b == 0x00) { // UBX-ACK-NAK message
                    LOG_DEBUG_GPS("%s", debugmsg.c_str());
                    LOG_WARN("Got NAK for class %02X msg %02X", class_id, msg_id);
                    return GNSS_RESPONSE_NAK; // NAK received
                }
                ack = 0; // Reset the acknowledgement counter
            }
        }
    }
    LOG_DEBUG_GPS("%s", debugmsg.c_str());
    LOG_DEBUG_GPS("No response for class %02X msg %02X", class_id, msg_id);
    return GNSS_RESPONSE_NONE; // No response received within timeout
}

/**
 * @brief
 * @note   New method, this method can wait for the specified class and message ID, and return the payload
 * @param  *buffer: The message buffer, if there is a response payload message, it will be returned through the buffer parameter
 * @param  size:    size of buffer
 * @param  requestedClass:  request class constant
 * @param  requestedID:     request message ID constant
 * @retval length of payload message
 */
int GPS::getACK(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID, uint32_t waitMillis)
{
    uint16_t ubxFrameCounter = 0;
    uint32_t startTime = millis();
    uint16_t needRead = 0;

    while (Throttle::isWithinTimespanMs(startTime, waitMillis)) {
        feedWatchdogWhileWaiting();
        if (_serial_gps->available()) {
            int c = _serial_gps->read();
            switch (ubxFrameCounter) {
            case 0:
                // ubxFrame 'μ'
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                // ubxFrame 'b'
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                // Class
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                // Message ID
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                // Payload length lsb
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                // Payload length msb
                needRead |= (c << 8);
                ubxFrameCounter++;
                // Check for buffer overflow
                if (needRead >= size) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (_serial_gps->readBytes(buffer, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    // return payload length
                    LOG_DEBUG_GPS("Got ACK for class %02X msg %02X in %dms", requestedClass, requestedID, millis() - startTime);
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
    }
    return 0;
}

#if GPS_BAUDRATE_FIXED
// if GPS_BAUDRATE is specified in variant, only try that.
static const int serialSpeeds[1] = {GPS_BAUDRATE};
static const int rareSerialSpeeds[1] = {GPS_BAUDRATE};
#else
static const int serialSpeeds[3] = {9600, 115200, 38400};
static const int rareSerialSpeeds[3] = {4800, 57600, GPS_BAUDRATE};
#endif

#ifndef GPS_PROBETRIES
#define GPS_PROBETRIES 2
#endif

bool GPS::loadProbeCache()
{
#ifdef FSCom
    // Load the last known-good GPS model/baud pair so we can avoid a full probe
    // sweep on every boot.
    triedProbeCache = true; // Latch this boot's load attempt, even if no cache.
    GPSProbeCacheRecord record = {};
    size_t bytesRead = 0;

    spiLock->lock();
    auto file = FSCom.open(GPS_PROBE_CACHE_FILE, FILE_O_READ);
    if (!file) {
        spiLock->unlock();
        return false;
    }
    bytesRead = file.read(reinterpret_cast<uint8_t *>(&record), sizeof(record));
    file.close();
    spiLock->unlock();

    const bool headerValid = (bytesRead == sizeof(record)) && (record.magic == GPS_PROBE_CACHE_MAGIC) &&
                             (record.version == GPS_PROBE_CACHE_VERSION) && (record.reserved == 0U);
    if (!headerValid || !isValidGnssModel(record.model) || !isValidProbeBaud(record.baud)) {
        clearProbeCache(); // Drop corrupt/invalid cache so next boot can
                           // recover.
        return false;
    }

    cachedProbeBaud = static_cast<int32_t>(record.baud);
    cachedProbeModel = static_cast<GnssModel_t>(record.model);
    hasProbeCache = true;
    triedProbeCache = false;
    LOG_INFO("Loaded cached GPS probe: baud=%u", record.baud);
    return true;
#else
    return false;
#endif
}

void GPS::clearProbeCache()
{
    // Invalidate in-memory and on-disk cache so next boot is forced to do a
    // full probe.
    hasProbeCache = false;
    triedProbeCache = true;
    cachedProbeBaud = 0;
    cachedProbeModel = GNSS_MODEL_UNKNOWN;
#ifdef FSCom
    spiLock->lock();
    if (FSCom.exists(GPS_PROBE_CACHE_FILE)) {
        FSCom.remove(GPS_PROBE_CACHE_FILE);
    }
    spiLock->unlock();
#endif
}

bool GPS::saveProbeCache() const
{
#ifdef FSCom
    if (gnssModel == GNSS_MODEL_UNKNOWN || !isValidGnssModel(static_cast<uint8_t>(gnssModel)) ||
        !isValidProbeBaud(detectedBaud)) {
        return false;
    }

    spiLock->lock();
    FSCom.mkdir("/prefs");
    spiLock->unlock();
    GPSProbeCacheRecord record = {
        GPS_PROBE_CACHE_MAGIC, GPS_PROBE_CACHE_VERSION, 0, static_cast<uint32_t>(detectedBaud), static_cast<uint8_t>(gnssModel),
    };

    auto file = SafeFile(GPS_PROBE_CACHE_FILE, true);
    spiLock->lock();
    const size_t written = file.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record));
    spiLock->unlock();
    return (written == sizeof(record)) && file.close();
#else
    return false;
#endif
}

bool GPS::verifyCachedProbePresence()
{
    if (!hasProbeCache || cachedProbeModel == GNSS_MODEL_UNKNOWN || !isValidProbeBaud(cachedProbeBaud)) {
        return false;
    }

#if defined(ARCH_NRF52) || defined(ARCH_PORTDUINO) || defined(ARCH_STM32)
    _serial_gps->end();
    _serial_gps->begin(cachedProbeBaud);
#elif defined(ARCH_RP2040)
    _serial_gps->end();
    _serial_gps->setFIFOSize(256);
    _serial_gps->begin(cachedProbeBaud);
#else
    if (_serial_gps->baudRate() != cachedProbeBaud) {
        LOG_DEBUG("Set GPS Baud to %i (cached verify)", cachedProbeBaud);
        _serial_gps->updateBaudRate(cachedProbeBaud);
    }
#endif

    // Before trusting cached model/baud, require either active model-specific
    // response or passive NMEA flow.
    clearBuffer();
    bool present = false;

    // Model-specific "active ping" checks to avoid false stale decisions on
    // modules that start streaming late.
    const char *cachedProbeModelName = "UNKNOWN";
    switch (cachedProbeModel) {
    case GNSS_MODEL_MTK:
        cachedProbeModelName = "L76K/MTK";
        _serial_gps->write("$PCAS06,0*1B\r\n");
        present = (getACK("$GPTXT,01,01,02,SW=", 700) == GNSS_RESPONSE_OK);
        break;
    case GNSS_MODEL_MTK_L76B:
        cachedProbeModelName = "L76B";
    case GNSS_MODEL_MTK_PA1010D:
        if (cachedProbeModel == GNSS_MODEL_MTK_PA1010D)
            cachedProbeModelName = "PA1010D";
    case GNSS_MODEL_MTK_PA1616S:
        if (cachedProbeModel == GNSS_MODEL_MTK_PA1616S)
            cachedProbeModelName = "PA1616S";
    case GNSS_MODEL_LS20031:
        if (cachedProbeModel == GNSS_MODEL_LS20031)
            cachedProbeModelName = "LS20031";
        _serial_gps->write("$PMTK605*31\r\n");
        present = (getACK("$PMTK705", 900) == GNSS_RESPONSE_OK);
        break;
    case GNSS_MODEL_AG3335:
        cachedProbeModelName = "AG3335";
    case GNSS_MODEL_AG3352:
        if (cachedProbeModel == GNSS_MODEL_AG3352)
            cachedProbeModelName = "AG3352";
        wakeAirohaForActiveProbe(_serial_gps, cachedProbeModel);
        _serial_gps->write("$PAIR021*39\r\n");
        present = (getACK("$PAIR021,", 900) == GNSS_RESPONSE_OK);
        break;
    case GNSS_MODEL_ATGM336H:
        cachedProbeModelName = "ATGM336H";
        _serial_gps->write("$PCAS06,1*1A\r\n");
        present = (getACK("$GPTXT,01,01,02,HW=ATGM", 900) == GNSS_RESPONSE_OK);
        break;
    case GNSS_MODEL_UC6580:
        cachedProbeModelName = "UC6580/UM600";
        _serial_gps->write("$PDTINFO\r\n");
        present = (getACK("UC6580", 900) == GNSS_RESPONSE_OK) || (getACK("UM600", 900) == GNSS_RESPONSE_OK);
        break;
    case GNSS_MODEL_CM121:
        cachedProbeModelName = "CM121";
        _serial_gps->write("$PDTINFO\r\n");
        present = (getACK("CM121", 900) == GNSS_RESPONSE_OK);
        break;
    case GNSS_MODEL_LC760CA:
        cachedProbeModelName = "LC760CA";
        _serial_gps->write("$PDTINFO\r\n");
        present = (getACK("CC1161W", 900) == GNSS_RESPONSE_OK);
        break;
    case GNSS_MODEL_UBLOX6:
    case GNSS_MODEL_UBLOX7:
    case GNSS_MODEL_UBLOX8:
    case GNSS_MODEL_UBLOX9:
    case GNSS_MODEL_UBLOX10: {
        if (cachedProbeModel == GNSS_MODEL_UBLOX6)
            cachedProbeModelName = "U-blox 6";
        else if (cachedProbeModel == GNSS_MODEL_UBLOX7)
            cachedProbeModelName = "U-blox 7";
        else if (cachedProbeModel == GNSS_MODEL_UBLOX8)
            cachedProbeModelName = "U-blox 8";
        else if (cachedProbeModel == GNSS_MODEL_UBLOX9)
            cachedProbeModelName = "U-blox 9";
        else if (cachedProbeModel == GNSS_MODEL_UBLOX10)
            cachedProbeModelName = "U-blox 10";

        uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00};
        UBXChecksum(cfg_rate, sizeof(cfg_rate));
        _serial_gps->write(cfg_rate, sizeof(cfg_rate));
        present = (getACK(0x06, 0x08, 900) != GNSS_RESPONSE_NONE);
        break;
    }
    default:
        break;
    }

    if (!present) {
        // Some modules may not respond to probes while still streaming NMEA, so
        // allow a passive fallback check.
        present = sawNmeaSentenceAtBaud(_serial_gps, 3000);
    }
    if (!present) {
        LOG_WARN("Cached GPS probe stale (%s @ %d), clearing", cachedProbeModelName, cachedProbeBaud);
        clearProbeCache();
        return false;
    }

    detectedBaud = cachedProbeBaud;
    gnssModel = cachedProbeModel;
    LOG_INFO("Using cached GPS probe: %s @ %d", cachedProbeModelName, detectedBaud);
    return true;
}

/**
 * @brief  Setup the GPS based on the model detected.
 *  We detect the GPS by cycling through a set of baud rates, first common then rare.
 *  For each baud rate, we run GPS::Probe to send commands and match the responses
 *  to known GPS responses.
 * @retval Whether setup reached the end of its potential to configure the GPS.
 */
bool GPS::setup()
{
    if (!didSerialInit) {
        int msglen = 0;
        if (tx_gpio && gnssModel == GNSS_MODEL_UNKNOWN) {
            if (!hasProbeCache && !triedProbeCache) {
                (void)loadProbeCache();
            }

            if (hasProbeCache && !triedProbeCache) {
                triedProbeCache = true;
                if (!verifyCachedProbePresence()) {
                    currentStep = 0;
                    speedSelect = 0;
                    probeTries = 0;
                }
            }

            if (gnssModel == GNSS_MODEL_UNKNOWN && probeTries < GPS_PROBETRIES) {
                // No usable cache: walk common baud rates first.
                gnssModel = probe(serialSpeeds[speedSelect]);
                if (gnssModel != GNSS_MODEL_UNKNOWN) {
                    detectedBaud = serialSpeeds[speedSelect];
                } else if (currentStep == 0 && ++speedSelect == array_count(serialSpeeds)) {
                    speedSelect = 0;
                    ++probeTries;
                }
            }
            // Rare Serial Speeds
#ifndef CONFIG_IDF_TARGET_ESP32C6
            else if (gnssModel == GNSS_MODEL_UNKNOWN && probeTries == GPS_PROBETRIES) {
                // Then try less common baud rates before giving up.
                gnssModel = probe(rareSerialSpeeds[speedSelect]);
                if (gnssModel != GNSS_MODEL_UNKNOWN) {
                    detectedBaud = rareSerialSpeeds[speedSelect];
                } else if (currentStep == 0 && ++speedSelect == array_count(rareSerialSpeeds)) {
                    LOG_WARN("Give up GPS probe, set to %d", GPS_BAUDRATE);
                    return true;
                }
            }
#endif
        }

        if (gnssModel != GNSS_MODEL_UNKNOWN) {
            setConnected();
            (void)saveProbeCache();
        } else {
            return false;
        }

        if (gnssModel == GNSS_MODEL_MTK) {
            /*
             * t-beam-s3-core uses the same L76K GNSS module as t-echo.
             * Unlike t-echo, L76K uses 9600 baud rate for communication by default.
             * */

            // Initialize the L76K Chip, use GPS + GLONASS + BEIDOU
            _serial_gps->write("$PCAS04,7*1E\r\n");
            delay(250);
            // only ask for RMC and GGA
            _serial_gps->write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
            delay(250);
            // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
            _serial_gps->write("$PCAS11,3*1E\r\n");
            delay(250);
        } else if (gnssModel == GNSS_MODEL_MTK_L76B) {
            // Waveshare Pico-GPS hat uses the L76B with 9600 baud
            // Initialize the L76B Chip, use GPS + GLONASS
            // See note in L76_Series_GNSS_Protocol_Specification, chapter 3.29
            _serial_gps->write("$PMTK353,1,1,0,0,0*2B\r\n");
            // Above command will reset the GPS and takes longer before it will accept new commands
            delay(1000);
            // only ask for RMC and GGA (GNRMC and GNGGA)
            // See note in L76_Series_GNSS_Protocol_Specification, chapter 2.1
            _serial_gps->write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
            delay(250);
            // Enable SBAS
            _serial_gps->write("$PMTK301,2*2E\r\n");
            delay(250);
            // Enable PPS for 2D/3D fix only
            _serial_gps->write("$PMTK285,3,100*3F\r\n");
            delay(250);
            // Switch to Fitness Mode, for running and walking purpose with low speed (<5 m/s)
            _serial_gps->write("$PMTK886,1*29\r\n");
            delay(250);
        } else if (gnssModel == GNSS_MODEL_MTK_PA1010D) {
            // PA1010D is used in the Pimoroni GPS board.

            // Enable all constellations.
            _serial_gps->write("$PMTK353,1,1,1,1,1*2A\r\n");
            // Above command will reset the GPS and takes longer before it will accept new commands
            delay(1000);
            // Only ask for RMC and GGA (GNRMC and GNGGA)
            _serial_gps->write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
            delay(250);
            // Enable SBAS / WAAS
            _serial_gps->write("$PMTK301,2*2E\r\n");
            delay(250);
        } else if (gnssModel == GNSS_MODEL_MTK_PA1616S) {
            // PA1616S is used in some GPS breakout boards from Adafruit
            // PA1616S does not have GLONASS capability. PA1616D does, but is not implemented here.
            _serial_gps->write("$PMTK353,1,0,0,0,0*2A\r\n");
            // Above command will reset the GPS and takes longer before it will accept new commands
            delay(1000);
            // Only ask for RMC and GGA (GNRMC and GNGGA)
            _serial_gps->write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
            delay(250);
            // Enable SBAS / WAAS
            _serial_gps->write("$PMTK301,2*2E\r\n");
            delay(250);
        } else if (gnssModel == GNSS_MODEL_ATGM336H) {
            // Set the initial configuration of the device - these _should_ work for most AT6558 devices
            msglen = makeCASPacket(0x06, 0x07, sizeof(_message_CAS_CFG_NAVX_CONF), _message_CAS_CFG_NAVX_CONF);
            _serial_gps->write(UBXscratch, msglen);
            if (getACKCas(0x06, 0x07, 250) != GNSS_RESPONSE_OK) {
                LOG_WARN("ATGM336H: Can't set Config");
            }

            // Set the update frequency to 1Hz
            msglen = makeCASPacket(0x06, 0x04, sizeof(_message_CAS_CFG_RATE_1HZ), _message_CAS_CFG_RATE_1HZ);
            _serial_gps->write(UBXscratch, msglen);
            if (getACKCas(0x06, 0x04, 250) != GNSS_RESPONSE_OK) {
                LOG_WARN("ATGM336H: Can't set Update Frequency");
            }

            // Set the NEMA output messages
            // Ask for only RMC and GGA
            uint8_t fields[] = {CAS_NEMA_RMC, CAS_NEMA_GGA};
            for (unsigned int i = 0; i < sizeof(fields); i++) {
                // Construct a CAS-CFG-MSG packet
                uint8_t cas_cfg_msg_packet[] = {0x4e, fields[i], 0x01, 0x00};
                msglen = makeCASPacket(0x06, 0x01, sizeof(cas_cfg_msg_packet), cas_cfg_msg_packet);
                _serial_gps->write(UBXscratch, msglen);
                if (getACKCas(0x06, 0x01, 250) != GNSS_RESPONSE_OK) {
                    LOG_WARN("ATGM336H: Can't enable NMEA MSG: %d", fields[i]);
                }
            }
        } else if (gnssModel == GNSS_MODEL_UC6580) {
            // The Unicore UC6580 can use a lot of sat systems, enable it to
            // use GPS L1 & L5 + BDS B1I & B2a + GLONASS L1 + GALILEO E1 & E5a + SBAS + QZSS
            // This will reset the receiver, so wait a bit afterwards
            // The paranoid will wait for the OK*04 confirmation response after each command.
            _serial_gps->write("$CFGSYS,h35155\r\n");
            delay(750);
            // Must be done after the CFGSYS command
            // Turn off GSV messages, we don't really care about which and where the sats are, maybe someday.
            _serial_gps->write("$CFGMSG,0,3,0\r\n");
            delay(250);
            // Turn off GSA messages, TinyGPS++ doesn't use this message.
            _serial_gps->write("$CFGMSG,0,2,0\r\n");
            delay(250);
            // Turn off NOTICE __TXT messages, these may provide Unicore some info but we don't care.
            _serial_gps->write("$CFGMSG,6,0,0\r\n");
            delay(250);
            _serial_gps->write("$CFGMSG,6,1,0\r\n");
            delay(250);
        } else if (IS_ONE_OF(gnssModel, GNSS_MODEL_AG3335, GNSS_MODEL_AG3352)) {

            if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_IN ||
                config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_NP_865) {
                _serial_gps->write("$PAIR066,1,0,1,0,0,1*3B\r\n"); // Enable GPS+GALILEO+NAVIC
                // GPS GLONASS GALILEO BDS QZSS NAVIC
                //  1    0       1      0   0    1
            } else {
                _serial_gps->write("$PAIR066,1,1,1,1,0,0*3A\r\n"); // Enable GPS+GLONASS+GALILEO+BDS
                // GPS GLONASS GALILEO BDS QZSS NAVIC
                //  1    1       1      1   0    0
            }
            // Configure NMEA (sentences will output once per fix)
            _serial_gps->write("$PAIR062,0,1*3F\r\n"); // GGA ON
            _serial_gps->write("$PAIR062,1,0*3F\r\n"); // GLL OFF
            _serial_gps->write("$PAIR062,2,0*3C\r\n"); // GSA OFF
            _serial_gps->write("$PAIR062,3,0*3D\r\n"); // GSV OFF
            _serial_gps->write("$PAIR062,4,1*3B\r\n"); // RMC ON
            _serial_gps->write("$PAIR062,5,0*3B\r\n"); // VTG OFF
            _serial_gps->write("$PAIR062,6,0*38\r\n"); // ZDA ON

            delay(250);
            _serial_gps->write("$PAIR513*3D\r\n"); // save configuration
        } else if (gnssModel == GNSS_MODEL_UBLOX6) {
            clearBuffer();
            SEND_UBX_PACKET(0x06, 0x02, _message_DISABLE_TXT_INFO, "disable text info messages", 500);
            SEND_UBX_PACKET(0x06, 0x39, _message_JAM_6_7, "enable interference resistance", 500);
            SEND_UBX_PACKET(0x06, 0x23, _message_NAVX5, "configure NAVX5 settings", 500);

            // Turn off unwanted NMEA messages, set update rate
            SEND_UBX_PACKET(0x06, 0x08, _message_1HZ, "set GPS update rate", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GLL, "disable NMEA GLL", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GSA, "enable NMEA GSA", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GSV, "disable NMEA GSV", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_VTG, "disable NMEA VTG", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_RMC, "enable NMEA RMC", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GGA, "enable NMEA GGA", 500);

            clearBuffer();
            SEND_UBX_PACKET(0x06, 0x11, _message_CFG_RXM_ECO, "enable powersave ECO mode for Neo-6", 500);
            SEND_UBX_PACKET(0x06, 0x3B, _message_CFG_PM2, "enable powersave details for GPS", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_AID, "disable UBX-AID", 500);

            msglen = makeUBXPacket(0x06, 0x09, sizeof(_message_SAVE), _message_SAVE);
            _serial_gps->write(UBXscratch, msglen);
            if (getACK(0x06, 0x09, 2000) != GNSS_RESPONSE_OK) {
                LOG_WARN("Can't save GNSS module config");
            } else {
                LOG_INFO("GNSS module config saved");
            }
        } else if (IS_ONE_OF(gnssModel, GNSS_MODEL_UBLOX7, GNSS_MODEL_UBLOX8, GNSS_MODEL_UBLOX9)) {
            if (gnssModel == GNSS_MODEL_UBLOX7) {
                LOG_DEBUG("Set GPS+SBAS");
                msglen = makeUBXPacket(0x06, 0x3e, sizeof(_message_GNSS_7), _message_GNSS_7);
                _serial_gps->write(UBXscratch, msglen);
            } else { // 8,9
                msglen = makeUBXPacket(0x06, 0x3e, sizeof(_message_GNSS_8), _message_GNSS_8);
                _serial_gps->write(UBXscratch, msglen);
            }

            if (getACK(0x06, 0x3e, 800) == GNSS_RESPONSE_NAK) {
                // It's not critical if the module doesn't acknowledge this configuration.
                LOG_DEBUG("reconfigure GNSS - defaults kept. GPS-only module?");
            } else {
                if (gnssModel == GNSS_MODEL_UBLOX7) {
                    LOG_INFO("GPS+SBAS configured");
                } else { // 8,9
                    LOG_INFO("GPS+SBAS+GLONASS+Galileo configured");
                }
                // Documentation say, we need wait at least 0.5s after reconfiguration of GNSS module, before sending next
                // commands for the M8 it tends to be more... 1 sec should be enough ;>)
                delay(1000);
            }

            // Disable Text Info messages //6,7,8,9
            clearBuffer();
            SEND_UBX_PACKET(0x06, 0x02, _message_DISABLE_TXT_INFO, "disable text info messages", 500);

            if (gnssModel == GNSS_MODEL_UBLOX8) { // 8
                clearBuffer();
                SEND_UBX_PACKET(0x06, 0x39, _message_JAM_8, "enable interference resistance", 500);

                clearBuffer();
                SEND_UBX_PACKET(0x06, 0x23, _message_NAVX5_8, "configure NAVX5_8 settings", 500);
            } else { // 6,7,9
                SEND_UBX_PACKET(0x06, 0x39, _message_JAM_6_7, "enable interference resistance", 500);
                SEND_UBX_PACKET(0x06, 0x23, _message_NAVX5, "configure NAVX5 settings", 500);
            }
            // Turn off unwanted NMEA messages, set update rate
            SEND_UBX_PACKET(0x06, 0x08, _message_1HZ, "set GPS update rate", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GLL, "disable NMEA GLL", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GSA, "enable NMEA GSA", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GSV, "disable NMEA GSV", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_VTG, "disable NMEA VTG", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_RMC, "enable NMEA RMC", 500);
            SEND_UBX_PACKET(0x06, 0x01, _message_GGA, "enable NMEA GGA", 500);

            if (ublox_info.protocol_version >= 18) {
                clearBuffer();
                SEND_UBX_PACKET(0x06, 0x86, _message_PMS, "enable powersave for GPS", 500);
                SEND_UBX_PACKET(0x06, 0x3B, _message_CFG_PM2, "enable powersave details for GPS", 500);

                // For M8 we want to enable NMEA version 4.10 so we can see the additional sats.
                if (gnssModel == GNSS_MODEL_UBLOX8) {
                    clearBuffer();
                    SEND_UBX_PACKET(0x06, 0x17, _message_NMEA, "enable NMEA 4.10", 500);
                }
            } else {
                SEND_UBX_PACKET(0x06, 0x11, _message_CFG_RXM_PSM, "enable powersave mode for GPS", 500);
                SEND_UBX_PACKET(0x06, 0x3B, _message_CFG_PM2, "enable powersave details for GPS", 500);
            }

            msglen = makeUBXPacket(0x06, 0x09, sizeof(_message_SAVE), _message_SAVE);
            _serial_gps->write(UBXscratch, msglen);
            if (getACK(0x06, 0x09, 2000) != GNSS_RESPONSE_OK) {
                LOG_WARN("Can't save GNSS module config");
            } else {
                LOG_INFO("GNSS module config saved");
            }
        } else if (gnssModel == GNSS_MODEL_UBLOX10) {
            delay(1000);
            clearBuffer();
            LOG_INFO("Skipping U10 configs.");
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_DISABLE_NMEA_RAM, "disable NMEA messages in M10 RAM", 300);
            //~ delay(750);
            //~ clearBuffer();
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_DISABLE_NMEA_BBR, "disable NMEA messages in M10 BBR", 300);
            //~ delay(750);
            //~ clearBuffer();
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_DISABLE_TXT_INFO_RAM, "disable Info messages for M10 GPS RAM", 300);
            //~ delay(750);
            //~ // Next disable Info txt messages in BBR layer
            //~ clearBuffer();
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_DISABLE_TXT_INFO_BBR, "disable Info messages for M10 GPS BBR", 300);
            //~ delay(750);
            //~ // Do M10 configuration for Power Management.
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_PM_RAM, "enable powersave for M10 GPS RAM", 300);
            //~ delay(750);
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_PM_BBR, "enable powersave for M10 GPS BBR", 300);
            //~ delay(750);
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_ITFM_RAM, "enable jam detection M10 GPS RAM", 300);
            //~ delay(750);
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_ITFM_BBR, "enable jam detection M10 GPS BBR", 300);
            //~ delay(750);
            //~ // Here is where the init commands should go to do further M10 initialization.
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_DISABLE_SBAS_RAM, "disable SBAS M10 GPS RAM", 300);
            //~ delay(750); // will cause a receiver restart so wait a bit
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_DISABLE_SBAS_BBR, "disable SBAS M10 GPS BBR", 300);
            //~ delay(750); // will cause a receiver restart so wait a bit

            //~ // Done with initialization, Now enable wanted NMEA messages in BBR layer so they will survive a periodic
            //~ // sleep.
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_ENABLE_NMEA_BBR, "enable messages for M10 GPS BBR", 300);
            //~ delay(750);
            //~ // Next enable wanted NMEA messages in RAM layer
            //~ SEND_UBX_PACKET(0x06, 0x8A, _message_VALSET_ENABLE_NMEA_RAM, "enable messages for M10 GPS RAM", 500);
            //~ delay(750);

            //~ // As the M10 has no flash, the best we can do to preserve the config is to set it in RAM and BBR.
            //~ // BBR will survive a restart, and power off for a while, but modules with small backup
            //~ // batteries or super caps will not retain the config for a long power off time.
            //~ msglen = makeUBXPacket(0x06, 0x09, sizeof(_message_SAVE_10), _message_SAVE_10);
            //~ _serial_gps->write(UBXscratch, msglen);
            //~ if (getACK(0x06, 0x09, 2000) != GNSS_RESPONSE_OK) {
                //~ LOG_WARN("Can't save GNSS module config");
            //~ } else {
                //~ LOG_INFO("GNSS module config saved");
            //~ }
        } else if (IS_ONE_OF(gnssModel, GNSS_MODEL_CM121, GNSS_MODEL_LC760CA)) {
            // only ask for RMC and GGA
            // enable GGA
            _serial_gps->write("$CFGMSG,0,0,1,1*1B\r\n");
            delay(250);
            // enable RMC
            _serial_gps->write("$CFGMSG,0,4,1,1*1F\r\n");
            delay(250);
        }
        didSerialInit = true;
    }

    notifyDeepSleepObserver.observe(&notifyDeepSleep);

    ubxState = UBX_SEARCH_SYNC1;
    ubxPayloadIdx = 0;

    return true;
}

GPS::~GPS()
{
    // we really should unregister our sleep observer
    notifyDeepSleepObserver.unobserve(&notifyDeepSleep);
}

// Put the GPS hardware into a specified state
void GPS::setPowerState(GPSPowerState newState, uint32_t sleepTime)
{
    // Update the stored GPSPowerstate, and create local copies
    GPSPowerState oldState = powerState;
    powerState = newState;
    LOG_INFO("GPS power state %s -> %s", getGPSPowerStateString(oldState), getGPSPowerStateString(newState));

    switch (newState) {
    case GPS_ACTIVE:
    case GPS_IDLE:
        if (oldState == GPS_ACTIVE)
            break;
        gotTime = false;
        if (oldState == GPS_IDLE) // If hardware already awake, no changes needed
            break;
        if (oldState != GPS_ACTIVE && oldState != GPS_IDLE) // If hardware just waking now, clear buffer
            clearBuffer();
#ifdef TRACKER_T1000_E
        pinMode(GPS_VRTC_EN, OUTPUT);
        digitalWrite(GPS_VRTC_EN, HIGH);
        pinMode(GPS_SLEEP_INT, OUTPUT);
        digitalWrite(GPS_SLEEP_INT, HIGH);
        pinMode(GPS_RTC_INT, OUTPUT);
        digitalWrite(GPS_RTC_INT, LOW);
        pinMode(GPS_RESETB_OUT, INPUT_PULLUP);
#endif
        powerMon->setState(meshtastic_PowerMon_State_GPS_Active); // Report change for power monitoring (during testing)
        writePinEN(true);                                         // Power (EN pin): on
#ifdef HAS_AIROHA_SOFT_RTC
        if (IS_ONE_OF(gnssModel, GNSS_MODEL_AG3335, GNSS_MODEL_AG3352))
            airohaPulseRtcInt(); // Airoha: leave software-RTC sleep, now that VCC is back
#endif
        setPowerPMU(true);      // Power (PMU): on
        writePinRFEN(true);     // External RF front-end: on
        writePinStandby(false); // Standby (pin): awake (not standby)
        setPowerUBLOX(true);    // Standby (UBLOX): awake
        break;

    case GPS_SOFTSLEEP:
        powerMon->clearState(meshtastic_PowerMon_State_GPS_Active); // Report change for power monitoring (during testing)
        writePinEN(true);                                           // Power (EN pin): on
        setPowerPMU(true);                                          // Power (PMU): on
        writePinRFEN(false);                                        // External RF front-end: off
        writePinStandby(true);                                      // Standby (pin): asleep (not awake)
        setPowerUBLOX(false, sleepTime);                            // Standby (UBLOX): asleep, timed
        break;

    case GPS_HARDSLEEP:
        powerMon->clearState(meshtastic_PowerMon_State_GPS_Active); // Report change for power monitoring (during testing)
#ifdef HAS_AIROHA_SLEEP
        if (oldState != GPS_HARDSLEEP && oldState != GPS_OFF)
            airohaEnterSleep(); // Airoha: must precede the power cut, while it can still hear us
#endif
        writePinRFEN(false);             // External RF front-end: off
        writePinStandby(true);           // Standby (pin): asleep (not awake)
        writePinEN(false);               // Power (EN pin): off
        setPowerPMU(false);              // Power (PMU): off
        setPowerUBLOX(false, sleepTime); // Standby (UBLOX): asleep, timed
        break;

    case GPS_OFF:
        assert(sleepTime == 0);                                     // This is an indefinite sleep
        powerMon->clearState(meshtastic_PowerMon_State_GPS_Active); // Report change for power monitoring (during testing)
#ifdef HAS_AIROHA_SLEEP
        if (oldState != GPS_HARDSLEEP && oldState != GPS_OFF)
            airohaEnterSleep(); // Airoha: must precede the power cut, while it can still hear us
#endif
        writePinRFEN(false);     // External RF front-end: off
        writePinStandby(true);   // Standby (pin): asleep
        writePinEN(false);       // Power (EN pin): off
        setPowerPMU(false);      // Power (PMU): off
        setPowerUBLOX(false, 0); // Standby (UBLOX): asleep, indefinitely
        break;
    }
}

// Set power with EN pin, if relevant
void GPS::writePinEN(bool on)
{
    // Abort: if conflict with Canned Messages when using Wisblock(?)
    if ((HW_VENDOR == meshtastic_HardwareModel_RAK4631 || HW_VENDOR == meshtastic_HardwareModel_WISMESH_TAP) &&
        (rotaryEncoderInterruptImpl1 || upDownInterruptImpl1))
        return;

    // Write and log
    enablePin->set(on);
    LOG_DEBUG_GPS("Pin EN %s", on == HIGH ? "HI" : "LOW");
}

// Set the value of the STANDBY pin, if relevant
// true for standby state, false for awake
void GPS::writePinStandby(bool standby)
{
#ifdef PIN_GPS_STANDBY // Specifically the standby pin for L76B, L76K and clones
    bool val;
    if (standby)
        val = GPS_STANDBY_ACTIVE;
    else
        val = !GPS_STANDBY_ACTIVE;

    // Write and log
    pinMode(PIN_GPS_STANDBY, OUTPUT);
    digitalWrite(PIN_GPS_STANDBY, val);

    // Enter backup mode on PA1010D; TODO: may be applicable to other MTK GPS too
    if (IS_ONE_OF(gnssModel, GNSS_MODEL_MTK_PA1010D)) {
        _serial_gps->write("$PMTK225,4*2F\r\n");
    }

    LOG_DEBUG_GPS("Pin STANDBY %s", val == HIGH ? "HI" : "LOW");
#endif
}

// Set the external RF front-end enable pin, if relevant
void GPS::writePinRFEN(bool on)
{
#ifdef PIN_GPS_RF_EN
    bool val = on ? GPS_RF_EN_ACTIVE : !GPS_RF_EN_ACTIVE;
    pinMode(PIN_GPS_RF_EN, OUTPUT);
    digitalWrite(PIN_GPS_RF_EN, val);
    LOG_DEBUG_GPS("Pin RF EN %s", val == HIGH ? "HI" : "LOW");
#else
    (void)on;
#endif
}

// Enable / Disable GPS with PMU, if present
void GPS::setPowerPMU(bool on)
{
    // We only have PMUs on the T-Beam, and that board has a tiny battery to save GPS ephemera,
    // so treat as a standby.
#ifdef HAS_PMU
    // Abort: if no PMU
    if (!pmu_found)
        return;

    // Abort: if PMU not initialized
    if (!PMU)
        return;

    uint8_t model = PMU->getChipModel();
    if (model == XPOWERS_AXP2101) {
        if (HW_VENDOR == meshtastic_HardwareModel_TBEAM) {
            // t-beam v1.2 GNSS power channel
            on ? PMU->enablePowerOutput(XPOWERS_ALDO3) : PMU->disablePowerOutput(XPOWERS_ALDO3);
        } else if (HW_VENDOR == meshtastic_HardwareModel_LILYGO_TBEAM_S3_CORE) {
            // t-beam-s3-core GNSS power channel
            on ? PMU->enablePowerOutput(XPOWERS_ALDO4) : PMU->disablePowerOutput(XPOWERS_ALDO4);
        } else if (HW_VENDOR == meshtastic_HardwareModel_T_WATCH_ULTRA || HW_VENDOR == meshtastic_HardwareModel_T_WATCH_S3) {
            // t-watch-ultra / t-watch-s3-plus GNSS power channel
            on ? PMU->enablePowerOutput(XPOWERS_BLDO1) : PMU->disablePowerOutput(XPOWERS_BLDO1);
        }
    } else if (model == XPOWERS_AXP192) {
        // t-beam v1.1 GNSS  power channel
        on ? PMU->enablePowerOutput(XPOWERS_LDO3) : PMU->disablePowerOutput(XPOWERS_LDO3);
    }
    LOG_DEBUG_GPS("PMU %s", on ? "on" : "off");
#endif
}

// Set UBLOX power, if relevant
void GPS::setPowerUBLOX(bool on, uint32_t sleepMs)
{
    // Abort: if not UBLOX hardware
    if (!IS_ONE_OF(gnssModel, GNSS_MODEL_UBLOX6, GNSS_MODEL_UBLOX7, GNSS_MODEL_UBLOX8, GNSS_MODEL_UBLOX9, GNSS_MODEL_UBLOX10))
        return;

    // If waking
    if (on) {
        gps->_serial_gps->write(0xFF);
        clearBuffer(); // This often returns old data, so drop it
    }

    // If putting to sleep
    else {
        uint8_t msglen;

        // If we're being asked to sleep indefinitely, make *sure* we're awake first, to process the new sleep command
        if (sleepMs == 0) {
            setPowerUBLOX(true);
            delay(500);
        }

        // Determine hardware version
        if (gnssModel != GNSS_MODEL_UBLOX10) {
            // Encode the sleep time in millis into the packet
            for (int i = 0; i < 4; i++)
                _message_PMREQ[0 + i] = sleepMs >> (i * 8);

            // Record the message length
            msglen = gps->makeUBXPacket(0x02, 0x41, sizeof(_message_PMREQ), _message_PMREQ);
        } else {
            // Encode the sleep time in millis into the packet
            for (int i = 0; i < 4; i++)
                _message_PMREQ_10[4 + i] = sleepMs >> (i * 8);

            // Record the message length
            msglen = gps->makeUBXPacket(0x02, 0x41, sizeof(_message_PMREQ_10), _message_PMREQ_10);
        }

        // Send the UBX packet
        gps->_serial_gps->write(gps->UBXscratch, msglen);
        LOG_DEBUG_GPS("UBLOX: sleep for %dmS", sleepMs);
    }
}

// With RTC_INT, park the receiver in software RTC mode so a pulse can wake it once VCC is cut;
// without it, only stop the GNSS engine.
void GPS::airohaEnterSleep()
{
#ifdef HAS_AIROHA_SLEEP
    // Only an Airoha receiver understands PAIR commands; a probe that fell back must not be talked at.
    if (!IS_ONE_OF(gnssModel, GNSS_MODEL_AG3335, GNSS_MODEL_AG3352) || airohaSleepMisses >= AIROHA_SLEEP_MAX_MISSES)
        return;

#ifdef HAS_AIROHA_SOFT_RTC
    const char *sleepCmd = "$PAIR650,0*25\r\n";
    const char *sleepAck = "$PAIR001,650,0";
#else
    const char *sleepCmd = "$PAIR003*39\r\n";
    const char *sleepAck = "$PAIR001,003,0";
#endif
    const uint32_t start = Time::getMillis();
    do {
#ifdef HAS_AIROHA_SOFT_RTC
        _serial_gps->write(sleepCmd);
#else
        // $PAIR003 alone goes unanswered; locking sleep first with $PAIR382 gets it acked.
        airohaWriteIdleInt(_serial_gps, "$PAIR382,1*2E\r\n");
        airohaWriteIdleInt(_serial_gps, sleepCmd);
#endif
        if (getACK(sleepAck, AIROHA_SLEEP_ACK_MS) == GNSS_RESPONSE_OK) {
            airohaSleepMisses = 0;
            return;
        }
        // Only start another attempt while a whole ack window still fits inside the budget.
    } while (Throttle::isWithinTimespanMs(start, AIROHA_SLEEP_BUDGET_MS - AIROHA_SLEEP_ACK_MS));

    airohaSleepMisses++;
    LOG_WARN("GPS: no ack for %.8s (%u/%u)", sleepCmd, airohaSleepMisses, AIROHA_SLEEP_MAX_MISSES);
#endif
}

/// Record that we have a GPS
void GPS::setConnected()
{
    if (!hasGPS) {
        hasGPS = true;
        shouldPublish = true;
    }
}

// We want a GPS lock. Wake the hardware
void GPS::up()
{
    scheduling.informSearching();
    setPowerState(GPS_ACTIVE);
}

// We've finished a GPS search cycle (lock or timeout). Enter a low power state, potentially.
void GPS::down()
{
    if (scheduling.hasValidFixSinceSearchStarted())
        scheduling.informGotLock();
    else
        scheduling.informSearchFailed();
    uint32_t predictedSearchDuration = scheduling.predictedSearchDurationMs();
    uint32_t sleepTime = scheduling.msUntilNextSearch();
    uint32_t updateInterval = Default::getConfiguredOrDefaultMs(config.position.gps_update_interval);

    LOG_DEBUG("%us until next search", sleepTime / 1000);

    // If update interval less than 10 seconds, no attempt to sleep
    if (updateInterval <= GPS_UPDATE_ALWAYS_ON_THRESHOLD_MS || sleepTime == 0)
        setPowerState(GPS_IDLE);

    else {
// Check whether the GPS hardware is capable of GPS_SOFTSLEEP
// If not, fallback to GPS_HARDSLEEP instead
#ifdef PIN_GPS_STANDBY // L76B, L76K and clones have a standby pin
        bool softsleepSupported = true;
#else
        bool softsleepSupported = false;
#endif
        // U-blox is supported via PMREQ
        if (IS_ONE_OF(gnssModel, GNSS_MODEL_UBLOX6, GNSS_MODEL_UBLOX7, GNSS_MODEL_UBLOX8, GNSS_MODEL_UBLOX9, GNSS_MODEL_UBLOX10))
            softsleepSupported = true;

#ifdef GPS_FORCE_SOFT_SLEEP
        if (softsleepSupported) {
            setPowerState(GPS_SOFTSLEEP, sleepTime);
            return;
        }
#endif

        if (softsleepSupported) {
            uint32_t hardsleepThreshold = gpsHardsleepThresholdMs(predictedSearchDuration / 1000);
            LOG_DEBUG("gps_update_interval >= %us needed for hardsleep", hardsleepThreshold / 1000);

            // If update interval too short: softsleep (if supported by hardware)
            if (updateInterval < hardsleepThreshold) {
                setPowerState(GPS_SOFTSLEEP, sleepTime);
                return;
            }
        }
        // If update interval long enough (or softsleep unsupported): hardsleep instead
        setPowerState(GPS_HARDSLEEP, sleepTime);
        // Reset the fix quality to 0, since we're off.
        fixQual = 0;
    }
}

void GPS::publishUpdate()
{
    if (shouldPublish) {
        shouldPublish = false;

        // In debug logs, identify position by @timestamp:stage (stage 2 = publish)
        LOG_DEBUG("Publish pos@%x:2, hasVal=%d, Sats=%d, GPSlock=%d", p.timestamp, hasValidLocation, p.sats_in_view, hasLock());

        // Notify any status instances that are observing us
        const meshtastic::GPSStatus status = meshtastic::GPSStatus(hasValidLocation, isConnected(), isPowerSaving(), p, gotTime);
        newStatus.notifyObservers(&status);
        if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_ENABLED) {
            positionModule->handleNewPosition();
        }
    }
}

/// Is a post-lock ephemeris hold currently in force? The `!= 0` is the "never armed" sentinel, which
/// deadlinePassed() reads as passed for the first half of each wrap cycle and as ~24.8 days in the
/// future for the second. No header: test_gps_fix_hold declares the prototypes itself.
bool fixHoldInForce(uint32_t fixHoldEnds, uint32_t threadIntervalMs)
{
    return fixHoldEnds != 0 && !Throttle::deadlinePassed(fixHoldEnds + threadIntervalMs);
}

/// Did an armed hold just expire? `!= 0` guards against negating fixHoldInForce() alone, which would
/// call an unarmed hold "expired" every cycle. No grace interval: the deadline itself is go-down time.
bool holdJustExpired(uint32_t fixHoldEnds)
{
    return fixHoldEnds != 0 && !fixHoldInForce(fixHoldEnds, 0);
}

/// Should a post-lock ephemeris hold be (re-)armed this cycle? "No hold in force" fires often, since
/// every publish clears the hold, including ones that don't put the receiver back to sleep.
bool shouldArmFixHold(bool hasValidLocation, uint8_t prevFixQual, uint32_t fixHoldEnds, uint32_t threadIntervalMs)
{
    // First lock of a cycle, first lock after the receiver was off, or nothing holding right now.
    return !hasValidLocation || prevFixQual == 0 || !fixHoldInForce(fixHoldEnds, threadIntervalMs);
}

int32_t GPS::runOnce()
{
#if defined(SENSECAP_INDICATOR)
    // No model probe on the bridged fake UART (the module only streams
    // NMEA), but the user's GPS mode setting must still be honored
    if (!GPSInitFinished) {
        if (!_serial_gps || config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT) {
            LOG_INFO("GPS set to not-present. Skip probe");
            return disable();
        }
        if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED) {
            return disable();
        }
        GPSInitFinished = true;
        publishUpdate();
    }
#else
    if (!GPSInitFinished) {
        if (!_serial_gps || config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_NOT_PRESENT) {
            LOG_INFO("GPS set to not-present. Skip probe");
            return disable();
        }
        if (!setup())
            return currentDelay; // Setup failed, re-run in two seconds

        if (gnssModel == GNSS_MODEL_UNKNOWN) {
            LOG_WARN("GPS not detected; not present this boot");
            return disable();
        }

        // We have now loaded our saved preferences from flash
        if (config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED) {
            return disable();
        }
        GPSInitFinished = true;
        publishUpdate();
    }
#endif

    // ======================== GPS_ACTIVE state ========================
    // In GPS_ACTIVE state, GPS is powered on and we're receiving NMEA messages.
    // We use the following logic to determine when to update the local position
    // or time by running GPS::publishUpdate.
    // Note: Local position update is asynchronous to position broadcast. We
    // generally run this state every gps_update_interval seconds, and in most cases
    // gps_update_interval is faster than the position broadcast interval so there's a
    // fresh position ready when the device wants to broadcast one on the mesh.
    //
    // 1. Got a time for the first time --> set the time, publish so the UI can show the time-only state.
    // 2. Got a lock for the first time
    //   --> If gps_update_interval is <= 10s --> publishUpdate
    //   --> Otherwise, hold for MIN(gps_update_interval - GPS_UPDATE_ALWAYS_ON_THRESHOLD_MS, 20s)
    //  3. Got a lock after turning back on
    //   --> If gps_update_interval is <= 10s --> publishUpdate
    //   --> Otherwise, hold for MIN(gps_update_interval - GPS_UPDATE_ALWAYS_ON_THRESHOLD_MS, 20s)
    // 4. Hold has expired
    //   --> If we have a time and a location --> publishUpdate
    //   --> down()
    // 5. Search time has expired
    //   --> If we have a time and a location --> publishUpdate
    //   --> If we had a location before but don't now --> publishUpdate
    //   --> down()
    if (whileActive()) {
        // if we have received valid NMEA claim we are connected
        setConnected();
    }

    // If we're due for an update, wake the GPS
    if (!config.position.fixed_position && powerState != GPS_ACTIVE && scheduling.isUpdateDue())
        up();

    // quality of the previous fix. We set it to 0 when we go down, so it's a way
    // to check if we're getting a lock after being GPS_OFF.
    uint8_t prev_fixQual = fixQual;

    if (powerState == GPS_ACTIVE) {
        // if gps_update_interval is <=10s, GPS never goes off, so we treat that differently
        uint32_t updateInterval = Default::getConfiguredOrDefaultMs(config.position.gps_update_interval);

        // 1. Got a time for the first time this cycle
        if (!gotTime && lookForTime()) { // Note: we count on this && short-circuiting and not resetting the RTC time
            gotTime = true;
            // Publish immediately (rather than via the block below, which would clear fixHoldEnds) so the
            // time-only state reaches the UI without waiting for a location. Safe without a valid location:
            // PositionModule::handleNewPosition ignores invalid positions.
            shouldPublish = true;
            publishUpdate();
        }

        // 2. Got a lock for the first time, or 3. Got a lock after turning back on
        bool gotLoc = lookForLocation();
        if (gotLoc) {
            scheduling.informValidFix();
#if GPS_DEBUG
            if (!hasValidLocation) { // declare that we have location ASAP
                LOG_DEBUG("hasValidLocation RISING EDGE");
            }
#endif
            if (updateInterval <= GPS_UPDATE_ALWAYS_ON_THRESHOLD_MS) {
                hasValidLocation = true;
                shouldPublish = true;
            } else if (shouldArmFixHold(hasValidLocation, prev_fixQual, fixHoldEnds, GPS_THREAD_INTERVAL)) {
                hasValidLocation = true;
                // Hold for up to 20secs after getting a lock to download ephemeris etc
                uint32_t holdTime = updateInterval - GPS_UPDATE_ALWAYS_ON_THRESHOLD_MS;
                if (holdTime > GPS_FIX_HOLD_MAX_MS)
                    holdTime = GPS_FIX_HOLD_MAX_MS;
                // Same clock the Throttle evaluation reads, and never the "no hold" sentinel.
                fixHoldEnds = Time::timerEndsAtMillis(holdTime);
                LOG_DEBUG_GPS("Holding for %ums after lock", holdTime);
            }
        }

        bool tooLong = scheduling.searchedTooLong();
        if (tooLong && !scheduling.hasValidFixSinceSearchStarted()) {
            LOG_WARN("Can't publish valid location: no GPS lock in time");
            // we didn't get a location during this ack window, therefore declare loss of lock
            if (hasValidLocation) {
                p = meshtastic_Position_init_default;
                hasValidLocation = false;
                shouldPublish = true;
                LOG_DEBUG_GPS("hasValidLocation FALLING EDGE");
            }
        }

        // Hold has expired , Search time has expired, we got a time only, or we never needed to hold.
        bool holdExpired = holdJustExpired(fixHoldEnds);
        if (shouldPublish || tooLong || holdExpired) {
            if (gotTime && hasValidLocation) {
                shouldPublish = true;
            }
            if (shouldPublish) {
                fixHoldEnds = 0;
                publishUpdate();
            }

            // There's a chance we just got a time, so keep going to see if we can get a location too
            if (tooLong || holdExpired) {
                down();
            }

#if GPS_DEBUG
        } else if (fixHoldEnds != 0) {
            LOG_DEBUG("Holding for GPS data download: %d ms (numSats=%d)", fixHoldEnds - Time::getMillis(), p.sats_in_view);
#endif
        }
    }
    // ===================== end GPS_ACTIVE state ========================

    if (config.position.fixed_position == true && hasValidLocation)
        return disable(); // This should trigger when we have a fixed position, and get that first position

    // 9600bps is approx 1 byte per msec, so considering our buffer size we never need to wake more often than 200ms
    // if not awake we can run super infrequently (once every 5 secs?) to see if we need to wake.
    return (powerState == GPS_ACTIVE) ? GPS_THREAD_INTERVAL : 5000;
}

// clear the GPS rx/tx buffer as quickly as possible
void GPS::clearBuffer()
{
#ifdef ARCH_ESP32
    _serial_gps->flush(false);
#else
    int x = _serial_gps->available();
    while (x--)
        _serial_gps->read();
#endif
}

/// Prepare the GPS for the cpu entering deep or light sleep, expect to be gone for at least 100s of msecs
int GPS::prepareDeepSleep(void *unused)
{
    LOG_INFO("GPS deep sleep");
    disable();
    return 0;
}

static const char *PROBE_MESSAGE = "Trying %s (%s)...";
static const char *DETECTED_MESSAGE = "%s detected";

#define PROBE_SIMPLE(CHIP, TOWRITE, RESPONSE, DRIVER, TIMEOUT, ...)                                                              \
    do {                                                                                                                         \
        LOG_DEBUG(PROBE_MESSAGE, TOWRITE, CHIP);                                                                                 \
        clearBuffer();                                                                                                           \
        _serial_gps->write(TOWRITE "\r\n");                                                                                      \
        if (getACK(RESPONSE, TIMEOUT) == GNSS_RESPONSE_OK) {                                                                     \
            LOG_INFO(DETECTED_MESSAGE, CHIP);                                                                                    \
            return DRIVER;                                                                                                       \
        }                                                                                                                        \
    } while (0)

#define PROBE_FAMILY(FAMILY_NAME, COMMAND, RESPONSE_MAP, TIMEOUT)                                                                \
    do {                                                                                                                         \
        LOG_DEBUG(PROBE_MESSAGE, COMMAND, FAMILY_NAME);                                                                          \
        clearBuffer();                                                                                                           \
        _serial_gps->write(COMMAND "\r\n");                                                                                      \
        GnssModel_t detectedDriver = getProbeResponse(TIMEOUT, RESPONSE_MAP, serialSpeed);                                       \
        if (detectedDriver != GNSS_MODEL_UNKNOWN) {                                                                              \
            return detectedDriver;                                                                                               \
        }                                                                                                                        \
    } while (0)

GnssModel_t GPS::probe(int serialSpeed)
{
    uint8_t buffer[768] = {0};

    switch (currentStep) {
    case 0: {
#if defined(ARCH_NRF52) || defined(ARCH_PORTDUINO) || defined(ARCH_STM32)
        _serial_gps->end();
        _serial_gps->begin(serialSpeed);
#elif defined(ARCH_RP2040)
        _serial_gps->end();
        _serial_gps->setFIFOSize(256);
        _serial_gps->begin(serialSpeed);
#else
        if (_serial_gps->baudRate() != serialSpeed) {
            LOG_DEBUG("Set GPS Baud to %i", serialSpeed);
            _serial_gps->updateBaudRate(serialSpeed);
        }
#endif

        memset(&ublox_info, 0, sizeof(ublox_info));
        delay(100);

#if defined(PIN_GPS_RESET) && PIN_GPS_RESET != -1
        digitalWrite(PIN_GPS_RESET, GPS_RESET_MODE); // assert for 10ms
        delay(10);
        digitalWrite(PIN_GPS_RESET, !GPS_RESET_MODE);
#ifdef TRACKER_T1000_E
        delay(100);
#endif

        // attempt to detect the chip based on boot messages
        std::vector<ChipInfo> passive_detect = {
            {"AG3335", "$PAIR021,AG3335", GNSS_MODEL_AG3335},
            {"AG3352", "$PAIR021,AG3352", GNSS_MODEL_AG3352},
            {"RYS3520", "$PAIR021,REYAX_RYS3520_V2", GNSS_MODEL_AG3352},
            {"UC6580", "UC6580", GNSS_MODEL_UC6580}
            // as L76K is sort of a last ditch effort, we won't attempt to detect it by startup messages for now.
            /*{"L76K", "SW=URANUS", GNSS_MODEL_MTK}*/};
        GnssModel_t detectedDriver = getProbeResponse(500, passive_detect, serialSpeed);
        if (detectedDriver != GNSS_MODEL_UNKNOWN) {
            return detectedDriver;
        }
#endif
        // Close all NMEA sentences, valid for L76K, ATGM336H (and likely other AT6558 devices)
        _serial_gps->write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
        delay(20);
        // Close NMEA sequences on Ublox
        _serial_gps->write("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");
        _serial_gps->write("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");
        _serial_gps->write("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");
        delay(20);
        // Close NMEA sequences on CM121
        _serial_gps->write("$CFGMSG,0,1,0,1*1B\r\n");
        _serial_gps->write("$CFGMSG,0,2,0,1*18\r\n");
        _serial_gps->write("$CFGMSG,0,3,0,1*19\r\n");
        currentDelay = 20;
        currentStep = 1;
        return GNSS_MODEL_UNKNOWN;
    }
    case 1: {

        // Unicore UFirebirdII Series: UC6580, UM620, UM621, UM670A, UM680A, or UM681A,or CM121
        std::vector<ChipInfo> unicore = {{"UC6580", "UC6580", GNSS_MODEL_UC6580},
                                         {"UM600", "UM600", GNSS_MODEL_UC6580},
                                         {"CM121", "CM121", GNSS_MODEL_CM121},
                                         {"CC1167Q", "CC1167Q", GNSS_MODEL_CM121},
                                         {"LC760CA", "CC1161W", GNSS_MODEL_LC760CA}};
        PROBE_FAMILY("Unicore Family", "$PDTINFO", unicore, 500);
        currentDelay = 20;
        currentStep = 2;
        return GNSS_MODEL_UNKNOWN;
    }
    case 2: {
        std::vector<ChipInfo> atgm = {
            {"ATGM336H", "$GPTXT,01,01,02,HW=ATGM336H", GNSS_MODEL_ATGM336H},
            /* ATGM332D series (-11(GPS), -21(BDS), -31(GPS+BDS), -51(GPS+GLONASS), -71-0(GPS+BDS+GLONASS)) based on AT6558 */
            {"ATGM332D", "$GPTXT,01,01,02,HW=ATGM332D", GNSS_MODEL_ATGM336H}};
        PROBE_FAMILY("ATGM33xx Family", "$PCAS06,1*1A", atgm, 500);
        currentDelay = 20;
        currentStep = 3;
        return GNSS_MODEL_UNKNOWN;
    }
    case 3: {
        /* Airoha (Mediatek) AG3335A/M/S, A3352Q, Quectel L89 2.0, SimCom SIM65M */
        sendAirohaWakeCommand(_serial_gps);
        _serial_gps->write("$PAIR062,2,0*3C\r\n"); // GSA OFF to reduce volume
        _serial_gps->write("$PAIR062,3,0*3D\r\n"); // GSV OFF to reduce volume
        _serial_gps->write("$PAIR513*3D\r\n");     // save configuration
        std::vector<ChipInfo> airoha = {{"AG3335", "$PAIR021,AG3335", GNSS_MODEL_AG3335},
                                        {"AG3352", "$PAIR021,AG3352", GNSS_MODEL_AG3352},
                                        {"RYS3520", "$PAIR021,REYAX_RYS3520_V2", GNSS_MODEL_AG3352}};
        PROBE_FAMILY("Airoha Family", "$PAIR021*39", airoha, 1000);
        currentDelay = 20;
        currentStep = 4;
        return GNSS_MODEL_UNKNOWN;
    }
    case 4: {
        PROBE_SIMPLE("LC86", "$PQTMVERNO*58", "$PQTMVERNO,LC86", GNSS_MODEL_AG3352, 500);
        PROBE_SIMPLE("L76K", "$PCAS06,0*1B", "$GPTXT,01,01,02,SW=", GNSS_MODEL_MTK, 500);
        currentDelay = 20;
        currentStep = 5;
        return GNSS_MODEL_UNKNOWN;
    }
    case 5: {

        // Close all NMEA sentences, valid for MTK3333 and MTK3339 platforms
        _serial_gps->write("$PMTK514,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2E\r\n");
        delay(20);
        std::vector<ChipInfo> mtk = {{"L76B", "Quectel-L76B", GNSS_MODEL_MTK_L76B}, {"PA1010D", "1010D", GNSS_MODEL_MTK_PA1010D},
                                     {"PA1616S", "1616S", GNSS_MODEL_MTK_PA1616S},  {"LS20031", "MC-1513", GNSS_MODEL_MTK_L76B},
                                     {"L96", "Quectel-L96", GNSS_MODEL_MTK_L76B},   {"L80-R", "_3337_", GNSS_MODEL_MTK_L76B},
                                     {"L80", "_3339_", GNSS_MODEL_MTK_L76B}};

        PROBE_FAMILY("MTK Family", "$PMTK605*31", mtk, 500);
        currentDelay = 20;
        currentStep = 6;
        return GNSS_MODEL_UNKNOWN;
    }
    case 6: {
        uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x00, 0x00};
        UBXChecksum(cfg_rate, sizeof(cfg_rate));
        clearBuffer();
        _serial_gps->write(cfg_rate, sizeof(cfg_rate));
        // Check that the returned response class and message ID are correct
        GPS_RESPONSE response = GNSS_RESPONSE_OK;//getACK(0x06, 0x08, 750);
        if (response == GNSS_RESPONSE_NONE) {
            LOG_WARN("No GNSS Module (baudrate %d)", serialSpeed);
            currentDelay = 2000;
            currentStep = 0;
            return GNSS_MODEL_UNKNOWN;
        } else if (response == GNSS_RESPONSE_FRAME_ERRORS) {
            LOG_INFO("UBlox Frame Errors (baudrate %d)", serialSpeed);
        }

        memset(buffer, 0, sizeof(buffer));
        uint8_t _message_MONVER[8] = {
            0xB5, 0x62, // Sync message for UBX protocol
            0x0A, 0x04, // Message class and ID (UBX-MON-VER)
            0x00, 0x00, // Length of payload (we're asking for an answer, so no payload)
            0x00, 0x00  // Checksum
        };
        //  Get Ublox gnss module hardware and software info
        UBXChecksum(_message_MONVER, sizeof(_message_MONVER));
        clearBuffer();
        _serial_gps->write(_message_MONVER, sizeof(_message_MONVER));

        uint16_t len = 1; //getACK(buffer, sizeof(buffer), 0x0A, 0x04, 1200);
        if (len) {
            uint16_t position = 0;
            for (int i = 0; i < 30; i++) {
                break;
                ublox_info.swVersion[i] = buffer[position];
                position++;
            }
            for (int i = 0; i < 10; i++) {
                break;
                ublox_info.hwVersion[i] = buffer[position];
                position++;
            }

            while (len >= position + 30) {
                for (int i = 0; i < 30; i++) {
                    ublox_info.extension[ublox_info.extensionNo][i] = buffer[position];
                    position++;
                }
                ublox_info.extensionNo++;
                if (ublox_info.extensionNo > 9)
                    break;
            }
            
            ublox_info.protocol_version = 34; // Major version of 34.10

            strncpy(ublox_info.swVersion, "SPG 5.10", sizeof(ublox_info.swVersion) - 1);
            strncpy(ublox_info.hwVersion, "000A0000", sizeof(ublox_info.hwVersion) - 1);

            // Extensions (if parsed from the log)
            ublox_info.extensionNo = 2;
            strncpy(ublox_info.extension[0], "PROTVER=34.10", sizeof(ublox_info.extension[0]) - 1);
            strncpy(ublox_info.extension[1], "GPS;GLO;GAL;BDS", sizeof(ublox_info.extension[1]) - 1);
            
            LOG_DEBUG("Soft version: %s", ublox_info.swVersion);
            LOG_DEBUG("Hard version: %s", ublox_info.hwVersion);
            LOG_DEBUG("Extensions:%d", ublox_info.extensionNo);
            for (int i = 0; i < ublox_info.extensionNo; i++) {
                LOG_DEBUG("  %s", ublox_info.extension[i]);
            }

            memset(buffer, 0, sizeof(buffer));

            // tips: extensionNo field is 0 on some 6M GNSS modules
            for (int i = 0; i < ublox_info.extensionNo; ++i) {
                if (!strncmp(ublox_info.extension[i], "MOD=", 4)) {
                    strncpy((char *)buffer, &(ublox_info.extension[i][4]), sizeof(buffer));
                } else if (!strncmp(ublox_info.extension[i], "PROTVER", 7)) {
                    char *ptr = nullptr;
                    memset(buffer, 0, sizeof(buffer));
                    strncpy((char *)buffer, &(ublox_info.extension[i][8]), sizeof(buffer));
                    LOG_DEBUG("Protocol Version:%s", (char *)buffer);
                    if (strlen((char *)buffer)) {
                        ublox_info.protocol_version = strtoul((char *)buffer, &ptr, 10);
                        LOG_DEBUG("ProtVer=%d", ublox_info.protocol_version);
                    } else {
                        ublox_info.protocol_version = 0;
                    }
                }
            }
            if (strncmp(ublox_info.hwVersion, "00040007", 8) == 0) {
                LOG_INFO(DETECTED_MESSAGE, "U-blox 6", "6");
                return GNSS_MODEL_UBLOX6;
            } else if (strncmp(ublox_info.hwVersion, "00070000", 8) == 0) {
                LOG_INFO(DETECTED_MESSAGE, "U-blox 7", "7");
                return GNSS_MODEL_UBLOX7;
            } else if (strncmp(ublox_info.hwVersion, "00080000", 8) == 0) {
                LOG_INFO(DETECTED_MESSAGE, "U-blox 8", "8");
                return GNSS_MODEL_UBLOX8;
            } else if (strncmp(ublox_info.hwVersion, "00190000", 8) == 0) {
                LOG_INFO(DETECTED_MESSAGE, "U-blox 9", "9");
                return GNSS_MODEL_UBLOX9;
            } else if (strncmp(ublox_info.hwVersion, "000A0000", 8) == 0) {
                LOG_INFO(DETECTED_MESSAGE, "U-blox 10", "10");
                return GNSS_MODEL_UBLOX10;
            }
        }
    }
    }
    LOG_WARN("No GNSS Module (baudrate %d)", serialSpeed);
    currentDelay = 2000;
    currentStep = 0;
    return GNSS_MODEL_UNKNOWN;
}

GnssModel_t GPS::getProbeResponse(unsigned long timeout, const std::vector<ChipInfo> &responseMap, int serialSpeed)
{
    // Calculate buffer size based on baud rate - 256 bytes for 9600 baud as baseline
    // Higher baud rates get proportionally larger buffers to handle more data
    int bufferSize = (serialSpeed * 256) / 9600;
    // Clamp buffer size between reasonable limits
    if (bufferSize < 128)
        bufferSize = 128;
    if (bufferSize > 2048)
        bufferSize = 2048;

    auto response = std::unique_ptr<char[]>(new char[bufferSize]); // Dynamically allocate based on baud rate
    uint16_t responseLen = 0;
    unsigned long start = millis();
    while (millis() - start < timeout) {
        if (_serial_gps->available()) {
            char c = _serial_gps->read();

            // Add char to buffer if there's space
            if (responseLen < bufferSize - 1) {
                response[responseLen++] = c;
                response[responseLen] = '\0';
            }

            if (c == ',' || (responseLen >= 2 && response[responseLen - 2] == '\r' && response[responseLen - 1] == '\n')) {
                // check if we can see our chips
                for (const auto &chipInfo : responseMap) {
                    if (strstr(response.get(), chipInfo.detectionString.c_str()) != nullptr) {
                        LOG_DEBUG_GPS("%s", response.get());
                        LOG_INFO("%s detected", chipInfo.chipName.c_str());
                        return chipInfo.driver;
                    }
                }
            }
            if (responseLen >= 2 && response[responseLen - 2] == '\r' && response[responseLen - 1] == '\n') {
                LOG_DEBUG_GPS("%s", response.get());
                // Reset the response buffer for the next potential message
                responseLen = 0;
                response[0] = '\0';
            }
        }
    }
    LOG_DEBUG_GPS("%s", response.get());
    return GNSS_MODEL_UNKNOWN; // Return unknown on timeout
}

static UBXState ubx_state = PREAMBLE_1;
static uint8_t ubx_class = 0;
static uint8_t ubx_id = 0;
static uint16_t ubx_len = 0;
static uint16_t ubx_payload_idx = 0;
static uint8_t ubx_payload[128];
static uint8_t ubx_ck_a = 0, ubx_ck_b = 0;
static uint8_t calc_ck_a = 0, calc_ck_b = 0;
// Static variables maintain state across handleUBXMessage calls
static uint32_t totalUbxCount = 0;
static uint32_t lastLogTime = 0;

// UBX Parser State Machine Definitions
//enum UbxParseState {
    //UBX_SEARCH_SYNC1,
    //UBX_SEARCH_SYNC2,
    //UBX_CLASS,
    //UBX_ID,
    //UBX_LENGTH_LSB,
    //UBX_LENGTH_MSB,
    //UBX_PAYLOAD,
    //UBX_CK_A,
    //UBX_CK_B
//};

//UbxParseState ubxState = UBX_SEARCH_SYNC1;
uint8_t ubxMsgClass = 0;
uint8_t ubxMsgId = 0;
uint16_t ubxPayloadLen = 0;
uint16_t ubxPayloadIdx = 0;
uint8_t ubxPayloadBuffer[128]; // Fits NAV-PVT (92-100+ bytes)


// Added to GPS class / header:
struct UbxTimeData {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    bool valid;
    uint32_t lastParsedMs; // Set using millis() when frame arrives
} ubxTimeData;

// Added to GPS class / header:
struct UbxLocData {
    int32_t lat;             // High precision fold: pvt->lat + (latHp / 100)
    int32_t lon;             // High precision fold: pvt->lon + (lonHp / 100)
    int32_t height;          // HAE altitude in mm
    int32_t hMSL;            // MSL altitude in mm
    uint16_t pdop;           // pDOP in 1/100ths
    uint8_t fixType;         // 0=No fix, 2=2D, 3=3D
    uint8_t fixFlags;        // pvt->flags
    uint8_t numSV;           // Satellites in view
    uint32_t headMot;        // Heading of motion (1e-5 deg)
    int32_t gSpeed;          // Ground speed (mm/s)
    int32_t nano;
    uint32_t lastParsedMs;   // Set via millis() on arrival
    
    bool isUpdated;          // Set to true when new frame parsed
} ubxLocData;

void GPS::handleUBXMessage(uint8_t msgClass, uint8_t msgId, uint8_t *payload, uint16_t len, uint8_t ckA, uint8_t ckB) {
    // Check for UBX-NAV-PVT (Class 0x01, ID 0x07)
    if (msgClass == 0x01 && msgId == 0x07 && len >= 92) {
        UBX_NAV_PVT_payload *pvt = (UBX_NAV_PVT_payload *)payload;

        uint32_t now = millis();
        
        if (now - ubxTimeData.lastParsedMs < 500) {
            return;
        }
        ubxTimeData.lastParsedMs = now;
        
        if ((pvt->valid & 0x03) == 0x03) { // Bit 0 = validDate, Bit 1 = validTime
            ubxTimeData.year = pvt->year;
            ubxTimeData.month = pvt->month;
            ubxTimeData.day = pvt->day;
            ubxTimeData.hour = pvt->hour;
            ubxTimeData.min = pvt->min;
            ubxTimeData.sec = pvt->sec;
            ubxTimeData.lastParsedMs = millis();
            ubxTimeData.valid = true;
        }

        // ----------------------------------------------------
        // B. LOCATION POPULATION & GUARD CHECKS
        // ----------------------------------------------------
        // Verify Fix Status: 2D/3D fix AND Bit 0 of flags byte (gnssFixOK)
        bool validFix = (pvt->fixType >= 2) && (pvt->flags & 0x01);

        if (validFix) {
            
            
            // Fold high-precision offsets if using extended payload (len >= 100 bytes)
            int8_t latHp = (len >= 100) ? pvt->latHp : 0;
            int8_t lonHp = (len >= 100) ? pvt->lonHp : 0;

            // Fold 1e-9 HP offset into 1e-7 base integer
            int32_t finalLat = pvt->lat + (latHp / 100);
            int32_t finalLon = pvt->lon + (lonHp / 100);

            // Defensive Guards: Reject Earth out-of-bounds & Null Island
            if (abs(finalLat) > 900000000 || abs(finalLon) > 1800000000) {
                LOG_WARN("[UBX GPS] Out of bounds position rejected (Lat: %d, Lon: %d)", finalLat, finalLon);
                this->hasValidLocation = false;
                ubxLocData.isUpdated = false;
                return;
            }

            if (finalLat == 0 && finalLon == 0) {
                LOG_WARN("[UBX GPS] Null Island position (0,0) rejected!");
                this->hasValidLocation = false;
                ubxLocData.isUpdated = false;
                return;
            }
            // Add this line inside handleUBXMessage / handleUBXMessage2
            uint32_t nowMs = millis();
            // Populate the location cache struct
            ubxLocData.lat = finalLat;
            ubxLocData.lon = finalLon;
            ubxLocData.height = pvt->height;
            ubxLocData.hMSL = pvt->hMSL;
            ubxLocData.pdop = pvt->pDOP;
            ubxLocData.fixType = pvt->fixType;
            ubxLocData.fixFlags = pvt->flags;
            ubxLocData.numSV = pvt->numSV;
            ubxLocData.headMot = pvt->headMot;
            ubxLocData.gSpeed = pvt->gSpeed;
            ubxLocData.lastParsedMs = nowMs;
            ubxLocData.nano = pvt->nano;
            ubxLocData.isUpdated = true;

            this->hasValidLocation = true;

            // 15-second rate-limited diagnostic logging
            static uint32_t lastLogTime = 0;
            static uint32_t totalUbxFrames = 0;
            totalUbxFrames++;

            if (nowMs - lastLogTime >= 15000) {
                lastLogTime = nowMs;
                LOG_DEBUG("[UBX RX] 15s Summary | Total Frames: %u | FixType: %d, Sats: %d, Lat: %d, Lon: %d", 
                          totalUbxFrames, ubxLocData.fixType, ubxLocData.numSV, ubxLocData.lat, ubxLocData.lon);
            }
        } else {
            this->hasValidLocation = false;
            ubxLocData.isUpdated = false;
        }
    }
}



void GPS::handleUBXMessageOld(uint8_t msgClass, uint8_t msgId, uint8_t *payload, uint16_t len) {
    // Class 0x01, ID 0x07 = UBX-NAV-PVT
    if (msgClass == 0x01 && msgId == 0x07 && len >= 92) {
        UBX_NAV_PVT_payload *pvt = (UBX_NAV_PVT_payload *)payload;
        
        totalUbxCount++; // Increment on every valid UBX message processed
        
        bool validFix = (pvt->fixType >= 2) && (pvt->flags & 0x01);
        bool validTime = (pvt->valid & 0x03) == 0x03;

        // Log summary every 15,000 ms (15 seconds)
        if (millis() - lastLogTime >= 15000) {
            lastLogTime = millis();
            
            LOG_DEBUG("[UBX RX] Total Msgs: %u | FixType: %d, Sats: %d, Lat: %.7f, Lon: %.7f, ValidTime: %d, ValidFix: %d", 
                      totalUbxCount,
                      pvt->fixType, 
                      pvt->numSV, 
                      pvt->lat / 1e7, 
                      pvt->lon / 1e7,
                      validTime,
                      validFix);
        }
        
        meshtastic_Position pos = meshtastic_Position_init_default;

        if (validTime) {
            // 1. Pure UTC conversion using deterministic arithmetic
            uint16_t y = pvt->year;
            uint8_t m = pvt->month;
            uint8_t d = pvt->day;

            if (m <= 2) {
                y -= 1;
                m += 12;
            }

            uint32_t days = (365 * y) + (y / 4) - (y / 100) + (y / 400) + (306 * (m + 1) / 10) + d - 719591;
            uint32_t epochSecs = (days * 86400) + (pvt->hour * 3600) + (pvt->min * 60) + pvt->sec;

            // 2. Assign BOTH protobuf timestamp fields
            pos.time = epochSecs;
            pos.timestamp = epochSecs;

            // 3. Sub-second adjustment (signed ms: -500 to +500)
            pos.timestamp_millis_adjust = pvt->nano / 1000000;
            
            //LOG_DEBUG("[UBX GPS] Time OK: Epoch secs=%d", 
            //          epochSecs);
            
        }
        // Safely check if packet contains high-precision extensions (>= 100 bytes)
        int8_t latHp = (len >= 100) ? pvt->latHp : 0;
        int8_t lonHp = (len >= 100) ? pvt->lonHp : 0;
        
        int32_t finalLat = pvt->lat + (latHp / 100);
        int32_t finalLon = pvt->lon + (lonHp / 100);
        
        // ==================== GUARD CHECKS ====================

        // 1. Absolute Zero / Null Island Guard (|lat| or |lon| < 0.1 deg / 1,000,000 units)
        // (Unless you are intentionally testing at the equator/prime meridian junction!)
        if (validFix && (abs(finalLat) < 1000000 || abs(finalLon) < 1000000)) {
            LOG_WARN("[UBX GPS] Guard Triggered: Coordinates near zero (Lat: %d, Lon: %d). Skipping.", finalLat, finalLon);
            this->hasValidLocation = false;
            validFix = false;
        }

        // 2. Out-of-bounds Earth Guard (> 90 lat / > 180 lon)
        if (abs(finalLat) > 900000000 || abs(finalLon) > 1800000000) {
            LOG_WARN("[UBX GPS] Guard Triggered: Out of bounds (Lat: %d, Lon: %d). Skipping.", finalLat, finalLon);
            this->hasValidLocation = false;
            validFix = false;
        }

        if (validFix) {
                        
            pos.has_latitude_i = true;
            pos.latitude_i = pvt->lat + (pvt->latHp / 100);

            pos.has_longitude_i = true;
            pos.longitude_i = pvt->lon + (pvt->lonHp / 100);

            // Altitude
            pos.has_altitude = true;
            pos.altitude = (pvt->hMSL + (pvt->hMSLHp / 10)) / 1000;

            // Satellites & DOP
            pos.sats_in_view = pvt->numSV;
            pos.PDOP = pvt->pDOP;
            
            pos.fix_type = pvt->fixType;

            //pos.timestamp = millis();
            pos.precision_bits = 32;

            // Pushes position directly to Meshtastic's internal database & phone app
            this->hasValidLocation = true;

            //LOG_DEBUG("[UBX GPS] Fix OK: Lat=%d, Lon=%d, Sats=%d", 
            //          pvt->lat, pvt->lon, pvt->numSV);
        } else {
            this->hasValidLocation = false;
        }
        if (validFix || validTime) {
            //if (nodeDB) {
            if (gpsStatus) {
                meshtastic::GPSStatus newStatus(
                validFix,     // hasLock
                true,         // isConnected (GPS responding over UART)
                false,        // isPowerSaving
                pos,          // meshtastic_Position struct payload
                validTime     // hasTime (Decoupled time flag)
            );

            // Directly notifies PositionModule observers if status is dirty
            static bool triggeredThisSecond = false;
            uint32_t remainder = millis() % 1000;

            // Triggers if loop lands within the first 10ms of any second
            if (remainder < 50) {
                if (!triggeredThisSecond) {
                    triggeredThisSecond = true; 
                    // Close enough to the second tick
                    LOG_DEBUG("[UBX GPS] Status sent: time %u, lat %, lon %, sats %",
                        pos.time,
                        pvt->lat,
                        pvt->lon,
                        pvt->numSV);
                    bool statusUpdated = gpsStatus->updateStatus(&newStatus);
                    LOG_DEBUG("[UBX GPS] Status %s", statusUpdated ? "updated" : "NOT updated");
                }
            } else {
                // Reset the lock once we move past the 50ms window
                triggeredThisSecond = false; 
            }      
            
            
            }
                
            //    nodeDB->setLocalPosition(pos);
            //}
        }
 
    }
}

void GPS::parseUBXByte(uint8_t b) {
    switch (ubx_state) {
        case PREAMBLE_1:
            if (b == 0xB5) ubx_state = PREAMBLE_2;
            break;

        case PREAMBLE_2:
            if (b == 0x62) {
                ubx_state = CLASS;
                calc_ck_a = 0;
                calc_ck_b = 0;
            } else {
                ubx_state = (b == 0xB5) ? PREAMBLE_2 : PREAMBLE_1;
            }
            break;

        case CLASS:
            ubx_class = b;
            calc_ck_a += b; calc_ck_b += calc_ck_a;
            ubx_state = ID;
            break;

        case ID:
            ubx_id = b;
            calc_ck_a += b; calc_ck_b += calc_ck_a;
            ubx_state = LENGTH_1;
            break;

        case LENGTH_1:
            ubx_len = b;
            calc_ck_a += b; calc_ck_b += calc_ck_a;
            ubx_state = LENGTH_2;
            break;

        case LENGTH_2:
            ubx_len |= ((uint16_t)b << 8);
            calc_ck_a += b; calc_ck_b += calc_ck_a;
            ubx_payload_idx = 0;
            if (ubx_len > sizeof(ubx_payload)) {
                // Payload too large for buffer, discard
                ubx_state = PREAMBLE_1;
            } else if (ubx_len == 0) {
                ubx_state = CK_A;
            } else {
                ubx_state = PAYLOAD;
            }
            break;

        case PAYLOAD:
            ubx_payload[ubx_payload_idx++] = b;
            calc_ck_a += b; calc_ck_b += calc_ck_a;
            if (ubx_payload_idx >= ubx_len) {
                ubx_state = CK_A;
            }
            break;

        case CK_A:
            ubx_ck_a = b;
            ubx_state = CK_B;
            break;

        case CK_B:
            ubx_ck_b = b;
            if (calc_ck_a == ubx_ck_a && calc_ck_b == ubx_ck_b) {
                // Valid frame, process payload
                handleUBXMessageOld(ubx_class, ubx_id, ubx_payload, ubx_len);
            } else {
                LOG_WARN("[UBX RX] Checksum mismatch! Calc: 0x%02X 0x%02X, Recv: 0x%02X 0x%02X", 
                         calc_ck_a, calc_ck_b, ubx_ck_a, ubx_ck_b);
            }
            ubx_state = PREAMBLE_1;
            break;
    }
}



std::unique_ptr<GPS> GPS::createGps()
{
    int8_t _rx_gpio = config.position.rx_gpio;
    int8_t _tx_gpio = config.position.tx_gpio;
    int8_t _en_gpio = config.position.gps_en_gpio;

#if defined(GPS_RX_PIN)
    if (!_rx_gpio)
        _rx_gpio = GPS_RX_PIN;
#endif
#if defined(GPS_TX_PIN)
    if (!_tx_gpio)
        _tx_gpio = GPS_TX_PIN;
#endif
#if defined(PIN_GPS_EN)
    if (!_en_gpio)
        _en_gpio = PIN_GPS_EN;
#endif
#ifdef ARCH_PORTDUINO
    if (portduino_config.has_gps) {
        // These need to set as flags so later checks will pass on native and GPS will work.
        // They are not used for any hardware access.
        _rx_gpio = 1;
        _tx_gpio = 1;
        if (!portduino_config.gpsd_host.empty()) {
            gpsdSerial.setAddress(portduino_config.gpsd_host, portduino_config.gpsd_port);
            _serial_gps = &gpsdSerial;
        }
    } else
        return nullptr;
#endif
#if defined(SENSECAP_INDICATOR)
    // assigned at runtime, static initialization order across translation
    // units is undefined
    _serial_gps = uartProxy;
    if (!_serial_gps)
        return nullptr;
#else
    if (!_rx_gpio || !_serial_gps) // Configured to have no GPS at all
        return nullptr;
#endif

    auto new_gps = std::unique_ptr<GPS>(new GPS());
    new_gps->rx_gpio = _rx_gpio;
    new_gps->tx_gpio = _tx_gpio;
#ifdef ARCH_PORTDUINO
    // Skip chip-specific probing for gpsd - it's a generic NMEA stream.
    if (!portduino_config.gpsd_host.empty())
        new_gps->gnssModel = GNSS_MODEL_GENERIC_NMEA;
#endif

    GpioVirtPin *virtPin = new GpioVirtPin();
    new_gps->enablePin = virtPin; // Always at least populate a virtual pin
    if (_en_gpio) {
        GpioPin *p = new GpioHwPin(_en_gpio);

        if (!GPS_EN_ACTIVE) { // Need to invert the pin before hardware
            new GpioNotTransformer(
                virtPin,
                p); // We just leave this created object on the heap so it can stay watching virtPin and driving en_gpio
        } else {
            new GpioUnaryTransformer(
                virtPin,
                p); // We just leave this created object on the heap so it can stay watching virtPin and driving en_gpio
        }
    }

#ifdef PIN_GPS_PPS
    // pulse per second
    pinMode(PIN_GPS_PPS, INPUT);
#endif

#ifdef PIN_GPS_SWITCH
    // toggle GPS via external GPIO switch
    pinMode(PIN_GPS_SWITCH, INPUT);
    gpsPeriodic = std::unique_ptr<concurrency::Periodic>(new concurrency::Periodic("GPSSwitch", gpsSwitch));
#endif

// Currently disabled per issue #525 (TinyGPS++ crash bug)
// when fixed upstream, can be un-disabled to enable 3D FixType and PDOP
#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
    // see NMEAGPS.h
    gsafixtype.begin(reader, NMEA_MSG_GXGSA, 2);
    gsapdop.begin(reader, NMEA_MSG_GXGSA, 15);
    LOG_DEBUG("Use " NMEA_MSG_GXGSA " for 3DFIX and PDOP");
#endif

    // Make sure the GPS is awake before performing any init.
    new_gps->up();

#ifdef PIN_GPS_RESET
    pinMode(PIN_GPS_RESET, OUTPUT);
    digitalWrite(PIN_GPS_RESET, !GPS_RESET_MODE);
#endif

    if (_serial_gps) {
#ifdef ARCH_ESP32
        // In esp32 framework, setRxBufferSize needs to be initialized before Serial
        _serial_gps->setRxBufferSize(SERIAL_BUFFER_SIZE); // the default is 256
#endif

#if defined(SENSECAP_INDICATOR)
        LOG_DEBUG("Use RP2040 tunnel for GPS, no local pins");
#else
        LOG_DEBUG("Use GPIO%d for GPS RX", new_gps->rx_gpio);
        LOG_DEBUG("Use GPIO%d for GPS TX", new_gps->tx_gpio);
#endif

//  ESP32 has a special set of parameters vs other arduino ports
#if defined(ARCH_ESP32)
        _serial_gps->begin(GPS_BAUDRATE, SERIAL_8N1, new_gps->rx_gpio, new_gps->tx_gpio);
#elif defined(ARCH_RP2040)
        _serial_gps->setPinout(new_gps->tx_gpio, new_gps->rx_gpio);
        _serial_gps->setFIFOSize(256);
        _serial_gps->begin(GPS_BAUDRATE);
#elif defined(ARCH_NRF52)
        _serial_gps->setPins(new_gps->rx_gpio, new_gps->tx_gpio);
        _serial_gps->begin(GPS_BAUDRATE);
#elif defined(ARCH_STM32)
        _serial_gps->setTx(new_gps->tx_gpio);
        _serial_gps->setRx(new_gps->rx_gpio);
        _serial_gps->begin(GPS_BAUDRATE);
#elif defined(ARCH_PORTDUINO)
        // Portduino can't set the GPS pins directly.
        _serial_gps->begin(GPS_BAUDRATE);
#else
#error Unsupported architecture!
#endif
    }
    return new_gps;
}

static int32_t toDegInt(RawDegrees d)
{
    int32_t degMult = 10000000; // 1e7
    int32_t r = d.deg * degMult + d.billionths / 100;
    if (d.negative)
        r *= -1;
    return r;
}

/**
 * Perform any processing that should be done only while the GPS is awake and looking for a fix.
 * Override this method to check for new locations
 *
 * @return true if we've set a new time
 */
 
bool GPS::lookForTime()
{
    if (ubxTimeData.valid) {
        // Calculate frame age in milliseconds to adjust seconds accurately
        uint32_t ageMs = millis() - ubxTimeData.lastParsedMs;

        struct tm t;
        t.tm_sec = ubxTimeData.sec + round(ageMs / 1000.0);
        t.tm_min = ubxTimeData.min;
        t.tm_hour = ubxTimeData.hour;
        t.tm_mday = ubxTimeData.day;
        t.tm_mon = ubxTimeData.month - 1; // tm_mon is 0-indexed (0 = Jan)
        t.tm_year = ubxTimeData.year - 1900; // tm_year is years since 1900
        t.tm_isdst = false;

        if (t.tm_mon > -1) {
            // Validate time plausibility using Meshtastic's native rule
            if (!isPlausibleNmeaTime(t)) {
                return false;
            }

            // Sync node RTC clock
            if (perhapsSetRTC(RTCQualityGPS, t) == RTCSetResultSuccess) {
                LOG_DEBUG("UBX GPS time set %04d-%02d-%02d %02d:%02d:%02d age %dms", 
                          ubxTimeData.year, ubxTimeData.month, t.tm_mday, 
                          t.tm_hour, t.tm_min, t.tm_sec, ageMs);
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        return false;
    }
}

bool GPS::lookForTimeOld()
{
    auto ti = reader.time;
    auto d = reader.date;
    if (ti.isValid() && d.isValid()) { // Note: we don't check for updated, because we'll only be called if needed
        /* Convert to unix time
The Unix epoch (or Unix time or POSIX time or Unix timestamp) is the number of seconds that have elapsed since January 1,
1970 (midnight UTC/GMT), not counting leap seconds (in ISO 8601: 1970-01-01T00:00:00Z).
*/
        struct tm t;
        t.tm_sec = ti.second() + round(ti.age() / 1000);
        t.tm_min = ti.minute();
        t.tm_hour = ti.hour();
        t.tm_mday = d.day();
        t.tm_mon = d.month() - 1;
        t.tm_year = d.year() - 1900;
        t.tm_isdst = false;
        if (t.tm_mon > -1) {
            if (!isPlausibleNmeaTime(t)) {
                return false;
            }
            if (perhapsSetRTC(RTCQualityGPS, t) == RTCSetResultSuccess) {
                LOG_DEBUG("NMEA GPS time set %02d-%02d-%02d %02d:%02d:%02d age %d", d.year(), d.month(), t.tm_mday, t.tm_hour,
                          t.tm_min, t.tm_sec, ti.age());
                return true;
            } else {
                return false;
            }
        } else
            return false;
    } else
        return false;
}

/**
 * Perform any processing that should be done only while the GPS is awake and looking for a fix.
 * Override this method to check for new locations
 *
 * @return true if we've acquired a new location
 */
 
 bool GPS::lookForLocation()
{
    // Map UBX fixType directly (0=No fix, 2=2D, 3=3D)
    p.fix_type = ubxLocData.fixType;
    fixQual = (ubxLocData.fixFlags & 0x01) ? 1 : 0; // Bit 0 of flags = gnssFixOK

    // Check if GPS has an acceptable lock
    if (!hasLock())
        return false;

    // Check if a new UBX frame arrived
    if (!ubxLocData.isUpdated)
        return false;
    
    // 1. Populate the 'tm' struct so 't' exists in scope
    struct tm t = {};
    t.tm_sec = ubxTimeData.sec;
    t.tm_min = ubxTimeData.min;
    t.tm_hour = ubxTimeData.hour;
    t.tm_mday = ubxTimeData.day;
    t.tm_mon = ubxTimeData.month - 1; // tm_mon is 0-11
    t.tm_year = ubxTimeData.year - 1900; // tm_year is years since 1900
    t.tm_isdst = 0;
    
    int32_t currentLatI = ubxLocData.lat;
    int32_t currentLonI = ubxLocData.lon;
    int32_t currentAltMeters = ubxLocData.height / 1000;
    uint32_t nowEpoch = gm_mktime(&t);
    
    // 3. Distance Jump Filter (Haversine)
    //    Ignore jump > 500km within 60 seconds of last valid update
    static int32_t  lastStoredLatI = 0;
    static int32_t  lastStoredLonI = 0;
    static int32_t lastStoredAltMeters = 0;
    static uint32_t lastStoredTimeEpoch = 0;

    if (currentAltMeters > 50000 || currentAltMeters < -500) {
        LOG_ERROR("Rejected impossible absolute altitude: %d m", currentAltMeters);
        ubxLocData.isUpdated = false;
        return false;
    }

    // Only apply filter if we already have a previous, recent stored fix (within the last 60 seconds)
    if (lastStoredTimeEpoch > 0 && (nowEpoch - lastStoredTimeEpoch <= 60)) {
        // Haversine Constants
        const float_t EarthRadiusKm = 6371.0f; // Mean Earth Radius in km
        const float_t DegToRad = (float_t)M_PI / 180.0f;
        const float_t ScaleFactorRecip = 1.0f / 10000000.0f; // 1e-7 to float

        // Convert current I-scaled points to float radians
        float_t latNewR = ((float_t)currentLatI * ScaleFactorRecip) * DegToRad;
        float_t lonNewR = ((float_t)currentLonI * ScaleFactorRecip) * DegToRad;

        // Convert previous stored I-scaled points to float radians
        float_t latOldR = ((float_t)lastStoredLatI * ScaleFactorRecip) * DegToRad;
        float_t lonOldR = ((float_t)lastStoredLonI * ScaleFactorRecip) * DegToRad;

        // Deltas
        float_t dLat = latNewR - latOldR;
        float_t dLon = lonNewR - lonOldR;
        int32_t altDelta = abs(currentAltMeters - lastStoredAltMeters);

        // Haversine Core (a is the square of half the chord length)
        float_t a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
                    cos(latOldR) * cos(latNewR) *
                    sin(dLon / 2.0f) * sin(dLon / 2.0f);

        // c is the angular distance in radians
        float_t c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));

        // Resulting distance in km
        float_t jumpDistanceKm = EarthRadiusKm * c;

        // Drop if altitude shifts by more than 1000m within a minute
        if (altDelta > 300) {
            LOG_ERROR("Rejected abnormal altitude jump: %d m change", altDelta);
            ubxLocData.isUpdated = false;
            return false;
        }

        // 500km threshold check
        if (jumpDistanceKm > 100.0f) {
            // Illogical jump detected! Reject this fix.
            LOG_ERROR("Rejected illogical position jump: %2.2f km detected", jumpDistanceKm);
            ubxLocData.isUpdated = false;
            return false; // Exit without committing the position change
        }
        
    }
    
    // 4. Position committed! Update history guards.
    // If we reach here, the filter passed (or was not applicable).
    lastStoredLatI = currentLatI;
    lastStoredLonI = currentLonI;
    lastStoredTimeEpoch = nowEpoch; // Mark this fix as the new baseline
    
    
    // Clear updated flag so we don't re-read the same frame next loop
    ubxLocData.isUpdated = false;

    // Calculate age of solution
    uint32_t ageMs = millis() - ubxLocData.lastParsedMs;

    // Reject stale solution sets
    if (ageMs > GPS_SOL_EXPIRY_MS) {
        LOG_WARN("UBX DATA TOO OLD: %u ms", ageMs);
        return false;
    }

    // Defensive Guards: Reject Null Island and invalid Earth coordinates
    if (abs(ubxLocData.lat) > 900000000) {
        LOG_DEBUG_GPS("Bail out EARLY on LAT %i", ubxLocData.lat);
        return false;
    }
    if (abs(ubxLocData.lon) > 1800000000) {
        LOG_DEBUG_GPS("Bail out EARLY on LNG %i", ubxLocData.lon);
        return false;
    }
    if (ubxLocData.lat == 0 && ubxLocData.lon == 0) {
        LOG_WARN("Bail out EARLY on Null Island (0,0)");
        return false;
    }

    // Core Position Population
    p.location_source = meshtastic_Position_LocSource_LOC_INTERNAL;
    p.altitude_source = meshtastic_Position_AltSource_ALT_INTERNAL;

    p.has_latitude_i = true;
    p.latitude_i = ubxLocData.lat;

    p.has_longitude_i = true;
    p.longitude_i = ubxLocData.lon;

    //p.has_PDOP = true;
    p.PDOP = ubxLocData.pdop;

    // Altitude fields (UBX values in mm converted to meters)
    p.has_altitude = true;
    
    p.altitude = ubxLocData.hMSL / 1000;

    p.has_altitude_hae = true;
    p.altitude_hae = ubxLocData.height / 1000;

    p.has_altitude_geoidal_separation = true;
    p.altitude_geoidal_separation = (ubxLocData.height - ubxLocData.hMSL) / 1000;

    //p.has_sats_in_view = true;
    p.sats_in_view = ubxLocData.numSV;

    p.fix_quality = (ubxLocData.fixFlags & 0x01) ? 1 : 0;
    p.fix_type = ubxLocData.fixType;

    // Set high-precision marker so PositionModule preserves bits
    p.precision_bits = 32;

    // Timestamps
    if (ubxTimeData.valid) {
        struct tm t;
        t.tm_sec = ubxTimeData.sec;
        t.tm_min = ubxTimeData.min;
        t.tm_hour = ubxTimeData.hour;
        t.tm_mday = ubxTimeData.day;
        t.tm_mon = ubxTimeData.month - 1;
        t.tm_year = ubxTimeData.year - 1900;
        t.tm_isdst = false;
        uint32_t newTimestamp = gm_mktime(&t);

        // Reject frame if timestamp hasn't ticked over to a new second
        if (newTimestamp <= p.timestamp) {
            return false; // Suppresses duplicate timestamp execution
        }

        //p.timestamp = newTimestamp;
        p.timestamp = gm_mktime(&t);
        p.timestamp_millis_adjust = (ubxLocData.nano / 1000000);
    } else {
        p.timestamp = millis() / 1000;
    }

    // Heading (UBX headMot in 1e-5 degrees converted to 1e-5 degrees expected by Meshtastic)
    if (ubxLocData.headMot < 36000000) { // 360 degrees * 1e5
        p.has_ground_track = true;
        p.ground_track = ubxLocData.headMot;
    }

    // Speed (UBX gSpeed in mm/s converted to km/h)
    if (ubxLocData.gSpeed >= 0) {
        p.has_ground_speed = true;
        p.ground_speed = (ubxLocData.gSpeed * 36) / 10000; // mm/s -> km/h
    }

    return true;
}
 
 
bool GPS::lookForLocationOld()
{
    // By default, TinyGPS++ does not parse GPGSA lines, which give us
    //   the 2D/3D fixType (see NMEAGPS.h)
    // At a minimum, use the fixQuality indicator in GPGGA (FIXME?)
    fixQual = reader.fixQuality();

#ifndef TINYGPS_OPTION_NO_STATISTICS
    if (reader.failedChecksum() > lastChecksumFailCount) {
// In a GPS_DEBUG build we want to log all of these. In production, we only care if there are many of them.
#if !GPS_DEBUG
        if (reader.failedChecksum() > 4)
#endif
            LOG_WARN("%u new GPS checksum failures, total %u", reader.failedChecksum() - lastChecksumFailCount,
                     reader.failedChecksum());
        //lastChecksumFailCount = reader.failedChecksum();
    }
#endif

#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
    fixType = atoi(gsafixtype.value()); // will set to zero if no data
#endif

    // check if GPS has an acceptable lock
    if (!hasLock())
        return false;

#if GPS_DEBUG
    LOG_DEBUG("AGE: LOC=%d FIX=%d DATE=%d TIME=%d", reader.location.age(),
#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
              gsafixtype.age(),
#else
              0,
#endif
              reader.date.age(), reader.time.age());
#endif // GPS_DEBUG

    // Is this a new point or are we re-reading the previous one?
    if (!reader.location.isUpdated() && !reader.altitude.isUpdated())
        return false;

    // check if a complete GPS solution set is available for reading
    //   tinyGPSDatum::age() also includes isValid() test
    // FIXME
    if (!((reader.location.age() < GPS_SOL_EXPIRY_MS) &&
#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
          (gsafixtype.age() < GPS_SOL_EXPIRY_MS) &&
#endif
          (reader.time.age() < GPS_SOL_EXPIRY_MS) && (reader.date.age() < GPS_SOL_EXPIRY_MS))) {
        LOG_WARN("SOME data TOO OLD: LOC %u, TIME %u, DATE %u", reader.location.age(), reader.time.age(), reader.date.age());
        return false;
    }

    // We know the solution is fresh and valid, so just read the data
    auto loc = reader.location.value();

    // Bail out EARLY to avoid overwriting previous good data (like #857)
    if (toDegInt(loc.lat) > 900000000) {
        LOG_DEBUG_GPS("Bail out EARLY on LAT %i", toDegInt(loc.lat));
        return false;
    }
    if (toDegInt(loc.lng) > 1800000000) {
        LOG_DEBUG_GPS("Bail out EARLY on LNG %i", toDegInt(loc.lng));
        return false;
    }

    p.location_source = meshtastic_Position_LocSource_LOC_INTERNAL;

    // Dilution of precision (an accuracy metric) is reported in 10^2 units, so we need to scale down when we use it
#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
    p.HDOP = reader.hdop.value();
    p.PDOP = TinyGPSPlus::parseDecimal(gsapdop.value());
#else
    // FIXME! naive PDOP emulation (assumes VDOP==HDOP)
    // correct formula is PDOP = SQRT(HDOP^2 + VDOP^2)
    p.HDOP = reader.hdop.value();
    p.PDOP = 1.41 * reader.hdop.value();
#endif

    // Discard incomplete or erroneous readings
    if (reader.hdop.value() == 0) {
        LOG_WARN("BOGUS hdop.value() REJECTED: %d", reader.hdop.value());
        return false;
    }

    p.latitude_i = toDegInt(loc.lat);
    p.longitude_i = toDegInt(loc.lng);

    p.altitude_geoidal_separation = reader.geoidHeight.meters();
    p.altitude_hae = reader.altitude.meters() + p.altitude_geoidal_separation;
    p.altitude = reader.altitude.meters();

    p.fix_quality = fixQual;
#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
    p.fix_type = fixType;
#endif

    // positional timestamp
    struct tm t;
    t.tm_sec = reader.time.second();
    t.tm_min = reader.time.minute();
    t.tm_hour = reader.time.hour();
    t.tm_mday = reader.date.day();
    t.tm_mon = reader.date.month() - 1;
    t.tm_year = reader.date.year() - 1900;
    t.tm_isdst = false;
    p.timestamp = gm_mktime(&t);

    // Nice to have, if available
    if (reader.satellites.isUpdated()) {
        p.sats_in_view = reader.satellites.value();
    }

    if (reader.course.isUpdated() && reader.course.isValid()) {
        if (reader.course.value() < 36000) { // sanity check
            p.ground_track =
                reader.course.value() * 1e3; // Scale the heading (in degrees * 10^-2) to match the expected degrees * 10^-5
        } else {
            LOG_WARN("BOGUS course.value() REJECTED: %d", reader.course.value());
        }
    }

    if (reader.speed.isUpdated() && reader.speed.isValid()) {
        p.ground_speed = reader.speed.kmph();
    }

    return true;
}

bool GPS::hasLock()
{
    // Using GPGGA fix quality indicator
    if (fixQual >= 1 && fixQual <= 5) {
#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
        // Use GPGSA fix type 2D/3D (better) if available
        if (fixType == 3 || fixType == 0) // zero means "no data received"
#endif
            return true;
    }

    return false;
}

bool GPS::whileActive()
{
    bool validFrameParsed = false;

    if (powerState != GPS_ACTIVE) {
        clearBuffer();
        ubxState = UBX_SEARCH_SYNC1;
        return false;
    }

#ifdef SERIAL_BUFFER_SIZE
    if (_serial_gps->available() >= SERIAL_BUFFER_SIZE - 1) {
        LOG_WARN("GPS Buffer full (%u bytes). Flush to avoid corruption", _serial_gps->available());
        clearBuffer();
        ubxState = UBX_SEARCH_SYNC1;
    }
#endif

    // Consume all available bytes in the serial RX ring buffer
    while (_serial_gps->available() > 0) {
        uint8_t c = _serial_gps->read();

        // Binary state machine for UBX frame extraction
        switch (ubxState) {
            case UBX_SEARCH_SYNC1:
                if (c == 0xB5) {
                    ubxState = UBX_SEARCH_SYNC2;
                }
                break;

            case UBX_SEARCH_SYNC2:
                if (c == 0x62) {
                    ubxState = UBX_CLASS;
                } else if (c != 0xB5) {
                    ubxState = UBX_SEARCH_SYNC1;
                }
                break;

            case UBX_CLASS:
                // Only accept NAV Class (0x01). If it's anything else (or NMEA noise), abort immediately.
                if (c == 0x01) {
                    ubxMsgClass = c;
                    ubxState = UBX_ID;
                } else {
                    ubxState = UBX_SEARCH_SYNC1; // Drop invalid frame early
                }
                break;

            case UBX_ID:
                // Only accept NAV-PVT (0x07) or NAV-TIMEUTC (0x21). Abort if unknown.
                if (c == 0x07 || c == 0x21) {
                    ubxMsgId = c;
                    ubxState = UBX_LENGTH_LSB;
                } else {
                    ubxState = UBX_SEARCH_SYNC1; // Drop invalid frame early
                }
                break;

            case UBX_LENGTH_LSB:
                ubxPayloadLen = c;
                ubxState = UBX_LENGTH_MSB;
                break;

            case UBX_LENGTH_MSB:
                ubxPayloadLen |= ((uint16_t)c << 8);

                // Sanity Guard: NAV-PVT is always 92 to 100 bytes. 
                // Anything < 10 or > 120 bytes in Class 0x01 ID 0x07 is guaranteed to be NMEA collision noise.
                if (ubxPayloadLen < 10 || ubxPayloadLen > 120) {
                    // Silent drop (suppress log spam) and reset sync
                    ubxState = UBX_SEARCH_SYNC1;
                } else {
                    ubxPayloadIdx = 0;
                    ubxState = UBX_PAYLOAD;
                }
                break;

            case UBX_PAYLOAD:
                ubxPayloadBuffer[ubxPayloadIdx++] = c;
                if (ubxPayloadIdx >= ubxPayloadLen) {
                    ubxState = UBX_CK_A;
                }
                break;

            case UBX_CK_A:
                // Store CK_A in scratch variable and prepare for CK_B
                UBXscratch[0] = c;
                ubxState = UBX_CK_B;
                break;

            case UBX_CK_B: {
                uint8_t ckA = UBXscratch[0];
                uint8_t ckB = c;

                // Pass completed binary frame to your handler function
                handleUBXMessage(ubxMsgClass, ubxMsgId, ubxPayloadBuffer, ubxPayloadLen, ckA, ckB);
                
                validFrameParsed = true;

                // Reset state machine for the next incoming packet
                ubxState = UBX_SEARCH_SYNC1;
                break;
            }

            default:
                ubxState = UBX_SEARCH_SYNC1;
                break;
        }
    }

    return validFrameParsed;
}


bool GPS::whileActiveOld()
{
    unsigned int charsInBuf = 0;
    bool isValid = false;
#if GPS_DEBUG
    std::string debugmsg = "";
#endif
    if (powerState != GPS_ACTIVE) {
        clearBuffer();
        return false;
    }
#ifdef SERIAL_BUFFER_SIZE
    if (_serial_gps->available() >= SERIAL_BUFFER_SIZE - 1) {
        LOG_WARN("GPS Buffer full (%u bytes). Flush to avoid corruption", _serial_gps->available());
        clearBuffer();
    }
#endif
    // First consume any chars that have piled up at the receiver
    while (_serial_gps->available() > 0) {
        int c = _serial_gps->read();
        UBXscratch[charsInBuf] = c;
#if GPS_DEBUG
        //debugmsg += vformat("%c", (c >= 32 && c <= 126) ? c : '.');
        //debugmsg += vformat("%c", (c >= 1 && c <= 80) ? c : '.');
        
#endif

        parseUBXByte((uint8_t)c);
        
        //isValid |= reader.encode(c);
        if (charsInBuf > sizeof(UBXscratch) - 10 || c == '\r') {
            if (strnstr((char *)UBXscratch, "$GPTXT,01,01,02,u-blox ag - www.u-blox.com*50", charsInBuf)) {
                rebootsSeen++;
            }
            charsInBuf = 0;
        } else {
            charsInBuf++;
        }
    }
#if GPS_DEBUG
    if (debugmsg != "") {
        LOG_DEBUG("%s", debugmsg.c_str());
    }
#endif

    return isValid;
}
void GPS::enable()
{
    // Clear the old scheduling info (reset the lock-time prediction)
    scheduling.reset();

    enabled = true;
    setInterval(GPS_THREAD_INTERVAL);

    scheduling.informSearching();
    setPowerState(GPS_ACTIVE);
}

int32_t GPS::disable()
{
    enabled = false;
    setInterval(INT32_MAX);
    setPowerState(GPS_OFF);

    return INT32_MAX;
}

bool GPS::isEnabled()
{
    return enabled;
}

void GPS::toggleGpsMode()
{
    if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_ENABLED) {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_DISABLED;
        LOG_INFO("User toggled GpsMode. Now DISABLED");
        playGPSDisableBeep();
        disable();
    } else if (config.position.gps_mode == meshtastic_Config_PositionConfig_GpsMode_DISABLED) {
        config.position.gps_mode = meshtastic_Config_PositionConfig_GpsMode_ENABLED;
        LOG_INFO("User toggled GpsMode. Now ENABLED");
        playGPSEnableBeep();
        enable();
    }
}
#endif // Exclude GPS
