#pragma once
#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_GPS

#include <memory>

#include "GPSStatus.h"
#include "GpioLogic.h"
#include "Observer.h"
#include "TinyGPS++.h"
#include "concurrency/OSThread.h"
#include "input/RotaryEncoderInterruptImpl1.h"
#include "input/UpDownInterruptImpl1.h"
#include "modules/PositionModule.h"

#ifdef SENSECAP_INDICATOR
#include "mesh/comms/UARTProxy.h"
#endif

// Allow defining the polarity of the ENABLE output.  default is active high
#ifndef GPS_EN_ACTIVE
#define GPS_EN_ACTIVE 1
#endif

// Allow defining the polarity of the STANDBY output.  default is LOW for standby
#ifndef GPS_STANDBY_ACTIVE
#define GPS_STANDBY_ACTIVE LOW
#endif

// Allow defining the polarity of an external GPS RF front-end enable. Default is active high.
#ifndef GPS_RF_EN_ACTIVE
#define GPS_RF_EN_ACTIVE HIGH
#endif

// UBX-NAV-PVT Payload (Class 0x01, ID 0x07 - 92 bytes)
#pragma pack(push, 1)
struct UBX_NAV_PVT_payload {
    uint32_t iTOW;       // GPS time of week (ms)
    uint16_t year;       // Year (UTC)
    uint8_t  month;      // Month (UTC)
    uint8_t  day;        // Day (UTC)
    uint8_t  hour;       // Hour (UTC)
    uint8_t  min;        // Minute (UTC)
    uint8_t  sec;        // Second (UTC)
    uint8_t  valid;      // Validity flags
    uint32_t tAcc;       // Time accuracy estimate (ns)
    int32_t  nano;       // Fraction of second (ns)
    uint8_t  fixType;    // GNSSfix Type: 0=No fix, 2=2D, 3=3D, etc.
    uint8_t  flags;      // Fix status flags
    uint8_t  flags2;     // Additional flags
    uint8_t  numSV;      // Number of satellites used
    int32_t  lon;        // Longitude (1e-7 deg)
    int32_t  lat;        // Latitude (1e-7 deg)
    int32_t  height;     // Height above ellipsoid (mm)
    int32_t  hMSL;       // Height above mean sea level (mm)
    uint32_t hAcc;       // Horizontal accuracy estimate (mm)
    uint32_t vAcc;       // Vertical accuracy estimate (mm)
    int32_t  velN;       // NED north velocity (mm/s)
    int32_t  velE;       // NED east velocity (mm/s)
    int32_t  velD;       // NED down velocity (mm/s)
    int32_t  gSpeed;     // Ground Speed (mm/s)
    int32_t  headMot;    // Heading of motion (1e-5 deg)
    uint32_t sAcc;       // Speed accuracy estimate (mm/s)
    uint32_t headAcc;    // Heading accuracy estimate (1e-5 deg)
    uint16_t pDOP;       // Position DOP (0.01 land)
    uint8_t  flags3;     // Additional flags
    uint8_t  reserved1[5];
    int32_t  headVeh;    // Heading of vehicle (1e-5 deg)
    int16_t  magDec;     // Magnetic declination (1e-2 deg)
    uint16_t magAcc;     // Magnetic declination accuracy (1e-2 deg)
    // High Precision Fields (Scale: deg * 1e-9)
    int8_t   latHp;      // Lat high-precision offset
    int8_t   lonHp;      // Lon high-precision offset
    int8_t   heightHp;   // Height offset (0.1 mm)
    int8_t   hMSLHp;     // hMSL offset (0.1 mm)
    uint32_t reserved2;  // Trailing payload alignment
};
#pragma pack(pop)

// UBX Parser State Machine States
enum UBXState {
    PREAMBLE_1,  // Expect 0xB5
    PREAMBLE_2,  // Expect 0x62
    CLASS,       // Message Class
    ID,          // Message ID
    LENGTH_1,    // Length low byte
    LENGTH_2,    // Length high byte
    PAYLOAD,     // Reading Payload
    CK_A,        // Checksum A
    CK_B         // Checksum B
};


// Compile receiver-specific sleep support when the board exposes the required wake controls.
#if defined(GPS_RTC_INT)
#define HAS_AIROHA_SOFT_RTC 1
#endif
// Without RTC_INT, a board whose RESETB is wired can still use backup mode and reset out of it.
#if !defined(HAS_AIROHA_SOFT_RTC) && defined(GPS_AIROHA_RESET_WAKE) && defined(PIN_GPS_RESET) && defined(GPS_SLEEP_INT) &&       \
    defined(PIN_GPS_EN)
#define HAS_AIROHA_RESET_WAKE 1
#endif
// Without RTC_INT only the GNSS engine is stopped; powering EN back up reboots the core on its own.
#if defined(HAS_AIROHA_SOFT_RTC) || (defined(GPS_SLEEP_INT) && defined(PIN_GPS_EN))
#define HAS_AIROHA_SLEEP 1
#endif


static constexpr uint32_t GPS_UPDATE_ALWAYS_ON_THRESHOLD_MS = 10 * 1000UL;
static constexpr uint32_t GPS_FIX_HOLD_MAX_MS = 20000;

typedef enum {
    GNSS_MODEL_ATGM336H,
    GNSS_MODEL_MTK,
    GNSS_MODEL_UBLOX6,
    GNSS_MODEL_UBLOX7,
    GNSS_MODEL_UBLOX8,
    GNSS_MODEL_UBLOX9,
    GNSS_MODEL_UBLOX10,
    GNSS_MODEL_UC6580,
    GNSS_MODEL_UNKNOWN,
    GNSS_MODEL_MTK_L76B,
    GNSS_MODEL_MTK_PA1010D,
    GNSS_MODEL_MTK_PA1616S,
    GNSS_MODEL_AG3335,
    GNSS_MODEL_AG3352,
    GNSS_MODEL_LS20031,
    GNSS_MODEL_CM121,
    GNSS_MODEL_LC760CA,
    // Keep GNSS_MODEL_GENERIC_NMEA last: isValidGnssModel() uses it as the exclusive upper bound
    // for values the probe cache is allowed to hold.
    GNSS_MODEL_GENERIC_NMEA // generic NMEA source (e.g. gpsd); skips chip-specific probe and init
} GnssModel_t;

typedef enum {
    GNSS_RESPONSE_NONE,
    GNSS_RESPONSE_NAK,
    GNSS_RESPONSE_FRAME_ERRORS,
    GNSS_RESPONSE_OK,
} GPS_RESPONSE;

enum GPSPowerState : uint8_t {
    GPS_ACTIVE,    // Awake and want a position
    GPS_IDLE,      // Awake, but not wanting another position yet
    GPS_SOFTSLEEP, // Physically powered on, but soft-sleeping
    GPS_HARDSLEEP, // Physically powered off, but scheduled to wake
    GPS_OFF        // Powered off indefinitely
};

struct ChipInfo {
    String chipName;        // The name of the chip (for logging)
    String detectionString; // The string to match in the response
    GnssModel_t driver;     // The driver to use
};
/**
 * A gps class that only reads from the GPS periodically and keeps the gps powered down except when reading
 *
 * When new data is available it will notify observers.
 */
class GPS : private concurrency::OSThread
{
  public:
    meshtastic_Position p = meshtastic_Position_init_default;
    

    /** This is normally bound to config.position.gps_en_gpio but some rare boards (like heltec tracker) need more advanced
     * implementations. Those boards will set this public variable to a custom implementation.
     *
     * Normally set by GPS::createGPS()
     */
    GpioVirtPin *enablePin = NULL;

    virtual ~GPS();

    /** We will notify this observable anytime GPS state has changed meaningfully */
    Observable<const meshtastic::GPSStatus *> newStatus;

    /**
     * Returns true if we succeeded
     */
    virtual bool setup();

    // re-enable the thread
    void enable();

    // Disable the thread
    int32_t disable() override;

    // Returns if the thread is enabled
    bool isEnabled();

    // toggle between enabled/disabled
    void toggleGpsMode();

    // Change the power state of the GPS - for power saving / shutdown
    void setPowerState(GPSPowerState newState, uint32_t sleepMs = 0);

    /// Returns true if we have acquired GPS lock.
    virtual bool hasLock();

    /// Return true if we are connected to a GPS
    bool isConnected() const { return hasGPS; }

    bool isPowerSaving() const { return config.position.gps_mode != meshtastic_Config_PositionConfig_GpsMode_ENABLED; }

    // Empty the input buffer as quickly as possible
    void clearBuffer();

    // Creates an instance of the GPS class.
    // Returns the new instance or null if the GPS is not present.
    static std::unique_ptr<GPS> createGps();

    // Wake the GPS hardware - ready for an update
    void up();

    // Let the GPS hardware save power between updates
    void down();

  private:
  
   void handleUBXMessage(uint8_t msgClass, uint8_t msgId, uint8_t *payload, uint16_t len, uint8_t ckA, uint8_t ckB);
  
    void handleUBXMessageOld(uint8_t msgClass, uint8_t msgId, uint8_t *payload, uint16_t len);
    void parseUBXByte(uint8_t b);
        //adde for UBX replacement
    meshtastic_Position ubxPos = meshtastic_Position_init_default;
    bool newUbxFrameAvailable = false;
    
    GPS() : concurrency::OSThread("GPS") {}

    /// Record that we have a GPS
    void setConnected();

    /** Subclasses should look for serial rx characters here and feed it to their GPS parser
     *
     * Return true if we received a valid message from the GPS
     */
    virtual bool whileActive();
    virtual bool whileActiveOld();

    /**
     * Perform any processing that should be done only while the GPS is awake and looking for a fix.
     * Override this method to check for new locations
     *
     * @return true if we've acquired a time
     */
    virtual bool lookForTime();
    virtual bool lookForTimeOld();

    /**
     * Perform any processing that should be done only while the GPS is awake and looking for a fix.
     * Override this method to check for new locations
     *
     * @return true if we've acquired a new location
     */
    virtual bool lookForLocation();
    virtual bool lookForLocationOld();
   
    
    // Load persisted GPS model+baud from /prefs.
    bool loadProbeCache();
    // Clear persisted GPS model+baud cache.
    void clearProbeCache();
    // Persist the currently detected GPS model+baud.
    bool saveProbeCache() const;
    // Verify the cached model+baud still maps to a live GPS device.
    bool verifyCachedProbePresence();

    GnssModel_t gnssModel = GNSS_MODEL_UNKNOWN;
    int32_t detectedBaud = GPS_BAUDRATE;
    int32_t cachedProbeBaud = 0;
    GnssModel_t cachedProbeModel = GNSS_MODEL_UNKNOWN;

    TinyGPSPlus reader;
    uint8_t fixQual = 0; // fix quality from GPGGA
    uint32_t lastChecksumFailCount = 0;
    uint8_t currentStep = 0;
    int32_t currentDelay = 2000;
    bool gotTime = false;

    enum UbxParseState {
        UBX_SEARCH_SYNC1,
        UBX_SEARCH_SYNC2,
        UBX_CLASS,
        UBX_ID,
        UBX_LENGTH_LSB,
        UBX_LENGTH_MSB,
        UBX_PAYLOAD,
        UBX_CK_A,
        UBX_CK_B
    };

    UbxParseState ubxState = UBX_SEARCH_SYNC1;
    uint8_t ubxMsgClass = 0;
    uint8_t ubxMsgId = 0;
    uint16_t ubxPayloadLen = 0;
    uint16_t ubxPayloadIdx = 0;
    uint8_t ubxPayloadBuffer[128]; // Fits NAV-PVT (92-100+ bytes)



#ifndef TINYGPS_OPTION_NO_CUSTOM_FIELDS
    // (20210908) TinyGps++ can only read the GPGSA "FIX TYPE" field
    // via optional feature "custom fields", currently disabled (bug #525)
    TinyGPSCustom gsafixtype; // custom extract fix type from GPGSA
    TinyGPSCustom gsapdop;    // custom extract PDOP from GPGSA
    uint8_t fixType = 0;      // fix type from GPGSA
#endif

    uint32_t fixHoldEnds = 0;
    uint32_t rx_gpio = 0;
    uint32_t tx_gpio = 0;

    uint8_t speedSelect = 0;
    uint8_t probeTries = 0;
    // Cache file is successfully loaded.
    bool hasProbeCache = false;
    // Ensures cached probe is attempted once per boot.
    bool triedProbeCache = false;

    /**
     * hasValidLocation - indicates that the position variables contain a complete
     *   GPS location, valid and fresh (< gps_update_interval + position_broadcast_secs)
     */
    bool hasValidLocation = false; // default to false, until we complete our first read

    bool shouldPublish = false; // If we've changed GPS state, this will force a publish the next loop()

    bool hasGPS = false; // Do we have a GPS we are talking to

    bool GPSInitFinished = false; // Init thread finished?
    bool GPSInitStarted = false;  // Init thread finished?

    GPSPowerState powerState = GPS_OFF; // GPS_ACTIVE if we want a location right now
#ifdef HAS_AIROHA_SLEEP
    uint8_t airohaSleepMisses = 0; // consecutive sleep commands that went unacked
#endif

    uint8_t numSatellites = 0;

    CallbackObserver<GPS, void *> notifyDeepSleepObserver = CallbackObserver<GPS, void *>(this, &GPS::prepareDeepSleep);

    /** If !NULL we will use this serial port to construct our GPS */
#if defined(SENSECAP_INDICATOR)
    static UARTProxy *_serial_gps;
#elif defined(ARCH_RP2040)
    static SerialUART *_serial_gps;
#elif defined(ARCH_NRF52)
    static Uart *_serial_gps;
#else
    static HardwareSerial *_serial_gps;
#endif

    // Create a ublox packet for editing in memory
    uint8_t makeUBXPacket(uint8_t class_id, uint8_t msg_id, uint8_t payload_size, const uint8_t *msg);
    uint8_t makeCASPacket(uint8_t class_id, uint8_t msg_id, uint8_t payload_size, const uint8_t *msg);

    // scratch space for creating ublox packets
    uint8_t UBXscratch[250] = {0};

    int rebootsSeen = 0;

    int getACK(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID, uint32_t waitMillis);
    GPS_RESPONSE getACK(uint8_t c, uint8_t i, uint32_t waitMillis);
    GPS_RESPONSE getACK(const char *message, uint32_t waitMillis);

    GPS_RESPONSE getACKCas(uint8_t class_id, uint8_t msg_id, uint32_t waitMillis);

    /// Prepare the GPS for the cpu entering deep sleep, expect to be gone for at least 100s of msecs
    /// always returns 0 to indicate okay to sleep
    int prepareDeepSleep(void *unused);

    /** Set power with EN pin, if relevant
     */
    void writePinEN(bool on);

    /** Set the value of the STANDBY pin, if relevant
     */
    void writePinStandby(bool standby);

    /** Set the external RF front-end enable pin, if relevant
     */
    void writePinRFEN(bool on);

    /** Set GPS power with PMU, if relevant
     */
    void setPowerPMU(bool on);

    /** Set UBLOX power, if relevant
     */
    void setPowerUBLOX(bool on, uint32_t sleepMs = 0);

    /** Send an Airoha receiver its sleep command before the power cut, if relevant
     */
    void airohaEnterSleep();

    /**
     * Tell users we have new GPS readings
     */
    void publishUpdate();

    virtual int32_t runOnce() override;

    GnssModel_t getProbeResponse(unsigned long timeout, const std::vector<ChipInfo> &responseMap, int serialSpeed);

    // Get GNSS model
    GnssModel_t probe(int serialSpeed);

    // delay counter to allow more sats before fixed position stops GPS thread
    uint8_t fixeddelayCtr = 0;
};

extern std::unique_ptr<GPS> gps;
#endif // Exclude GPS
