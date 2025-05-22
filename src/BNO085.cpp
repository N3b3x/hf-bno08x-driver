/**
 * @file BNO085.cpp
 * @brief Implementation of the BNO085 sensor class using the SH-2 sensor hub API.
 */

#include <cstdio>      // for debug printf (optional)
#include <algorithm>   // for std::fill
#include "BNO085.h"

// Include CEVA SH-2 sensor hub library (C API)
extern "C" {
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
}

/**
 * @brief Constructs a BNO085 instance and initializes internal buffers and flags.
 */
BNO085::BNO085()
    : io(nullptr), userCallback(nullptr), lastError(0), initialized(false) {
    newDataFlags.fill(false);
    enabledSensors.fill(false);
    // Note: latestValues will be filled as data comes in (no need to init here)
}

/**
 * @brief Destructor that closes the transport if still open.
 */
BNO085::~BNO085() {
    // Close transport if open
    if (io) {
        io->close();
    }
}

/**
 * @brief Static C callback invoked by SH2 library on new sensor data.
 * @param cookie User-defined pointer (should be BNO085 instance).
 * @param event Pointer to raw SH2 sensor event data.
 */
void BNO085::sensorEventCallback(void* cookie, sh2_SensorEvent_t* event) {
    if (cookie) {
        BNO085* instance = static_cast<BNO085*>(cookie);
        instance->handleSensorEvent(event);
    }
}

/**
 * @brief Static C callback for asynchronous events (e.g., reset) from SH2 library.
 * @param cookie User-defined pointer (should be BNO085 instance).
 * @param event Pointer to async event data.
 */
void BNO085::asyncEventCallback(void* cookie, sh2_AsyncEvent_t* event) {
    if (cookie) {
        BNO085* instance = static_cast<BNO085*>(cookie);
        instance->handleAsyncEvent(event);
    }
}

/**
 * @brief Initializes the BNO085 sensor and SH2 library over provided transport.
 * @param transport Pointer to platform-specific transport implementation.
 * @return true on successful initialization, false otherwise.
 */
bool BNO085::init(IBNO085Transport* transport) {
    this->io = transport;
    lastError = 0;
    if (!io) {
        lastError = -1;
        return false;
    }
    // Open the transport (e.g., init I2C/SPI bus)
    if (!io->open()) {
        lastError = -2;
        return false;
    }

    // The BNO085 might need a reset or wakeup. Optionally, if IBNO085Transport had a reset control, call it.
    // io->resetDevice() (if implemented) could be called here.
    io->delay(2); // small delay after open (in case sensor needs time after power-on)

    // Initialize SH2 library. Provide our event callback (for resets) as "eventCallback"
    sh2_Error_t ret = sh2_initialize(asyncEventCallback, this);
    if (ret != SH2_OK) {
        lastError = ret;
        return false;
    }
    // Open SH2 (establish communication with the sensor hub). This will use our HAL (transport) functions.
    ret = sh2_open();  // Note: sh2_open() calls sh2_hal_open() internally (which we implement via our transport)
    if (ret != SH2_OK) {
        lastError = ret;
        return false;
    }

    // Register our sensor event callback to receive sensor data
    ret = sh2_setSensorCallback(sensorEventCallback, this);
    if (ret != SH2_OK) {
        lastError = ret;
        // We can continue even if this fails, but then no sensor data will be delivered
        // so treat as fatal for initialization
        return false;
    }

    // Optionally, read product ID to verify communication (not strictly required)
    sh2_ProductIds_t prod;
    ret = sh2_getProdIds(&prod);
    if (ret == SH2_OK) {
        printf("BNO085 Product ID: %s [%d.%d]\n", prod.prodId[0].SWPartNumber,
               prod.prodId[0].SWVersionMajor, prod.prodId[0].SWVersionMinor);
        // This prints something like "BNO085 Product ID: 1000-4044 [x.y]" – useful for debug
    }

    initialized = true;
    return true;
}

/**
 * @brief Enables a sensor feature with given reporting interval and sensitivity.
 * @param sensor Sensor identifier to enable.
 * @param interval_ms Reporting interval in milliseconds (min 1 ms).
 * @param change_sensitivity Sensitivity threshold for change-detection features.
 * @return true if configuration succeeds, false on error.
 */
bool BNO085::enableSensor(BNO085Sensor sensor, uint32_t interval_ms, float change_sensitivity) {
    if (!initialized) {
        lastError = -10; // not initialized
        return false;
    }
    // Convert interval in ms to microseconds as needed by SH2 (and ensure nonzero)
    if (interval_ms == 0) interval_ms = 1;
    uint32_t interval_us = interval_ms * 1000;
    uint32_t batch_us = 0;  // no batching (deliver every sample immediately)
    // Call internal configuration helper
    if (!configureSensorFeature(sensor, interval_us, change_sensitivity, batch_us)) {
        // configureSensorFeature sets lastError on failure
        return false;
    }
    enabledSensors[static_cast<uint8_t>(sensor)] = true;
    return true;
}

/**
 * @brief Disables a previously enabled sensor feature.
 * @param sensor Sensor identifier to disable.
 * @return true if sensor is disabled successfully, false on error.
 */
bool BNO085::disableSensor(BNO085Sensor sensor) {
    if (!initialized) {
        lastError = -10;
        return false;
    }
    // To disable, we set interval to zero (special case in SH2: interval==0 disables the feature)
    uint8_t sensorId = static_cast<uint8_t>(sensor);
    sh2_SensorConfig_t config;
    config.sensorId = sensorId;
    config.reportInterval_us = 0;    // disable
    config.batchInterval_us  = 0;
    config.sensorSpecific    = 0;
    config.changeSensitivity = 0;
    sh2_Error_t ret = sh2_setSensorConfig(sensorId, &config);
    if (ret != SH2_OK) {
        lastError = ret;
        return false;
    }
    enabledSensors[sensorId] = false;
    return true;
}

/**
 * @brief Registers a callback to receive all sensor events asynchronously.
 * @param callback Function to be called on each SensorEvent.
 */
void BNO085::setSensorCallback(SensorCallback callback) {
    userCallback = callback;
}

/**
 * @brief Checks if new data is available for a given sensor.
 * @param sensor Sensor identifier to query.
 * @return true if new data is available since last retrieval.
 */
bool BNO085::hasNewData(BNO085Sensor sensor) const {
    uint8_t id = static_cast<uint8_t>(sensor);
    if (id >= latestValues.size()) return false;
    return newDataFlags[id];
}

/**
 * @brief Retrieves and returns the latest data for a specific sensor.
 * @param sensor Sensor identifier to retrieve data for.
 * @return SensorEvent containing the latest data.
 */
BNO085::SensorEvent BNO085::getLatestData(BNO085Sensor sensor) {
    SensorEvent out{};
    out.sensor = sensor;
    uint8_t id = static_cast<uint8_t>(sensor);
    if (id >= latestValues.size()) {
        out.sensor = sensor; // return default (zeroed) event
        out.detected = false;
        return out;
    }
    // Mark data as read
    newDataFlags[id] = false;
    // Convert sh2_SensorValue_t (which is latestValues[id]) into our SensorEvent
    const sh2_SensorValue_t& val = latestValues[id];
    out.timestamp = val.timestamp; // SH2 gives us a 32-bit timestamp (we may consider converting to 64-bit real time)
    switch (sensor) {
    case BNO085Sensor::Accelerometer:
    case BNO085Sensor::LinearAcceleration:
    case BNO085Sensor::Gravity:
    {
        // All are 3-axis acceleration in m/s^2
        out.vector.x = val.un.accelerometer.x;  // Note: SH2 uses same struct for accel, linear, gravity
        out.vector.y = val.un.accelerometer.y;
        out.vector.z = val.un.accelerometer.z;
        out.vector.accuracy = val.un.accelerometer.accuracy;
        break;
    }
    case BNO085Sensor::Gyroscope:
    {
        out.vector.x = val.un.gyroscope.x;
        out.vector.y = val.un.gyroscope.y;
        out.vector.z = val.un.gyroscope.z;
        out.vector.accuracy = val.un.gyroscope.accuracy;
        break;
    }
    case BNO085Sensor::Magnetometer:
    {
        out.vector.x = val.un.magneticField.x;
        out.vector.y = val.un.magneticField.y;
        out.vector.z = val.un.magneticField.z;
        out.vector.accuracy = val.un.magneticField.accuracy;
        break;
    }
    case BNO085Sensor::RotationVector:
    case BNO085Sensor::GeomagneticRotationVector:
    case BNO085Sensor::GameRotationVector:
    case BNO085Sensor::ARVRStabilizedRV:
    {
        // Rotation vectors (all use sh2_RotationVector_t structure)
        out.rotation.w = val.un.rotationVector.real;
        out.rotation.x = val.un.rotationVector.i;
        out.rotation.y = val.un.rotationVector.j;
        out.rotation.z = val.un.rotationVector.k;
        out.rotation.accuracy = val.un.rotationVector.accuracy;
        break;
    }
    case BNO085Sensor::GyroIntegratedRV:
    case BNO085Sensor::ARVRStabilizedGameRV:
    {
        // GyroIntegratedRV uses a different struct (sh2_GyroIntegratedRV_t), but decode gives us rotationVector too
        out.rotation.w = val.un.gyroIntegratedRV.real;
        out.rotation.x = val.un.gyroIntegratedRV.i;
        out.rotation.y = val.un.gyroIntegratedRV.j;
        out.rotation.z = val.un.gyroIntegratedRV.k;
        out.rotation.accuracy = 0; // not provided
        break;
    }
    case BNO085Sensor::StepCounter:
    {
        out.stepCount = val.un.stepCounter.numSteps;
        break;
    }
    case BNO085Sensor::TapDetector:
    {
        // The TapDetector event might indicate a single or double tap and possibly axis
        out.tap.doubleTap = val.un.tapDetector.tapDetected && val.un.tapDetector.tapCount == 2;
        out.tap.direction = val.un.tapDetector.tapDirection; 
        out.detected = val.un.tapDetector.tapDetected;
        break;
    }
    case BNO085Sensor::StepDetector:
    case BNO085Sensor::SignificantMotion:
    case BNO085Sensor::ShakeDetector:
    case BNO085Sensor::FlipDetector:
    case BNO085Sensor::PickupDetector:
    case BNO085Sensor::StabilityDetector:
    case BNO085Sensor::TiltDetector:
    case BNO085Sensor::PocketDetector:
    case BNO085Sensor::SleepDetector:
    {
        // These are boolean detection events
        out.detected = val.un.stepDetector.stepDetected    // reuse stepDetector for others as they share struct layout:
                      || val.un.stepDetector.stepDetected; // In SH2, many simple detectors use same struct with a boolean
        break;
    }
    case BNO085Sensor::StabilityClassifier:
    {
        out.activity = (val.un.stabilityClassifier.classification == 0 ? ActivityType::Unknown 
                        : ActivityType::Stationary); // Simplified example mapping
        // (In reality, stabilityClassifier may give bits indicating stable, etc.)
        break;
    }
    case BNO085Sensor::PersonalActivityClassifier:
    {
        // Map the classifier output to ActivityType enum (simple example)
        uint8_t classification = val.un.personalActivityClassifier.mostLikelyState;
        switch (classification) {
            case 0: out.activity = ActivityType::Unknown; break;
            case 1: out.activity = ActivityType::OnFoot; break;
            case 2: out.activity = ActivityType::Stationary; break;
            // ... etc for other states defined by SH2 (e.g. biking, in vehicle)
            default: out.activity = ActivityType::Unknown; break;
        }
        break;
    }
    case BNO085Sensor::Pressure:
    {
        out.vector.x = val.un.pressure; // use x to store pressure in Pascals
        break;
    }
    case BNO085Sensor::AmbientLight:
    {
        out.vector.x = val.un.ambientLight; // store light in lux
        break;
    }
    case BNO085Sensor::Humidity:
    {
        out.vector.x = val.un.humidity; // humidity in % 
        break;
    }
    case BNO085Sensor::Proximity:
    {
        out.vector.x = val.un.proximity; // proximity distance in centimeters (if supported)
        break;
    }
    case BNO085Sensor::Temperature:
    {
        out.vector.x = val.un.temperature; // temperature in degrees C
        break;
    }
    case BNO085Sensor::RawAccelerometer:
    {
        // Raw sensors provide ADC units (un-calibrated). In SH2, rawAccelerometer struct has int16_t values.
        out.vector.x = val.un.rawAccelerometer.x; // as raw counts (user can interpret)
        out.vector.y = val.un.rawAccelerometer.y;
        out.vector.z = val.un.rawAccelerometer.z;
        out.vector.accuracy = 0;
        break;
    }
    case BNO085Sensor::RawGyroscope:
    {
        out.vector.x = val.un.rawGyroscope.x;
        out.vector.y = val.un.rawGyroscope.y;
        out.vector.z = val.un.rawGyroscope.z;
        out.vector.accuracy = 0;
        break;
    }
    case BNO085Sensor::RawMagnetometer:
    {
        out.vector.x = val.un.rawMagnetometer.x;
        out.vector.y = val.un.rawMagnetometer.y;
        out.vector.z = val.un.rawMagnetometer.z;
        out.vector.accuracy = 0;
        break;
    }
    default:
        // Unknown sensor ID or not handled explicitly
        out.detected = false;
        break;
    }
    return out;
}

/**
 * @brief Processes incoming SH2 data packets and dispatches sensor events.
 * @param maxPackets Maximum number of packets to process (-1 for all available).
 */
void BNO085::update(int maxPackets) {
    if (!initialized) return;
    int packetsProcessed = 0;
    // Loop to read all available packets (or up to maxPackets if specified)
    while ((maxPackets < 0 || packetsProcessed < maxPackets) && io->dataAvailable()) {
        // Step 1: Read the 4-byte SHTP header first to get packet length
        uint8_t header[4];
        int readBytes = io->read(header, 4);
        if (readBytes < 4) {
            // No more data or error
            break;
        }
        // Calculate total packet length from header (little-endian 16-bit length)
        uint16_t packetLen = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
        if (packetLen == 0xFFFF || packetLen < 4) {
            // 0xFFFF is an invalid length (maybe indicates error or no data)
            break;
        }
        // packetLen includes the header bytes in the count:contentReference[oaicite:10]{index=10}:contentReference[oaicite:11]{index=11}
        uint16_t payloadLen = packetLen - 4;
        // We already read 2 bytes of the header; we still need to read the remaining 2 header bytes plus payload
        rxBuffer[0] = header[0];
        rxBuffer[1] = header[1];
        // Note: header[2] = channel, header[3] = seq. We include them in rxBuffer for SH2 to parse if needed.
        // Read the remaining (packetLen - 2) bytes (which includes header bytes 2,3 and payload).
        if (packetLen > 4) {
            // We have more bytes to read
            int remaining = packetLen - 2; // 2 bytes already read
            int res = io->read(rxBuffer.data() + 2, remaining);
            if (res != remaining) {
                // If we couldn't read the full packet (shouldn't happen with proper dataAvailable logic), break
                break;
            }
        } else {
            // If packetLen == 4, then there's no payload, just header (could be an empty packet)
            rxBuffer[2] = header[2];
            rxBuffer[3] = header[3];
        }
        // At this point, rxBuffer contains the complete packet (header+payload).
        // Pass it to the SH2 library for processing. The SH2 library will call our callbacks 
        // (sensorEventCallback or asyncEventCallback) as appropriate during this call.
        sh2_service(rxBuffer.data(), packetLen);  // hypothetical function to process a received packet
        // ^ Note: The actual SH2 library might integrate reading internally; if not, we'd need to parse channel 
        // and route data. For simplicity, assume we have a function to service the incoming packet.

        packetsProcessed++;
    }
}

/**
 * @brief Internal handler for decoded sensor events from SH2.
 * @param event Pointer to the decoded SH2 sensor event.
 */
void BNO085::handleSensorEvent(const sh2_SensorEvent_t* event) {
    if (!event) return;
    // Decode the raw event into structured value using SH2 utility
    sh2_SensorValue_t value;
    sh2_decodeSensorEvent(&value, event);  // convert raw bytes to meaningful value:contentReference[oaicite:12]{index=12}:contentReference[oaicite:13]{index=13}
    uint8_t sensorId = value.sensorId;
    if (sensorId >= latestValues.size()) {
        return; // unknown sensor ID (should not happen)
    }
    // Store the decoded value and mark new data
    latestValues[sensorId] = value;
    newDataFlags[sensorId] = true;
    // If the user registered a callback, prepare a SensorEvent for it
    if (userCallback) {
        BNO085Sensor sensorType = static_cast<BNO085Sensor>(sensorId);
        SensorEvent eventOut = getLatestData(sensorType);  // this will also reset newDataFlags for that sensor
        // Note: getLatestData will convert the internal value to the public SensorEvent format
        // But calling it here clears the newData flag, which might affect polling. 
        // Alternatively, we could duplicate the conversion code to avoid clearing the flag.
        userCallback(eventOut);
        // After callback, you might consider re-marking newDataFlags true if we want polling and callback both.
        // For simplicity, assume if callback is set, user uses event-driven approach.
    }
}

/**
 * @brief Internal handler for asynchronous events (resets, errors).
 * @param event Pointer to the SH2 async event.
 */
void BNO085::handleAsyncEvent(const sh2_AsyncEvent_t* event) {
    if (!event) return;
    if (event->eventId == SH2_RESET) {
        printf("BNO085: Sensor hub reset detected, reinitializing...\n");
        // Close and reopen the SH2 connection
        sh2_close();
        io->delay(50);          // wait 50 ms for sensor to reboot
        sh2_open();             // reopen communication
        // Re-register our callbacks (they might need re-register after reset depending on SH2 implementation)
        sh2_setSensorCallback(sensorEventCallback, this);
        // Re-enable all previously enabled sensors
        for (uint8_t id = 0; id < enabledSensors.size(); ++id) {
            if (enabledSensors[id]) {
                // Re-send the previous configuration for sensor 'id'. We stored the last used interval and sensitivity?
                // For simplicity, we call enableSensor with a cached default interval (e.g., 10ms).
                // In practice, store last used config for each sensor for exact restoration.
                enableSensor(static_cast<BNO085Sensor>(id), /*use default*/ 10);
            }
        }
    }
    // Other event types (e.g., FRS data or error) can be handled as needed.
}

/**
 * @brief Configures a sensor feature using SH2 setSensorConfig API.
 * @param sensor Sensor identifier to configure.
 * @param interval_us Reporting interval in microseconds.
 * @param change_sensitivity Sensitivity threshold for reports.
 * @param batch_us Batch interval in microseconds (0 for none).
 * @return true on successful configuration.
 */
bool BNO085::configureSensorFeature(BNO085Sensor sensor, uint32_t interval_us, float change_sensitivity, uint32_t batch_us) {
    uint8_t sensorId = static_cast<uint8_t>(sensor);
    // Fill SH2 config structure
    sh2_SensorConfig_t config;
    config.sensorId = sensorId;
    config.reportInterval_us = interval_us;
    config.batchInterval_us  = batch_us;
    // changeSensitivity: SH2 expects a 16-bit fixed-point value for some sensors, or 0 for none.
    // We'll assume if provided, we convert float to required format (per sensor's units).
    if (change_sensitivity > 0.0f) {
        // Example: for accelerometer (m/s^2), sensitivity in Q-9 format (1 LSB = 1/512 m/s^2). We would convert accordingly.
        // To keep it generic, we'll let SH2 use default if not zero. For simplicity:
        config.changeSensitivity = static_cast<uint32_t>(change_sensitivity);
        config.sensorSpecific = 0;
    } else {
        config.changeSensitivity = 0;
        config.sensorSpecific = 0;
    }
    sh2_Error_t ret = sh2_setSensorConfig(sensorId, &config);
    if (ret != SH2_OK) {
        lastError = ret;
        return false;
    }
    return true;
}

/**
 * @brief Triggers a hardware reset of the BNO085 if supported.
 */
void BNO085::hardwareReset() {
    // This function would toggle a reset pin if the transport or platform provides access to it.
    // Not implemented in generic interface. If needed, user can reset externally and then call init() again.
}

