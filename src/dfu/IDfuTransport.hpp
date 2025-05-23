#pragma once

#include <cstdint>

extern "C" {
struct sh2_Hal_t;
}

/**
 * @brief Abstract interface for DFU hardware access.
 *
 * Users must derive from this class and implement the virtual
 * methods to adapt the DFU routines to a specific platform.
 */
class IDfuTransport {
public:
    virtual ~IDfuTransport() = default;

    /** Open the transport interface. */
    virtual int open() = 0;
    /** Close the transport interface. */
    virtual void close() = 0;
    /**
     * Read bytes from the device.
     * @param data Buffer to fill
     * @param len  Number of bytes to read
     * @param timestamp Optional timestamp from the transport
     * @return Number of bytes read or negative error code
     */
    virtual int read(uint8_t *data, unsigned len, uint32_t *timestamp) = 0;
    /**
     * Write bytes to the device.
     * @param data Buffer of data to send
     * @param len  Number of bytes to send
     * @return Number of bytes written or negative error code
     */
    virtual int write(const uint8_t *data, unsigned len) = 0;
    /** Return current time in microseconds. */
    virtual uint32_t getTimeUs() = 0;

    /**
     * Retrieve the underlying C HAL pointer used by the vendor
     * SH-2 library. Implementations should return a pointer to
     * a sh2_Hal_t structure that dispatches to this object.
     */
    virtual sh2_Hal_t *nativeHal() = 0;
};

