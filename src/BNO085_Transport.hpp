// BNO085_Transport.hpp - Abstract transport interface for BNO085 (I2C, SPI, etc.)
#ifndef BNO085_TRANSPORT_HPP
#define BNO085_TRANSPORT_HPP

#include <cstdint>

/**
 * @class IBNO085Transport
 * @brief Abstract interface for BNO085 communication transport (I2C, SPI, etc.).
 */
class IBNO085Transport {
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IBNO085Transport() = default;
    /**
     * @brief Opens the communication bus.
     * @return true if successful, false otherwise.
     */
    virtual bool open() = 0;
    /**
     * @brief Closes the communication bus.
     */
    virtual void close() = 0;
    /**
     * @brief Writes data to the sensor.
     * @param data Pointer to buffer containing data to write.
     * @param length Number of bytes to write.
     * @return Number of bytes written, or negative on error.
     */
    virtual int write(const uint8_t* data, uint32_t length) = 0;
    /**
     * @brief Reads data from the sensor.
     * @param data Pointer to buffer to receive data.
     * @param length Number of bytes to read.
     * @return Number of bytes read, 0 if no data, or negative on error.
     */
    virtual int read(uint8_t* data, uint32_t length) = 0;
    /**
     * @brief Checks if new data is available.
     * @return true if data is available or if not implemented, always true.
     */
    virtual bool dataAvailable() { return true; }
    /**
     * @brief Delays execution for a specified time.
     * @param ms Delay duration in milliseconds.
     */
    virtual void delay(uint32_t ms) = 0;
};

#endif // BNO085_TRANSPORT_HPP
