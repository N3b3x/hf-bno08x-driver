#include "BNO085_Transport.hpp"
class DummyTransport : public IBNO085Transport {
public:
    bool open() override { return true; }
    void close() override {}
    int write(const uint8_t*, uint32_t) override { return 0; }
    int read(uint8_t*, uint32_t) override { return 0; }
    void delay(uint32_t) override {}
    uint32_t getTimeUs() override { return 0; }
};
