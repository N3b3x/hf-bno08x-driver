#include "../../common/DummyTransport.hpp"
#include "BNO085.hpp"
extern "C" void app_main(void) {
  DummyTransport t;
  BNO085 imu(&t);
  imu.begin();
}
