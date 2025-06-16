#include "../common/DummyTransport.hpp"
#include "BNO085.hpp"
int main() {
  DummyTransport t;
  BNO085 imu(&t);
  imu.begin();
  return 0;
}
