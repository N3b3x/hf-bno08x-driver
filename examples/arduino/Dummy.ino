#include <BNO085.hpp>
#include "../common/DummyTransport.hpp"
DummyTransport t;
BNO085 imu(&t);
void setup(){
  imu.begin();
}
void loop(){
}
