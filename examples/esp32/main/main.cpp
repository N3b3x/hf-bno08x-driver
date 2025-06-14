#include "BNO085.hpp"
#include "../../common/DummyTransport.hpp"
extern "C" void app_main(void){
    DummyTransport t;
    BNO085 imu(&t);
    imu.begin();
}
