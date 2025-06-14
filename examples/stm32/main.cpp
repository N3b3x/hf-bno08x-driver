#include "BNO085.hpp"
#include "../common/DummyTransport.hpp"
int main(){
    DummyTransport t;
    BNO085 imu(&t);
    imu.begin();
    return 0;
}
