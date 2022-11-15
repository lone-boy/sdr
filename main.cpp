#include <iostream>
#include "sdrDevice.h"




using namespace mp;
int main() {
    std::shared_ptr<sdrDevice> iio_device = sdrDevice::make_sdrDevice("iio","ip:192.168.1.10");
    iio_device->sdr_open();
    iio_device->sdr_check(0);

    return 0;
}
