#include <iostream>
#include "sdrDevice.h"

using namespace mp;
int main() {
    std::shared_ptr<sdrDevice> sdr_device = sdrDevice::make_sdrDevice("uhd","192.168.1.10");
    sdr_device->sdr_open();
    sdr_device->sdr_set_bandwidth(90.2f);
    std::shared_ptr<sdrDevice> iio_device = sdrDevice::make_sdrDevice("iio","192.168.1.10");


    std::cout << "Hello, World!" << std::endl;
    return 0;
}
