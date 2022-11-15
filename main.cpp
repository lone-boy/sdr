#include <iostream>
#include "sdrDevice.h"
#include "unistd.h"
void rx_data_call_back(mp::sdr_transfer *transfer){
}


using namespace mp;
int main() {
    std::shared_ptr<sdrDevice> iio_device = sdrDevice::make_sdrDevice("iio","ip:192.168.1.10");
    iio_device->sdr_open();
    iio_device->sdr_check(0);
    iio_device->sdr_set_rx_samplecnt(1024);
    iio_device->sdr_start_rx(rx_data_call_back);
    while(1){
        sleep(1);
    }

    iio_device->sdr_close();

    return 0;
}
