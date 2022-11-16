#include <iostream>
#include "sdrDevice.h"
#include "unistd.h"

/* iq data recv call back */
void rx_data_call_back(mp::sdr_transfer *transfer){
    printf("recv sample %d\n",transfer->length);
    for(int i = 0;i<transfer->length;i++)
        printf("i=%d q=%d\n",transfer->data[i*2],transfer->data[i*2+1]);
}

using namespace mp;
int main() {
//    std::shared_ptr<sdrDevice> iio_device = sdrDevice::make_sdrDevice("iio","ip:192.168.1.10");
//    iio_device->sdr_open();
//    iio_device->sdr_check(0);
//    iio_device->sdr_set_rx_samplecnt(1024);
//    iio_device->sdr_start_rx(rx_data_call_back);
//    sleep(5);
//    iio_device->sdr_set_reset_rx_samplecount(2048);
//    sleep(5);
//    iio_device->sdr_stop_rx();

    std::shared_ptr<sdrDevice> uhd_device = sdrDevice::make_sdrDevice("uhd","addr=192.168.1.10");
    uhd_device->sdr_open();
    uhd_device->sdr_check(0);
    uhd_device->sdr_set_rx_samplecnt(1024);
    uhd_device->sdr_set_lo_frequency(100.1e6);
    uhd_device->sdr_set_gain(50,0);
    uhd_device->sdr_set_bandwidth(2e6,0);
    uhd_device->sdr_set_samplerate(2e6*1.5f,0);

    uhd_device->sdr_start_rx(rx_data_call_back);

    sleep(5);
    uhd_device->sdr_stop_rx();
    while(1){
        sleep(1);
    }

//    iio_device->sdr_close();

    return 0;
}
