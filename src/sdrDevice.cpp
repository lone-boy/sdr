//
// Created by jcc on 22-11-14.
//
#include "sdrDevice.h"

#include <utility>

using namespace mp;

sdrDevice::~sdrDevice() {
    /* NOP */
}

class iioDevice :public sdrDevice{
public:
    iioDevice(std::string  ip) :_device(std::move(ip)){}
    bool sdr_open() override{
        std::cout << "iio device open" << std::endl;
        return true;
    }
    void sdr_set_samplerate(double sample_rate) override{
        std::cout << "iio device sdr_set_samplerate" << std::endl;
    }
    void sdr_set_gain(double gain) override{
        std::cout << "iio device sdr_set_gain" << std::endl;

    }
    void sdr_set_bandwidth(double bandwidth) override{
        std::cout << "iio device sdr_set_bandwidth" << std::endl;
    }


private:
    string _device;
    double _sample_rate;
    double _gain;
    double _band_width;
};

class uhdDevice : public sdrDevice{
public:
    uhdDevice(std::string  ip) :_device(std::move(ip)){}
    bool sdr_open() override{
        std::cout << "uhd device open" << std::endl;
        return true;
    }
    void sdr_set_samplerate(double sample_rate) override{
        std::cout << "uhd device sdr_set_samplerate" << std::endl;
    }
    void sdr_set_gain(double gain) override{
        std::cout << "uhd device sdr_set_gain" << std::endl;
    }
    void sdr_set_bandwidth(double bandwidth) override{
        std::cout << "uhd device sdr_set_bandwidth" << std::endl;

    }

private:
    string _device;
    double _sample_rate;
    double _gain;
    double _band_width;
};



sdrDevice::sdr_sptr sdrDevice::make_sdrDevice(const std::string& driver_type,const std::string& ip) {
    if(driver_type == "uhd"){
        std::cout<<"run uhd driver return uhd sptr "<<std::endl;
        return sdr_sptr(new iioDevice(ip));
    }
    else if(driver_type == "iio"){
        std::cout << "create iio return sptr" << std::endl;
        return sdr_sptr (new uhdDevice(ip));
    }
    return nullptr;
}