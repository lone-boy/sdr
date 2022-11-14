//
// Created by jcc on 22-11-14.
//

#ifndef SDR_SDRDEVICE_H
#define SDR_SDRDEVICE_H

#include "string"
#include "iostream"
#include <memory>

namespace mp{

    using namespace std;

class sdrDevice{
    /* device string
     * d_string:pluto or uhd
     * */


public:
    typedef std::shared_ptr<sdrDevice> sdr_sptr;
    virtual ~sdrDevice(void) = 0;

    static sdr_sptr make_sdrDevice(const std::string& driver_type,
                                         const std::string& ip);

    virtual bool sdr_open() = 0;
    virtual void sdr_set_samplerate(double sample_rate) = 0;
    virtual void sdr_set_gain(double gain) = 0;
    virtual void sdr_set_bandwidth(double bandwidth) = 0;

};

}

#endif //SDR_SDRDEVICE_H
