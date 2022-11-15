//
// Created by jcc on 22-11-14.
//
#include "sdrDevice.h"
#include "iio.h"
#include <utility>
#include "log.h"


using namespace mp;

sdrDevice::~sdrDevice() {
    /* NOP */
}

class iioDevice :public sdrDevice{
    enum iodev {RX,TX};

public:
    iioDevice(std::string  ip) :
    _device(std::move(ip))
    ,_rx(nullptr)
    ,_rx_buf(nullptr)
    ,_rx0_i(nullptr)
    ,_rx0_q(nullptr){
        _log.INFO("Create IIO device ...");
    }
    ~iioDevice(){
        _log.INFO("Release IIO device ...");
        if(_rx0_i) iio_channel_disable(_rx0_i);
        if(_rx0_q) iio_channel_disable(_rx0_q);
        if(_rx_buf) iio_buffer_destroy(_rx_buf);
        if(_ctx) iio_context_destroy(_ctx);
    }
    void sdr_open() override{
        std::string info("Find device with ");
        info += _device;
        _log.INFO("iio device open ...");
        if (this->connect_device(_device)){
            info += string(" Succed ...");
            _log.INFO(info);
        }
        else{
            info += string(" Failed ...");
            _log.WARNING(info);
        }
    }
    void sdr_set_samplerate(double sample_rate, int channel) override{
        this->set_ad9361_fs_hz(RX,channel,(long long)sample_rate);
    }
    void sdr_set_bandwidth(double bandwidth, int channel) override{
        this->set_ad9361_bd_hz(RX,channel,(long long)bandwidth);
    }
    void sdr_set_lo_frequency(double frequency, int channel) override{
        this->set_ad9361_lo_hz(RX,channel,(long long)frequency);
    }
    void sdr_set_gain(double gain, int channel) override{
        this->set_ad9361_rx_gain(RX,channel,(int)gain);
    }
    bool sdr_check(int channel) override{
        if(_rx0_i) iio_channel_disable(_rx0_i);
        if(_rx0_q) iio_channel_disable(_rx0_q);
        set_ad9361_rx_gain_mode(RX,"manual",channel);
        this->sdr_set_bandwidth(1e6,0);
        this->sdr_set_gain(50,0);
        this->sdr_set_lo_frequency(100.1e6,0);
        this->sdr_set_samplerate(1e6,0);
        if(not get_ad9361_stream_dev(_ctx,RX,&_rx))
            this->_log.ERROR("No stream dev ...");
        if(not get_ad9361_stream_ch(_ctx, RX, _rx, 0, &_rx0_i)){
            this->_log.ERROR("No stream ch rx i");
        }
        if(not get_ad9361_stream_ch(_ctx, RX, _rx, 1, &_rx0_q)){
            this->_log.ERROR("No stream ch rx q");
        }
        this->enable_iio_channel();
        return true;
    }

private:
    log _log;


    string _device;
    double _sample_rate;
    double _gain;
    double _band_width;
    int _chid;
    /* iio */
    iio_context *_ctx;
    iio_channel *_rx0_i;
    iio_channel *_rx0_q;
    iio_buffer  *_rx_buf;
    iio_device  *_rx;
    void *_p_data;

    char _tmp_str[64]{};


    bool connect_device(const std::string &ip);

    void errchk(int v,const char *what);
    void wr_ch_lli(struct iio_channel *chn,const char *what,long long val);
    void wr_ch_str(struct iio_channel *chn,const char *what,const char *str);
    char *get_ch_name(const char *type,int id);
    struct iio_device *get_ad9361_phy(struct iio_context *ctx);
    bool get_ad9361_stream_dev(struct iio_context *ctx,enum iodev d,struct iio_device **dev);
    bool get_ad9361_stream_ch(struct iio_context *ctx,enum iodev d,struct iio_device *dev,
                              int chid,struct iio_channel **chn);
    bool get_phy_chan(struct iio_context *ctx,enum iodev d,int chid,struct iio_channel **chn);
    bool get_lo_chan(struct iio_context *ctx,enum iodev d,struct iio_channel **chn);
    void set_ad9361_lo_hz(enum iodev type,int chid,long long lo_hz);
    void set_ad9361_bd_hz(enum iodev type,int chid,long long bd_hz);
    void set_ad9361_fs_hz(enum iodev type,int chid,long long fs);
    bool set_ad9361_rx_gain(iodev d, long long value, int chid);
    bool set_ad9361_rx_gain_mode(iodev d, const char *mode, int chid);
    void enable_iio_channel();
};

bool iioDevice::connect_device(const string &ip) {
    string info("Connect IIO device with ");
    info += ip;
    info += string(" ...");
    _log.INFO(info);
    _ctx = iio_create_context_from_uri(ip.c_str());
    if(_ctx == nullptr)
        return false;

    return true;
}

void iioDevice::errchk(int v, const char *what) {
    if (v < 0) {
        fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
    }
}

void iioDevice::wr_ch_lli(struct iio_channel *chn, const char *what, long long int val) {
    errchk(iio_channel_attr_write_longlong(chn,what,val),what);
}

void iioDevice::wr_ch_str(struct iio_channel *chn, const char *what, const char *str) {
    errchk(static_cast<int>(iio_channel_attr_write(chn, what, str)), what);
}

char *iioDevice::get_ch_name(const char *type, int id) {
    snprintf(_tmp_str,sizeof(_tmp_str),"%s%d",type,id);
    return _tmp_str;
}

struct iio_device *iioDevice::get_ad9361_phy(struct iio_context *ctx) {
    struct iio_device *dev = iio_context_find_device(_ctx,"ad9361-phy");
    if(dev == nullptr)
        this->_log.ERROR("No ad9361-phy found ...");

    return dev;
}

bool iioDevice::get_ad9361_stream_dev(struct iio_context *ctx, iioDevice::iodev d, struct iio_device **dev) {
    switch (d) {
        case TX: *dev = iio_context_find_device(_ctx,"cf-ad9361-dds-core-lpc"); return *dev != nullptr;
        case RX: *dev = iio_context_find_device(_ctx, "cf-ad9361-lpc");  return *dev != nullptr;
        default:
            this->_log.ERROR("No stream dev found ...");return false;
    }
}

bool iioDevice::get_ad9361_stream_ch(struct iio_context *ctx, iioDevice::iodev d, struct iio_device *dev, int chid,
                                     struct iio_channel **chn) {
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != nullptr;
}

bool iioDevice::get_phy_chan(struct iio_context *ctx, iioDevice::iodev d, int chid, struct iio_channel **chn) {
    switch (d) {
        case RX: *chn = iio_device_find_channel(get_ad9361_phy(_ctx), get_ch_name("voltage", chid), false);
            return *chn !=nullptr;
        case TX: *chn = iio_device_find_channel(get_ad9361_phy(_ctx), get_ch_name("voltage", chid), true);
            return *chn != nullptr;
        default:
            this->_log.ERROR("No phy channel found ..."); return false;
    }
}

bool iioDevice::get_lo_chan(struct iio_context *ctx, iioDevice::iodev d, struct iio_channel **chn) {
    switch (d) {
        // LO chan is always output, i.e. true
        case RX: *chn = iio_device_find_channel(get_ad9361_phy(_ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
        case TX: *chn = iio_device_find_channel(get_ad9361_phy(_ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
        default: this->_log.ERROR("No lo channel found ...");return false;
    }
}

void iioDevice::set_ad9361_lo_hz(iioDevice::iodev type, int chid, long long int lo_hz) {
    struct iio_channel *chn = nullptr;
    if (!get_lo_chan(_ctx, type, &chn)) { return; }
    wr_ch_lli(chn, "frequency", lo_hz);
}

void iioDevice::set_ad9361_bd_hz(iioDevice::iodev type, int chid, long long int bd_hz) {
    struct iio_channel *chn = nullptr;
    if (!get_phy_chan(_ctx, type, chid, &chn)) {	return; }
    wr_ch_lli(chn, "rf_bandwidth",       bd_hz);
}

void iioDevice::set_ad9361_fs_hz(iioDevice::iodev type, int chid, long long int fs) {
    struct iio_channel *chn = nullptr;
    if (!get_phy_chan(_ctx, type, chid, &chn)) {	return; }
    wr_ch_lli(chn, "sampling_frequency",       fs);
}

bool iioDevice::set_ad9361_rx_gain(iioDevice::iodev d, long long value, int chid) {
    struct iio_channel *chn = nullptr;
    if(!get_phy_chan(_ctx,d,chid,&chn)) { return false;}
    wr_ch_lli(chn,"hardwaregain",value);
    return true;
}

bool iioDevice::set_ad9361_rx_gain_mode(iioDevice::iodev d, const char *mode, int chid) {
    struct iio_channel *chn = nullptr;
    if(!get_phy_chan(_ctx,d,chid,&chn)) { return false;}
    wr_ch_str(chn,"gain_control_mode",mode);
    return true;
}

void iioDevice::enable_iio_channel() {
    iio_channel_enable(_rx0_i);
    iio_channel_enable(_rx0_q);
}


/**
 * -----------------------------------------------------------------------------------------------------------------
 * UHD INTERFACE
 * -----------------------------------------------------------------------------------------------------------------
 * */

class uhdDevice : public sdrDevice{
public:
    uhdDevice(std::string  ip) :_device(std::move(ip)){}
    void sdr_open() override{
        std::cout << "uhd device open" << std::endl;
    }
    void sdr_set_samplerate(double sample_rate, int channel) override{
        std::cout << "uhd device sdr_set_samplerate" << std::endl;
    }
    void sdr_set_gain(double gain, int channel) override{
        std::cout << "uhd device sdr_set_gain" << std::endl;
    }
    void sdr_set_bandwidth(double bandwidth, int channel) override{
        std::cout << "uhd device sdr_set_bandwidth" << std::endl;
    }
    void sdr_set_lo_frequency(double frequency, int channel) override{

    }
    bool sdr_check(int channel) override{
    }

private:
    string _device;
    double _sample_rate;
    double _gain;
    double _band_width;
};



sdrDevice::sdr_sptr sdrDevice::make_sdrDevice(const std::string& driver_type,const std::string& ip) {
    if(driver_type == "uhd"){
        return sdr_sptr(new uhdDevice(ip));
    }
    else if(driver_type == "iio"){
        return sdr_sptr (new iioDevice(ip));
    }
    return nullptr;
}