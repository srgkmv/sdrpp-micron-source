#pragma once
#include <dsp/stream.h>
#include <dsp/types.h>
#include <memory>
#include <vector>
#include <string>
#include <thread>

#define NUM_USB_XFERS 1
struct libusb_device_handle;
struct libusb_transfer;

namespace micron {
    struct Info {
        std::string serial;
        unsigned int index;

        bool operator==(const Info& b) const {
            return serial == b.serial && index == b.index;
        }
    };

    enum MicronSamplerate {
        MICRON_SAMP_RATE_48KHZ = 0,
        MICRON_SAMP_RATE_96KHZ,
        MICRON_SAMP_RATE_192KHZ,
        MICRON_SAMP_RATE_240KHZ,
        MICRON_SAMP_RATE_384KHZ,
        MICRON_SAMP_RATE_480KHZ,
        MICRON_SAMP_RATE_640KHZ,
        MICRON_SAMP_RATE_768KHZ,
        MICRON_SAMP_RATE_960KHZ,
        MICRON_SAMP_RATE_1536KHZ
    };


    class Client {

        libusb_device_handle *openDevice(const std::string & serial);
    public:
        Client(std::string serial);

        void close();

        void start();
        void stop();

        void setSamplerate(MicronSamplerate samplerate);
        void setFrequency(double freq);
        void setAtt(int att);
        void setGain(int gain);


        void controlRx();

        dsp::stream<dsp::complex_t> out;

        

        void worker();


        bool bexit;
        int enabled;
        int sampleRateIndex;
        unsigned int freq;
        int att;
        int gain;
        std::thread workerThread;
        libusb_device_handle *  deviceHandle;
        libusb_transfer *       transfer[NUM_USB_XFERS];
        std::vector< char > recvBuffer;
        std::string serial;

    };

    std::vector<Info> discover();
    std::shared_ptr<Client> open(std::string serial);
}
