#pragma once
#include <dsp/stream.h>
#include <dsp/types.h>
#include <memory>
#include <vector>
#include <string>
#include <thread>

struct ftdi_context;

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

        uint8_t * g_receiveBuffer;
        int g_bytesReceived;
        int g_preamble_displacement_extra;
        int g_outputCount;


        bool CleanUSBBuffer();
        void ReceiveData();
        void ProcessData();
        int FindPreamble(uint8_t *data, int dataLen);
        void ProcessBlock(uint8_t *block);
        void ProcessIQ24(uint8_t * block, int count);
        void ProcessIQ16(uint8_t * block, int count);
    public:
        Client(std::string serial);

        void close();

        void start();
        void stop();

        void setSamplerate(MicronSamplerate samplerate);
        void setFrequency(double freq);
        void setAtt(int att);


        void controlRx();

        dsp::stream<dsp::complex_t> out;

        

        void worker();


        bool bexit;
        int enabled;
        int sampleRateIndex;
        unsigned int freq;
        int att;
        std::thread workerThread;
        void * deviceHandle;
        std::string serial;

    };

    std::vector<Info> discover();
    std::shared_ptr<Client> open(std::string serial);
}
