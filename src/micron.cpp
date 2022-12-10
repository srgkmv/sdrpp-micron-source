#include "micron.h"
#include <spdlog/spdlog.h>
#include <libftdi1/ftdi.h>
#include <chrono>

#define MICRON_VEN 0x0403
#define MICRON_DEV 0x6014
#define MICRON_CTRL_PACKET_SIZE 32
#define MICRON_DATA_PACKET_SIZE 508

namespace micron {
    Client::Client(std::string serial) {
        this->serial = serial;
        enabled = 0x1;
        sampleRateIndex = MICRON_SAMP_RATE_384KHZ;
        freq = 0;
        gain = 0;
        bexit = false;

        if ((ftdi = ftdi_new()) == 0)
        {
           fprintf(stderr, "ftdi_new failed\n");
           return;
        }

        if (ftdi_set_interface(ftdi, INTERFACE_A) < 0)
        {
           fprintf(stderr, "ftdi_set_interface failed\n");
           ftdi_free(ftdi);
           return;
        }

        if (ftdi_usb_open_desc(ftdi, MICRON_VEN, MICRON_DEV, nullptr, serial.c_str()) < 0)
        {
           fprintf(stderr, "ftdi_usb_open_desc failed\n");
           ftdi_free(ftdi);
           return;
        }

        /* A timeout value of 1 results in may skipped blocks */
        if(ftdi_set_latency_timer(ftdi, 2))
        {
           fprintf(stderr,"Can't set latency, Error %s\n",ftdi_get_error_string(ftdi));
           ftdi_usb_close(ftdi);
           ftdi_free(ftdi);
           return;
        }

        if (ftdi_set_bitmode(ftdi,  0xff, BITMODE_SYNCFF) < 0)
        {
            fprintf(stderr,"Can't set bitmode, Error %s\n",ftdi_get_error_string(ftdi));
            ftdi_usb_close(ftdi);
            ftdi_free(ftdi);
            return;
        }

        if(ftdi_usb_purge_rx_buffer(ftdi) < 0)
        {
           fprintf(stderr,"Can't rx purge\n",ftdi_get_error_string(ftdi));
           return;
        }

        // Start worker
        workerThread = std::thread(&Client::worker, this);
    }

    void Client::close() {
        bexit = true;
        // Wait for worker to exit
        out.stopWriter();
        if (workerThread.joinable()) { workerThread.join(); }
        out.clearWriteStop();
        ftdi_free(ftdi);
    }

    void Client::start() {
        enabled = 0x1;
        controlRx();
    }

    void Client::stop() {
        enabled = 0x0;
        controlRx();
    }

    void Client::setSamplerate(MicronSamplerate samplerate) {
        sampleRateIndex = samplerate;
        controlRx();
    }

    void Client::setFrequency(double freq) {
        this->freq = freq;
        controlRx();
    }

    void Client::setGain(int gain) {
        this->gain = gain;
        controlRx();
    }

    void Client::controlRx()
    {
        unsigned char lBa[MICRON_CTRL_PACKET_SIZE];
        //Configure RX0
        for(int i =0 ;i < 7; i++) //reamble
            lBa[i] = 0x55;
        lBa[7] = 0xd5; //delimiter
        strcpy((char*)lBa+8, "RX0");
        lBa[11] = ((char)enabled);   //enable
        lBa[12] = ((char)sampleRateIndex);   // rate  1 for 96 kHz,
        *(unsigned int*)(lBa + 13) = __bswap_32(freq); //4 bytes frequency
        lBa[17] = ((char)gain);   //attenuation
        for(int i =0 ;i < 14; i++) //14 binary zeroes
            lBa[18+i] = 0x00;

        uint32_t iBytesWritten = 0;
        //FT_Write(m_pDeviceHandle, lBa.data(), lBa.size(), &iBytesWritten);
        iBytesWritten = ftdi_write_data(ftdi,  lBa, MICRON_CTRL_PACKET_SIZE);
        if(iBytesWritten > 0)
        {
            printf("%i bytes written\n", iBytesWritten);
        }

    }

    void Client::worker() {
        using namespace std::chrono_literals;

        uint8_t rbuf[MICRON_DATA_PACKET_SIZE];       

        while (!bexit) {
            uint32_t iBytesRead = ftdi_read_data(ftdi, rbuf, MICRON_DATA_PACKET_SIZE);
            if(iBytesRead > 0)
            {
                int rx_id = -1;
                if(rbuf[8] == 'R' && rbuf[9] == 'X' && rbuf[10] == '0')
                    rx_id = 0;
                if(rx_id != 0)
                {
                    continue;
                }
                uint8_t * iq = (uint8_t *)rbuf + 8 + 3+1+1+1+2;

                if(sampleRateIndex < 8)
                {
                    //82 IQ pairs formatted as “I2 I1 I0, Q2 Q1 Q0…..”,  MSB is first, 24 bits per sample
                    for(int i = 0; i < 82; i++)
                    {
                        // Convert to 32bit
                        int32_t sq = ((uint32_t)iq[(i*6) + 0] << 16) | ((uint32_t)iq[(i*6) + 1] << 8) | (uint32_t)iq[(i*6) + 2];
                        int32_t si = ((uint32_t)iq[(i*6) + 3] << 16) | ((uint32_t)iq[(i*6) + 4] << 8) | (uint32_t)iq[(i*6) + 5];

                        // Sign extend
                        si = (si << 8) >> 8;
                        sq = (sq << 8) >> 8;

                        // Convert to float (IQ swapped for some reason)
                        out.writeBuf[i].im = (float)si / (float)0x1000000;
                        out.writeBuf[i].re = (float)sq / (float)0x1000000;
                    }
                    out.swap(81);
                }
                else
                {
                    //123 IQ pairs formatted as "I1 I0, Q1 Q0..... ", MSB is first, 16 bits per sample
                    for(int i = 0; i < 123; i++)
                    {

                        // Convert to 32bit
                        int16_t sq = ((uint32_t)iq[(i*4) + 0] << 8) | (uint32_t)iq[(i*4) + 1];
                        int16_t si = ((uint32_t)iq[(i*4) + 2] << 8) | (uint32_t)iq[(i*4) + 3];

                        // Sign extend
                        //si = (si << 16) >> 16;
                        //sq = (sq << 16) >> 16;
                        //*i = (*i >> 2);


                        // Convert to float (IQ swapped for some reason)
                        out.writeBuf[i].im = (float)si / (float)0x10000;//00;
                        out.writeBuf[i].re = (float)sq / (float)0x10000;//00;
                    }
                    out.swap(123);
                }

            }
            else
                std::this_thread::sleep_for(100ms);

        }
    }

    std::vector<Info> discover() {

        std::vector<Info> devices;

        ftdi_context * ftdi = nullptr;
        if ((ftdi = ftdi_new()) == 0)
        {
           fprintf(stderr, "ftdi_new failed\n");
           return devices;
        }

        if (ftdi_set_interface(ftdi, INTERFACE_A) < 0)
        {
           fprintf(stderr, "ftdi_set_interface failed\n");
           ftdi_free(ftdi);
           return devices;
        }

        ftdi_device_list * pFtdiDeviceList = nullptr;
        if(ftdi_usb_find_all(ftdi, &pFtdiDeviceList, MICRON_VEN, MICRON_DEV) < 0)
        {
           fprintf(stderr, "ftdi_set_interface failed\n");
           ftdi_free(ftdi);
           return devices;
        }
        ftdi_device_list * pDevIterator = pFtdiDeviceList;
        int iIndex = 0;
        while (pDevIterator && pDevIterator->dev) {

            libusb_device * pDev = pDevIterator->dev;
            char sManufacturer[128];
            char sDescription[128];
            char sSerial[128];
            if(ftdi_usb_get_strings(ftdi, pDev, sManufacturer, 128, sDescription, 128, sSerial, 128) >= 0)
            {
                // Analyze
                Info info;
                info.serial = std::string(sSerial);
                info.index = iIndex;


                devices.push_back(info);
            }
            pDevIterator = pDevIterator->next;
            iIndex++;
        }
        
        ftdi_list_free(&pFtdiDeviceList);
        ftdi_free(ftdi);
        return devices;
    }

    std::shared_ptr<Client> open(std::string serial) {
        return std::make_shared<Client>(serial);
    }

}
