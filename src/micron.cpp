#include "micron.h"
#include <utils/flog.h>
#include <libusb-1.0/libusb.h>
#include <chrono>

#define MICRON_VEN 0x0403
#define MICRON_DEV 0x6014
#define MICRON_CTRL_PACKET_SIZE 32
#define MICRON_DATA_PACKET_SIZE 508
#define ISB_ISO_NUMPACKETS 1
#define EP_ISO_IN	0x81
#define EP_ISO_OUT	0x2
#define USB_TIMEOUT 1000
#define PACKET_SIZE 508
#define PACKET_DATA_SIZE 492

const unsigned char pPreamble[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5};
std::vector<char> preamble(pPreamble, pPreamble + sizeof(pPreamble));

template<typename T>
int find_vector(const std::vector<T> & haystack, const std::vector<T> & needle)
{
    for(auto i = haystack.begin(); i != haystack.end() - needle.size(); i++)
    {
        bool found = true;
        for(auto j = needle.begin(), k = i; j != needle.end(); j++,k++)
        {
            if(*k != *j)
            {
                found = false;
                break;
            }
        }
        if(found)
            return i - haystack.begin();
    }
    return -1;
}

namespace micron {
libusb_device_handle * Client::openDevice(const std::string &serial)
{
    libusb_device ** deviceList = nullptr;
    ssize_t deviceCount = libusb_get_device_list(nullptr, &deviceList);

    for (size_t idx = 0; idx < deviceCount; ++idx)
    {
        libusb_device *device = deviceList[idx];
        libusb_device_descriptor desc = {0};
        int rc = libusb_get_device_descriptor(device, &desc);
        if(rc >= 0 && desc.idVendor == 0x0403 && desc.idProduct == 0x6014)
        {
            libusb_device_handle *handle;
            try
            {
                libusb_open(device, &handle);
                if (handle != nullptr)
                {
                    unsigned char serialBuffer[128];
                    if (libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serialBuffer, 128) >= 0)
                    {
                        serialBuffer[127] = '\0';
                        if(std::string((char*)serialBuffer) == serial)
                        {
                            return handle;
                        }
                    }
                }
                libusb_close(handle);
            } catch (libusb_error &e) {
                flog::error("Unable to open device. Error {}", (int)e);
            }
        }
    }
    flog::error("Unable to find device with serial {}", serial);
    return nullptr;
}

Client::Client(std::string serial) {
        this->serial = serial;
        enabled = 0x1;
        sampleRateIndex = MICRON_SAMP_RATE_384KHZ;
        freq = 0;
        att = 0;
        bexit = false;

        int rc = libusb_init(nullptr);
        if (rc < 0) {
            flog::error("Error initializing libusb: {}", libusb_error_name(rc));
            return;
        }
        deviceHandle = openDevice(serial);

        if (!deviceHandle) {
            flog::error("Error finding USB device");
            return;
        }

        rc = libusb_claim_interface(deviceHandle, 0);
        if (rc < 0) {
            flog::error("Error claiming interface: {}", libusb_error_name(rc));
            return;
        }

        libusb_control_transfer(deviceHandle, 0x40, 0,  0x0000, 1,   0, 0, 0);
        libusb_control_transfer(deviceHandle, 0x40, 3,  0x042e, 513, 0, 0, 0);
        libusb_control_transfer(deviceHandle, 0x40, 9,  0x0002, 1,   0, 0, 0);
        libusb_control_transfer(deviceHandle, 0x40, 11, 0x40ff, 1,   0, 0, 0);  //BITMODE_SYNCFF (0x40) BITMASK (0xff)
        libusb_control_transfer(deviceHandle, 0x40, 0,  0x0001, 1,   0, 0, 0);

        for(int i =0; i < NUM_USB_XFERS; i++)
        {
            transfer[i] = libusb_alloc_transfer(ISB_ISO_NUMPACKETS);
            if (transfer[i] == nullptr)
            {
                flog::error("Micron: Unable to allocate transfer");
                return;
            }
            int pktsize_in = 512;
            int bufsize_in = pktsize_in * ISB_ISO_NUMPACKETS;
            unsigned char * isoc_databuf = (unsigned char *)(malloc(bufsize_in));

            libusb_fill_bulk_transfer(transfer[i], deviceHandle, EP_ISO_IN, isoc_databuf, bufsize_in, [](struct libusb_transfer *transfer)
                {
                    int act_len;
                    unsigned char *ptr;
                    int pkts_in = ISB_ISO_NUMPACKETS;

                    Client * THIS = (Client*) transfer->user_data;

                    //Transfer complete call back got.Check if transfer succeeded
                    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
                        flog::warn("Micron: Transfer not completed. Status: {}", (int)transfer->status);
                        libusb_free_transfer(transfer);
                    }


                    if(transfer->actual_length > 2)
                    {
                        if(transfer->actual_length-2 != MICRON_DATA_PACKET_SIZE)
                        {
                            flog::warn("Micron: Broken packet received: {} bytes", transfer->actual_length);
                        }

                        std::vector<char> packet(transfer->buffer+2, transfer->buffer + transfer->actual_length);
                        THIS->recvBuffer.insert(THIS->recvBuffer.end(), packet.begin(), packet.end());


                        int iPreambleStart = -1;
                        while(THIS->recvBuffer.size())
                        {
                            iPreambleStart = find_vector(THIS->recvBuffer, preamble);
                            if(iPreambleStart < 0)
                            {
                                flog::warn("Micron: Preamble not found!");
                                break;
                            }
                            if(iPreambleStart > 0)
                                flog::warn("Micron: Packet loss detected!");
                            if(THIS->recvBuffer.size() - iPreambleStart < PACKET_SIZE)
                            {
                                //flog::warn("Micron: Packet not full. Buffer size: {}, recv len: {}", THIS->recvBuffer.size() - iPreambleStart, transfer->actual_length);
                                break;
                            }
                            unsigned const char * buf = ((unsigned const char *)THIS->recvBuffer.data()) + iPreambleStart;


                            int rx_id = 0;
                            if(buf[8] == 'B' && buf[9] == 'S' && buf[10] == '0')
                                flog::warn("Bandscpe data received!");
                            else if(buf[8] == 'R' && buf[9] == 'X' && buf[10] == '0')
                            {
                                static uint16_t seq = 0;
                                seq = buf[14];
                                int16_t * pPacketData = (int16_t *)(buf + 8 + 3+1+1+1+2);
                                uint8_t * iq = (uint8_t *)buf + 8 + 3+1+1+1+2;

                                if(THIS->sampleRateIndex < 8)
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
                                        THIS->out.writeBuf[i].im = (float)si / (float)0x1000000;
                                        THIS->out.writeBuf[i].re = (float)sq / (float)0x1000000;
                                    }
                                    THIS->out.swap(82);
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
                                        THIS->out.writeBuf[i].im = (float)si / (float)0x10000;//00;
                                        THIS->out.writeBuf[i].re = (float)sq / (float)0x10000;//00;
                                    }
                                    THIS->out.swap(123);
                                }
                            }
                            THIS->recvBuffer.erase(THIS->recvBuffer.begin(), THIS->recvBuffer.begin() + iPreambleStart + PACKET_SIZE);
                        }
                    }
                    if (THIS->bexit == false)
                    {
                        if(libusb_submit_transfer(transfer) < 0)
                            flog::error("Micron: error re-submitting transfer");
                    }
                }, this, 0);
            libusb_submit_transfer(transfer[i]);
        }

        // Start worker
        workerThread = std::thread(&Client::worker, this);
    }

    void Client::close() {
        stop();
        bexit = true;
        // Wait for worker to exit
        out.stopWriter();
        if (workerThread.joinable()) { workerThread.join(); }
        out.clearWriteStop();
        //ftdi_free(ftdi);
        if (deviceHandle)
        {
            libusb_control_transfer(deviceHandle, 0x40, 11, 0x00ff, 1,   0, 0, 0);  //BITMODE_RESET (0x00) BITMASK (0xff)
            libusb_release_interface(deviceHandle, 0);
            libusb_close(deviceHandle);
        }
        libusb_exit(nullptr);
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

    void Client::setAtt(int att) {
        this->att = att;
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
        lBa[17] = ((char)att);   //attenuation
        lBa[18] = 0x0;//nbs_on
        lBa[19] = 0x0;//nbs_period
        lBa[20] = 0x0;
        for(int i = 21; i < MICRON_CTRL_PACKET_SIZE; i++)
            lBa[i] = 0x00;

        int32_t iBytesWritten = 0;
        //FT_Write(m_pDeviceHandle, lBa.data(), lBa.size(), &iBytesWritten);
        //iBytesWritten = ftdi_write_data(ftdi,  lBa, MICRON_CTRL_PACKET_SIZE);
        int r = libusb_bulk_transfer(deviceHandle, EP_ISO_OUT, lBa, MICRON_CTRL_PACKET_SIZE, &iBytesWritten, USB_TIMEOUT);
        if (r)
        {
            flog::error("Micron: Error configuring RX0: {}", libusb_error_name(r));
        }
        else if(iBytesWritten > 0)
        {
            flog::info("Micron: {} bytes written", iBytesWritten);
        }
    }

    void Client::worker() {
        using namespace std::chrono_literals;
        while(!bexit)
        {
            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 1000;
            libusb_handle_events_timeout(nullptr, &tv);
            //std::this_thread::sleep_for(100ms);
        }
    }


    std::vector<Info> discover() {
        std::vector<Info> devices;
        int rc = libusb_init(nullptr);
        if (rc < 0) {
            flog::error("Micron: Error initializing libusb: {}", libusb_error_name(rc));
            return devices;
        }
        libusb_device ** deviceList = nullptr;
        ssize_t deviceCount = libusb_get_device_list(nullptr, &deviceList);

        for (size_t idx = 0; idx < deviceCount; ++idx)
        {
            libusb_device *device = deviceList[idx];
            libusb_device_descriptor desc = {0};
            rc = libusb_get_device_descriptor(device, &desc);
            if(rc >= 0 && desc.idVendor == 0x0403 && desc.idProduct == 0x6014)
            {
                libusb_device_handle *handle;
                try
                {
                    libusb_open(device, &handle);
                    if (handle != nullptr)
                    {
                        unsigned char serial[128];
                        if (libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serial, 128) >= 0)
                        {
                            serial[127] = '\0';
                            Info info;
                            info.serial = std::string((char*)serial);
                            info.index = idx;
                            devices.push_back(info);
                        }
                    }
                    libusb_close(handle);
                } catch (libusb_error &e) {
                    flog::error("Micron: Unable to open device. Error {}", (int)e);
                }
            }
        }
        libusb_exit(nullptr);
        return devices;
    }

    std::shared_ptr<Client> open(std::string serial) {
        return std::make_shared<Client>(serial);
    }
}
