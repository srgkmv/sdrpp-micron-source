#include "micron.h"
#include <utils/flog.h>
#include <ftd2xx.h>
#include <chrono>

using namespace std::chrono_literals;

#define DEVICE_NAME		"SDR-Micron"
#define USB_BUFFER_SIZE					65536
#define USB_READ_TIMEOUT				100
#define USB_WRITE_TIMEOUT				100
#define RECEIVE_BUFFER_SIZE				(2 * USB_BUFFER_SIZE + RECEIVE_BLOCK_SIZE)
#define RECEIVE_BLOCK_SIZE				508		/* 16 bytes header + 492 bytes IQ data */
#define RECEIVE_BLOCK_HEADER_SIZE		16
#define FIRMWARE_VERSION_HIGH_OFFSET	11
#define FIRMWARE_VERSION_LOW_OFFSET		12
#define IQ24_SIZE						6
#define IQ24_PER_BLOCK					82
#define IQ16_SIZE						4
#define IQ16_PER_BLOCK					123
#define OUTPUT_BLOCK_LEN				32*512	/* Not bytes, but number of 24-bit IQ pairs! */


#define MICRON_VEN 0x0403
#define MICRON_DEV 0x6014
#define MICRON_CTRL_PACKET_SIZE 32
#define MICRON_DATA_PACKET_SIZE 508

#ifdef WIN32
#define __bswap_32 _byteswap_ulong
#endif

namespace micron {
void Client::ReceiveData()
{

    DWORD bytesAvailable;
    FT_STATUS status = FT_GetQueueStatus(deviceHandle, &bytesAvailable);
    if (status != FT_OK)
    {
        flog::error("FT_GetQueueStatus() failed, status = {}", (int)status);
        std::this_thread::sleep_for(2000ms);
        return;
    }

    DWORD bytesToReceive = 508; // USB_BUFFER_SIZE / 2;
    if (bytesToReceive < bytesAvailable)
        bytesToReceive = bytesAvailable;

    // Just a sanity check. Normally g_bytesReceived holds a partial block,
    // hence this case should never happen.
    if (g_bytesReceived > RECEIVE_BLOCK_SIZE)
    {
        flog::error("ReceiveData() -- g_bytesReceived > RECEIVE_BLOCK_SIZE");
        g_bytesReceived = 0;
    }

    if (g_bytesReceived + bytesToReceive > RECEIVE_BUFFER_SIZE)
    {
        flog::warn("ReceiveData() -- receive buffer overflow");
        bytesToReceive = RECEIVE_BUFFER_SIZE - g_bytesReceived;
    }

    DWORD bytesReceived;
    status = FT_Read(deviceHandle, g_receiveBuffer + g_bytesReceived, bytesToReceive, &bytesReceived);
    if (status != FT_OK)
    {
        flog::error("FT_Read() failed, status = %i", (int)status);
        std::this_thread::sleep_for(2000ms);
        return;
    }

    g_bytesReceived += bytesReceived;
    // Just a sanity check. Normally g_bytesReceived will be less than RECEIVE_BUFFER_SIZE.
    if (g_bytesReceived > RECEIVE_BUFFER_SIZE)
    {
        flog::error("ReceiveData() -- g_bytesReceived > RECEIVE_BUFFER_SIZE");
        g_bytesReceived = RECEIVE_BUFFER_SIZE;
    }

    // QUOTE: The ft245 driver does not have a circular buffer for input; bytes are just appended
    //        to the buffer. When all bytes are read and the buffer goes empty, the pointers are reset to zero.
    //        Be sure to empty out the ft245 frequently so its buffer does not overflow.
    // Just in case some additional bytes have arrived into the USB buffer while I was reading from it...
    status = FT_GetQueueStatus(deviceHandle, &bytesAvailable);
    if (status != FT_OK)
    {
        flog::error("FT_GetQueueStatus() failed, status = {}", (int)status);
        std::this_thread::sleep_for(2000ms);
        return;
    }
    if(bytesAvailable> 1000) flog::warn("bytesAvailable {} bytes", (int)bytesAvailable);

    if (bytesAvailable == 0)
        return;	// Best case: no additional bytes

    //LOG_INFO(("ReceiveData() -- additional %i bytes available, receiving it", bytesAvailable));

    bytesToReceive = bytesAvailable;
    if (g_bytesReceived + bytesToReceive > RECEIVE_BUFFER_SIZE)
    {
        flog::warn("ReceiveData() -- receive buffer overflow while receiving additional bytes");
        bytesToReceive = RECEIVE_BUFFER_SIZE - g_bytesReceived;
    }

    if (bytesToReceive == 0)
        return;

    status = FT_Read(deviceHandle, g_receiveBuffer + g_bytesReceived, bytesToReceive, &bytesReceived);
    if (status != FT_OK)
    {
        flog::error("FT_Read() failed, status = {}", (int)status);
        std::this_thread::sleep_for(2000ms);
        return;
    }


    g_bytesReceived += bytesReceived;
    // Just a sanity check. Normally g_bytesReceived will be less than RECEIVE_BUFFER_SIZE.
    if (g_bytesReceived > RECEIVE_BUFFER_SIZE)
    {
        flog::error("ReceiveData() -- g_bytesReceived > RECEIVE_BUFFER_SIZE");
        g_bytesReceived = RECEIVE_BUFFER_SIZE;
    }
}

void Client::ProcessData()
{
    uint8_t * data = g_receiveBuffer;
    int dataLen = g_bytesReceived;
    while (dataLen != 0)
    {
        int preamble_displacement = FindPreamble(data, dataLen);
        if (preamble_displacement < 0)
        {
            if (dataLen > RECEIVE_BLOCK_SIZE - 1)
            {
                int bytesToDiscard = dataLen - (RECEIVE_BLOCK_SIZE - 1);
                data += bytesToDiscard;
                dataLen = (RECEIVE_BLOCK_SIZE - 1);
                g_preamble_displacement_extra = bytesToDiscard;
            }

            if (data != g_receiveBuffer)
                memmove(g_receiveBuffer, data, dataLen);

            break;
        }

        int full_preamble_displacement = preamble_displacement + g_preamble_displacement_extra;
        if (full_preamble_displacement != 0)
        {
            flog::warn("ProcessData() -- preamble displacement {} bytes", full_preamble_displacement);
            //DumpBlock(data);
            g_preamble_displacement_extra = 0;
        }

        data += preamble_displacement;
        dataLen -= preamble_displacement;
        ProcessBlock(data);
        data += RECEIVE_BLOCK_SIZE;
        dataLen -= RECEIVE_BLOCK_SIZE;
    }

    g_bytesReceived = dataLen;
}

int Client::FindPreamble(uint8_t *data, int dataLen)
{
    if (dataLen < RECEIVE_BLOCK_SIZE)
        return -1;	// not a full valid block

    int offset = 0;
    int maxOffset = dataLen - RECEIVE_BLOCK_SIZE;
    while (offset <= maxOffset)
    {
        if (data[offset + 7] != 0xD5)
            offset++;
        else if ((data[offset] == 0x55)
                 && (data[offset + 1] == 0x55)
                 && (data[offset + 2] == 0x55)
                 && (data[offset + 3] == 0x55)
                 && (data[offset + 4] == 0x55)
                 && (data[offset + 5] == 0x55)
                 && (data[offset + 6] == 0x55))
        {
            return offset;	// preamble found
        }
        else
            offset += 8;
    }

    return -1;	// not found
}

void Client::ProcessBlock(uint8_t * block)
{
    /*if ((g_firmware_version_high != block[FIRMWARE_VERSION_HIGH_OFFSET]) ||
        (g_firmware_version_low != block[FIRMWARE_VERSION_LOW_OFFSET]))
    {
        // Extract firmware version from packet
        g_firmware_version_high = block[FIRMWARE_VERSION_HIGH_OFFSET];
        g_firmware_version_low = block[FIRMWARE_VERSION_LOW_OFFSET];
        g_firmware_version_changed = true;
    }*/

    block += RECEIVE_BLOCK_HEADER_SIZE;
    uint8_t * iq = (uint8_t *)block;
    if (sampleRateIndex < 8)
    {
        // 24 bits per sample
        if (g_outputCount + IQ24_PER_BLOCK < OUTPUT_BLOCK_LEN)
            ProcessIQ24(block, IQ24_PER_BLOCK);
        else
        {
            int iqBeforeSend = OUTPUT_BLOCK_LEN - g_outputCount;
            int iqAfterSend = IQ24_PER_BLOCK - iqBeforeSend;
            if (iqBeforeSend != 0)
                ProcessIQ24(block, iqBeforeSend);
            //SendOutput();
            if (iqAfterSend != 0)
                ProcessIQ24(block + iqBeforeSend * IQ24_SIZE, iqAfterSend);
        }
    }
    else
    {
        // 16 bits per sample
        if (g_outputCount + IQ16_PER_BLOCK < OUTPUT_BLOCK_LEN)
            ProcessIQ16(block, IQ16_PER_BLOCK);
        else
        {
            int iqBeforeSend = OUTPUT_BLOCK_LEN - g_outputCount;
            int iqAfterSend = IQ16_PER_BLOCK - iqBeforeSend;
            if (iqBeforeSend != 0)
                ProcessIQ16(block, iqBeforeSend);
            //SendOutput();
            if (iqAfterSend != 0)
                ProcessIQ16(block + iqBeforeSend * IQ16_SIZE, iqAfterSend);
        }
    }
}

void Client::ProcessIQ24(uint8_t *block, int count)
{
    //82 IQ pairs formatted as “I2 I1 I0, Q2 Q1 Q0…..”,  MSB is first, 24 bits per sample
    for(int i = 0; i < count; i++)
    {
        // Convert to 32bit
        int32_t sq = ((uint32_t)block[(i*6) + 0] << 16) | ((uint32_t)block[(i*6) + 1] << 8) | (uint32_t)block[(i*6) + 2];
        int32_t si = ((uint32_t)block[(i*6) + 3] << 16) | ((uint32_t)block[(i*6) + 4] << 8) | (uint32_t)block[(i*6) + 5];

        // Sign extend
        si = (si << 8) >> 8;
        sq = (sq << 8) >> 8;

        // Convert to float (IQ swapped for some reason)
        out.writeBuf[i].im = (float)si / (float)0x1000000;
        out.writeBuf[i].re = (float)sq / (float)0x1000000;
    }
    out.swap(count);
}

void Client::ProcessIQ16(uint8_t *block, int count)
{
    for(int i = 0; i < count; i++)
    {

        // Convert to 32bit
        int16_t sq = ((uint32_t)block[(i*4) + 0] << 8) | (uint32_t)block[(i*4) + 1];
        int16_t si = ((uint32_t)block[(i*4) + 2] << 8) | (uint32_t)block[(i*4) + 3];

        // Sign extend
        //si = (si << 16) >> 16;
        //sq = (sq << 16) >> 16;
        //*i = (*i >> 2);


        // Convert to float (IQ swapped for some reason)
        out.writeBuf[i].im = (float)si / (float)0x10000;//00;
        out.writeBuf[i].re = (float)sq / (float)0x10000;//00;
    }
    out.swap(count);
}

Client::Client(std::string serial) {
    this->serial = serial;
    enabled = 0x1;
    sampleRateIndex = MICRON_SAMP_RATE_384KHZ;
    freq = 0;
    att = 0;
    bexit = false;

    FT_HANDLE handle;
    FT_STATUS status = FT_OpenEx((void*)serial.c_str(), FT_OPEN_BY_SERIAL_NUMBER, &handle);
    if (status != FT_OK)
    {
        flog::error("FT_OpenEx(\"{}\") failed, status = {}", serial, (int)status);
        return;
    }

    status = FT_SetBitMode(handle, 255, 64);
    if (status != FT_OK)
    {
        flog::error("FT_SetBitMode() failed, status = {}", (int)status);
        FT_Close(handle);
        return;
    }

    status = FT_SetTimeouts(handle, USB_READ_TIMEOUT, USB_WRITE_TIMEOUT);
    if (status != FT_OK)
    {
        flog::error("FT_SetTimeouts() failed, status = {}", (int)status);
        FT_Close(handle);
        return;
    }

    status = FT_SetLatencyTimer(handle, 2);
    if (status != FT_OK)
    {
        flog::error("FT_SetLatencyTimer() failed, status = {}", (int)status);
        FT_Close(handle);
        return ;
    }

    status = FT_SetUSBParameters(handle, USB_BUFFER_SIZE, USB_BUFFER_SIZE);
    if (status != FT_OK)
    {
        flog::error("FT_SetUSBParameters() failed, status = {}", (int)status);
        FT_Close(handle);
        return;
    }

    deviceHandle = handle;

    std::this_thread::sleep_for(1500ms);	// wait 1.5 sec for device initialization
    flog::info("{} was opened successfully", DEVICE_NAME);

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
    FT_STATUS status = FT_SetBitMode(deviceHandle, 255, 0);
    if (status != FT_OK)
    {
        flog::error("CloseRxDevice() -- FT_SetBitMode() failed, status = {}", (int)status);
    }

    status = FT_Close(deviceHandle);
    if (status != FT_OK)
    {
        flog::error("CloseRxDevice() -- FT_Close() failed, status = {}", (int)status);
    }
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


    DWORD bytesWritten;
    FT_STATUS status = FT_Write(deviceHandle, lBa, MICRON_CTRL_PACKET_SIZE, &bytesWritten);
    if (status != FT_OK)
    {
        flog::error("FT_Write(RX_DEVICE_CONTROL_PACKET) failed, status = {}", (int)status);
        return;
    }
}

void Client::worker() {
    g_receiveBuffer = new BYTE[RECEIVE_BUFFER_SIZE];
    g_bytesReceived = 0;
    g_preamble_displacement_extra = 0;
    g_outputCount = 0;

    while (!bexit) {
        ReceiveData();
        ProcessData();
    }
}

    std::vector<Info> discover() {

        std::vector<Info> devices;

        DWORD numDevices;
        FT_STATUS status = FT_CreateDeviceInfoList(&numDevices);
        if (status != FT_OK)
        {
        flog::error("FT_CreateDeviceInfoList() failed, status = {}", (int)status);
            return devices;
        }

        flog::info("Found {} FTDI devices", (int)numDevices);
        int iIndex = 0;
        for (DWORD i = 0; i < numDevices; i++)
        {
            DWORD flags;
            DWORD type;
            DWORD id;
            DWORD locId;
            char serialNumber[16];
            char description[64];
            FT_HANDLE handle;
            status = FT_GetDeviceInfoDetail(i, &flags, &type, &id, &locId, serialNumber, description, &handle);
            if (status != FT_OK)
            {
                flog::error("FT_GetDeviceInfoDetail({}) failed, status = {}", (int)i, (int)status);
            }
            else if (strncmp(description, DEVICE_NAME, sizeof DEVICE_NAME) == 0)
            {
                flog::info("{} found", DEVICE_NAME);
                Info info;
                info.serial = std::string(serialNumber);
                info.index = iIndex;

                devices.push_back(info);
                iIndex++;
            }
        }
        return devices;
    }

    std::shared_ptr<Client> open(std::string serial) {
        return std::make_shared<Client>(serial);
    }

}
