// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>
#include "SpecIOLink-Device.hpp"
#include <EEPROM.h>
#include "hardware/watchdog.h"

// Native Pico SDK integration for RP2040 devices
#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
#include "hardware/uart.h"
#include "hardware/gpio.h"
#define PICO_UART_ID uart0
#define PICO_TX_PIN  0
#define PICO_RX_PIN  1
#endif

using namespace lwIOLink;

static DeviceSettings g_settings;
static const uint32_t SETTINGS_MAGIC = 0xDEADBEEF;

// Logging Configuration
static const bool LOGGING_ENABLED = true;

//Checksum constants (Figure A.3)
static constexpr uint8_t ck8_seed = 0x52;
static constexpr uint8_t status_bit_offset = 0x06;
static constexpr uint8_t eventFlag_bit_offset = 0x07;
static constexpr uint8_t ckt_offset = 0x01;
static uint8_t mseq_ckt_mask = 0xC0;
static uint8_t ck6_mask = 0x3F;
static constexpr uint8_t timebase_bitoffset = 0x06;
static constexpr uint8_t multiplier_mask = 0x3F;
static constexpr uint8_t MC_ChMask = 0x60;
static constexpr uint8_t MC_AddrMask = 0x1F;
static constexpr uint8_t MC_ChBitOffset = 5;

uint32_t constexpr Device::TimeBaseLUT[TotalTimeEncodings];
uint32_t constexpr Device::TimeOffsetLUT[TotalTimeEncodings];

static volatile bool wakeup_signal = false;
static unsigned g_WuPin = 0;

// Wakeup interrupt disabled for validation scenario (always-active communication)
// See initTransceiver() for re-enabling instructions for low-power deployments
/*
void WakeupIRQ() {
    wakeup_signal = true;  // Signal to state machine that wakeup occurred
}
*/

// Utils namespace implementation
namespace lwIOLink {
namespace Utils {
static constexpr uint8_t TotalTimeEncodings = 3;
static constexpr uint32_t TimeBaseLUT[TotalTimeEncodings] = {100, 400, 1600}; //us
static constexpr uint32_t TimeOffsetLUT[TotalTimeEncodings] = {0, 6400, 32000}; //us

static uint16_t vendorId = 0xABCD;

uint32_t DecodeCycleTime(uint8_t encoded_time) {
    const uint8_t timebase_code = encoded_time >> timebase_bitoffset;
    const uint8_t multiplier = encoded_time & multiplier_mask;
    return TimeOffsetLUT[timebase_code] + (multiplier * TimeBaseLUT[timebase_code]);
}

uint8_t EncodeCycleTime(uint32_t cycleTime_us) {
    uint8_t timebase_code;
    uint8_t multiplier;
    const unsigned max_multiplier = 63;
    if (cycleTime_us < 400) cycleTime_us = 400;
    else if (cycleTime_us > 132800) cycleTime_us = 132800;

    if (cycleTime_us <= 6300) timebase_code = 0;
    else if (cycleTime_us <= 31600) timebase_code = 1;
    else timebase_code = 2;
    
    for (multiplier = 0; multiplier <= max_multiplier; multiplier++) {
        uint32_t cycletime_match = TimeOffsetLUT[timebase_code] + (multiplier * TimeBaseLUT[timebase_code]);
        if (cycletime_match >= cycleTime_us) break;
    }
    return (timebase_code << timebase_bitoffset | multiplier);
}

// Standard CRC32 Implementation (Ethernet/ZIP/Gzip)
// Poly: 0x04C11DB7, Init: 0xFFFFFFFF, RefIn: True, RefOut: True, XorOut: 0xFFFFFFFF
uint32_t CalculateCRC32(const void* data, size_t length) {
    const uint8_t* p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFF;
    
    while (length--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320; // Reflected Poly 0x04C11DB7
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

} // namespace Utils
} // namespace SpecIOLink

void __attribute__((weak)) Device::OnEventsProcessed() {}
void __attribute__((weak)) Device::OnNewCycle() {}

uint8_t Device::GetChecksum(const uint8_t *data, uint8_t length) const
{
    uint8_t checksum = ck8_seed;
    
    // Iterate over each byte in the frame
    for (uint8_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
        
        // Process 8 bits for the current byte
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (checksum & 0x80)
            {
                checksum = (checksum << 1) ^ 0xE5; // 0xE5 is the IO-Link Polynomial
            }
            else
            {
                checksum = (checksum << 1);
            }
        }
    }
    return checksum & 0x3F;
}

uint8_t EncodePD(uint8_t size_bytes)
{
    if (size_bytes <= 2) return (size_bytes * 8) & 0x1F; 
    return (1 << 7) | ((size_bytes - 1) & 0x1F);
}

// Temporary buffer for ISDU Response
static uint8_t isduResponseBuffer[32];
static uint8_t isduResponseSize = 0;
static bool isduResponsePending = false;

// Recalculate CRC32 checksum over parameter content (VendorName + ProductName + ApplicationTag)
// Uses fixed-size buffers (64+64+32 bytes) for deterministic calculation
static void UpdateParameterChecksum() {
    
    uint32_t crc = 0xFFFFFFFF;

    uint8_t dsBuffer[160];
    memcpy(dsBuffer, g_settings.VendorName, 64);
    memcpy(dsBuffer + 64, g_settings.ProductName, 64);
    memcpy(dsBuffer + 128, g_settings.ApplicationTag, 32);
    
    g_settings.ParameterChecksum = Utils::CalculateCRC32(dsBuffer, 160);
}

void Device::HandleISDU(uint8_t* buffer, uint8_t size, bool isRead)
{
    if (isRead) {
        // Respond with previously buffered ISDU response or idle state
        if (isduResponsePending) {
            uint8_t safeSize = (isduResponseSize > ODSize.Op) ? ODSize.Op : isduResponseSize;
            memcpy(ODBuffer, isduResponseBuffer, safeSize);
            isduResponsePending = false; 
        } else {
            memset(ODBuffer, 0, ODSize.Op); 
        }
        return;
    }

    // Handle ISDU Request from Master
    // Format: [Service(4b) | Length(4b)] [Index High] [Index Low] [Subindex] [Data...]
    // Minimum 3 bytes required: Service/Length + Index(2 bytes)
    if (size < 3) return; // Minimum frame size check

    uint8_t service = buffer[0] >> 4;
    uint8_t payloadLen = buffer[0] & 0x0F;
    uint16_t index = (buffer[1] << 8) | buffer[2]; // Big-endian index
    uint8_t subindex = (size >= 4) ? buffer[3] : 0;
    uint8_t *dataStart = (size > 4) ? &buffer[4] : nullptr;
    uint8_t dataLen = payloadLen;

    // Check for user ISDU callback first
    if (_userISDUHandler != nullptr) {
        bool isWrite = (service == 0xA || service == 0x2);
        if (_userISDUHandler(index, subindex, dataStart, dataLen, isWrite)) {
            if (isWrite) {
                isduResponseBuffer[0] = 0x20 | (payloadLen & 0x0F);
                isduResponseSize = 1;
            } else {
                // For reads, user fills response buffer
                _userISDUHandler(index, subindex, &isduResponseBuffer[1], dataLen, isWrite);
                isduResponseBuffer[0] = 0x10 | (dataLen & 0x0F);
                isduResponseSize = 1 + dataLen;
            }
            isduResponsePending = true;
            return;
        }
    }

    // Handle vendor test index (0x9999) for diagnostics
    if (service == 0xA && index == 0x9999) {
        // Trigger a synthetic event for testing purposes
        SendEvent(EventCodes::HARDWARE_FAULT, 3); // Error
        // Prepare a simple ACK (Write Confirm) response
        isduResponseBuffer[0] = 0x20 | (payloadLen & 0x0F);
        isduResponseSize = 1;
        isduResponsePending = true;
        return;
    }

    // Validate payload length against buffer size
    if ((service == 0xA || service == 0x2) && (static_cast<uint16_t>(3) + payloadLen + 1 > size)) {
        // Malformed PDU
        isduResponseBuffer[0] = 0xF0 | (service & 0x0F);
        isduResponseBuffer[1] = 0x80;
        isduResponseBuffer[2] = 0x30;
        isduResponseSize = 3;
        isduResponsePending = true;
        return;
    }

    
    if (service == 0x9 || service == 0x1) { // READ
        const void* pData = nullptr;
        uint8_t pLen = 0;
        bool found = false;
        uint32_t tempCRC = 0;
        uint8_t tempBuf[4];

        if (index == 0x0010) { pData = g_settings.VendorName; pLen = strlen(g_settings.VendorName); found = true; }
        else if (index == 0x0012) { pData = g_settings.ProductName; pLen = strlen(g_settings.ProductName); found = true; }
        else if (index == 0x0018) { pData = g_settings.ApplicationTag; pLen = strlen(g_settings.ApplicationTag); found = true; }
        else if (index == 0x001A) { // Parameter Checksum
            tempCRC = g_settings.ParameterChecksum; // Ensure correct endianness (Big Endian)
            tempBuf[0] = (tempCRC >> 24) & 0xFF;
            tempBuf[1] = (tempCRC >> 16) & 0xFF;
            tempBuf[2] = (tempCRC >> 8) & 0xFF;
            tempBuf[3] = tempCRC & 0xFF;
            pData = tempBuf;
            pLen = 4;
            found = true;
        }
        
        if (found) {
            // Build Read Response (truncate if necessary)
            uint8_t effectiveLen = pLen;
            if (effectiveLen > (ODSize.Op - 1)) {
                // Header takes 1 byte, leaving ODSize-1 for payload
                effectiveLen = 15; 
            }
            
            // Check buffer overflow for response buffer too
            if (effectiveLen > (sizeof(isduResponseBuffer)-1)) effectiveLen = sizeof(isduResponseBuffer)-1;

            isduResponseBuffer[0] = 0x10 | (effectiveLen & 0x0F);
            memcpy(&isduResponseBuffer[1], pData, effectiveLen);
            isduResponseSize = 1 + effectiveLen;
            isduResponsePending = true;
        } else {
             // Error Response: Index Not Available
             isduResponseBuffer[0] = 0xF0 | (service & 0x0F); 
             isduResponseBuffer[1] = 0x80; // Error Code High
             isduResponseBuffer[2] = 0x30; // Error Code Low (Index Not Available)
             isduResponseSize = 3;
             isduResponsePending = true;
        }

    } else if (service == 0xA || service == 0x2) { // WRITE
        dataLen = payloadLen;
        dataStart = &buffer[4];
        
        bool found = false;
        bool writeSuccess = true;
        uint16_t errorCode = 0;
        
        if (index == 0x0010 || index == 0x0012 || index == 0x0018) {
             found = true;
             if (index == 0x0010) { // VendorName (64 max)
                 if (dataLen > 64) { errorCode = 0x8030; writeSuccess = false; }
                 else {
                     memset(g_settings.VendorName, 0, 64);
                     memcpy(g_settings.VendorName, dataStart, dataLen);
                 }
             } else if (index == 0x0012) { // ProductName
                 if (dataLen > 64) { errorCode = 0x8030; writeSuccess = false; }
                 else {
                     memset(g_settings.ProductName, 0, 64);
                     memcpy(g_settings.ProductName, dataStart, dataLen);
                 }
             } else if (index == 0x0018) { // ApplicationTag
                 if (dataLen > 32) { errorCode = 0x8030; writeSuccess = false; }
                 else {
                     memset(g_settings.ApplicationTag, 0, 32);
                     memcpy(g_settings.ApplicationTag, dataStart, dataLen);
                 }
             }

             if (writeSuccess) {
                 UpdateParameterChecksum();
                 EEPROM.put(0, g_settings);
                 EEPROM.commit();
                 SendEvent(EventCodes::PARAM_CHANGED, 1);
             }
        } else {
             // Index Not Found
             found = false;
             errorCode = 0x8030;
        }
        
        if (found && writeSuccess) {
            isduResponseBuffer[0] = 0x20 | (payloadLen & 0x0F);
            isduResponseSize = 1;
            isduResponsePending = true;
        } else {
            isduResponseBuffer[0] = 0xF0 | (service & 0x0F);
            isduResponseBuffer[1] = (errorCode >> 8) & 0xFF;
            isduResponseBuffer[2] = errorCode & 0xFF;
            isduResponseSize = 3;
            isduResponsePending = true;
        }
    }
}

uint8_t Device::GetMseqCap() const
{
    uint8_t PreopCode;
    uint8_t OpCode;
    // Encode Preop size per spec Table A.8
    if (ODSize.Preop == 1) PreopCode = 0;
    else if (ODSize.Preop == 2) PreopCode = 1;
    else if (ODSize.Preop == 8) PreopCode = 2;
    else PreopCode = 3;
    
    // Encode Op size per spec Table A.10
    if (ODSize.Op == 1)
    {
        if ((Pd.In.Size > 0 && Pd.Out.Size >= 3) ||
            (Pd.In.Size >= 3 && Pd.Out.Size > 0)
       )
        {
            OpCode = 4;
        }
        else
        {
            OpCode = 0;
        }
    }
    else if (ODSize.Op == 2)
    {
        if (Pd.In.Size == 0 && Pd.Out.Size == 0)
        {
            OpCode = 1;
        }
        else
        {
            OpCode = 5;
        }
    }
    else if (ODSize.Op == 8)
    {
        OpCode = 6;
    }
    else
    {
        OpCode = 7;
    }
    return PreopCode << 4 | OpCode << 1 | static_cast<uint8_t> (ISDUSupported);
}

uint32_t Device::DecodeCycleTime(uint8_t encoded_time) const
{
    return Utils::DecodeCycleTime(encoded_time);
}

uint8_t Device::EncodeCycleTime(uint32_t cycleTime_us) const
{
    return Utils::EncodeCycleTime(cycleTime_us);
}

// Static member initialization
Device::ODSize_t Device::ODSize = {
    .Startup = 2,  // Force Type 1 (2 bytes)
    .Preop = 2,    // Force Type 1 (2 bytes)
    .Op = 2        // Force Type 1 (2 bytes)
};

Device::Device(uint8_t PDIn, uint8_t PDOut, uint32_t min_cycletime)
{
    EventMemory[EventStatusCodeAddr] = Event::StatusCodeDefault;
    Pd.Out.Size = PDOut;
    Pd.In.Size = PDIn;
    
    // Increase ODSize to 16 to support larger ISDU frames
    ODSize.Startup = 16;
    ODSize.Preop   = 16;
    ODSize.Op      = 16;
    
    // MAP SETUP (Standard V1.1 - Direct Indexing)
    ParameterPage1[0x00] = 0;                              // MasterCommand
    ParameterPage1[0x01] = 0;                              // MasterCycleTime
    // Set MinCycleTime to 10ms (encoded as 0x59: TimeBase 01 + Multiplier 25)
    ParameterPage1[0x02] = 0x59;                           // MinCycleTime: 10ms
    
    ParameterPage1[0x03] = GetMseqCap();                   // MSeqCap (now 0x11 for Type 1)
    ParameterPage1[0x04] = 0x11;                           // RevisionID (V1.1)
    ParameterPage1[0x05] = EncodePD(PDIn);                 // ProcessDataIn
    ParameterPage1[0x06] = EncodePD(PDOut);                // ProcessDataOut
    ParameterPage1[0x07] = 0xAB;                           // VID1 (Vendor ID MSB)
    ParameterPage1[0x08] = 0xCD;                           // VID2 (Vendor ID LSB)
    ParameterPage1[0x09] = 0x00;                           // DID1 (Device ID MSB)
    ParameterPage1[0x0A] = 0x00;                           // DID2
    ParameterPage1[0x0B] = 0x05;                           // DID3 (Device ID LSB)
    ParameterPage1[0x0C] = 0x00;                           // FID1
    ParameterPage1[0x0D] = 0x00;                           // FID2
    
    CurrentCycleTime = DecodeCycleTime(ParameterPage1[0x02]);
}

bool Device::GetPDOut(uint8_t *buffer, PDStatus *pStatus) const
{
    memcpy(buffer, Pd.Out.Data, Pd.Out.Size);
    if (deviceMode == operate)
    {
        *pStatus = status.PDOut;
        return true;
    }
    else
    {
        return false;
    }
}

        
Device::EventResult Device::SetEvent(Event::POD newEvent)
{
   EventResult result = EventResult::EventOK;
   do
   {
        if (ReadingEventMemory == true)
        {
            result = EventResult::ProcessingEvents;
            break;
        }
        if (TotalEvents < MaxEvents )
        {
           const uint8_t memory_offset = Event::SizeStatusCode + (TotalEvents * Event::SizeRawEvent);
           const uint8_t QualifierOffset = memory_offset;
           const uint8_t EventCodeMSB = memory_offset + 1U;
           const uint8_t EventCodeLSB = memory_offset + 2U;
           EventMemory[QualifierOffset] = newEvent.EventQualifier;
           EventMemory[EventCodeMSB] = static_cast<uint8_t>(newEvent.EventCode >> 8U);
           EventMemory[EventCodeLSB] = static_cast<uint8_t>(newEvent.EventCode & 0xFF);

           EventMemory[EventStatusCodeAddr] |= 1UL << TotalEvents;
           
           TotalEvents++;
        }
        else
        {
            result = EventResult::EventMemoryFull;
            break;
        }
      
   } while(0);   
   return result;
   
}

// Event queue with priority sorting
void Device::SendEvent(uint16_t code, uint8_t qualifier)
{
    // Prevent duplicates (simple linear search)
    for (uint8_t i = 0; i < eventCount; ++i) {
        if (eventQueue[i].Code == code) return; // already present
    }

    if (eventCount < EVENT_QUEUE_SIZE) {
        eventQueue[eventCount].Code = code;
        eventQueue[eventCount].Qualifier = qualifier;
        eventQueue[eventCount].Active = true;
        eventCount++;
        SortEvents(); // Keep high-priority events first
    }
}

// Bubble sort (small fixed-size queue, acceptable)
void Device::SortEvents()
{
    for (int i = 0; i < (int)eventCount - 1; ++i) {
        for (int j = 0; j < (int)eventCount - i - 1; ++j) {
            if (eventQueue[j].Qualifier < eventQueue[j+1].Qualifier) {
                EventObj tmp = eventQueue[j];
                eventQueue[j] = eventQueue[j+1];
                eventQueue[j+1] = tmp;
            }
        }
    }
}

bool Device::HasEvents()
{
    return eventCount > 0;
}

void Device::HandleDiagnosis(uint8_t addr, uint8_t* outByte)
{
    if (eventCount == 0) { *outByte = 0; return; }
    EventObj &e = eventQueue[0]; // Highest priority at index 0

    if (addr == 0) *outByte = static_cast<uint8_t>((e.Code >> 8) & 0xFF);
    else if (addr == 1) *outByte = static_cast<uint8_t>(e.Code & 0xFF);
    else if (addr == 2) {
        *outByte = e.Qualifier;
        // Remove head by shifting
        for (uint8_t i = 0; i + 1 < eventCount; ++i) eventQueue[i] = eventQueue[i+1];
        eventCount--;
        // Zero the freed slot
        if (eventCount < EVENT_QUEUE_SIZE) eventQueue[eventCount].Active = false;
    } else *outByte = 0;
}

lwIOLink::Mode Device::GetMode() const
{
    return deviceMode;
}

bool Device::SetPDInStatus(PDStatus pd_status)
{
    if (deviceMode == operate)
    {
        status.PDIn = pd_status;
        return true;
    }
    else
    {
        return false;
    }
}

bool Device::SetPDIn(uint8_t *pData, uint8_t len)
{
    if (deviceMode == operate)
    {
        if (len <= Pd.In.Size)
        {
            memcpy(Pd.In.Data, pData, len);
        }
        return true;
    }
    else
    {
        return false;
    }
}

void Device::ProcessMessage()
{
    const uint8_t MasterOD_offset = MCSize + ChecksumSize; // start of OD within rxBuffer
    ParseMC();
    memset(ODBuffer, 0, sizeof(ODBuffer));	//Clear OD
    
    // STATE MACHINE ENFORCEMENT
    // START: Only Page (Param) and Diagnosis allowed.
    // PREOPERATE: ISDU Allowed. Cyclic PD Forbidden.
    // OPERATE: All allowed.
    
    bool allowCyclic = (deviceMode == operate);
    bool allowISDU = (deviceMode == preoperate || deviceMode == operate);

    const uint8_t od_size = (deviceMode == start) ? ODSize.Startup
                        : (deviceMode == preoperate) ? ODSize.Preop
                        : ODSize.Op;
    
    switch (message.channel)
    {
        case Page:
             // Page 1 (DP1) addresses 0x00-0x0F, Page 2 maps addresses 0x10-0x1F
            if (MasterAccess == MCAccess::Read)
            {
                if (message.addr < 0x10) {
                    ODBuffer[0] = ParameterPage1[message.addr];
                } else {
                    // Page 2: ProductName data from address 0x10-0x1F
                    uint8_t offset = message.addr - 0x10;
                    if (offset < strlen(g_settings.ProductName)) {
                        ODBuffer[0] = g_settings.ProductName[offset];
                    } else {
                        ODBuffer[0] = 0x00; // Padding
                    }
                }
            }
            else
            {
                auto write_param = static_cast<DP1_Param> (message.addr);
                const uint8_t ODWrite = rxBuffer[MasterOD_offset];
                
                if (write_param == DP1_Param::MasterCommand) {
                    Cmd = static_cast<MasterCommands> (ODWrite);
                    NewCmd = true;
                }
                else if (write_param == DP1_Param::MasterCycleTime) {
                    CurrentCycleTime = DecodeCycleTime(ODWrite);
                }
                else if (write_param == DP1_Param::SystemCommand) {
                    switch (ODWrite) {
                        case 0x80: // Device Reset
                            watchdog_reboot(0, 0, 0);
                            break;
                        case 0x82: // Restore Factory Settings
                            g_settings.magic = 0;
                            EEPROM.put(0, g_settings);
                            EEPROM.commit();
                            watchdog_reboot(0, 0, 0);
                            break;
                        case 0xA0: // Parametrization Start
                            parametrization_active = true;
                            break;
                        case 0xA1: // Parametrization End
                            parametrization_active = false;
                            break;
                    }
                }
            }
            break;
            
        case ISDU:
            if (allowISDU) {
                if (MasterAccess == MCAccess::Write) {
                     HandleISDU(&rxBuffer[MasterOD_offset], od_size, false);
                } else {
                     HandleISDU(nullptr, 0, true);
                }
            }
            break;
        
        case Process:
             // Process Data Channel - only allowed in OPERATE mode
             if (allowCyclic) {
                 const uint8_t *rxOD = &rxBuffer[MasterOD_offset];
                 const uint8_t *rxPDOut = &rxBuffer[MasterOD_offset + od_size];
                 memcpy(Pd.Out.Data, rxPDOut, Pd.Out.Size);

                 // Handle embedded ISDU request in OD (0x00 = Idle)
                 if (od_size > 0 && rxOD[0] != 0x00) {
                     HandleISDU(const_cast<uint8_t*>(rxOD), od_size, false);

                     // Put response into ODBuffer so SetResponse() transmits it back
                     if (isduResponsePending) {
                         const uint8_t safeSize = (isduResponseSize > od_size) ? od_size : isduResponseSize;
                         memcpy(ODBuffer, isduResponseBuffer, safeSize);
                         if (safeSize < od_size) {
                             memset(&ODBuffer[safeSize], 0, od_size - safeSize);
                         }
                         isduResponsePending = false;
                     }
                 } else {
                     // No request: keep OD idle in response
                     if (od_size > 0) memset(ODBuffer, 0, od_size);
                 }
             }
             break;
             
        case Diagnosis:
            if (MasterAccess == MCAccess::Read) {
                if (message.addr < 3) {
                    // Lightweight event queue interface for quick reads
                    HandleDiagnosis(static_cast<uint8_t>(message.addr), &ODBuffer[0]);
                    if (ODSize.Op > 1) memset(&ODBuffer[1], 0, ODSize.Op - 1);
                } else {
                    // Full EventMemory fallback
                    if(TotalEvents > 0 && !ReadingEventMemory) {
                        ReadingEventMemory = true;
                    }
                    if(message.addr < sizeof(EventMemory)) {
                        const uint8_t read_bytes = sizeof(EventMemory) - message.addr;
                        uint8_t od_size;
                        // Determine ODSize for response based on Mode
                        if (deviceMode == start) od_size = ODSize.Startup;
                        else if (deviceMode == preoperate) od_size = ODSize.Preop;
                        else od_size = ODSize.Op;
                        
                        uint8_t copy_size = (read_bytes < od_size) ? read_bytes : od_size;
                        memcpy(ODBuffer,&EventMemory[message.addr],copy_size);
                    }
                }
            } else {
                // Acknowledge and clear pending events
                if (message.addr == EventStatusCodeAddr) {
                    ReadingEventMemory = false;
                    TotalEvents = 0;
                    EventMemory[EventStatusCodeAddr] = Event::StatusCodeDefault;
                    EventsProcessed = true;
                    for (uint8_t i = 0; i < eventCount; ++i) eventQueue[i].Active = false;
                    eventCount = 0;
                }
            }
            break;
    }
}

uint8_t Device::SetResponse()
{
    memset(txBuffer, 0, sizeof(txBuffer));
    
    uint8_t channel = (lastMC & MC_ChMask) >> MC_ChBitOffset;
    bool isType0 = (channel == Page || channel == Diagnosis);

    if (isType0) {
    // --- TYPE 0 FRAME RESPONSE ---
    txBuffer[0] = lastMC;
    txBuffer[1] = lastCKT; 
    
    bool eventFlag = (HasEvents() || (TotalEvents > 0));
    uint8_t status_val = static_cast<uint8_t>(status.PDIn << status_bit_offset)
                       | static_cast<uint8_t>(eventFlag << eventFlag_bit_offset);
    
    if (MasterAccess == MCAccess::Read)
        {
            
            txBuffer[2] = ODBuffer[0]; 
            
            uint8_t mseq_bits = txBuffer[1] & mseq_ckt_mask;
            txBuffer[1] = mseq_bits;
            uint8_t ck6 = GetChecksum(txBuffer, 3);
            txBuffer[1] = mseq_bits | (ck6 & ck6_mask);
            return 3; 
        } else {
            // Write response: [MC] [CKT] = 2 bytes
            txBuffer[0] = lastMC; 
            
            uint8_t mseq_bits = lastCKT & mseq_ckt_mask;
            uint8_t calcBuf[2] = {txBuffer[0], mseq_bits};
            uint8_t ck6 = GetChecksum(calcBuf, 2);
            txBuffer[1] = mseq_bits | (ck6 & ck6_mask);
            return 2;
        }
    }

    // Type 1/2 frames
    bool isRead = (lastMC >> 7) == static_cast<uint8_t>(MCAccess::Read);
    uint8_t tx_size = ChecksumSize; 
    uint8_t od_size = 0;
    uint8_t pd_size = 0;
    
    switch (deviceMode) {
        case start: od_size = ODSize.Startup; break;
        case preoperate: od_size = ODSize.Preop; break;
        case operate: od_size = ODSize.Op; pd_size = Pd.In.Size; break;
    }
    
    if (isRead) {
        memcpy(txBuffer, ODBuffer, od_size);
        tx_size += od_size;
    }
    
    if (deviceMode == operate) {
        uint8_t pd_offset = (isRead) ? od_size : 0;
        
        // Use linked external data source if available, otherwise use internal buffer
        if (this->processDataIn != nullptr) {
             memcpy(&txBuffer[pd_offset], this->processDataIn, this->processDataSize);
        } else {
             memcpy(&txBuffer[pd_offset], Pd.In.Data, pd_size);
        }
        
        tx_size += pd_size;
    }
    
    const uint8_t checksum_offset = tx_size - 1;
    bool eventFlag = (HasEvents() || (TotalEvents > 0));
    const uint8_t status_encoded = static_cast<uint8_t>(status.PDIn << status_bit_offset) | static_cast<uint8_t>(eventFlag << eventFlag_bit_offset);
    txBuffer[checksum_offset] = status_encoded;
    uint8_t ck6 = GetChecksum(txBuffer, tx_size);
    txBuffer[checksum_offset] |= ck6;
    
    return tx_size;
}


inline void Device::ParseMC()
{
    message.channel = (rxBuffer[MCOffset] & MC_ChMask) >> MC_ChBitOffset;
    message.addr = rxBuffer[MCOffset] & MC_AddrMask;
}


void Device::begin(const HWConfig config)
{
    // Initialize persistent settings from EEPROM
    EEPROM.begin(512);
    EEPROM.get(0, g_settings);
    if (g_settings.magic != SETTINGS_MAGIC) {
        g_settings.magic = SETTINGS_MAGIC;
        //strncpy(g_settings.VendorName, "MyVendor", sizeof(g_settings.VendorName));
        // To this (Long string to force truncation):
        strncpy(g_settings.VendorName, "MyVendorNameIsSuperLongAndExceedsbuffer", sizeof(g_settings.VendorName));
        strncpy(g_settings.ProductName, "MyDevice", sizeof(g_settings.ProductName));
        memset(g_settings.ApplicationTag, 0, sizeof(g_settings.ApplicationTag));
        strncpy(g_settings.ApplicationTag, "MyID", sizeof(g_settings.ApplicationTag));
        g_settings.ParameterChecksum = 0; // Initialize
        UpdateParameterChecksum(); // Calculate initial
        EEPROM.put(0, g_settings);
        EEPROM.commit();
    }

    TxEn = config.Pin.TxEN;
    WuPin = config.Pin.Wakeup;
    SerialPort = &config.SerialPort;

#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
    // Native Pico SDK UART initialization
    uart_init(PICO_UART_ID, config.Baud);
    gpio_set_function(PICO_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(PICO_UART_ID, 8, 1, UART_PARITY_EVEN);
    uart_set_hw_flow(PICO_UART_ID, false, false);
    uart_set_fifo_enabled(PICO_UART_ID, true);
    // Flush initial receive buffer
    while(uart_is_readable(PICO_UART_ID)) uart_getc(PICO_UART_ID);
#else
    static_cast<HardwareSerial*>(SerialPort)->begin(static_cast<uint32_t>(config.Baud), SERIAL_8E1);
    while(SerialPort->available()) SerialPort->read();
#endif

    tbit_us = 1000000 / static_cast<uint32_t>(config.Baud);
    initTransciever(config.WakeupMode);

    // Link the internal settings to the public pointer
    this->parameters = &g_settings;
}

void Device::attachISDUCallback(ISDUHandler handler) {
    _userISDUHandler = handler;
}

inline uint8_t Device::GetMasterTXSize()
{
    // Extract Channel and Access from the MC byte
    uint8_t channel = (lastMC & MC_ChMask) >> MC_ChBitOffset;
    bool isRead = (lastMC >> 7) == static_cast<uint8_t>(MCAccess::Read);
    uint8_t addr = lastMC & MC_AddrMask;
    
    // Type 0 frames: Page and Diagnosis channels
    if (channel == Page || channel == Diagnosis) {
        // Type 0 frame format: [MC][CKT] + optional payload
        return isRead ? 2 : 3;
    }
    
    // Type 1/2 frames: Process and ISDU channels
    // Note: Process PDOut frames may carry payload even with "Read" access bit
    uint8_t od_size = 0;
    uint8_t pd_size = 0;

    // Cyclic PDOut includes OD + PDOut even if access bit is "Read" (MC=0x80)
    bool isCyclicPDOutOperate = (channel == Process) && ((lastMC == 0x80) || 
        (addr == (static_cast<uint8_t>(MasterCommands::PDOutOperate) & MC_AddrMask)));
    if (isCyclicPDOutOperate) {
        if (deviceMode == start) od_size = ODSize.Startup;
        else if (deviceMode == preoperate) od_size = ODSize.Preop;
        else od_size = ODSize.Op;

        pd_size = Pd.Out.Size;
        return MasterMetadataOffset + od_size + pd_size;
    }
    
    if (deviceMode == start)
    {
        if (!isRead)
        {
            od_size = ODSize.Startup;
            pd_size = Pd.Out.Size;
        }
    }
    else if (deviceMode == preoperate)
    {
        if (!isRead)
        {
            od_size = ODSize.Preop;
            pd_size = Pd.Out.Size;
        }
    }
    else  // operate mode
    {
        if (!isRead)
        {
            od_size = ODSize.Op;
            pd_size = Pd.Out.Size;
        }
    }
    return MasterMetadataOffset + od_size + pd_size;
}

void Device::SaveMasterFrame(const uint8_t rx_byte)
{
    // Reset on inter-byte timeout (5ms)
    if (rxCnt > 0) {
        unsigned long now = micros();
        if ((now - lastByteTimestamp) > 5000) {
            rxCnt = 0;
            ExpectedRXCnt = 0xFF;
        }
    }
    lastByteTimestamp = micros();

    rxBuffer[rxCnt] = rx_byte; 
    
    // Determine expected frame size based on MC
    if (rxCnt == MCOffset)
    {
        lastMC = rxBuffer[MCOffset];
        MasterAccess = static_cast<MCAccess>(rxBuffer[MCOffset] >> 7);
        ExpectedRXCnt = GetMasterTXSize();
    }
    
    rxCnt++;
    
    // Protect against buffer overflow
    if (rxCnt > ExpectedRXCnt && ExpectedRXCnt != 0xFF) { 
        rxCnt = 0; 
        ExpectedRXCnt = 0xFF; 
        return; 
    }

    // Validate complete frame
    if (rxCnt == ExpectedRXCnt)
    {
        const uint8_t original_ckt = rxBuffer[ckt_offset];
        lastCKT = original_ckt;
        
        const uint8_t master_checksum = original_ckt & ck6_mask;
        
        // Calculate checksum (ignoring CKT checksum bits)
        rxBuffer[ckt_offset] &= mseq_ckt_mask;
        const uint8_t calculated_checksum = GetChecksum(rxBuffer, rxCnt);
        rxBuffer[ckt_offset] = original_ckt;

        if (master_checksum == calculated_checksum) {
            NewMasterMsg = true;
            currentMSeq = static_cast<MSeqType>((original_ckt & mseq_ckt_mask) >> 6);
        } else {
            // Checksum mismatch - log diagnostic info if enabled
            if (LOGGING_ENABLED) {
                Serial.print("\n[ERR] Checksum mismatch (calc=0x");
                Serial.print(calculated_checksum, HEX);
                Serial.print(", rx=0x");
                Serial.print(master_checksum, HEX);
                Serial.print(")\n[BUF] ");
                for(uint8_t i=0; i<rxCnt; i++) {
                    Serial.print("0x");
                    Serial.print(rxBuffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
            }
            rxCnt = 0;
            ExpectedRXCnt = 0xFF;
        }
    }
}

void Device::ResetRX()
{
    rxCnt = 0;
    ExpectedRXCnt = 0xFF;
#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
    while(uart_is_readable(PICO_UART_ID)) uart_getc(PICO_UART_ID);
#else
    while (SerialPort->available() > 0) SerialPort->read();
#endif
}

void Device::initTransciever(int wakeup_mode) const
{
    pinMode(TxEn, OUTPUT);
    pinMode(WuPin, INPUT_PULLUP);
    digitalWrite(TxEn, LOW);  // Start in RX mode (TIOL1123 active HIGH)
    g_WuPin = WuPin;

#ifdef ARDUINO_RASPBERRY_PI_PICO
    // WAKEUP INTERRUPT DISABLED FOR VALIDATION SCENARIO
    // See WakeupIRQ() implementation above for rationale.
    // To enable: Uncomment line below and rebuild
    // attachInterrupt(digitalPinToInterrupt(WuPin), WakeupIRQ, wakeup_mode);
    (void)wakeup_mode; 
#else
    // FOR OTHER PLATFORMS: Uncomment to enable wakeup detection
    // attachInterrupt(digitalPinToInterrupt(WuPin), WakeupIRQ, CHANGE);
#endif
}

void Device::DeviceRsp(uint8_t *data, uint8_t len, bool isWrite)
{
    // Enable transmitter
    delayMicroseconds(10); 
    digitalWrite(TxEn, HIGH);

#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
    uart_write_blocking(PICO_UART_ID, data, len);
    uart_tx_wait_blocking(PICO_UART_ID);
    
    // Consume echo bytes (half-duplex bus)
    for (uint8_t i = 0; i < len; i++) {
        uint32_t timeout = 0;
        while (!uart_is_readable(PICO_UART_ID) && timeout < 2000) {
            delayMicroseconds(1);
            timeout++;
        }
        if (uart_is_readable(PICO_UART_ID)) {
            volatile uint8_t echo = uart_getc(PICO_UART_ID);
            (void)echo;
        }
    }
    // Flush any remaining bytes
    while(uart_is_readable(PICO_UART_ID)) uart_getc(PICO_UART_ID);
#else
    for (uint8_t i = 0; i < len; i++) SerialPort->write(data[i]);
    SerialPort->flush();
#endif

    // Return to receive mode
    delayMicroseconds(5); 
    digitalWrite(TxEn, LOW);
    
    // Reset state machine for next frame
    rxCnt = 0;
    ExpectedRXCnt = 0xFF;
}

void Device::run()
{
    // Lambdas to abstract hardware reading
    auto isDataAvailable = [&]() -> bool {
#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
        return uart_is_readable(PICO_UART_ID);
#else
        return SerialPort->available() > 0;
#endif
    };
    
    auto readByte = [&]() -> uint8_t {
#if defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ARCH_RP2040)
        return uart_getc(PICO_UART_ID);
#else
        return SerialPort->read();
#endif
    };

    switch (deviceState)
    {
        case wait_wake:
            deviceState = wait_valid_frame;
            digitalWrite(TxEn, LOW);
            break;
            
        case wait_valid_frame:
            if (isDataAvailable()) {
                uint8_t b = readByte();
                // --- DEBUG: SEE IF BYTES ARE ARRIVING ---
                // If you see this, Hardware is OK, Stack is the issue.
                // If you don't see this, Hardware is BROKEN.
                if (LOGGING_ENABLED) { Serial.print("."); } 
                // ----------------------------------------
                SaveMasterFrame(b);
                if (NewMasterMsg) {
                    deviceState = run_mode;
                    Serial.println("\n[STATE] wait_valid_frame -> run_mode (Frame received)");
                }
            }
            break;
            
        case run_mode:
        {
            unsigned long current_time = micros();
            while (isDataAvailable()) {
                SaveMasterFrame(readByte());
            }
            
            // Timeout guard for cycle loss (disabled for debugging)
            if (deviceMode == operate &&
                (current_time - LastMessage) > CurrentCycleTime)
            {
                // Optional: Reset if silence is too long (disabled for debugging)
                // deviceMode = start;
                // deviceState = wait_wake;
                // Serial.println("[TIMEOUT] Cycle lost");
            }
            
            if (NewMasterMsg)
            {
                NewMasterMsg = false;
                uint8_t capturedRxLen = rxCnt;
                uint8_t capturedMC = lastMC;

                ProcessMessage();
                const uint8_t tx_size = SetResponse();
                bool isWrite = (MasterAccess == MCAccess::Write);

                DeviceRsp(txBuffer, tx_size, isWrite);

                if (LOGGING_ENABLED)
                {
                    Serial.print("\n[RX] MC=0x");
                    Serial.print(capturedMC, HEX);
                    Serial.print(" Data: ");
                    for (uint8_t i = 0; i < capturedRxLen; i++) {
                        Serial.print(rxBuffer[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
                    
                    Serial.print("[TX] ");
                    for (uint8_t i = 0; i < tx_size; i++) {
                        Serial.print(txBuffer[i], HEX);
                        Serial.print(" ");
                    }
                    Serial.println();
                }

                if (EventsProcessed == true)
                {
                    EventsProcessed = false;
                    OnEventsProcessed();
                }
                LastMessage = micros();
                if (deviceMode == operate)
                {
                    OnNewCycle();
                }
            }
            break;
        }
    }
    
    // Process commands
    if (NewCmd)
    {
        NewCmd = false;
        switch (Cmd)
        {
            case MasterCommands::Operate:
                LastMessage = micros();
                deviceMode = operate;
                status.PDOut = Invalid;
                Serial.println("[CMD] Mode -> OPERATE");
                break;
            case MasterCommands::PDOutOperate:
                status.PDOut = Valid;
                Serial.println("[CMD] PDOut -> VALID");
                break;
            case MasterCommands::Preoperate:
                deviceMode = preoperate;
                Serial.println("[CMD] Mode -> PREOPERATE");
                break;
        }
    }
}