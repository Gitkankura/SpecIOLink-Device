// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#pragma once
#include <stdint.h>
#include <Arduino.h>

namespace lwIOLink
{   

    //IO-Link device modes
    enum Mode
    {
        start,
        preoperate,
        operate
    };

    // Persisted settings structure
    struct DeviceSettings {
        uint32_t magic;
        char VendorName[64];
        char ProductName[64];
        char ApplicationTag[32];
        uint32_t ParameterChecksum;
        // NOTE: DS_Upload_Request removed. DS is now signaled via standard Event: 0x6350
    };

    // Standard IO-Link Event Codes
    namespace EventCodes {
        constexpr uint16_t NO_EVENT = 0x0000;
        constexpr uint16_t TEMP_OVERRUN = 0x4000;
        constexpr uint16_t HARDWARE_FAULT = 0x5000;
        constexpr uint16_t PARAM_CHANGED = 0x6350; // Triggers Data Storage
    }

    // Event Object used by the priority queue
    struct EventObj {
        uint16_t Code;
        uint8_t Qualifier; // 1=Notification, 2=Warning, 3=Error
        bool Active;
    };

    // Utility functions for testing
    namespace Utils
    {
        // Static utility functions that can be tested independently
        uint32_t DecodeCycleTime(uint8_t encoded_time);
        uint8_t EncodeCycleTime(uint32_t cycleTime_us);
        uint32_t CalculateCRC32(const void* data, size_t length);
    }
    
    // IO-Link Protocol Enums - moved from private Device class for reusability
    
    // Communication channels (Table A.1)
    enum Channel
    {
        Process = 0,
        Page,
        Diagnosis,
        ISDU
    };
    
    // Master Command access types (Table A.2)
    enum class MCAccess
    {
        Read = 1,
        Write = 0
    };
    
    // M-sequence types (Table A.3)
    enum class MSeqType
    {
        Type0 = 0,
        Type1,
        Type2
    };
    
    // Master Commands (Table B.2)
    enum MasterCommands
    {
        MasterIdent = 0x95,
        DeviceIden = 0x96,
        DeviceStartup = 0x97,
        PDOutOperate = 0x98,
        Operate = 0x99,
        Preoperate = 0x9A,
    };
    
    // Direct Parameter Page 1 addresses (Table B.1)
    enum DP1_Param
    {
        MasterCommand    = 0x00,
        MasterCycleTime  = 0x01,
        MinCycleTime     = 0x02,  // CRITICAL: Master timeout calculation depends on this
        MSeqCap          = 0x03,
        RevisionID       = 0x04,  // MUST be 0x11 for IO-Link V1.1
        ProcessDataIn    = 0x05,
        ProcessDataOut   = 0x06,
        VID1             = 0x07,  // Vendor ID MSB
        VID2             = 0x08,  // Vendor ID LSB
        DID1             = 0x09,  // Device ID MSB
        DID2             = 0x0A,
        DID3             = 0x0B,  // Device ID LSB
        FID1             = 0x0C,
        FID2             = 0x0D,
        Reserved_0E      = 0x0E,
        SystemCommand    = 0x0F
    };
    
    //IO-Link baudrates
    enum BaudRate: uint32_t
    {
        COM1 = 4800,
        COM2 = 38400,
        COM3 = 230400
    };
    //Table A.5
    enum PDStatus
    {
        Valid = 0,
        Invalid = 1
    };

    namespace Event
    {
        constexpr unsigned SizeRawEvent = 3U;
        constexpr unsigned SizeStatusCode = 1U;
        constexpr unsigned StatusCodeDefault = 0x80; //Event details << 6
        //Figure A.24
        namespace Qualifier
        {
            namespace BitOffset
            {
               constexpr unsigned  Instance = 0x00;
               constexpr unsigned Source = 0x03;
               constexpr unsigned Type = 0x04;
               constexpr unsigned Mode = 0x06;
            }
            enum Instance
            {
                Unknown = 0x00,
                Rsv1 = 0x01,
                Rsv2 = 0x02,
                Rsv3 = 0x03,
                Application = 0x04,
                Rsv5 = 0x05,
                Rsv6 = 0x06,
                Rsv7 = 0x07
            };  
            enum Source
            {
                Device = 0x00,
                Master = 0x01
            };  
            enum Type
            {
                Reserved = 0x00,
                SingleShot = 0x01,
                Disappears = 0x02,
                Appears =  0x03
            };
        }
        // Table 58
        struct POD
        {
            POD(Qualifier::Instance instance,Qualifier::Type type,uint16_t event_code)
            {
                EventQualifier = instance << Qualifier::BitOffset::Instance
                                 | Qualifier::Master << Qualifier::BitOffset::Source
                                 | type << Qualifier::BitOffset::Type;
                 EventCode = event_code;
            }
            POD()
            {
                EventQualifier = Qualifier::Master << Qualifier::BitOffset::Source;
                EventCode = 0x00;
            }
            uint8_t EventQualifier;
            uint16_t EventCode;
        };
    }

    // Definição do tipo de função para lidar com parâmetros customizados do utilizador
    typedef bool (*ISDUHandler)(uint16_t index, uint8_t subIndex, uint8_t* data, uint8_t len, bool isWrite);

    class Device
    {
      public:
        /*
           Device Constructor
           PDIn = total # bytes the device sends (Device->Master)
           PDOut = total # bytes the device gets (Master->Device)
           min_cycletime = minimum cycle time in us  (See B.1.3)
        */
        Device(uint8_t PDIn, uint8_t PDOut,uint32_t min_cycletime);
        
        struct HWConfig
        {
            Stream &SerialPort; 		 //reference to Arduino Serial port
            BaudRate Baud;			// baud rate for transmission
            int WakeupMode; 	/* Type of interrupt to detect wakeup from transceiver   
                            FALLING, RISING */
            struct Pin_t
            {
                unsigned TxEN;  	 //Digital Output pin used to enable TX of data
                unsigned Wakeup;	/* Digital Input pin used to get Wakeup Requests
                            Must be interrupt capable. */
    #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO)
                /* Some Arduino boards require to specify the UART Pins                 
                */
                unsigned Tx;	
                unsigned Rx;
    #endif
            };
            Pin_t Pin;
        };
        
        /*
           Begin the IOLink Device Hardware
        */
        void begin(const HWConfig config);
        /*
           Run the IOLink Device, should be called in a loop
        */
        void run();
        /*
          Get the Process Data Output
          buffer = Pointer to Buffer where to store PDOut
          pStats = Pointer to memory where status of PDOut will be saved
          Returns false if the device is not in operate mode
        */
        bool GetPDOut(uint8_t * buffer, PDStatus * pStatus) const;
        /*
          Sets the Process Data Input
          Returns false if the device is not in operate mode
          or the len > than the available PDIn Size
        */
        bool SetPDIn(uint8_t * pData, uint8_t len);
        /* 
         *  Sets the validity of PDIn
         *  Return false if not in operate mode
         *  By default the PDIN status is set to valid
         *  See Table A.5
         */
        bool SetPDInStatus(PDStatus pd_status);
        // Gets the current Mode of the IOLink device
        Mode GetMode() const;
        /* Callback for whenever an IOLink Cycle is completed in operate mode
	* Can be used to update the PDIN data, read a sensor or do a specific operation
	* for the application
	*/
        static void OnNewCycle(void);
        
        enum class EventResult
        {
            EventOK,         /* Event Added, will be processed by the master */
            EventMemoryFull, /* Cant add events to Device memory */
            ProcessingEvents /* The Master is already reading events, wait until it finished */
        };
        
        /*
          Add an event to the device, which will be processed by the Master Asynchronously
          Note: Function is not concurrent safe. Do not Call inside ISR
        */
        EventResult SetEvent(Event::POD newEvent);
        /* User Callback for whenever IO-Link Master has processed Events [Optional] 
	* Events sent from the IO-Link device are read from the so-called Event Memory.
	* The Master can take up some IO-Link MSeq Frames to complete reading the data.
	* This function is called once the Master confirms it has read the complete Event Memory
	* It can be useful to call this function in case more events are pending to be sent.
	*/
        static void OnEventsProcessed(void);

        // [NEW] Interface for Generic Sensors
        void attachISDUCallback(ISDUHandler handler);
        
        // NOTE: Wakeup Interrupt Handler (WakeupIRQ) is IMPLEMENTED but DISABLED
        // See lwIOLink.cpp for WakeupIRQ() implementation and rationale.
        // Can be enabled by uncommenting attachInterrupt() in initTransciever()
        
        // Public Pointers for Direct Memory Access from main.cpp
        DeviceSettings* parameters = nullptr; 
        uint8_t* processDataIn = nullptr;      
        uint8_t  processDataSize = 2;          

        // [MOVED FROM PRIVATE] Event API required for user logic
        void SendEvent(uint16_t code, uint8_t qualifier);
        bool HasEvents();

      private:
        // [NEW] Storage for user callback
        ISDUHandler _userISDUHandler = nullptr;
        
        static unsigned constexpr MaxPD = 32;
        static unsigned constexpr MaxOD = 32;
        static unsigned constexpr SizeDP1 = 16;
        static unsigned constexpr ChecksumSize = 1;
        static unsigned constexpr MCSize = 1;
        static unsigned constexpr MaxIOLMsgSize =  MCSize + ChecksumSize + MaxOD + MaxPD; 
        static unsigned constexpr MasterMetadataOffset = MCSize + ChecksumSize;
        static unsigned constexpr MCOffset = 0;
        static unsigned constexpr CKTFrameOffset = 1;
        static unsigned constexpr EventFlagBitOffset = 6; //Figure A.3
        static unsigned constexpr MaxEvents = 6; //Table 58
        static unsigned constexpr EventStatusCodeAddr = 0x00;
        
        /* Cycle time constants See B.3*/
        static unsigned constexpr TotalTimeEncodings = 3;
        static uint32_t constexpr TimeBaseLUT[TotalTimeEncodings] = {100, 400, 1600}; //us
        static uint32_t constexpr TimeOffsetLUT[TotalTimeEncodings] = {0,6400,32000}; //us
        
        
        uint8_t TotalEvents = 0;
        bool ReadingEventMemory = false;
        bool EventsProcessed = false;
        uint8_t EventMemory[Event::SizeStatusCode + (MaxEvents * Event::SizeRawEvent)];
        

        //ISDU Supported
        static bool constexpr ISDUSupported = true;
        
        // ISDU Error Codes (Table C.1)
        enum ISDUError {
            NoError = 0x00,
            IndexNotFound = 0x8030, // Service refused
            SubindexNotFound = 0x8030, // merged
            ServiceNotAvailable = 0x8020,
            AccessDenied = 0x8023,
            ParamValueOutOfRange = 0x8030
        };

        struct Parameter {
            uint16_t index;
            uint8_t subindex;
            uint8_t access; // 0=RW, 1=RO, 2=WO
            uint8_t length;
            void* data;
        };

        //Message struct for IO-Link
        struct ioLink_message_t
        {
            uint8_t channel;
            uint8_t addr;
        };

        struct ODSize_t
        {
            uint8_t Startup;
            uint8_t Preop;
            uint8_t Op;
        };
        //OD Size for the device (1,2,8 or 32)
        static ODSize_t ODSize;  // Removed constexpr to allow runtime initialization
        
        // Data Link State Machine for frame parsing
        enum DL_State
        {
            DL_IDLE,        // Waiting for MC byte
            DL_RX_MC,       // Received MC, waiting for CKT
            DL_RX_CKT,      // Received CKT, calculate expected length
            DL_RX_DATA,     // Receiving data bytes
            DL_ERROR        // Error state, discard until next frame
        };
        
        //States for interpreting IO-Link data
        enum ioLink_states
        {
            wait_wake, //Waiting for wakeup signal
            wait_valid_frame, //Waiting for a valid startup message
            run_mode  //Master started communication with device
        };

        struct Status
        {
            PDStatus PDIn;
            PDStatus PDOut;
        };

        struct PDBuffer
        {
            uint8_t Data[MaxPD];
            size_t Size;
        };

        struct PD
        {
            PDBuffer Out;
            PDBuffer In;
        };

        uint8_t GetMasterTXSize();
        //Reset the RX Line
        void ResetRX();
        //Get the MSeq Cap according B.1.4
        uint8_t GetMseqCap() const;
        //Process an incoming Master Message
        void ProcessMessage();
        //Handle ISDU requests
        void HandleISDU(uint8_t* buffer, uint8_t size, bool isRead);
        //Sets the Response for the Master, returns the total bytes to send
        uint8_t SetResponse();
        uint8_t rxBuffer[MaxIOLMsgSize]{};
        uint8_t txBuffer[MaxIOLMsgSize]{};
        void initTransciever(int wakeup_mode) const;
        void TxByteBitBang(uint8_t b);
        // Parse the MC (Figure A.1)
        void ParseMC();
        //Send device response to master (isWrite: true for Write commands, false for Read)
        void DeviceRsp(uint8_t *data, uint8_t len, bool isWrite);
        //Prepare the device message
        void prepareMessage(uint8_t od_size);
        /*
        * Called whenever a new RX frame 
        * from the master is recieved
        * rx_byte = byte recieved from the UART
        */
        void SaveMasterFrame(uint8_t rx_byte);
        //Generate IOLink checksum
        uint8_t GetChecksum(const uint8_t *data, uint8_t length) const;
        //Decode cycletime according to Table B.3, return value time in microseconds
        uint32_t DecodeCycleTime(uint8_t encoded_time) const;
        /* Encode cycletime according to Table B.3
        *  If the cycle time time cannot be encoded 
        *  the nearest greater cycle time will be used.
        *  e.g., cycleTime_us = 1333 us -> encoded value = 1340 us
        */
        uint8_t EncodeCycleTime(uint32_t cycleTime_us) const;
        Stream * SerialPort = nullptr; 
        ioLink_message_t message;
        PD Pd{};
        uint8_t ODBuffer[MaxOD]{};
        /* Process Data status */
        Status status = { .PDIn = Valid, .PDOut = Invalid};
        /* Table B.1 */
        uint8_t ParameterPage1[SizeDP1]{};
        bool NewCmd = false;
        unsigned TxEn;
        uint8_t _txPin;
        unsigned WuPin;
        MasterCommands Cmd;
        Mode deviceMode = start;
        MCAccess MasterAccess;
        uint8_t ExpectedRXCnt = 0;
        ioLink_states deviceState = wait_wake;
        bool NewMasterMsg = false;
        uint8_t rxCnt = 0;
        unsigned long CurrentCycleTime = 0;
        unsigned long LastMessage = 0;
        
        // Added for robust RX parsing
        uint32_t tbit_us = 0;
        unsigned long lastByteTimestamp = 0;
        MSeqType currentMSeq = MSeqType::Type0;
        
        // Dedicated copy for Type-0 Echo
        uint8_t lastMC = 0;
        uint8_t lastCKT = 0;
        
        // Data Link State Machine variables
        DL_State dlState = DL_IDLE;          // Current DL parser state
        bool parametrization_active = false; // System Command Flag
        uint8_t dlRxCnt = 0;                 // Byte counter in current phase
        uint8_t dlExpectedLen = 0;           // Expected frame length (after CKT received)
        uint8_t dlMC = 0;                    // Master Command byte
        uint8_t dlCKT = 0;                   // Checksum/M-sequence byte (NEVER modified)
        unsigned long dlSyncTime = 0;        // Timestamp when MC received (for timeout)
        bool dlFrameValid = false;           // True if current frame is valid and ready
        
        // Validate Master Command byte (filter noise patterns)
        bool IsValidMC(uint8_t mc_byte) const;

        // --- Priority Event Queue (Diagnosis) ---
        // Events are stored in an array and kept sorted so Errors (Qualifier=3)
        // have higher priority than Warnings (2) and Notifications (1).
        static const uint8_t EVENT_QUEUE_SIZE = 4;
        EventObj eventQueue[EVENT_QUEUE_SIZE];
        uint8_t eventCount = 0;

        // Handle a diagnosis read from the Master (addr==0..2 -> Event fields)
        void HandleDiagnosis(uint8_t addr, uint8_t* outByte);

        // Internal helpers for event management
        void SortEvents(); // Keep highest qualifier at index 0

        // Event API moved to public
       // void SendEvent(uint16_t code, uint8_t qualifier);
       // bool HasEvents();

    };
};
