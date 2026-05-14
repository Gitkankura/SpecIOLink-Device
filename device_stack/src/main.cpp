// SPDX-License-Identifier: GPL-3.0+
#include "SpecIOLink-Device.hpp"
#include <Arduino.h>
#include <EEPROM.h>

using namespace lwIOLink;

// ==========================================
// USER AREA: DEFINE YOUR SENSOR MEMORY
// ==========================================

// 1. Define Persistent Parameters (Saved in EEPROM)
struct MySensorParams {
    uint16_t switchPoint; // Example: A threshold value
    uint32_t counter;     // Example: An event counter
};

// Global Variables
Device ioLinkDevice(2, 2, 10000);
MySensorParams sensorParams;      // struct instance


uint16_t sensorProcessData = 0;   // The live sensor value (Sent to Master)
uint16_t outputDataFromMaster = 0;// Data received from Master

// 2. Define the Handler for Custom Parameters (ISDU)
// This function is called automatically when the Master requests a custom Index
bool Sensor_ISDU_Handler(uint16_t index, uint8_t subIndex, uint8_t* data, uint8_t len, bool isWrite) {
    
    // Custom Parameter: Switch Point (Index 0x0042)
    if (index == 0x0042) {
        if (isWrite) {
            // Master is writing: Convert bytes to uint16
            sensorParams.switchPoint = (data[0] << 8) | data[1];
            
            // Save to EEPROM (Offset 256 to avoid system area)
            EEPROM.put(256, sensorParams); 
            EEPROM.commit();
            return true;
        } else {
            // Master is reading: Convert uint16 to bytes
            data[0] = (sensorParams.switchPoint >> 8) & 0xFF;
            data[1] = sensorParams.switchPoint & 0xFF;
            return true;
        }
    }
    
    // Custom Parameter: Reset Counter (Index 0x0043 - Write Only)
    if (index == 0x0043 && isWrite) {
        if (data[0] == 0x01) { // Command 0x01 = Reset
            sensorParams.counter = 0;
            EEPROM.put(256, sensorParams);
            EEPROM.commit();
            return true;
        }
    }

    return false; // Index not found (Stack will send Error 0x8030)
}

void setup()
{
    // Debug USB
    Serial.begin(115200); 
    
    // SAFETY DELAY: Dá tempo ao teu computador para criar a porta COM
    // antes de o Pico começar a falar.
    delay(3000); 
    
    Serial.println("\n\n=== IO-Link Device: NATIVE PICO MODE ACTIVATED ===");

    // 3. Initialize Persistence
    EEPROM.begin(512);
    EEPROM.get(256, sensorParams); // Load user params from offset 256

    // Initialize Defaults if memory is empty/fresh (0xFFFF is default erased state)
    if (sensorParams.switchPoint == 0xFFFF) {
        sensorParams.switchPoint = 2048; // Default threshold (half of 12-bit ADC)
        sensorParams.counter = 0;
        EEPROM.put(256, sensorParams);
        EEPROM.commit();
    }

    // 4. Configure Hardware (Pico)
    Device::HWConfig config = {
        .SerialPort = Serial1, 
        .Baud = BaudRate::COM2,
        .WakeupMode = FALLING,
        .Pin = { .TxEN = 2, .Wakeup = 3, .Tx = 0, .Rx = 1 }
    };
    
    // 5. LINK THE GENERIC POINTERS (The Magic Step)
    // This tells the stack: "When sending Cyclic Data, look at this variable."
    ioLinkDevice.processDataIn = (uint8_t*)&sensorProcessData;
    ioLinkDevice.processDataSize = 2; // 2 Bytes
    
    // 6. Register your Callback
    ioLinkDevice.attachISDUCallback(Sensor_ISDU_Handler);

    // 7. Start Stack
    ioLinkDevice.begin(config);

    // Optional: Rename the Device via the public parameters pointer
    if(ioLinkDevice.parameters) {
        strcpy(ioLinkDevice.parameters->VendorName, "MyGenericSensor");
        strcpy(ioLinkDevice.parameters->ProductName, "GenSensor-01");
    }

    Serial.println("Stack iniciada. Aguardando dados no GP1...");
}

void loop()
{
    // 1. Run the Protocol Stack
    ioLinkDevice.run();
    
    // 2. Sensor Application Logic
    // Only run sensor logic if we are in OPERATE mode (saving power/cpu otherwise)
    if (ioLinkDevice.GetMode() == Mode::operate)
    {
        // A. SIMULATE SENSOR READING
        // If you have a real sensor, replace this with analogRead(26)
        // Here we simulate a sawtooth wave for testing
        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 100) {
            sensorProcessData++; 
            lastUpdate = millis();
        }

        // B. READ OUTPUT FROM MASTER (Optional)
        // If the Master sends data to us (e.g., turn on LED)
        PDStatus status;
        if (ioLinkDevice.GetPDOut((uint8_t*)&outputDataFromMaster, &status)) {
            // Example: If Master sends > 0, turn on built-in LED
            // digitalWrite(LED_BUILTIN, outputDataFromMaster > 0 ? HIGH : LOW);
        }

        // C. EVENT LOGIC (ALARM)
        // Check if our simulated value exceeds the configured threshold
        static bool alarmActive = false;
        if (sensorProcessData > sensorParams.switchPoint && !alarmActive) {
            alarmActive = true;
            // Trigger Standard Event: "Process variable range over-run" (0x8C10)
            // Qualifier 2 = Warning
            ioLinkDevice.SendEvent(0x8C10, 2); 
            Serial.println("[APP] Threshold Exceeded! Event sent.");
        } 
        else if (sensorProcessData < sensorParams.switchPoint && alarmActive) {
            alarmActive = false; // Reset alarm state
        }
    }
}
