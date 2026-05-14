================================================================================
IO-LINK DEVICE STACK V1.1 (RP2040)
================================================================================
Version:       1.0.0 (Gold / Thesis Release)
Platform:      Raspberry Pi Pico (RP2040)
Target Spec:   IEC 61131-9 (IO-Link V1.1 / V1.1.4 behavior)
Author:        Manuel Kankura Salazar
License:       GPL-3.0+

[!] CERTIFICATION DISCLAIMER:
This software package implements IO-Link protocol behaviors according to the
specification, but it has NOT undergone formal conformance testing by the
IO-Link Consortium. It is intended for experimental, research, and retrofit
prototyping purposes only.

--------------------------------------------------------------------------------
1. OVERVIEW
--------------------------------------------------------------------------------
This software package implements an IO-Link V1.1 Device Stack designed to
"Retrofit" legacy analog sensors into the Industry 4.0 ecosystem.

The stack handles low-level protocol behavior:
  - Physical Layer (validation): UART 8E1 at 38.4 kbaud (COM2).
  - Data Link Layer: M-sequence framing with checksum validation.
  - Application Layer: Cyclic Process Data (PD) and Acyclic On-Request Data (ISDU).

KEY FEATURES:
  * Generic Sensor API: Link any C++ variable directly to the IO-Link map.
  * Data Storage: Automatic parameter persistence to EEPROM (with CRC32).
  * Event Handling: Priority-based diagnosis queue (Errors > Warnings > Notifications).
  * Safety: Buffer overflow protection (ISDU truncation to OD size).

NOTES (IMPORTANT):
  - OD_SIZE is configured as 16 bytes (On-Demand payload) to support ISDU services.
  - The default MinCycleTime exposed in Direct Parameter Page 1 is 10 ms.

--------------------------------------------------------------------------------
2. HARDWARE SETUP
--------------------------------------------------------------------------------
The stack is configured for the Raspberry Pi Pico (RP2040).

[WARNING]: The RP2040 is a 3.3V device.
- For Logic Validation: Connect directly to a 3.3V Master (e.g. Raspberry Pi 4).
- For Industrial Use: You MUST use an IO-Link Transceiver (24V interface) to
  connect to real IO-Link cabling and field wiring.

PINOUT CONFIGURATION (Defined in main.cpp):
  [GP0] UART0 TX  ----> Connect to Master RX (or Transceiver TXIN)
  [GP1] UART0 RX  ----> Connect to Master TX (or Transceiver RXOUT)
  [GP2] TX_ENABLE ----> Transceiver Direction Pin (HIGH = Transmit)
  [GP3] WAKE_UP   ----> Transceiver Wake Pin (Active LOW/FALLING)

* Note: For Direct 3.3V UART testing (No Transceiver), leave GP2/GP3 disconnected
  and cross-connect UARTs (TX->RX, RX->TX) with a common Ground.

--------------------------------------------------------------------------------
3. DIRECTORY STRUCTURE
--------------------------------------------------------------------------------
/src
  ├── main.cpp                 # USER AREA: Define your sensor logic here.
  ├── SpecIOLink-Device.hpp    # STACK HEADER: The generic API definition.
  ├── SpecIOLink-Device.cpp    # STACK SOURCE: Protocol implementation (Do not edit).
/tools
  ├── master.py        # Python IO-Link Master Stack (Validation Tool).
  ├── run_master.py    # Automated Test Bench script.

--------------------------------------------------------------------------------
4. HOW TO USE (QUICK START)
--------------------------------------------------------------------------------
1. Install Arduino IDE with "Raspberry Pi Pico/RP2040" board support 
   (Earle Philhower core recommended).
2. Open `main.cpp`.
3. Select Board: "Raspberry Pi Pico".
4. Upload the code.
5. The device starts in STARTUP mode and awaits Master interaction 
   (Wake-Up / startup commands).

--------------------------------------------------------------------------------
5. CUSTOMIZING FOR YOUR SENSOR
--------------------------------------------------------------------------------
The stack uses a "Generic Link" architecture. You do not need to modify the
protocol files. All customization happens in `main.cpp`.

Step A: Define Your Memory
   Create a struct to hold your parameters (Thresholds, Counters, etc.).
   > struct MySensorParams { uint16_t limit; ... };

Step B: Define Your ISDU Handler
   Implement `Sensor_ISDU_Handler`. This function intercepts Read/Write requests
   from the Master for specific Indexes (e.g., 0x0042).
   - Return `true` if you handled the request.
   - Return `false` to let the stack send an "Index Not Found" error.
   - Use `EEPROM.put()` inside this handler to save changes permanently.

Step C: Link Pointers in setup()
   > ioLinkDevice.processDataIn = (uint8_t*)&myVariable;
   > ioLinkDevice.attachISDUCallback(Sensor_ISDU_Handler);

Step D: Write Logic in loop()
   Update your `myVariable` based on sensor readings. The stack automatically
   transmits this value in the next IO-Link cycle (cycle time is Master-defined;
   MinCycleTime is 10 ms by default).
   Use `ioLinkDevice.SendEvent(code, qualifier)` to trigger alarms.

--------------------------------------------------------------------------------
6. VALIDATION (PYTHON MASTER)
--------------------------------------------------------------------------------
A custom Python Master is provided to validate the device without an expensive PLC.

Requirements:
  - Raspberry Pi 4 (or PC with USB-UART adapter).
  - Python 3 + PySerial.

Running the Test Bench:
  $ python3 run_master.py

Menu Options:
  [1] Basic Communication: Verifies Startup Handshake and cyclic exchange.
  [2] Persistence: Writes a tag, reboots device, reads tag back.
  [4] Full Subset Validation: Stresses error handling and invalid indexes.
  [6] Events & DS: Triggers faults and validates Event priority handling.

NOTE:
  - The master uses OD_SIZE = 16 bytes for ISDU payload transport.

--------------------------------------------------------------------------------
7. MEMORY MAP (DEFAULT)
--------------------------------------------------------------------------------
Index   | R/W | Description
--------|-----|-----------------------------------------------------------------
0x0002  |  R  | MinCycleTime (10ms)
0x0010  | R/W | Vendor Name ("MyGenericSensor") - Stored in EEPROM
0x0012  | R/W | Product Name ("GenSensor-01") - Stored in EEPROM
0x0018  | R/W | Application Tag (User ID) - Triggers Data Storage Event (0x6350)
0x0042  | R/W | **Custom Parameter** (Example: Switch Point)
0x0043  |  W  | **Custom Command** (Example: Reset Counter)
--------------------------------------------------------------------------------
| `0x0012` | R/W | Product Name ("GenSensor-01") - Stored in EEPROM |
| `0x0018` | R/W | Application Tag (User ID) - Triggers Data Storage Event (0x6350) |
| `0x0042` | R/W | **Custom Parameter** (Example: Switch Point) |
| `0x0043` | W | **Custom Command** (Example: Reset Counter) |
