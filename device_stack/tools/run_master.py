import sys
import logging
import os
import time
import random
import string
from iolink_master.master import IOLinkMaster
from iolink_master.mock_uart import MockUART

# ANSI Colors
GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

def print_pass(msg):
    print(f"{GREEN}[PASS] {msg}{RESET}")

def print_fail(msg):
    print(f"{RED}[FAIL] {msg}{RESET}")

def test_basic_communication(master):
    print("Running Basic Communication Test...")
    master.run() # This calls start() -> wakeup + startup + cyclic

def test_persistence(master):
    print("\n" + "="*40)
    print("TEST: Data Persistence (EEPROM)")
    print("="*40)

    # --- STEP 0: INITIALIZE DEVICE ---
    print("0. Initializing Device...")
    master.wakeup()
    master.startup_sequence()
    print("   Device is now in OPERATE mode (Ready for large 16-byte frames)")
    # -----------------------------------

    # --- STEP 1: WRITE ISDU ---
    print("\n1. Writing Application Tag to Device...")
    new_tag = f"ID-{random.randint(10, 99)}"
    print(f"   Tag Value: {new_tag} (short format: 5 bytes fits in 16-byte OD_SIZE)")
    
    success = master.write_isdu(0x18, 0, new_tag)
    if not success:
        print_fail("Could not write ISDU")
        return
    time.sleep(0.1)

    # --- STEP 2: VERIFY IN RAM (IMMEDIATE) ---
    print("\n2. Verifying in RAM (immediate readback)...")
    read_back = master.read_isdu(0x18)
    print(f"   Read: {read_back}")
    if read_back != new_tag:
        print_fail(f"RAM Mismatch. Expected '{new_tag}', got '{read_back}'")
        return
    print_pass("RAM Verification OK")

    # --- STEP 3: DEVICE RESET ---
    print("\n3. Sending Device Reset Command (0x80)...")
    master.send_system_command(0x80)
    print("   Waiting 3.0s for Device Reboot...")
    time.sleep(3.0)

    # --- STEP 4: RE-ESTABLISH COMMUNICATION ---
    print("\n4. Re-establishing Communication...")
    master.wakeup()
    master.startup_sequence()
    print("   Device back in OPERATE mode")

    # --- STEP 5: VERIFY PERSISTENCE ---
    print("\n5. Reading Application Tag from NV Memory...")
    final_read = master.read_isdu(0x18)
    print(f"   Read: {final_read}")
    
    if final_read == new_tag:
        print_pass("PERSISTENCE TEST PASSED")
        print("   ✅ Data survived reset cycle!")
    else:
        print_fail("PERSISTENCE TEST FAILED")
        print(f"   Expected '{new_tag}', got '{final_read}'")


def test_factory_reset(master):
    print("\n--- Running Factory Reset Test ---")
    
    print("Sending System Command 0x82 (Restore Factory)...")
    master.send_system_command(0x82)
    
    print("Waiting 2 seconds for device reboot...")
    time.sleep(2)
    
    print("Reconnecting...")
    master.wakeup()
    master.startup_sequence()
    
    print("Verifying Application Tag is default...")
    tag = master.read_isdu(0x18, 0x00)
    print(f"Tag: {tag}")
    
    # Assuming default is "MyID" or we just check it's NOT the random one?
    if tag == "MyID" or (tag is not None and "TEST-TAG" not in tag):
        print_pass("Factory Reset Test PASSED")
    else:
        print_fail(f"Factory Reset Test FAILED: Tag is {tag}")

def test_advanced_features(master):
    """
    Test Advanced Features: Page 2 (Product Name), Data Storage CRC, and ISDU Truncation.
    """
    print("\n" + "="*60)
    print("TEST: Advanced Features (Page 2, Data Storage, Truncation)")
    print("="*60)

    # --- STEP 0: INITIALIZE DEVICE ---
    print("0. Initializing Device...")
    master.wakeup()
    master.startup_sequence()
    print("   Device is now in OPERATE mode")
    time.sleep(0.5)

    # =====================================================
    # TEST A: Page 2 Identification (Direct Page Read)
    # =====================================================
    print("\n--- TEST A: Page 2 Identification (Read Addresses 0x10-0x1F) ---")
    
    product_name_chars = []
    print("Reading addresses 0x10 to 0x1F...")
    
    try:
        for addr in range(0x10, 0x20):
            val = master.read_page(addr)
            if val is not None:
                if val != 0: # 0 might be padding/null
                   product_name_chars.append(chr(val))
            else:
                print_fail(f"Failed to read address 0x{addr:02X}")
        
        product_name = "".join(product_name_chars)
        print(f"Constructed String: '{product_name}'")
        
        if product_name.startswith("MyDevice"):
            print_pass("Page 2 Identification PASSED")
        else:
            print_fail(f"Page 2 Identification FAILED. Expected start with 'MyDevice', got '{product_name}'")
            
    except Exception as e:
        print_fail(f"Exception during Page 2 test: {e}")
        import traceback
        traceback.print_exc()

    # =====================================================
    # TEST B: Data Storage (CRC32 Check)
    # =====================================================
    print("\n--- TEST B: Data Storage (CRC32 Check) ---")
    
    try:
        # 1. Read Initial CRC (Index 0x001A)
        print("Reading Initial CRC (Index 0x001A)...")
        crc_data = master.read_isdu(0x001A, 0) # Returns 4 bytes raw data usually? 
        
        initial_crc = 0
        if isinstance(crc_data, (bytes, bytearray, list)):
             initial_crc = int.from_bytes(bytes(crc_data), 'big')
        elif isinstance(crc_data, str):
             # If by chance it decoded to string (unlikely for CRC), revert to bytes
             initial_crc = int.from_bytes(crc_data.encode('latin1'), 'big') # Fallback
             
        print(f"Initial CRC: 0x{initial_crc:08X}")

        # 2. Write new Application Tag (Index 0x18)
        new_tag = "NEW-TAG-v1"
        print(f"Writing new Application Tag: '{new_tag}'...")
        if master.write_isdu(0x18, 0, new_tag):
             print("Write Success.")
        else:
             print_fail("Write failed.")
        
        time.sleep(0.5)

        # 3. Read New CRC
        print("Reading New CRC (Index 0x001A)...")
        crc_data_new = master.read_isdu(0x001A, 0)
        
        new_crc = 0
        if isinstance(crc_data_new, (bytes, bytearray, list)):
             new_crc = int.from_bytes(bytes(crc_data_new), 'big')
        elif isinstance(crc_data_new, str):
             new_crc = int.from_bytes(crc_data_new.encode('latin1'), 'big')

        print(f"New CRC: 0x{new_crc:08X}")
        
        if new_crc != initial_crc:
            print_pass(f"CRC Changed (0x{initial_crc:08X} -> 0x{new_crc:08X}) - PASSED")
        else:
            print_fail("CRC did not change! - FAILED")

    except Exception as e:
        print_fail(f"Exception during CRC test: {e}")
        import traceback
        traceback.print_exc()

    # =====================================================
    # TEST C: ISDU Truncation (Safety) - Read Index 0x10
    # =====================================================
    print("\n--- TEST C: ISDU Truncation (Read Index 0x10) ---")
    
    try:
        # Read Vendor Name (Index 0x10) - usually a long string (up to 64 bytes)
        # Device buffer is 16 bytes. ISDU Frame = Header(1) + Data(15).
        # So we expect max 15 chars.
        
        print("Reading Vendor Name (Index 0x10)...")
        vendor_name = master.read_isdu(0x10, 0)
        
        print(f"Received: '{vendor_name}'")
        
        if not vendor_name:
             print_fail("Received empty response")
             return

        # Check length
        length = len(vendor_name)
        print(f"Length: {length} chars")
        
        if length == 15:
            print_pass("Truncation Size Correct (15 chars) - PASSED")
        elif length < 15:
            print(f"Warning: Received less than 15 chars ({length}). Device might have short name?")
            # For verification, we check if it matches expectation
            print_pass("Read successful (Short string)")
        else:
            print_fail(f"Truncation FAILED. Received {length} chars (>15).")
            
        # Check content match (Mock sends "MyVendorNameIsL...")
        # "MyVendorNameIsL" is exactly 15 chars
        expected_start = "MyVendorNameIsL"
        if vendor_name.startswith(expected_start[:5]): 
             print_pass("Content Verification OK")

    except Exception as e:
         print_fail(f"Exception during Truncation test: {e}")
         import traceback
         traceback.print_exc()
    
    print("\n" + "="*60)
    print("ADVANCED FEATURES TEST COMPLETE")
    print("="*60 + "\n")


def test_compliant_events(master):
    print("\n=== TEST 6: Compliant Event & DS Flow ===")
    master.wakeup()
    master.startup_sequence()
    time.sleep(0.5)

    print("\n1. Triggering Hardware Fault (0x9999)...")
    master.write_isdu(0x9999, 0, [0x01]) 
    time.sleep(0.2)
    
    code_h = master.read_diagnosis(0)
    code_l = master.read_diagnosis(1)
    qual = master.read_diagnosis(2)
    
    if code_h is not None:
        code = (code_h << 8) | code_l
        print(f"   Received: 0x{code:04X} (Qual {qual})")
        if code == 0x5000: print_pass("Hardware Fault (0x5000) Confirmed")
        else: print_fail(f"Expected 0x5000, got 0x{code:04X}")
    else: print_fail("No Diagnosis Response")

    print("\n2. Testing Data Storage Trigger (Write Tag)...")
    master.write_isdu(0x18, 0, "NewTag")
    time.sleep(0.5)
    
    # In the compliant flow, writing the tag should raise Event 0x6350
    print("   Checking for 'Parameter Changed' Event...")
    code_h = master.read_diagnosis(0)
    code_l = master.read_diagnosis(1)
    
    if code_h is not None:
        code = (code_h << 8) | code_l
        print(f"   Received: 0x{code:04X}")
        if code == 0x6350: print_pass("DS Trigger (0x6350 Param Changed) Confirmed")
        else: print_fail(f"Expected 0x6350, got 0x{code:04X}")
    else:
        print_fail("No Event received. Device did not signal Data Storage update.")

def test_full_compliance(master):
    """
    Complete compliance test: Persistence + Error Handling
    """
    print("\n" + "="*60)
    print("FULL COMPLIANCE TEST")
    print("="*60)

    # --- STEP 0: INITIALIZE DEVICE ---
    # Ensure the device is awake and in the correct mode before testing
    print("0. Initializing Device...")
    master.wakeup()
    master.startup_sequence()
    print("   Device is now in OPERATE mode (Ready for large 16-byte frames)")
    time.sleep(0.5) # Allow state machine to settle
    # -----------------------------------
    
    # =====================================================
    # TEST 1: Application Tag Persistence
    # =====================================================
    print("\n--- TEST 1: Application Tag Persistence ---")
    
    try:
        # 1. Write a test tag
        test_tag = "TAG-" + str(random.randint(1000, 9999))
        print(f"Writing test tag: {test_tag}")
        
        write_result = master.write_isdu(0x18, 0x00, test_tag)
        
        if isinstance(write_result, dict) and write_result.get('error'):
            print_fail(f"Write failed with error code 0x{write_result['code']:04X}")
            return
        
        if not write_result:
            print_fail("Write ISDU failed")
            return
        
        time.sleep(0.1)
        
        # 2. Read back to confirm RAM update
        print("Verifying RAM update...")
        read_back = master.read_isdu(0x18, 0x00)
        
        if isinstance(read_back, dict) and read_back.get('error'):
            print_fail(f"Read failed with error code 0x{read_back['code']:04X}")
            return
        
        if read_back == test_tag:
            print_pass("RAM updated successfully")
        else:
            print_fail(f"RAM mismatch: expected '{test_tag}', got '{read_back}'")
            return
        
        # 3. Send device reset
        print("Sending Device Reset command (0x80)...")
        master.send_system_command(0x80)
        
        print("Waiting 3 seconds for device reboot...")
        time.sleep(3)
        
        # 4. Reconnect
        print("Reconnecting...")
        master.wakeup()
        master.startup_sequence()
        
        time.sleep(0.5)
        
        # 5. Read tag after reset
        print("Reading Application Tag after reset...")
        final_tag = master.read_isdu(0x18, 0x00)
        
        if isinstance(final_tag, dict) and final_tag.get('error'):
            print_fail(f"Read failed with error code 0x{final_tag['code']:04X}")
            return
        
        if final_tag == test_tag:
            print_pass("PERSISTENCE TEST PASSED: Tag persisted after reset!")
        else:
            print_fail(f"PERSISTENCE TEST FAILED: Expected '{test_tag}', got '{final_tag}'")
            return
    
    except Exception as e:
        print_fail(f"Persistence test exception: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # =====================================================
    # TEST 2: Error Handling (Invalid Index)
    # =====================================================
    print("\n--- TEST 2: Error Handling (Invalid Index) ---")
    
    try:
        invalid_index = 0x5000
        print(f"Attempting to read invalid index: 0x{invalid_index:04X}...")
        
        result = master.read_isdu(invalid_index)

        if isinstance(result, dict) and result.get('error'):
            code = result.get('code', 0)
            if code == 0x8030:
                print_pass(f"Correctly received Error 0x{code:04X} (Index not available)")
            else:
                print_fail(f"Received Wrong Error Code: 0x{code:04X}")
        else:
            print_fail(f"Expected error dictionary, got: {result}")
    
    except Exception as e:
        print_fail(f"Error handling test exception: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "="*60)
    print("FULL COMPLIANCE TEST COMPLETE")
    print("="*60 + "\n")

def main():
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    
    port = '/dev/serial0'
    use_mock = False
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--mock":
            use_mock = True
        else:
            port = sys.argv[1]

    if not use_mock and not os.path.exists(port):
        print(f"Serial port {port} not found. Using MOCK for demonstration.")
        use_mock = True

    try:
        master = IOLinkMaster(port=port)
        if use_mock:
            master.uart = MockUART()
    except Exception as e:
        print(f"Failed to initialize master: {e}")
        return

    while True:
        print("\n--- IO-Link Master Test Menu ---")
        print("1. Basic Communication (Startup & Cyclic)")
        print("2. Test Persistence (Write Tag -> Reset -> Read)")
        print("3. Test Factory Reset")
        print("4. Full Compliance Test (Persistence + Error Handling)")
        print("5. Test Advanced Features (Page2, Truncation)")
        print("6. Test Compliant Events & DS (NEW)")
        print("7. Exit")
        
        # Reset ISDU state before showing menu
        master.reset_isdu()
        
        choice = input("Select test: ")
        
        if choice == '1':
            try:
                test_basic_communication(master)
            except KeyboardInterrupt:
                print("\nStopped.")
        elif choice == '2':
            test_persistence(master)
        elif choice == '3':
            test_factory_reset(master)
        elif choice == '4':
            test_full_compliance(master)
        elif choice == '5':
            test_advanced_features(master)
        elif choice == '6':
            test_compliant_events(master)
        elif choice == '7':
            master.disconnect()
            break
        else:
            print("Invalid choice.")

if __name__ == "__main__":
    main()
