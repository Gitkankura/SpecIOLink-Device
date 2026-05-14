import logging
import time
from enum import Enum, auto
from iolink_master.checksum import Checksum
from iolink_master.uart import UARTLayer

class State(Enum):
    INACTIVE = auto()
    STARTUP = auto()
    PREOPERATE = auto()
    OPERATE = auto()

class IOLinkMaster:
    OD_SIZE = 16  # On-Demand Data payload size (must match device firmware)
    PD_SIZE = 2   # Process Data payload size (must match device firmware)
    
    def __init__(self, port='/dev/ttyAMA0'):
        self.state = State.INACTIVE
        self.uart = UARTLayer(port=port)
        self.checksum_calc = Checksum()
        self.logger = logging.getLogger("MASTER")
        self.seq = 0 
        self.device_seq = 0 
        # ISDU State Variables
        self.isdu_active = False
        self.isdu_step = 0
        self.isdu_tx_buffer = [] 
        self.isdu_rx_buffer = [] 
        self.isdu_req_service = 0
        self.isdu_req_index = 0
        self.isdu_req_subindex = 0
        self.isdu_done = False
        self.isdu_error = False
        
        # ISDU Queue for handling requests during cyclic exchange
        self.isdu_queue = None  # Current pending ISDU request
        self.isdu_response = None  # Response received from device

    def log(self, msg):
        self.logger.info(msg) 

    def transition_to(self, new_state):
        old_state = self.state
        self.state = new_state
        self.log(f"[STATE TRANSITION] {old_state.name} -> {new_state.name}")
        self.log(f"[STATE] {new_state.name}")

    def run(self):
        self.start()

    def start(self):
        self.wakeup()
        try:
            self.startup_sequence()
        except KeyboardInterrupt:
            self.log("[MASTER] Interrupted by user")
        except Exception as e:
            self.log(f"[ERROR] Exception during sequence: {e}")
            import traceback
            traceback.print_exc()
        # Note: Don't disconnect here - let the caller decide when to disconnect
        # This allows multiple tests to share the same connection
            
    def disconnect(self):
        """Explicitly disconnect from the device."""
        self.uart.disconnect()
        self.reset_isdu()
            
    def wakeup(self):
        self.log("[MASTER] Enter STARTUP")
        self.log("[MASTER] Performing WURQ...") 
        self.uart.connect()
        self.log("[MASTER] COM2 selected (38.4k)")
        self.transition_to(State.STARTUP)

    def startup_sequence(self):
        self.golden_startup_sequence()

    def read_param(self, address, desc):
        # Ensure port is open
        if not self.uart.ser or not self.uart.ser.is_open:
            self.log(f"[MASTER] Port not open, reconnecting...")
            self.uart.connect()
        
        mc = 0xA0 | (address & 0x0F)
        masked_ckt = (self.seq & 0x03) << 6
        ck6 = self.checksum_calc.calculate([mc, masked_ckt])
        ckt = self.checksum_calc.build_ckt(self.seq, ck6)
        tx_frame = [mc, ckt]
        
        self.log(f"[MASTER -> DEVICE] READ {desc} (Addr 0x{address:02X})")
        
        rx_len = 3
        response_received = False
        r_data = None
        attempt = 1
        while not response_received and attempt <= 3:
            if attempt > 1:
                self.log(f"[RETRY] No valid reply, resending request (attempt {attempt})")
            self.uart.send_bytes(tx_frame)
            time.sleep(0.08)  
            self.seq = (self.seq + 1) % 4

            rx_frame = self.uart.read_bytes(rx_len)

            if len(rx_frame) == rx_len:
                r_mc = rx_frame[0]
                r_ckt = rx_frame[1]
                r_data = rx_frame[2]
                r_seq, r_ck6 = self.checksum_calc.parse_ckt(r_ckt)
                masked_rckt = (r_seq & 0x03) << 6
                calc_ck6 = self.checksum_calc.calculate([r_mc, masked_rckt, r_data])

                if r_ck6 == calc_ck6:
                    response_received = True
                else:
                    self.log(f"[INTERPRETATION] Checksum ERROR! Expected 0x{calc_ck6:02X}, Got 0x{r_ck6:02X}")
            else:
                self.log(f"[ERROR] Incomplete RX.")
                time.sleep(0.05)
            attempt += 1
        return r_data

    def read_page(self, address):
        """
        Reads a byte from Page 2 (Addresses 0x10 to 0x1F) using Type 0 frames.
        Note: The mask for Page 2 is 0x1F, allowing access up to address 31.
        """
        # Ensure port is open
        if not self.uart.ser or not self.uart.ser.is_open:
            self.log(f"[MASTER] Port not open, reconnecting...")
            self.uart.connect()

        # MC calculation uses 5-bit mask (0x1F) to include Page 2 addresses
        mc = 0xA0 | (address & 0x1F)
        masked_ckt = (self.seq & 0x03) << 6
        ck6 = self.checksum_calc.calculate([mc, masked_ckt])
        ckt = self.checksum_calc.build_ckt(self.seq, ck6)
        tx_frame = [mc, ckt]

        self.log(f"[MASTER -> DEVICE] READ PAGE 2 (Addr 0x{address:02X})")

        rx_len = 3
        response_received = False
        r_data = None
        attempt = 1
        while not response_received and attempt <= 3:
            if attempt > 1:
                self.log(f"[RETRY] No valid reply, resending request (attempt {attempt})")
            self.uart.send_bytes(tx_frame)
            time.sleep(0.08)  
            self.seq = (self.seq + 1) % 4

            rx_frame = self.uart.read_bytes(rx_len)

            if len(rx_frame) == rx_len:
                r_mc = rx_frame[0]
                r_ckt = rx_frame[1]
                r_data = rx_frame[2]
                r_seq, r_ck6 = self.checksum_calc.parse_ckt(r_ckt)
                masked_rckt = (r_seq & 0x03) << 6
                calc_ck6 = self.checksum_calc.calculate([r_mc, masked_rckt, r_data])

                if r_ck6 == calc_ck6:
                    response_received = True
                else:
                    self.log(f"[INTERPRETATION] Checksum ERROR! Expected 0x{calc_ck6:02X}, Got 0x{r_ck6:02X}")
            else:
                self.log(f"[ERROR] Incomplete RX.")
                time.sleep(0.05)
            attempt += 1
        return r_data

    def read_diagnosis(self, address):
        """Reads a byte from the Diagnosis channel (MC base 0xC0).
        Returns the data byte or None on failure."""
        # Ensure port is open
        if not self.uart.ser or not self.uart.ser.is_open:
            self.log(f"[MASTER] Port not open, reconnecting...")
            self.uart.connect()

        mc = 0xC0 | (address & 0x1F)
        masked_ckt = (self.seq & 0x03) << 6
        ck6 = self.checksum_calc.calculate([mc, masked_ckt])
        ckt = self.checksum_calc.build_ckt(self.seq, ck6)
        tx_frame = [mc, ckt]

        self.log(f"[MASTER -> DEVICE] READ DIAG (Addr 0x{address:02X})")

        rx_len = 3
        response_received = False
        r_data = None
        attempt = 1
        while not response_received and attempt <= 3:
            if attempt > 1:
                self.log(f"[RETRY] No valid reply, resending diag request (attempt {attempt})")
            self.uart.send_bytes(tx_frame)
            time.sleep(0.08)
            self.seq = (self.seq + 1) % 4

            rx_frame = self.uart.read_bytes(rx_len)

            if len(rx_frame) == rx_len:
                r_mc = rx_frame[0]
                r_ckt = rx_frame[1]
                r_data = rx_frame[2]
                r_seq, r_ck6 = self.checksum_calc.parse_ckt(r_ckt)
                masked_rckt = (r_seq & 0x03) << 6
                calc_ck6 = self.checksum_calc.calculate([r_mc, masked_rckt, r_data])

                if r_ck6 == calc_ck6:
                    response_received = True
                else:
                    self.log(f"[INTERPRETATION] Checksum ERROR! Expected 0x{calc_ck6:02X}, Got 0x{r_ck6:02X}")
            else:
                self.log(f"[ERROR] Incomplete DIAG RX.")
                time.sleep(0.05)
            attempt += 1
        return r_data

    def write_param(self, address, data, desc):
        # Ensure port is open
        if not self.uart.ser or not self.uart.ser.is_open:
            self.log(f"[MASTER] Port not open, reconnecting...")
            self.uart.connect()
        
        mc = 0x20 | (address & 0x0F)
        masked_ckt = (self.seq & 0x03) << 6
        ck6 = self.checksum_calc.calculate([mc, masked_ckt, data])
        ckt = self.checksum_calc.build_ckt(self.seq, ck6)
        tx_frame = [mc, ckt, data]
        
        self.log(f"[MASTER -> DEVICE] WRITE {desc} (Addr 0x{address:02X}, Data 0x{data:02X})")
        
        self.uart.send_bytes(tx_frame)
        self.seq = (self.seq + 1) % 4
        
        rx_len = 2
        rx_frame = self.uart.read_bytes(rx_len)
        
        if len(rx_frame) == rx_len:
             r_mc = rx_frame[0]
             r_ckt = rx_frame[1]
             r_seq, r_ck6 = self.checksum_calc.parse_ckt(r_ckt)
             masked_rckt = (r_seq & 0x03) << 6
             calc_ck6 = self.checksum_calc.calculate([r_mc, masked_rckt])
             
             if r_ck6 == calc_ck6:
                 return True
             else:
                 self.log(f"[INTERPRETATION] Checksum ERROR!")
        else:
            self.log(f"[ERROR] Incomplete RX.")
        return False

    def reset_isdu(self):
        """Reset ISDU state variables."""
        self.isdu_queue = None
        self.isdu_response = None
        self.isdu_active = False
        self.isdu_error = False

    def _queue_isdu_request(self, service, index, subindex=0, data=None):
        """Queue an ISDU request to be embedded into the next cyclic frame(s)."""
        self.isdu_response = None
        self.isdu_queue = {
            'service': service,
            'index': index,
            'subindex': subindex,
            'data': list(data) if data else []
        }

    def _parse_isdu_response_from_od(self, od_bytes, expected_write_len=None):
        if not od_bytes:
            return None, None

        # Trim trailing padding
        trimmed = list(od_bytes)
        while trimmed and trimmed[-1] == 0:
            trimmed.pop()
        if not trimmed:
            return None, None

        service = trimmed[0]
        service_rsp = (service >> 4) & 0x0F
        data_len = service & 0x0F

        if service_rsp == 0xF:
            error_code = 0
            if len(trimmed) >= 3:
                error_code = (trimmed[1] << 8) | trimmed[2]
                self.log(f"[ISDU/CYCLIC] Device Error: 0x{error_code:04X}")
            else:
                self.log("[ISDU/CYCLIC] Device Error (No Code)")
            return 'error', error_code

        if service_rsp == 0x1:
            payload = trimmed[1:]
            if data_len > 0 and len(payload) > data_len:
                payload = payload[:data_len]
            try:
                clean_payload = bytes(payload).rstrip(b'\x00')
                return 'read', clean_payload.decode('utf-8')
            except Exception:
                return 'read', payload

        if service_rsp == 0x2:
            if expected_write_len is None:
                return 'write', True
            return 'write', (data_len == expected_write_len)

        return None, None

    def _pad_isdu_frame(self, frame):
        """
        Pads an ISDU frame to OD_SIZE (16 bytes).
        Device expects exactly 16 bytes of payload data after service/index/subindex header.
        """
        # Frame format: [Service] [IndexHigh] [IndexLow] [Subindex] [Data...]
        # We need to pad the [Data...] portion to fill OD_SIZE
        if len(frame) < 4:
            return frame  # No data portion yet
        
        header = frame[:4]  # Service, Index High, Index Low, Subindex
        data = frame[4:]
        
        if len(data) < self.OD_SIZE:
            padding = [0] * (self.OD_SIZE - len(data))
            padded_data = data + padding
            result = header + padded_data
            self.log(f"[PADDING] Frame padded from {len(frame)} to {len(result)} bytes")
            return result
        
        return frame

    def send_system_command(self, command_byte):
        """
        Sends a system command (writes to address 0x0F).
        
        Commands:
        - 0x80: Device Reset
        - 0x82: Factory Restore
        """
        self.log(f"[SYSTEM CMD] Sending System Command 0x{command_byte:02X}...")
        return self.write_param(0x0F, command_byte, "System Command")

    def write_master_command(self, cmd):
        """
        Writes a Master Command to Address 0x00 (Page 1).
        
        Commands:
        - 0x9A: PREOPERATE (On-Request, direct ISDU access)
        - 0x99: OPERATE (Cyclic mode)
        """
        self.log(f"[MASTER CMD] Writing 0x{cmd:02X} to Master Command (Addr 0x00)")
        return self.write_param(0x00, cmd, "Master Command")

    def set_preoperate(self):
        """
        Forces the device into PREOPERATE mode (0x9A).
        This is required for simple ISDU access without cyclic interleaving.
        In PREOPERATE, the device responds to Type 0 (On-Request) commands directly,
        making ISDU read/write operations straightforward and synchronous.
        """
        self.log("[MASTER] Forcing PREOPERATE mode for ISDU access...")
        success = self.write_master_command(0x9A)
        time.sleep(0.1)  # Give device time to switch state
        return success

    def golden_startup_sequence(self):
        mct = self.read_param(0x02, "MinCycleTime")
        if mct: self.log(f"[DEVICE -> MASTER] MinCycleTime = 0x{mct:02X}")
        mseq = self.read_param(0x03, "M-Sequence Capability")
        if mseq: self.log(f"[DEVICE -> MASTER] M-Seq Cap = 0x{mseq:02X}")
        
        self.read_param(0x04, "Revision ID")
        self.read_param(0x05, "PD Input Length")
        self.read_param(0x06, "PD Output Length")
        self.read_param(0x07, "VendorID MSB")
        self.read_param(0x08, "VendorID LSB")
        self.read_param(0x09, "DeviceID Byte 1")
        self.read_param(0x0A, "DeviceID Byte 2")
        self.read_param(0x0B, "DeviceID Byte 3")
        
        self.transition_to(State.PREOPERATE)
        self.write_param(0x01, 0x36, "MasterCycleTime")
        time.sleep(0.05) 
        self.write_param(0x00, 0x9A, "Master Command: PREOPERATE")
        
        time.sleep(0.05)
        self.write_param(0x00, 0x99, "Master Command: OPERATE")
        self.transition_to(State.OPERATE)
        
        self.cyclic_exchange(duration=2.0) 

    def cyclic_exchange(self, duration=None, isdu_handler=None):
        # Ensure port is open
        if not self.uart.ser or not self.uart.ser.is_open:
            self.log(f"[MASTER] Port not open, reconnecting...")
            self.uart.connect()
        
        self.log(f"[MASTER] Entering CYCLIC EXCHANGE (Duration: {duration if duration else 'Infinite'})")
        i = 0
        start_time = time.time()
        
        try:
            while True:
                if duration and (time.time() - start_time > duration):
                    break
                if isdu_handler and isdu_handler.isdu_response is not None:
                    break
                    
                # Prepare payload (Type 2): [OD(16)] [PD(2)]
                od_out = [0x00] * self.OD_SIZE
                pd_out = [0x00] * self.PD_SIZE

                # Add ISDU into OD area ONLY if explicitly requested via isdu_handler parameter
                # If ISDU data is smaller than OD_SIZE, we pad with zeros.
                if isdu_handler and self.isdu_queue:
                    req = self.isdu_queue
                    service_byte = req['service']
                    index = req['index']
                    subindex = req['subindex']

                    isdu_payload = [service_byte, (index >> 8) & 0xFF, index & 0xFF, subindex & 0xFF]
                    if req.get('data'):
                        isdu_payload.extend(req['data'])

                    if len(isdu_payload) > self.OD_SIZE:
                        self.log(
                            f"[CYCLIC] ISDU payload too large for OD ({len(isdu_payload)} > {self.OD_SIZE}); truncating"
                        )
                        isdu_payload = isdu_payload[: self.OD_SIZE]

                    od_out[: len(isdu_payload)] = isdu_payload
                    self.log(f"[CYCLIC] Including ISDU in OD payload ({len(isdu_payload)}/{self.OD_SIZE}): {isdu_payload}")

                # Build cyclic frame
                # Master TX length must be: 1 (MC) + 1 (CKT) + OD_SIZE + PD_SIZE
                mc = 0x80
                masked_ckt = (self.seq & 0x03) << 6
                calc_data = [mc, masked_ckt] + od_out + pd_out
                ck6 = self.checksum_calc.calculate(calc_data)
                ckt = self.checksum_calc.build_ckt(self.seq, ck6)

                tx_frame = [mc, ckt] + od_out + pd_out
                self.uart.send_bytes(tx_frame)
                self.seq = (self.seq + 1) % 4
                time.sleep(0.02)
                
                # Read response (Type 2): [OD(16)] [PD(2)] [Status(1)]
                rx_len = self.OD_SIZE + self.PD_SIZE + 1
                rx_frame = self.uart.read_bytes(rx_len)

                self.log(f"[CYCLIC RX] Expected {rx_len} bytes, got {len(rx_frame)} bytes")
                if len(rx_frame) == rx_len:
                    r_od = rx_frame[0 : self.OD_SIZE]
                    r_pd = rx_frame[self.OD_SIZE : self.OD_SIZE + self.PD_SIZE]
                    r_status_byte = rx_frame[self.OD_SIZE + self.PD_SIZE]

                    self.log(f"[CYCLIC RX] OD={r_od} PD={r_pd} STATUS=0x{r_status_byte:02X}")

                    # If an ISDU request is pending, treat non-zero OD bytes as the ISDU response.
                    if isdu_handler and self.isdu_queue:
                        if any(b != 0 for b in r_od):
                            self.isdu_response = r_od
                            self.log(f"[CYCLIC] Received ISDU response in OD: {r_od}")
                elif len(rx_frame) > 0:
                    self.log(f"[CYCLIC RX] Raw bytes: {rx_frame}")
                
                time.sleep(0.01)
                i += 1
        except KeyboardInterrupt:
            self.log("[MASTER] Cyclic exchange stopped")

    def read_isdu(self, index, subindex=0):
        """
        Reads an ISDU parameter using Type 0 (On-Request) framing.
        This is a direct, synchronous read that bypasses cyclic exchange.
        
        Protocol:
        1. Send READ request: [0x90] [IndexHigh] [IndexLow] [SubIndex] [Checksum]
        2. Device responds with: [0x1X] [Data...] [Checksum]  (X = length)
                          or: [0xFX] [ErrorHigh] [ErrorLow] [Checksum]  (error)
        """
        self.log(f"[ISDU] Reading Index 0x{index:04X}...")

        # In OPERATE, ISDU must be transported inside cyclic (Type 2) frames.
        if self.state == State.OPERATE:
            self._queue_isdu_request(service=0x90, index=index, subindex=subindex, data=None)
            self.cyclic_exchange(duration=0.5, isdu_handler=self)
            response = self.isdu_response
            self.isdu_queue = None

            if response is None:
                self.log("[ISDU/CYCLIC] No ISDU response received")
                return None

            parsed_type, value = self._parse_isdu_response_from_od(response)
            if parsed_type == 'read':
                self.log(f"[ISDU/CYCLIC] Read Success: {value}")
                return value
            if parsed_type == 'error':
                return {'error': True, 'code': value}

            self.log(f"[ISDU/CYCLIC] Unrecognized response: {response}")
            return None

        # In PREOPERATE (or any non-OPERATE state), fall back to raw Type 0.
        
        # Ensure port is open
        if not self.uart.ser or not self.uart.ser.is_open:
            self.log(f"[MASTER] Port not open, reconnecting...")
            self.uart.connect()
        
        try:
            # Build Type 0 READ ISDU request
            service = 0x90  # Read Service
            payload = [
                service,
                (index >> 8) & 0xFF,  # Index High
                index & 0xFF,          # Index Low
                subindex & 0xFF        # Subindex
            ]
            
            # Add checksum
            masked_ckt = (self.seq & 0x03) << 6
            calc_data = payload.copy()
            calc_data.insert(1, masked_ckt)
            ck6 = self.checksum_calc.calculate(calc_data)
            ckt = self.checksum_calc.build_ckt(self.seq, ck6)
            
            tx_frame = [payload[0], ckt] + payload[1:]
            
            # Pad ISDU frame to OD_SIZE (16 bytes of data after header)
            # But only the data portion after [Service, IndexH, IndexL, Subindex]
            header_and_ckt = tx_frame[:2]  # [Service, CKT]
            isdu_payload = tx_frame[2:]     # [IndexH, IndexL, Subindex]
            if len(isdu_payload) < 4 + self.OD_SIZE:
                padding_needed = (4 + self.OD_SIZE) - len(isdu_payload)
                isdu_payload = isdu_payload + [0] * padding_needed
                tx_frame = header_and_ckt + isdu_payload
                self.log(f"[PADDING] READ request padded to {len(tx_frame)} bytes total")
            
            self.log(f"[ISDU -> DEVICE] READ Type 0 Request: Service=0x{service:02X}, Index=0x{index:04X}, Subindex=0x{subindex:02X}")
            
            # Send request
            self.uart.send_bytes(tx_frame)
            self.seq = (self.seq + 1) % 4
            time.sleep(0.05)  # Wait for device response
            
            # Read response
            rx_len = 20  # Estimate maximum ISDU response length
            rx_frame = self.uart.read_bytes(rx_len)
            
            if len(rx_frame) < 2:
                self.log(f"[ISDU] No valid response (got {len(rx_frame)} bytes)")
                return None
            
            # Parse response
            r_service = rx_frame[0]
            r_ckt = rx_frame[1]
            r_seq, r_ck6 = self.checksum_calc.parse_ckt(r_ckt)
            
            # Validate checksum
            masked_rckt = (r_seq & 0x03) << 6
            if len(rx_frame) > 2:
                resp_data = rx_frame[2:-1] if len(rx_frame) > 3 else rx_frame[2:]
                calc_ck6 = self.checksum_calc.calculate([r_service, masked_rckt] + resp_data)
            else:
                calc_ck6 = self.checksum_calc.calculate([r_service, masked_rckt])
            
            if r_ck6 != calc_ck6:
                self.log(f"[ISDU] Checksum ERROR! Expected 0x{calc_ck6:02X}, got 0x{r_ck6:02X}")
                return None
            
            service_rsp = (r_service >> 4) & 0x0F
            data_len = r_service & 0x0F
            
            # Parse service response
            if service_rsp == 0xF:
                # Error response
                if len(rx_frame) >= 4:
                    error_code = (rx_frame[2] << 8) | rx_frame[3]
                    self.log(f"[ISDU] Device Error: 0x{error_code:04X}")
                return None
                
            elif service_rsp == 0x1:
                # Success response - extract data
                if len(rx_frame) > 2:
                    payload = rx_frame[2:-1] if len(rx_frame) > 3 else rx_frame[2:]
                    # Trim to data_len if specified
                    if data_len > 0 and len(payload) > data_len:
                        payload = payload[:data_len]
                    
                    try:
                        clean_payload = bytes(payload).rstrip(b'\x00')
                        result = clean_payload.decode('utf-8')
                        self.log(f"[ISDU] Read Success: {result}")
                        return result
                    except:
                        self.log(f"[ISDU] Read Success (raw): {payload}")
                        return payload
                return None
            else:
                self.log(f"[ISDU] Unknown service response: 0x{r_service:02X}")
                return None
                
        except Exception as e:
            self.log(f"[ISDU] Exception during read: {e}")
            import traceback
            traceback.print_exc()
            return None

    def write_isdu(self, index, subindex, data):
        """
        Writes an ISDU parameter using Type 0 (On-Request) framing.
        This is a direct, synchronous write that bypasses cyclic exchange.
        
        Protocol:
        1. Send WRITE request: [0xAX] [IndexHigh] [IndexLow] [SubIndex] [Data...] [Checksum]
                         (X = length of data)
        2. Device responds with: [0x2X] [Checksum]  (X = length echoed back)
                          or: [0xFX] [ErrorHigh] [ErrorLow] [Checksum]  (error)
        """
        self.log(f"[ISDU] Writing Index 0x{index:04X}...")

        # Convert data to bytes if needed (used by both transports)
        if isinstance(data, str):
            data_bytes = [ord(c) for c in data]
        else:
            data_bytes = list(data)

        # In OPERATE, ISDU must be transported inside cyclic (Type 2) frames.
        if self.state == State.OPERATE:
            data_len = len(data_bytes)
            service = 0xA0 | (data_len & 0x0F)
            self._queue_isdu_request(service=service, index=index, subindex=subindex, data=data_bytes)
            self.cyclic_exchange(duration=0.5, isdu_handler=self)
            response = self.isdu_response
            self.isdu_queue = None

            if response is None:
                self.log("[ISDU/CYCLIC] No ISDU response received")
                return False

            parsed_type, value = self._parse_isdu_response_from_od(response, expected_write_len=data_len)
            if parsed_type == 'write':
                if value:
                    self.log("[ISDU/CYCLIC] Write Success")
                else:
                    self.log("[ISDU/CYCLIC] Write Failed (length mismatch)")
                return bool(value)
            if parsed_type == 'error':
                return False

            self.log(f"[ISDU/CYCLIC] Unrecognized response: {response}")
            return False

        # In PREOPERATE (or any non-OPERATE state), fall back to raw Type 0.
        
        # Ensure port is open
        if not self.uart.ser or not self.uart.ser.is_open:
            self.log(f"[MASTER] Port not open, reconnecting...")
            self.uart.connect()
        
        try:
            data_len = len(data_bytes)
            
            # Build Type 0 WRITE ISDU request
            service = 0xA0 | (data_len & 0x0F)  # Write Service with length
            payload = [
                service,
                (index >> 8) & 0xFF,  # Index High
                index & 0xFF,          # Index Low
                subindex & 0xFF        # Subindex
            ] + data_bytes
            
            # Add checksum
            masked_ckt = (self.seq & 0x03) << 6
            calc_data = payload.copy()
            calc_data.insert(1, masked_ckt)
            ck6 = self.checksum_calc.calculate(calc_data)
            ckt = self.checksum_calc.build_ckt(self.seq, ck6)
            
            tx_frame = [payload[0], ckt] + payload[1:]
            
            # Pad ISDU frame to OD_SIZE (16 bytes of data after service/index/subindex header)
            # Frame: [Service, CKT] [IndexH, IndexL, Subindex] [Data...] [Padding]
            header_and_ckt = tx_frame[:2]  # [Service, CKT]
            isdu_payload = tx_frame[2:]     # [IndexH, IndexL, Subindex, Data...]
            if len(isdu_payload) < 4 + self.OD_SIZE:
                padding_needed = (4 + self.OD_SIZE) - len(isdu_payload)
                isdu_payload = isdu_payload + [0] * padding_needed
                tx_frame = header_and_ckt + isdu_payload
                self.log(f"[PADDING] WRITE request padded to {len(tx_frame)} bytes total")
            
            self.log(f"[ISDU -> DEVICE] WRITE Type 0 Request: Service=0x{service:02X}, Index=0x{index:04X}, Subindex=0x{subindex:02X}, Data({data_len})="+str(data_bytes))
            
            # Send request
            self.uart.send_bytes(tx_frame)
            self.seq = (self.seq + 1) % 4
            time.sleep(0.05)  # Wait for device to write to EEPROM
            
            # Read response
            rx_len = 5  # Minimal response: [Service+Len] [CKT]
            rx_frame = self.uart.read_bytes(rx_len)
            
            if len(rx_frame) < 2:
                self.log(f"[ISDU] No valid response (got {len(rx_frame)} bytes)")
                return False
            
            # Parse response
            r_service = rx_frame[0]
            r_ckt = rx_frame[1]
            r_seq, r_ck6 = self.checksum_calc.parse_ckt(r_ckt)
            
            # Validate checksum
            masked_rckt = (r_seq & 0x03) << 6
            if len(rx_frame) > 2:
                resp_data = rx_frame[2:-1] if len(rx_frame) > 3 else rx_frame[2:]
                calc_ck6 = self.checksum_calc.calculate([r_service, masked_rckt] + resp_data)
            else:
                calc_ck6 = self.checksum_calc.calculate([r_service, masked_rckt])
            
            if r_ck6 != calc_ck6:
                self.log(f"[ISDU] Checksum ERROR! Expected 0x{calc_ck6:02X}, got 0x{r_ck6:02X}")
                return False
            
            service_rsp = (r_service >> 4) & 0x0F
            response_len = r_service & 0x0F
            
            # Parse service response
            if service_rsp == 0xF:
                # Error response
                if len(rx_frame) >= 4:
                    error_code = (rx_frame[2] << 8) | rx_frame[3]
                    self.log(f"[ISDU] Write Failed! Device Error: 0x{error_code:04X}")
                else:
                    self.log(f"[ISDU] Write Failed! Device Error")
                return False
                
            elif service_rsp == 0x2:
                # Success response - write confirmed
                if response_len == data_len:
                    self.log(f"[ISDU] Write Success - Device confirmed {response_len} bytes")
                    return True
                else:
                    self.log(f"[ISDU] Write Partial - Sent {data_len}, Device confirmed {response_len}")
                    return True
            else:
                self.log(f"[ISDU] Unknown service response: 0x{r_service:02X}")
                return False
                
        except Exception as e:
            self.log(f"[ISDU] Exception during write: {e}")
            import traceback
            traceback.print_exc()
            return False

def main():
    logging.basicConfig(level=logging.INFO)
    master = IOLinkMaster()
    master.run()

if __name__ == "__main__":
    main()
