/*===========================================================================
 * KronCommunication — Implementation
 *
 * Sections:
 *   1. CRC-16/IBM (Modbus)        — fully implemented
 *   2. Modbus RTU PDU build/parse — fully implemented
 *   3. Modbus TCP ADU build/parse — fully implemented
 *   4. Hardware-dependent stubs   — TODO: fill in per target MCU/OS
 *===========================================================================*/

#include "kroncomm.h"

/* ============================================================
 * §1  CRC-16/IBM  (Modbus variant)
 *
 *   Polynomial : 0xA001  (reflected 0x8005)
 *   Init       : 0xFFFF
 *   Output XOR : 0x0000
 *   Reflect in : true   (LSB-first processing)
 *
 * The CRC is appended to the RTU frame as [CRC_lo, CRC_hi].
 * ============================================================ */
uint16_t KRON_CRC16_Modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i, j;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0u; j < 8u; j++) {
            if (crc & 0x0001u) {
                crc = (uint16_t)((crc >> 1u) ^ 0xA001u);
            } else {
                crc >>= 1u;
            }
        }
    }
    return crc;
}

/* ============================================================
 * §2  Modbus RTU PDU build / parse
 * ============================================================ */

/* Internal: write big-endian uint16 into buf at offset, advance offset */
static void put16(uint8_t *buf, uint16_t *off, uint16_t val)
{
    buf[(*off)++] = (uint8_t)(val >> 8u);
    buf[(*off)++] = (uint8_t)(val & 0xFFu);
}

/* Internal: read big-endian uint16 from buf at offset, advance offset */
static uint16_t get16(const uint8_t *buf, uint16_t *off)
{
    uint16_t val = (uint16_t)((uint16_t)buf[*off] << 8u) | buf[(*off) + 1u];
    *off += 2u;
    return val;
}

uint16_t KRON_ModbusRTU_BuildRequest(uint8_t        *buf,
                                     uint8_t         slaveAddr,
                                     uint8_t         fc,
                                     uint16_t        startAddr,
                                     uint16_t        quantity,
                                     const uint16_t *writeData)
{
    if (!buf) return 0u;

    uint16_t off = 0u;

    buf[off++] = slaveAddr;
    buf[off++] = fc;

    switch (fc) {
    /* --- Read FCs: Addr(2) + Qty(2) --- */
    case MODBUS_FC_READ_COILS:
    case MODBUS_FC_READ_DISCRETE_INPUTS:
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS:
        put16(buf, &off, startAddr);
        put16(buf, &off, quantity);
        break;

    /* --- Write Single Coil: Addr(2) + Value(2) --- */
    case MODBUS_FC_WRITE_SINGLE_COIL:
        put16(buf, &off, startAddr);
        /* 0xFF00 = ON, 0x0000 = OFF */
        put16(buf, &off, writeData ? writeData[0] : 0u);
        break;

    /* --- Write Single Register: Addr(2) + Value(2) --- */
    case MODBUS_FC_WRITE_SINGLE_REGISTER:
        put16(buf, &off, startAddr);
        put16(buf, &off, writeData ? writeData[0] : 0u);
        break;

    /* --- Write Multiple Coils: Addr(2) + Qty(2) + ByteCount(1) + Data --- */
    case MODBUS_FC_WRITE_MULTIPLE_COILS: {
        if (!writeData || quantity == 0u || quantity > 1968u) return 0u;
        uint8_t  byteCount = (uint8_t)((quantity + 7u) / 8u);
        uint16_t numRegs   = (uint16_t)((quantity + 15u) / 16u);
        put16(buf, &off, startAddr);
        put16(buf, &off, quantity);
        buf[off++] = byteCount;
        /* Pack uint16 words into bytes, LSB first */
        uint16_t i;
        for (i = 0u; i < numRegs; i++) {
            buf[off++] = (uint8_t)(writeData[i] & 0xFFu);
            if ((i * 16u + 8u) < quantity) {
                buf[off++] = (uint8_t)(writeData[i] >> 8u);
            }
        }
        break;
    }

    /* --- Write Multiple Registers: Addr(2) + Qty(2) + ByteCount(1) + Data --- */
    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        /* Spec limit for FC16 is 123 registers (not 125 as for reads): the
         * request frame also carries Addr+Qty+ByteCount, so 125 would push a
         * 259-byte ADU into a 256-byte RTU buffer. */
        if (!writeData || quantity == 0u || quantity > KRONCOMM_MODBUS_MAX_WRITE_REGS)
            return 0u;
        put16(buf, &off, startAddr);
        put16(buf, &off, quantity);
        buf[off++] = (uint8_t)(quantity * 2u);
        uint16_t i;
        for (i = 0u; i < quantity; i++) {
            put16(buf, &off, writeData[i]);
        }
        break;
    }

    default:
        return 0u;  /* unsupported function code */
    }

    /* Append CRC16 [lo, hi] */
    uint16_t crc = KRON_CRC16_Modbus(buf, off);
    buf[off++] = (uint8_t)(crc & 0xFFu);
    buf[off++] = (uint8_t)(crc >> 8u);

    return off;
}

uint8_t KRON_ModbusRTU_ParseResponse(const uint8_t *buf,
                                     uint16_t       len,
                                     uint8_t        expectedSlaveAddr,
                                     uint8_t        expectedFC,
                                     uint16_t      *readData,
                                     uint16_t       maxRegs,
                                     uint8_t       *exceptionCode)
{
    if (!buf || len < 4u) return KRONCOMM_ERR_FRAME;

    if (exceptionCode) *exceptionCode = 0u;

    /* Verify CRC over all bytes except the last two */
    uint16_t crcCalc = KRON_CRC16_Modbus(buf, (uint16_t)(len - 2u));
    uint16_t crcRecv = (uint16_t)buf[len - 2u] | ((uint16_t)buf[len - 1u] << 8u);
    if (crcCalc != crcRecv) return KRONCOMM_ERR_CRC;

    if (buf[0] != expectedSlaveAddr) return KRONCOMM_ERR_FRAME;

    /* Exception response: FC | 0x80 */
    if (buf[1] == (expectedFC | 0x80u)) {
        if (exceptionCode && len >= 5u) *exceptionCode = buf[2];
        return KRONCOMM_ERR_EXCEPTION;
    }

    if (buf[1] != expectedFC) return KRONCOMM_ERR_FRAME;

    /* Parse read response (FC01-04) */
    if (expectedFC >= MODBUS_FC_READ_COILS &&
        expectedFC <= MODBUS_FC_READ_INPUT_REGISTERS)
    {
        if (len < 5u) return KRONCOMM_ERR_FRAME;
        uint8_t  byteCount = buf[2];
        uint16_t off       = 3u;

        /* The byte count is attacker/peer controlled — it must agree exactly
         * with the frame we actually received (Addr+FC+BC + data + CRC),
         * otherwise we would read past the end of the frame into adjacent or
         * stale buffer memory and report it as valid data. */
        if ((uint16_t)byteCount + 5u != len) return KRONCOMM_ERR_FRAME;

        if (expectedFC == MODBUS_FC_READ_COILS ||
            expectedFC == MODBUS_FC_READ_DISCRETE_INPUTS)
        {
            /* Bit-unpacked into readData: each bit → one uint16 word (0 or 1) */
            uint16_t i;
            for (i = 0u; i < byteCount && (i * 8u) < maxRegs; i++) {
                uint8_t byte = buf[off++];
                uint8_t b;
                for (b = 0u; b < 8u && (i * 8u + b) < maxRegs; b++) {
                    if (readData) readData[i * 8u + b] = (byte >> b) & 0x01u;
                }
            }
        } else {
            /* FC03/04: each register = 2 bytes big-endian */
            uint16_t numRegs = (uint16_t)(byteCount / 2u);
            if (numRegs > maxRegs) numRegs = maxRegs;
            uint16_t i;
            for (i = 0u; i < numRegs; i++) {
                if (readData) readData[i] = get16(buf, &off);
                else off += 2u;
            }
        }
    }
    /* Write FCs (05,06,0F,10): response echoes address + value/qty — no extra parse needed */

    return KRONCOMM_OK;
}

/* ============================================================
 * §3  Modbus TCP ADU build / parse
 *
 * MBAP Header (6 bytes):
 *   [0..1] Transaction ID (big-endian)
 *   [2..3] Protocol ID = 0x0000
 *   [4..5] Remaining length (UnitId + PDU)
 *   [6]    Unit ID
 *   [7]    Function Code
 *   ...    PDU data
 * ============================================================ */
uint16_t KRON_ModbusTCP_BuildRequest(uint8_t        *buf,
                                     uint16_t        transactionId,
                                     uint8_t         unitId,
                                     uint8_t         fc,
                                     uint16_t        startAddr,
                                     uint16_t        quantity,
                                     const uint16_t *writeData)
{
    if (!buf) return 0u;

    /* Build the PDU first into a temporary region starting at byte 7
     * (after the 6-byte MBAP header + unit id byte).
     * We reuse KRON_ModbusRTU_BuildRequest on a temp buffer (without
     * the slave address and CRC), or build manually. */

    uint8_t  pdu[KRONCOMM_MODBUS_TCP_FRAME_SIZE];
    /* Build RTU-style without slave address: put a dummy addr=0 and
     * strip the first byte + last 2 bytes (CRC).                      */
    uint16_t rtuLen = KRON_ModbusRTU_BuildRequest(pdu, 0x00u, fc, startAddr,
                                                   quantity, writeData);
    if (rtuLen < 4u) return 0u;  /* BuildRequest failed */

    /* PDU = rtu[1 .. rtuLen-3]  (skip addr at [0], skip CRC at end)   */
    uint16_t pduLen = (uint16_t)(rtuLen - 3u);  /* subtract addr + 2 CRC bytes */

    /* MBAP header */
    uint16_t off = 0u;
    put16(buf, &off, transactionId);           /* [0..1] Transaction ID */
    put16(buf, &off, 0x0000u);                 /* [2..3] Protocol ID    */
    put16(buf, &off, (uint16_t)(pduLen + 1u)); /* [4..5] Length = UnitId + PDU */
    buf[off++] = unitId;                       /* [6]    Unit ID        */

    /* Copy PDU (fc + data, skip leading 0x00 addr byte from RTU build) */
    uint16_t i;
    for (i = 0u; i < pduLen; i++) {
        buf[off++] = pdu[1u + i];
    }

    return off;
}

uint8_t KRON_ModbusTCP_ParseResponse(const uint8_t *buf,
                                     uint16_t       len,
                                     uint16_t       expectedTransId,
                                     uint8_t        expectedUnitId,
                                     uint8_t        expectedFC,
                                     uint16_t      *readData,
                                     uint16_t       maxRegs,
                                     uint8_t       *exceptionCode)
{
    if (!buf || len < 8u) return KRONCOMM_ERR_FRAME;

    /* len is the number of bytes actually received from the socket and is
     * therefore peer controlled. Reject anything that cannot be a legal TCP
     * ADU before it reaches the fixed-size synth[] buffer below. */
    if (len > KRONCOMM_MODBUS_TCP_FRAME_SIZE) return KRONCOMM_ERR_FRAME;

    if (exceptionCode) *exceptionCode = 0u;

    /* Validate MBAP */
    uint16_t transId   = ((uint16_t)buf[0] << 8u) | buf[1];
    uint16_t protoId   = ((uint16_t)buf[2] << 8u) | buf[3];
    uint16_t mbapLen   = ((uint16_t)buf[4] << 8u) | buf[5];
    uint8_t  unitId    = buf[6];
    uint8_t  fc        = buf[7];

    if (transId != expectedTransId) return KRONCOMM_ERR_FRAME;
    if (protoId != 0x0000u)        return KRONCOMM_ERR_FRAME;
    /* MBAP length counts UnitId + PDU, so the full ADU is mbapLen + 6 bytes. */
    if (mbapLen < 2u)              return KRONCOMM_ERR_FRAME;
    if ((uint16_t)(mbapLen + 6u) != len) return KRONCOMM_ERR_FRAME;
    if (unitId  != expectedUnitId) return KRONCOMM_ERR_FRAME;

    /* Exception response */
    if (fc == (expectedFC | 0x80u)) {
        if (exceptionCode && len >= 9u) *exceptionCode = buf[8];
        return KRONCOMM_ERR_EXCEPTION;
    }
    if (fc != expectedFC) return KRONCOMM_ERR_FRAME;

    /* Re-use RTU parser on a synthetic frame [unitId + tcp_payload + dummy_crc].
     * We build a fake RTU buffer from the TCP payload and add a correct CRC. */
    uint8_t  synth[KRONCOMM_MODBUS_TCP_FRAME_SIZE + 2u];
    /* buf[6] = unit ID (already placed as synth[0]).
     * buf[7..] = FC + PDU data.  Skip buf[6] to avoid duplicating unit ID. */
    synth[0] = unitId;
    uint16_t pduLen = (uint16_t)(len - 7u);   /* FC + data, excluding unit ID */
    uint16_t i;
    for (i = 0u; i < pduLen; i++) synth[1u + i] = buf[7u + i];
    uint16_t synthLen = (uint16_t)(1u + pduLen);
    uint16_t crc = KRON_CRC16_Modbus(synth, synthLen);
    synth[synthLen]     = (uint8_t)(crc & 0xFFu);
    synth[synthLen + 1u] = (uint8_t)(crc >> 8u);

    return KRON_ModbusRTU_ParseResponse(synth, (uint16_t)(synthLen + 2u),
                                        unitId, expectedFC,
                                        readData, maxRegs, exceptionCode);
}

/* ============================================================
 * §4  Hardware-dependent stubs
 *
 * Each function below contains:
 *   (void) casts  — suppress unused-parameter warnings
 *   TODO block    — describes exact behavior to implement
 * ============================================================ */

/* ----------------------------------------------------------
 * MODBUS_RTU_MASTER
 * ---------------------------------------------------------- */
void MODBUS_RTU_MASTER_Init(MODBUS_RTU_MASTER *inst)
{
    (void)inst;
    /* TODO:
     * Configure UART peripheral:
     *   - Baudrate = inst->Baudrate
     *   - Parity   = inst->Parity   (0=None, 1=Even, 2=Odd)
     *   - StopBits = inst->StopBits
     *   - DataBits = 8 (Modbus RTU always uses 8 data bits)
     *   - Enable RTS/DE line for RS-485 direction control
     * Enable RX interrupt or DMA.
     * Clear all output flags. */
}

void MODBUS_RTU_MASTER_Call(MODBUS_RTU_MASTER *inst, uint32_t currentTime)
{
    (void)inst;
    (void)currentTime;
    /* TODO:
     * Rising edge of Execute (Execute=true, _prevExecute=false):
     *   1. If Busy: set ErrorCode=KRONCOMM_ERR_BUSY, Error=true, return.
     *   2. Validate SlaveAddress (1–247), Quantity.
     *   3. Call KRON_ModbusRTU_BuildRequest() to fill a local TX buffer.
     *   4. Assert RTS/DE line (RS-485 TX enable).
     *   5. Transmit buffer over UART (blocking or DMA).
     *   6. Deassert RTS/DE (RS-485 RX mode).
     *   7. Set Busy=true, Done=false, Error=false, _prevExecute=true.
     *   8. Record currentTime as request start time.
     *
     * While Busy (waiting for response):
     *   - Poll UART RX buffer for incoming bytes.
     *   - Accumulate bytes until frame complete (inter-character timeout).
     *   - Check (currentTime - startTime) against inst->Timeout [ms].
     *     If timeout: set Error=true, ErrorCode=KRONCOMM_ERR_TIMEOUT, Busy=false.
     *   - When frame complete:
     *       Call KRON_ModbusRTU_ParseResponse().
     *       On KRONCOMM_OK: copy ReadData, set Done=true, Busy=false.
     *       On error:       set Error=true, ErrorCode=result, Busy=false.
     *
     * When Execute=false AND Done=true: clear Done.
     * Update _prevExecute = Execute. */
}

/* ----------------------------------------------------------
 * MODBUS_RTU_SLAVE
 * ---------------------------------------------------------- */
void MODBUS_RTU_SLAVE_Init(MODBUS_RTU_SLAVE *inst)
{
    (void)inst;
    /* TODO:
     * Configure UART as for MASTER_Init but in listen (slave) mode.
     * Enable RX interrupt/DMA to capture incoming frames continuously. */
}

void MODBUS_RTU_SLAVE_Call(MODBUS_RTU_SLAVE *inst, uint32_t currentTime)
{
    (void)inst;
    (void)currentTime;
    /* TODO:
     * Each scan:
     *   - Clear CoilWritten, RegisterWritten flags from previous scan.
     *   - If a complete frame has been received in the RX buffer:
     *       1. Call KRON_CRC16_Modbus() to validate CRC.
     *       2. Check SlaveAddress matches inst->SlaveAddress (or 0xFF broadcast).
     *       3. Dispatch on FunctionCode:
     *          FC01: reply with Coils[StartAddr..Qty-1]
     *          FC02: reply with DiscreteInputs[StartAddr..Qty-1]
     *          FC03: reply with HoldingRegisters[StartAddr..Qty-1]
     *          FC04: reply with InputRegisters[StartAddr..Qty-1]
     *          FC05: write Coils[StartAddr], set CoilWritten=true
     *          FC06: write HoldingRegisters[StartAddr], set RegisterWritten=true
     *          FC0F: write Coils[StartAddr..Qty-1], set CoilWritten=true
     *          FC10: write HoldingRegisters[StartAddr..Qty-1], set RegisterWritten=true
     *          Other: send exception 0x01 (Illegal Function)
     *       4. Build response using put16() helpers, compute and append CRC.
     *       5. Assert RTS/DE, transmit response, deassert RTS/DE.
     *   - Set Active=true when UART is correctly configured and no fault. */
}

/* ----------------------------------------------------------
 * MODBUS_TCP_CLIENT
 * ---------------------------------------------------------- */
void MODBUS_TCP_CLIENT_Init(MODBUS_TCP_CLIENT *inst)
{
    (void)inst;
    /* TODO:
     * Initialize TCP/IP stack (lwIP, FreeRTOS+TCP, etc.).
     * Create a TCP socket.
     * No connection yet — Connect input drives the actual connect. */
}

void MODBUS_TCP_CLIENT_Call(MODBUS_TCP_CLIENT *inst, uint32_t currentTime)
{
    (void)inst;
    (void)currentTime;
    /* TODO:
     * Rising edge of Connect:
     *   - Open TCP socket to ServerIP:ServerPort.
     *   - On success: set Connected=true.
     *   - On failure: set Error=true, ErrorCode=KRONCOMM_ERR_HARDWARE.
     *
     * Falling edge of Connect (or socket closed remotely):
     *   - Close TCP socket.
     *   - Set Connected=false, Done=false, Busy=false.
     *
     * Rising edge of Execute (while Connected):
     *   - Increment _transactionCounter, store in TransactionId.
     *   - Call KRON_ModbusTCP_BuildRequest() into a TX buffer.
     *   - Send buffer over TCP socket.
     *   - Set Busy=true, Done=false.
     *   - Record currentTime as request start time.
     *
     * While Busy:
     *   - Poll socket for response bytes.
     *   - On timeout: Error=true, ErrorCode=KRONCOMM_ERR_TIMEOUT, Busy=false.
     *   - On full response: call KRON_ModbusTCP_ParseResponse().
     *     Set Done/Error accordingly.
     *
     * Update _prevConnect, _prevExecute. */
}

/* ----------------------------------------------------------
 * MQTT_CLIENT
 * ---------------------------------------------------------- */
void MQTT_CLIENT_Init(MQTT_CLIENT *inst)
{
    (void)inst;
    /* TODO:
     * Initialize TCP/IP stack and MQTT client library context.
     * Configure callbacks for incoming messages:
     *   On message: copy topic/payload into inst->RcvTopic/RcvPayload,
     *               set MsgReceived=true for one scan. */
}

void MQTT_CLIENT_Call(MQTT_CLIENT *inst, uint32_t currentTime)
{
    (void)inst;
    (void)currentTime;
    /* TODO:
     * Clear MsgReceived from the previous scan.
     *
     * Rising edge of Connect:
     *   - Build MQTT CONNECT packet with ClientId, Username, Password,
     *     KeepAlive, CleanSession.
     *   - Establish TCP connection to BrokerIP:BrokerPort.
     *   - Send CONNECT, wait for CONNACK.
     *   - On success: set Connected=true.
     *   - On failure: set Error=true, ErrorCode=KRONCOMM_ERR_NOT_CONNECTED.
     *
     * While Connected:
     *   - Run MQTT keep-alive: send PINGREQ every KeepAlive seconds.
     *   - Call MQTT library poll/receive to handle inbound PUBLISH,
     *     PUBACK, SUBACK packets.
     *
     * Rising edge of Publish (while Connected):
     *   - Send PUBLISH with PubTopic, PubPayload[0..PubPayloadLen-1],
     *     PubQos, PubRetain.
     *   - On success: PubDone=true.
     *
     * Rising edge of Subscribe (while Connected):
     *   - Send SUBSCRIBE with SubTopic, SubQos.
     *   - On SUBACK: SubDone=true.
     *
     * Update _prevConnect, _prevPublish, _prevSubscribe. */
}

void MQTT_CLIENT_Disconnect(MQTT_CLIENT *inst)
{
    (void)inst;
    /* TODO:
     * Send MQTT DISCONNECT packet.
     * Close TCP socket.
     * Set Connected=false, PubDone=false, SubDone=false. */
}

/* ----------------------------------------------------------
 * CAN_NODE
 * ---------------------------------------------------------- */
void CAN_NODE_Init(CAN_NODE *inst)
{
    (void)inst;
    /* TODO:
     * Configure CAN peripheral (STM32 bxCAN / FDCAN or equivalent):
     *   - Baudrate: compute prescaler + BS1 + BS2 from inst->Baudrate
     *     and the APB clock frequency.
     *   - Configure acceptance filter bank:
     *     FilterId   = inst->FilterId
     *     FilterMask = inst->FilterMask  (0 = accept all)
     *   - Enable RX FIFO interrupt.
     *   - Start CAN peripheral. */
}

void CAN_NODE_Call(CAN_NODE *inst, uint32_t currentTime)
{
    (void)inst;
    (void)currentTime;
    /* TODO:
     * Clear SendDone and MsgReceived from previous scan.
     *
     * Enable / Disable:
     *   - Rising edge of Enable:  start CAN peripheral, Active=true.
     *   - Falling edge of Enable: stop CAN peripheral, Active=false.
     *
     * Rising edge of Send (while Active):
     *   - Load TxFrame into a CAN mailbox (Id, DataLen, Data, IsExtended, IsRTR).
     *   - Request transmission.
     *   - On TX-complete interrupt/flag: set SendDone=true.
     *   - On TX error: set Error=true, ErrorCode=KRONCOMM_ERR_HARDWARE.
     *
     * Receive (interrupt-driven or polled):
     *   - When a frame passes the filter and sits in RX FIFO:
     *     Copy to inst->RxFrame, set MsgReceived=true.
     *
     * Update TxErrorCount, RxErrorCount from CAN error registers.
     * Update _prevSend. */
}

/* ----------------------------------------------------------
 * UART
 * ---------------------------------------------------------- */
void UART_Init(UART *inst)
{
    (void)inst;
    /* TODO:
     * Configure UART peripheral:
     *   - Baudrate, DataBits, Parity, StopBits from inst fields.
     *   - If RtsEnable: configure RTS/DE GPIO for RS-485 direction control.
     *   - Enable RX DMA or interrupt for continuous receive.
     *   - Enable TX DMA or interrupt for non-blocking transmit. */
}

void UART_Call(UART *inst, uint32_t currentTime)
{
    (void)inst;
    (void)currentTime;
    /* TODO:
     * Enable / Disable:
     *   - Rising  edge of Enable: start UART peripheral, Active=true.
     *   - Falling edge of Enable: stop  UART peripheral, Active=false.
     *
     * Rising edge of Send (while Active, TxLen > 0):
     *   - If RtsEnable: assert RTS/DE (TX direction).
     *   - Transmit TxData[0..TxLen-1] via DMA or interrupt.
     *   - On TX-complete: set SendDone=true.
     *   - If RtsEnable: deassert RTS/DE (RX direction).
     *
     * Receive (interrupt/DMA):
     *   - Accumulate bytes into inst->RxData[].
     *   - Set RxLen = byte count, RxAvailable=true.
     *   - Guard against KRONCOMM_UART_BUFFER_SIZE overflow.
     *
     * Update _prevSend. */
}

void UART_ClearRx(UART *inst)
{
    (void)inst;
    /* TODO:
     * Reset RX DMA counter / interrupt buffer pointer.
     * Set inst->RxLen = 0, inst->RxAvailable = false.
     * Zero inst->RxData if needed. */
}
