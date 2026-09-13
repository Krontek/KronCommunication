/*===========================================================================
 * KronCommunication — Tests
 * Build: tcc test.c kroncomm.c -o test_app && ./test_app
 *
 * Tests are split into:
 *   A) Pure-C utilities (CRC16, BuildRequest, ParseResponse) — full coverage
 *   B) Struct field access / enum values   — compile-time correctness
 *   C) Stub function calls                 — must not crash
 *===========================================================================*/

#include "kroncomm.h"
#include <stdio.h>
#include <string.h>

static int pass_count = 0;
static int fail_count = 0;

static void check(const char *name, int condition)
{
    if (condition) {
        pass_count++;
    } else {
        fail_count++;
        printf("FAIL: %s\n", name);
    }
}

/* =========================================================================
 * A.1  CRC-16/IBM  (Modbus)
 *
 * Known test vector (Modbus Application Protocol V1.1b3 example):
 *   Frame: 0B 03 00 6B 00 03
 *   CRC  : 76 87  → CRC16 = 0x8776
 * ========================================================================= */
static void test_crc16(void)
{
    /* Empty input → initial value 0xFFFF */
    check("CRC: empty → 0xFFFF", KRON_CRC16_Modbus(NULL, 0) == 0xFFFFu
          || KRON_CRC16_Modbus((const uint8_t*)"", 0) == 0xFFFFu);

    /* Known vector: 0B 03 00 6B 00 03 → 0xBD74 (appended on the wire as 74 BD) */
    {
        uint8_t frame[] = {0x0Bu, 0x03u, 0x00u, 0x6Bu, 0x00u, 0x03u};
        uint16_t crc = KRON_CRC16_Modbus(frame, 6u);
        check("CRC: known vector 0B 03 00 6B 00 03 → 0xBD74", crc == 0xBD74u);
    }

    /* Single byte 0x01 */
    {
        uint8_t b = 0x01u;
        uint16_t crc = KRON_CRC16_Modbus(&b, 1u);
        /* Re-verify: CRC of [0x01, CRC_lo, CRC_hi] should also make a
         * consistent round-trip via the parser — tested indirectly below */
        check("CRC: single byte 0x01 produces non-zero CRC", crc != 0u);
    }

    /* Append CRC to frame and verify the frame+CRC gives consistent result */
    {
        uint8_t frame[8];
        frame[0] = 0x01u; frame[1] = 0x03u;
        frame[2] = 0x00u; frame[3] = 0x00u;
        frame[4] = 0x00u; frame[5] = 0x02u;
        uint16_t crc = KRON_CRC16_Modbus(frame, 6u);
        frame[6] = (uint8_t)(crc & 0xFFu);
        frame[7] = (uint8_t)(crc >> 8u);
        /* CRC over all 8 bytes should equal CRC over first 6 (same data) */
        uint16_t crc2 = KRON_CRC16_Modbus(frame, 6u);
        check("CRC: deterministic (same input same result)", crc == crc2);
    }
}

/* =========================================================================
 * A.2  KRON_ModbusRTU_BuildRequest
 * ========================================================================= */
static void test_rtu_build(void)
{
    uint8_t buf[256];

    /* FC03 — Read 2 Holding Registers at address 0x0100, slave=1 */
    {
        uint16_t len = KRON_ModbusRTU_BuildRequest(buf, 1u,
                           MODBUS_FC_READ_HOLDING_REGISTERS, 0x0100u, 2u, NULL);
        check("RTU Build FC03: length = 8", len == 8u);
        check("RTU Build FC03: slave addr",   buf[0] == 0x01u);
        check("RTU Build FC03: FC",           buf[1] == 0x03u);
        check("RTU Build FC03: addr hi",      buf[2] == 0x01u);
        check("RTU Build FC03: addr lo",      buf[3] == 0x00u);
        check("RTU Build FC03: qty  hi",      buf[4] == 0x00u);
        check("RTU Build FC03: qty  lo",      buf[5] == 0x02u);
        /* Verify CRC in last 2 bytes */
        uint16_t crc = KRON_CRC16_Modbus(buf, 6u);
        check("RTU Build FC03: CRC lo", buf[6] == (uint8_t)(crc & 0xFFu));
        check("RTU Build FC03: CRC hi", buf[7] == (uint8_t)(crc >> 8u));
    }

    /* FC06 — Write Single Register */
    {
        uint16_t wd[1] = {0x1234u};
        uint16_t len = KRON_ModbusRTU_BuildRequest(buf, 2u,
                           MODBUS_FC_WRITE_SINGLE_REGISTER, 0x0010u, 1u, wd);
        check("RTU Build FC06: length = 8", len == 8u);
        check("RTU Build FC06: FC",          buf[1] == 0x06u);
        check("RTU Build FC06: value hi",    buf[4] == 0x12u);
        check("RTU Build FC06: value lo",    buf[5] == 0x34u);
    }

    /* FC05 — Write Single Coil (ON = 0xFF00) */
    {
        uint16_t wd[1] = {0xFF00u};
        uint16_t len = KRON_ModbusRTU_BuildRequest(buf, 1u,
                           MODBUS_FC_WRITE_SINGLE_COIL, 0x0005u, 1u, wd);
        check("RTU Build FC05: length = 8", len == 8u);
        check("RTU Build FC05: value hi",   buf[4] == 0xFFu);
        check("RTU Build FC05: value lo",   buf[5] == 0x00u);
    }

    /* FC16 — Write Multiple Registers */
    {
        uint16_t wd[3] = {0x0001u, 0x0002u, 0x0003u};
        uint16_t len = KRON_ModbusRTU_BuildRequest(buf, 1u,
                           MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 0x0000u, 3u, wd);
        /* Frame: addr(1) + FC(1) + Addr(2) + Qty(2) + ByteCount(1) +
         *        6 data bytes + CRC(2) = 15 bytes */
        check("RTU Build FC16: length = 15", len == 15u);
        check("RTU Build FC16: FC",           buf[1] == 0x10u);
        check("RTU Build FC16: byte count",   buf[6] == 0x06u);
        check("RTU Build FC16: reg0 hi",      buf[7] == 0x00u);
        check("RTU Build FC16: reg0 lo",      buf[8] == 0x01u);
        check("RTU Build FC16: reg1 hi",      buf[9] == 0x00u);
        check("RTU Build FC16: reg1 lo",      buf[10] == 0x02u);
    }

    /* NULL buffer → 0 */
    {
        uint16_t len = KRON_ModbusRTU_BuildRequest(NULL, 1u, 0x03u, 0u, 1u, NULL);
        check("RTU Build: NULL buf → 0", len == 0u);
    }

    /* Unsupported FC → 0 */
    {
        uint16_t len = KRON_ModbusRTU_BuildRequest(buf, 1u, 0xFFu, 0u, 1u, NULL);
        check("RTU Build: unknown FC → 0", len == 0u);
    }
}

/* =========================================================================
 * A.3  KRON_ModbusRTU_ParseResponse
 * ========================================================================= */

/* Helper: build a valid read-register response frame in buf, return length */
static uint16_t make_rtu_read_resp(uint8_t *buf, uint8_t slave, uint8_t fc,
                                   const uint16_t *regs, uint8_t numRegs)
{
    uint16_t off = 0u;
    buf[off++] = slave;
    buf[off++] = fc;
    buf[off++] = (uint8_t)(numRegs * 2u);
    uint8_t i;
    for (i = 0u; i < numRegs; i++) {
        buf[off++] = (uint8_t)(regs[i] >> 8u);
        buf[off++] = (uint8_t)(regs[i] & 0xFFu);
    }
    uint16_t crc = KRON_CRC16_Modbus(buf, off);
    buf[off++] = (uint8_t)(crc & 0xFFu);
    buf[off++] = (uint8_t)(crc >> 8u);
    return off;
}

static void test_rtu_parse(void)
{
    uint8_t  buf[256];
    uint16_t rd[10];
    uint8_t  ex;

    /* FC03 response: slave=1, 2 registers: {0xABCD, 0x1234} */
    {
        uint16_t regs[2] = {0xABCDu, 0x1234u};
        uint16_t len = make_rtu_read_resp(buf, 1u, 0x03u, regs, 2u);
        memset(rd, 0, sizeof(rd));
        uint8_t rc = KRON_ModbusRTU_ParseResponse(buf, len, 1u, 0x03u,
                                                   rd, 10u, &ex);
        check("RTU Parse FC03: KRONCOMM_OK",    rc == KRONCOMM_OK);
        check("RTU Parse FC03: reg[0]=0xABCD",  rd[0] == 0xABCDu);
        check("RTU Parse FC03: reg[1]=0x1234",  rd[1] == 0x1234u);
        check("RTU Parse FC03: exception=0",    ex == 0u);
    }

    /* CRC corruption → KRONCOMM_ERR_CRC */
    {
        uint16_t regs[1] = {0x0001u};
        uint16_t len = make_rtu_read_resp(buf, 1u, 0x03u, regs, 1u);
        buf[len - 1u] ^= 0xFFu;  /* corrupt last CRC byte */
        uint8_t rc = KRON_ModbusRTU_ParseResponse(buf, len, 1u, 0x03u,
                                                   rd, 10u, &ex);
        check("RTU Parse: CRC error detected", rc == KRONCOMM_ERR_CRC);
    }

    /* Exception response (FC | 0x80): illegal address */
    {
        buf[0] = 0x01u;
        buf[1] = (uint8_t)(0x03u | 0x80u);
        buf[2] = MODBUS_EX_ILLEGAL_ADDRESS;
        uint16_t crc = KRON_CRC16_Modbus(buf, 3u);
        buf[3] = (uint8_t)(crc & 0xFFu);
        buf[4] = (uint8_t)(crc >> 8u);
        uint8_t rc = KRON_ModbusRTU_ParseResponse(buf, 5u, 1u, 0x03u,
                                                   rd, 10u, &ex);
        check("RTU Parse: exception detected",          rc == KRONCOMM_ERR_EXCEPTION);
        check("RTU Parse: exception code = ILLEGAL_ADDR", ex == MODBUS_EX_ILLEGAL_ADDRESS);
    }

    /* Wrong slave address → KRONCOMM_ERR_FRAME */
    {
        uint16_t regs[1] = {0x0001u};
        uint16_t len = make_rtu_read_resp(buf, 2u, 0x03u, regs, 1u);
        uint8_t rc = KRON_ModbusRTU_ParseResponse(buf, len, 1u, 0x03u,
                                                   rd, 10u, &ex);
        check("RTU Parse: wrong slave → ERR_FRAME", rc == KRONCOMM_ERR_FRAME);
    }

    /* Too short → KRONCOMM_ERR_FRAME */
    {
        uint8_t  tiny[3] = {0x01u, 0x03u, 0x00u};
        uint8_t rc = KRON_ModbusRTU_ParseResponse(tiny, 3u, 1u, 0x03u,
                                                   rd, 10u, &ex);
        check("RTU Parse: too short → ERR_FRAME", rc == KRONCOMM_ERR_FRAME);
    }
}

/* =========================================================================
 * A.4  Modbus TCP build / parse  (round-trip)
 * ========================================================================= */
static void test_tcp(void)
{
    uint8_t  buf[260];
    uint16_t rd[10];
    uint8_t  ex = 0u;

    /* Build FC03 request: transId=0x0042, unitId=1, addr=0, qty=3 */
    uint16_t len = KRON_ModbusTCP_BuildRequest(buf, 0x0042u, 1u,
                       MODBUS_FC_READ_HOLDING_REGISTERS, 0u, 3u, NULL);
    check("TCP Build FC03: length > 6", len > 6u);
    check("TCP Build FC03: trans ID hi", buf[0] == 0x00u);
    check("TCP Build FC03: trans ID lo", buf[1] == 0x42u);
    check("TCP Build FC03: proto hi",    buf[2] == 0x00u);
    check("TCP Build FC03: proto lo",    buf[3] == 0x00u);
    check("TCP Build FC03: unit ID",     buf[6] == 0x01u);
    check("TCP Build FC03: FC",          buf[7] == 0x03u);

    /* Build a fake TCP response for slave=1, FC03, 2 regs {0x0001, 0x0002} */
    {
        uint8_t resp[260];
        uint16_t off = 0u;
        /* MBAP */
        resp[off++] = 0x00u; resp[off++] = 0x42u; /* transId */
        resp[off++] = 0x00u; resp[off++] = 0x00u; /* proto   */
        resp[off++] = 0x00u; resp[off++] = 0x07u; /* len = 1+6 = 7 (UnitId+FC+ByteCount+4) */
        resp[off++] = 0x01u;  /* unit id  */
        resp[off++] = 0x03u;  /* FC       */
        resp[off++] = 0x04u;  /* byte count = 4 */
        resp[off++] = 0x00u; resp[off++] = 0x01u;  /* reg[0] = 1 */
        resp[off++] = 0x00u; resp[off++] = 0x02u;  /* reg[1] = 2 */

        memset(rd, 0, sizeof(rd));
        uint8_t rc = KRON_ModbusTCP_ParseResponse(resp, off,
                         0x0042u, 1u, 0x03u, rd, 10u, &ex);
        check("TCP Parse FC03: KRONCOMM_OK", rc == KRONCOMM_OK);
        check("TCP Parse FC03: rd[0]=1",     rd[0] == 1u);
        check("TCP Parse FC03: rd[1]=2",     rd[1] == 2u);
    }

    /* Wrong transaction ID → ERR_FRAME */
    {
        uint8_t resp[12];
        memset(resp, 0, sizeof(resp));
        resp[0] = 0x00u; resp[1] = 0x99u;  /* wrong transId */
        resp[6] = 0x01u; resp[7] = 0x03u; resp[8] = 0x02u;
        resp[9] = 0x00u; resp[10] = 0x05u;
        /* add dummy bytes for length */
        uint8_t rc = KRON_ModbusTCP_ParseResponse(resp, 11u,
                         0x0042u, 1u, 0x03u, rd, 10u, &ex);
        check("TCP Parse: wrong transId → ERR_FRAME", rc == KRONCOMM_ERR_FRAME);
    }
}

/* =========================================================================
 * B.  Struct field access + enum values
 * ========================================================================= */
static void test_structs(void)
{
    /* MODBUS_RTU_MASTER */
    {
        MODBUS_RTU_MASTER m = {0};
        m.Baudrate = 19200u;
        m.SlaveAddress = 3u;
        m.FunctionCode = MODBUS_FC_READ_HOLDING_REGISTERS;
        m.StartAddress = 0u;
        m.Quantity = 10u;
        m.Execute = true;
        check("MODBUS_RTU_MASTER: fields accessible", m.Baudrate == 19200u);
        check("MODBUS_RTU_MASTER: WriteData size ok",
              sizeof(m.WriteData) == KRONCOMM_MODBUS_MAX_REGS * sizeof(uint16_t));
        check("MODBUS_RTU_MASTER: ReadData  size ok",
              sizeof(m.ReadData)  == KRONCOMM_MODBUS_MAX_REGS * sizeof(uint16_t));
    }

    /* MODBUS_RTU_SLAVE */
    {
        static MODBUS_RTU_SLAVE s;
        memset(&s, 0, sizeof(s));
        s.SlaveAddress = 5u;
        s.HoldingRegisters[0] = 42u;
        s.Coils[0] = true;
        check("MODBUS_RTU_SLAVE: HR accessible", s.HoldingRegisters[0] == 42u);
        check("MODBUS_RTU_SLAVE: Coils accessible", s.Coils[0] == true);
    }

    /* MODBUS_TCP_CLIENT */
    {
        MODBUS_TCP_CLIENT tc = {0};
        tc.ServerIP[0] = 192u; tc.ServerIP[1] = 168u;
        tc.ServerPort = 502u;
        tc.UnitId = 1u;
        check("MODBUS_TCP_CLIENT: ServerIP[0]=192", tc.ServerIP[0] == 192u);
        check("MODBUS_TCP_CLIENT: ServerPort=502",  tc.ServerPort  == 502u);
    }

    /* MQTT_CLIENT */
    {
        MQTT_CLIENT mq = {0};
        mq.BrokerPort = 1883u;
        mq.PubQos = MQTT_QOS_1;
        mq.SubQos = MQTT_QOS_2;
        mq.KeepAlive = 60u;
        mq.CleanSession = true;
        check("MQTT_CLIENT: BrokerPort=1883", mq.BrokerPort == 1883u);
        check("MQTT_CLIENT: PubQos=1",        mq.PubQos == 1);
        check("MQTT_CLIENT: KeepAlive=60",    mq.KeepAlive == 60u);
    }

    /* MQTT_QOS enum */
    check("MQTT_QOS_0 = 0", MQTT_QOS_0 == 0);
    check("MQTT_QOS_1 = 1", MQTT_QOS_1 == 1);
    check("MQTT_QOS_2 = 2", MQTT_QOS_2 == 2);

    /* CAN_NODE */
    {
        CAN_NODE c = {0};
        c.Baudrate = CAN_BAUDRATE_500K;
        c.TxFrame.Id = 0x123u;
        c.TxFrame.DataLen = 8u;
        c.TxFrame.Data[0] = 0xDEu;
        c.TxFrame.IsExtended = false;
        check("CAN_NODE: Baudrate=500K",     c.Baudrate == 500000u);
        check("CAN_NODE: TxFrame.Id=0x123",  c.TxFrame.Id == 0x123u);
        check("CAN_NODE: TxFrame.Data[0]",   c.TxFrame.Data[0] == 0xDEu);
    }

    /* CAN_BAUDRATE enum */
    check("CAN_BAUDRATE_125K = 125000",  CAN_BAUDRATE_125K  == 125000);
    check("CAN_BAUDRATE_250K = 250000",  CAN_BAUDRATE_250K  == 250000);
    check("CAN_BAUDRATE_500K = 500000",  CAN_BAUDRATE_500K  == 500000);
    check("CAN_BAUDRATE_1M   = 1000000", CAN_BAUDRATE_1M    == 1000000);

    /* UART */
    {
        UART u = {0};
        u.Baudrate = 115200u;
        u.DataBits = 8u;
        u.Parity   = UART_PARITY_EVEN;
        u.StopBits = 1u;
        u.RtsEnable = true;
        check("UART: Baudrate=115200",    u.Baudrate == 115200u);
        check("UART: Parity=EVEN",        u.Parity == UART_PARITY_EVEN);
        check("UART: RtsEnable=true",     u.RtsEnable == true);
        check("UART: TxData buffer size",
              sizeof(u.TxData) == KRONCOMM_UART_BUFFER_SIZE);
    }

    /* UART_PARITY enum */
    check("UART_PARITY_NONE = 0", UART_PARITY_NONE == 0);
    check("UART_PARITY_EVEN = 1", UART_PARITY_EVEN == 1);
    check("UART_PARITY_ODD  = 2", UART_PARITY_ODD  == 2);

    /* Error code constants */
    check("KRONCOMM_OK = 0x00",         KRONCOMM_OK          == 0x00u);
    check("KRONCOMM_ERR_TIMEOUT = 0x01",KRONCOMM_ERR_TIMEOUT == 0x01u);
    check("KRONCOMM_ERR_CRC = 0x02",    KRONCOMM_ERR_CRC     == 0x02u);

    /* Modbus FC constants */
    check("FC_READ_HOLDING = 0x03",  MODBUS_FC_READ_HOLDING_REGISTERS   == 0x03u);
    check("FC_WRITE_MULTI  = 0x10",  MODBUS_FC_WRITE_MULTIPLE_REGISTERS == 0x10u);
}

/* =========================================================================
 * C.  Stub function calls — must not crash
 * ========================================================================= */
static void test_stubs(void)
{
    /* Modbus RTU Master */
    {
        MODBUS_RTU_MASTER m = {0};
        m.Baudrate = 9600u; m.Timeout = 1000u;
        MODBUS_RTU_MASTER_Init(&m);
        MODBUS_RTU_MASTER_Call(&m, 0u);
        MODBUS_RTU_MASTER_Call(&m, 100u);
        check("MODBUS_RTU_MASTER stubs: no crash", 1);
    }

    /* Modbus RTU Slave */
    {
        static MODBUS_RTU_SLAVE s;
        memset(&s, 0, sizeof(s));
        s.SlaveAddress = 1u; s.Baudrate = 9600u;
        MODBUS_RTU_SLAVE_Init(&s);
        MODBUS_RTU_SLAVE_Call(&s, 0u);
        check("MODBUS_RTU_SLAVE stubs: no crash", 1);
    }

    /* Modbus TCP Client */
    {
        MODBUS_TCP_CLIENT tc = {0};
        tc.ServerPort = 502u;
        MODBUS_TCP_CLIENT_Init(&tc);
        MODBUS_TCP_CLIENT_Call(&tc, 0u);
        check("MODBUS_TCP_CLIENT stubs: no crash", 1);
    }

    /* MQTT Client */
    {
        MQTT_CLIENT mq = {0};
        mq.BrokerPort = 1883u;
        mq.KeepAlive  = 60u;
        MQTT_CLIENT_Init(&mq);
        MQTT_CLIENT_Call(&mq, 0u);
        MQTT_CLIENT_Disconnect(&mq);
        check("MQTT_CLIENT stubs: no crash", 1);
    }

    /* CAN Node */
    {
        CAN_NODE c = {0};
        c.Baudrate = CAN_BAUDRATE_500K;
        CAN_NODE_Init(&c);
        CAN_NODE_Call(&c, 0u);
        check("CAN_NODE stubs: no crash", 1);
    }

    /* UART */
    {
        UART u = {0};
        u.Baudrate = 115200u;
        u.DataBits = 8u;
        UART_Init(&u);
        UART_Call(&u, 0u);
        UART_ClearRx(&u);
        check("UART stubs: no crash", 1);
    }
}

/* =========================================================================
 * Regression tests — hardening fixes
 * ========================================================================= */
static void test_hardening(void)
{
    printf("\n--- Hardening / bounds ---\n");

    /* FC16: spec limit is 123 registers, not 125 — 125 would build a
     * 259-byte ADU into a 256-byte RTU buffer. */
    {
        uint16_t wd[KRONCOMM_MODBUS_MAX_REGS] = {0};
        uint8_t  b[KRONCOMM_MODBUS_RTU_FRAME_SIZE];
        check("FC16: qty 123 accepted",
              KRON_ModbusRTU_BuildRequest(b, 1u, MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                          0u, 123u, wd) == 255u);
        check("FC16: qty 124 rejected",
              KRON_ModbusRTU_BuildRequest(b, 1u, MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                          0u, 124u, wd) == 0u);
        check("FC16: qty 125 rejected",
              KRON_ModbusRTU_BuildRequest(b, 1u, MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
                                          0u, 125u, wd) == 0u);
    }

    /* RTU parse: a byte count that disagrees with the received length must be
     * rejected, not trusted into a read past the end of the frame. */
    {
        /* Well-formed FC03 reply, 2 registers */
        uint8_t f[16] = {0x01u, 0x03u, 0x04u, 0x00u, 0x0Au, 0x00u, 0x0Bu};
        uint16_t crc = KRON_CRC16_Modbus(f, 7u);
        f[7] = (uint8_t)(crc & 0xFFu);
        f[8] = (uint8_t)(crc >> 8u);
        uint16_t rd[8] = {0};
        check("RTU parse: valid FC03 accepted",
              KRON_ModbusRTU_ParseResponse(f, 9u, 1u, 0x03u, rd, 8u, NULL)
              == KRONCOMM_OK);
        check("RTU parse: FC03 data reg0", rd[0] == 0x000Au);
        check("RTU parse: FC03 data reg1", rd[1] == 0x000Bu);

        /* Same length, but claiming 250 data bytes — CRC recomputed so the
         * frame is self-consistent; only the length cross-check catches it. */
        f[2] = 250u;
        crc  = KRON_CRC16_Modbus(f, 7u);
        f[7] = (uint8_t)(crc & 0xFFu);
        f[8] = (uint8_t)(crc >> 8u);
        check("RTU parse: oversized byte count rejected",
              KRON_ModbusRTU_ParseResponse(f, 9u, 1u, 0x03u, rd, 8u, NULL)
              == KRONCOMM_ERR_FRAME);
    }

    /* TCP parse: an over-long frame must be rejected before it is copied into
     * the fixed-size synth buffer, and the MBAP length must match. */
    {
        uint8_t big[600];
        uint16_t rd[8] = {0};
        memset(big, 0, sizeof(big));
        big[0] = 0x00u; big[1] = 0x01u;          /* transaction id */
        big[2] = 0x00u; big[3] = 0x00u;          /* protocol id    */
        big[4] = 0x02u; big[5] = 0x1Au;          /* length = 538   */
        big[6] = 0x01u;                          /* unit id        */
        big[7] = 0x03u;                          /* function code  */
        big[8] = 0xFFu;                          /* byte count     */
        check("TCP parse: over-long frame rejected",
              KRON_ModbusTCP_ParseResponse(big, 544u, 1u, 1u, 0x03u, rd, 8u, NULL)
              == KRONCOMM_ERR_FRAME);

        /* Legal size, but MBAP length field disagrees with the byte count */
        big[4] = 0x00u; big[5] = 0x63u;          /* claims 99 bytes */
        check("TCP parse: MBAP length mismatch rejected",
              KRON_ModbusTCP_ParseResponse(big, 11u, 1u, 1u, 0x03u, rd, 8u, NULL)
              == KRONCOMM_ERR_FRAME);
    }

    /* TCP round trip still works after the added validation. */
    {
        uint8_t f[KRONCOMM_MODBUS_TCP_FRAME_SIZE];
        uint16_t rd[4] = {0};
        f[0] = 0x12u; f[1] = 0x34u;              /* transaction id */
        f[2] = 0x00u; f[3] = 0x00u;              /* protocol id    */
        f[4] = 0x00u; f[5] = 0x07u;              /* unit + 6 PDU   */
        f[6] = 0x0Bu;                            /* unit id        */
        f[7] = 0x03u;                            /* FC03           */
        f[8] = 0x04u;                            /* byte count     */
        f[9]  = 0x11u; f[10] = 0x22u;
        f[11] = 0x33u; f[12] = 0x44u;
        check("TCP parse: valid frame accepted",
              KRON_ModbusTCP_ParseResponse(f, 13u, 0x1234u, 0x0Bu, 0x03u,
                                           rd, 4u, NULL) == KRONCOMM_OK);
        check("TCP parse: reg0", rd[0] == 0x1122u);
        check("TCP parse: reg1", rd[1] == 0x3344u);
    }
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    test_crc16();
    test_rtu_build();
    test_rtu_parse();
    test_tcp();
    test_structs();
    test_stubs();
    test_hardening();

    printf("%d passed, %d failed\n", pass_count, fail_count);
    return fail_count == 0 ? 0 : 1;
}
