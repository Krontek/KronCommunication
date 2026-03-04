#include "kroncomm.h"
#include <stdio.h>
int main(void) {
    /* 1. CRC actual value */
    uint8_t frame[] = {0x0B, 0x03, 0x00, 0x6B, 0x00, 0x03};
    printf("CRC actual: 0x%04X\n", KRON_CRC16_Modbus(frame, 6));

    /* 2. FC16 length */
    uint8_t buf[256];
    uint16_t wd[3] = {1, 2, 3};
    uint16_t len = KRON_ModbusRTU_BuildRequest(buf, 1, 0x10, 0, 3, wd);
    printf("FC16 len: %u\n", len);
    for(int i=0;i<len;i++) printf("%02X ", buf[i]);
    printf("\n");

    /* 3. TCP parse — check what ParseResponse returns */
    uint8_t resp[13];
    uint16_t off = 0;
    resp[off++]=0x00; resp[off++]=0x42;
    resp[off++]=0x00; resp[off++]=0x00;
    resp[off++]=0x00; resp[off++]=0x07;
    resp[off++]=0x01; resp[off++]=0x03; resp[off++]=0x04;
    resp[off++]=0x00; resp[off++]=0x01;
    resp[off++]=0x00; resp[off++]=0x02;
    uint16_t rd[10]; uint8_t ex=0;
    uint8_t rc = KRON_ModbusTCP_ParseResponse(resp, off, 0x42, 1, 0x03, rd, 10, &ex);
    printf("TCP parse rc: %u, rd[0]=%u rd[1]=%u ex=%u\n", rc, rd[0], rd[1], ex);
    return 0;
}
