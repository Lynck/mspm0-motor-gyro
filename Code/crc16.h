#ifndef CRC16_H
#define CRC16_H

#include "ti_msp_dl_config.h"

/*
 * Modbus CRC16 查表法 — 标准 Modbus-RTU 校验
 * 多项式: 0xA001
 * 入参: puchMsg 数据缓冲区指针, usDataLen 数据长度(字节)
 * 返回值: 16位 CRC (低字节在前，即 Modbus 标准 Little-Endian)
 */
unsigned short CRC16(uint8_t *puchMsg, unsigned short usDataLen);

#endif
