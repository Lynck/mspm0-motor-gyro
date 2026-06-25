#ifndef CRC16_H
#define CRC16_H

#include "ti_msp_dl_config.h"

/* Modbus CRC16 查表法 — 标准 Modbus-RTU 校验 */
unsigned short CRC16(uint8_t *puchMsg, unsigned short usDataLen);

#endif
