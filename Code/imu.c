#include "imu.h"
uint8_t test_comm[20];

void test(void)
{
	test_comm[0]=0xaa;
	test_comm[1]=0x06;
	test_comm[2]=0x01;
	test_comm[3]=0x02;
	test_comm[4]=0x01;
	uint16_t aa;
	aa = CRC16(test_comm, 5);

	 test_comm[5] = aa & 0xFF;        // CRC_L
   test_comm[6] = (aa >> 8) & 0xFF; // CRC_H

}



volatile int16_t gyro_angle_raw;
volatile int16_t gyro_dps_raw;
volatile uint8_t gyro_rx_done;


void Gyro_ParseFrame(uint8_t data)
{
    static uint8_t state = 0;
    static uint8_t frame[9];
    static uint8_t idx = 0;
    uint16_t crc_calc;
    uint16_t crc_recv;

    switch(state)
    {
        case 0:     // ��ַ
            if(data == 0x0A)
            {
                idx = 0;
                frame[idx++] = data;
                state = 1;
            }
            break;

        case 1:     // ������
            if(data == 0x03)
            {
                frame[idx++] = data;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2:     // �ֽ���
            if(data == 0x04)
            {
                frame[idx++] = data;
                state = 3;
            }
            else
            {
                state = 0;
            }
            break;

        case 3:     // ����ʣ��6�ֽ�
            frame[idx++] = data;

            if(idx >= 9)
            {
                /* CRCУ�� */
                crc_calc = CRC16(frame, 7);
                crc_recv = frame[7] |
                          ((uint16_t)frame[8] << 8);

                if(crc_calc == crc_recv)
                {
                    /* �Ƕ� */
                    gyro_angle_raw =
                        (int16_t)(
                            ((uint16_t)frame[3] << 8) |
                             frame[4]);

                    /* ���ٶ� */
                    gyro_dps_raw =
                        (int16_t)(
                            ((uint16_t)frame[5] << 8) |
                             frame[6]);

                    gyro_rx_done = 1;
                }

                state = 0;
            }
            break;

        default:
            state = 0;
            break;
    }
}


	
void Gyro_ConfigReportRateRx(void)//AA 06 01 01 01 AD 00
{

		uint8_t idx = 0;
		uint8_t frame[20];

    frame[idx++] = 0xAA;        // ��վ��ַ
    frame[idx++] = 0x06;        // �����룺д�������ּĴ���
    frame[idx++] = 0x01;        // ��ʼ�Ĵ������ֽ�
	
	
    frame[idx++] = 0x01;        // ��ʼ�Ĵ������ֽ�

  
    // �Ĵ��� 1
    frame[idx++] = 0x01;

//    uint16_t crc = CRC16(frame, idx);
//	frame[idx++] = crc & 0xFF;
//	frame[idx++] = (crc >> 8) & 0xFF;
    frame[idx++] = 0xad;
    frame[idx++] = 0x00;

    // ����
    for (uint8_t i = 0; i < idx; i++)
    {
        while (DL_UART_isBusy(MSPMotor_INST));
        DL_UART_Main_transmitData(MSPMotor_INST, frame[i]);
    }
}




void Gyro_RequestData(void)
{
    uint8_t idx = 0;
    uint8_t frame[8];

    frame[idx++] = 0x0A;        // 从站地址
    frame[idx++] = 0x03;        // 功能码：读保持寄存器
    frame[idx++] = 0x00;        // 起始地址高字节
    frame[idx++] = 0x00;        // 起始地址低字节
    frame[idx++] = 0x00;        // 寄存器数量高字节
    frame[idx++] = 0x02;        // 寄存器数量低字节（读2个寄存器：角度+角速度）

    uint16_t crc = CRC16(frame, idx);
    frame[idx++] = crc & 0xFF;       // CRC_L
    frame[idx++] = (crc >> 8) & 0xFF; // CRC_H

    for (uint8_t i = 0; i < idx; i++)
    {
        while (DL_UART_isBusy(MSPMotor_INST));
        DL_UART_Main_transmitData(MSPMotor_INST, frame[i]);
    }
}
