/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-26     LCKFB     first version
 */
#include "board.h"
#include "stdio.h"

#define RE_0_BUFF_LEN_MAX	128






void board_init(void)
{
	// SYSCFG��ʼ��
	SYSCFG_DL_init();
	//��������жϱ�־
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//ʹ�ܴ����ж�
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	
	printf("Board Init [[ ** LCKFB ** ]]\r\n");
}

//����δ�ʱ��ʵ�ֵľ�ȷus��ʱ
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // ������Ҫ��ʱ���� = �ӳ�΢���� * ÿ΢���ʱ����
    ticks = __us * (32000000 / 1000000);

    // ��ȡ��ǰ��SysTickֵ
    told = SysTick->VAL;

    while (1)
    {
        // �ظ�ˢ�»�ȡ��ǰ��SysTickֵ
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // ����ﵽ����Ҫ��ʱ���������˳�ѭ��
            if (tcnt >= ticks)
                break;
        }
    }
}
//����δ�ʱ��ʵ�ֵľ�ȷms��ʱ
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

//���ڷ��͵����ַ�
void uart0_send_char(char ch)
{
	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while( DL_UART_isBusy(UART_0_INST) == true );
	//���͵����ַ�
	DL_UART_Main_transmitData(UART_0_INST, ch);

}
//���ڷ����ַ���
void uart0_send_string(char* str)
{
	//��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
	while(*str!=0&&str!=0)
	{
		//�����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
		uart0_send_char(*str++);
	}
}


#if !defined(__MICROLIB)
//��ʹ��΢��Ļ�����Ҫ�������ĺ���
#if (__ARMCLIB_VERSION <= 6000000)
//�����������AC5  �Ͷ�����������ṹ��
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
#endif


//printf�����ض���
int fputc(int ch, FILE *stream)
{
	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while( DL_UART_isBusy(UART_0_INST) == true );
	
	DL_UART_Main_transmitData(UART_0_INST, ch);
	
	return ch;
}

static uint8_t  rxBuf[7];   // 7 �ֽڻ���
static uint8_t  idx   = 0;  // �����ֽڼ���
 uint8_t  frame_ok = 0;
 int16_t  target_x = 0;
 int16_t  target_y = 0;

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);

            /* ˳����� 7 �ֽ� */
            if (idx < 7)
                rxBuf[idx++] = byte;

            /* �յ��� 7 �ֽں������� */
            if (idx == 7)
            {
                /* У��֡ͷ��֡β */
                if (rxBuf[0] == 0xAA &&
                    rxBuf[1] == 0xFF &&
                    rxBuf[6] == 0xEE)
                {
                    /* С����ƴ�� X��Y */
                    target_x = (int16_t)(rxBuf[3] << 8 | rxBuf[2]);
                    target_y = (int16_t)(rxBuf[5] << 8 | rxBuf[4]);
                    frame_ok = 1;      /* ֪ͨ��ѭ�� */
              motorA.now = target_x;
  	          motorB.now = target_y;   
                }

                idx = 0;               /* ��λ��׼����һ֡ */
            }
            break;
        }

        default:
            break;
    }
}