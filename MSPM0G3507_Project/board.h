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
#ifndef	__BOARD_H__
#define __BOARD_H__
#define FRAME_LEN 5

#include "ti_msp_dl_config.h"
#include "pid.h"
extern  uint8_t  recv0_buff[FRAME_LEN]; // ����������
extern  uint8_t  frame_ok ;          // 1 = �յ�������Ч֡
extern  int16_t target_x ;          // ������������
extern  int16_t target_y ;



void board_init(void);

void delay_us(unsigned long __us);
void delay_ms(unsigned long ms);
void delay_1us(unsigned long __us);
void delay_1ms(unsigned long ms);

void uart0_send_char(char ch);
void uart0_send_string(char* str);



#endif
