/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include <stdio.h>
#include "bsp_sg90.h"
#include <stdio.h>
#include "oled.h"
#include "pid.h"


void uart1_send_char(char ch);
void uart1_send_string(char* str);
int servo_angle_x,servo_angle_y;
char message[20];
char uart_send[100];
int i=0;
static float servo_x = 90, servo_y = 90;
int main(void)
{
    //开发板初始化
    board_init();
    OLED_Init();     //初始化OLED
    OLED_Clear();
  pid_init(&motorA, POSITION_PID,0.095,0.0002,0.06); //0.095,0.0005,0.005
  pid_init(&motorB, POSITION_PID,0.095,0.0002,0.06); //0.095,0.0002,0.06
    DL_GPIO_setPins(LED1_PORT,LED1_PIN_22_PIN);
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	
	
	  //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
      //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
		
		
		
    Set_Servo_Angle_x(90);
   delay_ms(1000);
    Set_Servo_Angle_y(90);
   delay_ms(1000);


    while(1) 
    {
	  motorA.target = 0;
	 	motorB.target = 0;
//      sprintf(message,"x:%d    ",target_x);
//			OLED_ShowString(0,0,(uint8_t *)message,16,1);//6*12 “ABC”
//			sprintf(message,"y:%d    ",target_y);
//			OLED_ShowString(0,15,(uint8_t *)message,16,1);//6*12 “ABC”
//      sprintf(message,"x_out:%d    ",(int)motorA.out);
//			OLED_ShowString(0,30,(uint8_t *)message,16,1);//6*12 “ABC”
//			sprintf(message,"y_out:%d    ",(int)motorB.out);
//			OLED_ShowString(0,45,(uint8_t *)message,16,1);//6*12 “ABC”		 
//		  sprintf(message,"count:%d    ",num);
//			OLED_ShowString(0,0,(uint8_t *)message,16,1);//6*12 “ABC”		 
			 sprintf(uart_send,"pid:%d,%d,%d,%d \n",(int)motorA.target,(int)motorA.now,(int)motorA.out,(int)servo_x);
	   	 uart1_send_string(uart_send);
//		 
//         delay_ms(10);
//       OLED_Refresh();
    }
}








//定时器的中断服务函数 已配置为50ms的周期
void TIMER_0_INST_IRQHandler(void)
{
    //如果产生了定时器中断
    switch( DL_TimerG_getPendingInterrupt(TIMER_0_INST) )
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断


    pid_cal(&motorA); 
	  pid_cal(&motorB); 	
	  servo_y += motorB.out;
	  servo_x += motorA.out;
    if (servo_x > 180) servo_x = 180;
    if (servo_x < 0)   servo_x = 0;
    if (servo_y > 180) servo_y = 180;
    if (servo_y < 0)   servo_y = 0;
	  Set_Servo_Angle_x(servo_x);
		 Set_Servo_Angle_y(servo_y);
            break;

        default://其他的定时器中断
            break;
    }
}
void uart1_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_1_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_1_INST, ch);
}
//串口发送字符串
void uart1_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart1_send_char(*str++);
    }
}





