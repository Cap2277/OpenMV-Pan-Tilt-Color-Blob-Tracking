/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-08     LCKFB-LP    first version
 */

#ifndef _BSP_SG90_H
#define _BSP_SG90_H


#include "board.h"
void Set_Servo_Angle_x(unsigned int angle);
void Set_Servo_Angle_y(unsigned int angle);
unsigned int Get_Servo_Angle(void);

#endif  

