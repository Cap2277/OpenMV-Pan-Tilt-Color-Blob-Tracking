#include "board.h"
#include "pid.h"
#include "stdlib.h"
#include <math.h>
#define MAX_DUTY 70

#define PID_DEAD_ZONE 30
#define MAX_I_OUT 100  // 积分项限幅防止积分过大
pid_t motorA;
pid_t motorB;
uint8_t motorA_dir, motorB_dir;


void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
}



void motor_target_set(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}
	
	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}
}

	

void pid_control()
{
	// 1.设定目标速度
	//motor_target_set(20, 20);
	// 2.获取当前速度
//	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
//	if(motorB_dir){motorB.now =  speed2;}else{motorB.now = - speed2;}
//	Encoder_count1 = 0;
//	Encoder_count2 = 0;
//	// 3.输入PID控制器进行计算
//	pid_cal(&motorA);
//	pid_cal(&motorB);
//	// 4.PID的输出值 输入给电机
//	set_left_PWM(motorA.out);
//	set_right_PWM(motorB.out);
}


void pid_cal(pid_t *pid)
{
    // 计算当前误差
    pid->error[0] = pid->target - pid->now;

    // === 死区处理 ===
    if (abs(pid->error[0]) < PID_DEAD_ZONE) {
        pid->out = 0;
        pid->iout = 0;  // 清除积分防止积累偏移
        pid->error[2] = pid->error[1];
        pid->error[1] = pid->error[0];
        return;
    }

    // ========== 增量式 PID ==========
    if (pid->pid_mode == DELTA_PID)
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];  // 这里只是本轮积分项
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);

        float delta = pid->pout + pid->iout + pid->dout;
        pid->out += delta;  // 增量输出直接叠加
    }

    // ========== 位置式 PID ==========
    else if (pid->pid_mode == POSITION_PID)
    {
        pid->pout = pid->p * pid->error[0];

        // == 抗积分饱和：只有在输出未饱和时积分 ==
        if (fabs(pid->out) < MAX_DUTY) {
            pid->iout += pid->i * pid->error[0];

            // 积分限幅
            if (pid->iout > MAX_I_OUT) pid->iout = MAX_I_OUT;
            else if (pid->iout < -MAX_I_OUT) pid->iout = -MAX_I_OUT;
        }

        pid->dout = pid->d * (pid->error[0] - pid->error[1]);

        pid->out = pid->pout + pid->iout + pid->dout;
    }

    // === 记录历史误差 ===
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    // === 输出限幅 ===
    if (pid->out > MAX_DUTY) pid->out = MAX_DUTY;
    else if (pid->out < -MAX_DUTY) pid->out = -MAX_DUTY;
}
