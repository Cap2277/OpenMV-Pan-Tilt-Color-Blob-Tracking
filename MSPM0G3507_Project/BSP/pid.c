#include "board.h"
#include "pid.h"
#include "stdlib.h"
#include <math.h>
#define MAX_DUTY 70

#define PID_DEAD_ZONE 30
#define MAX_I_OUT 100  // �������޷���ֹ���ֹ���
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
	// 1.�趨Ŀ���ٶ�
	//motor_target_set(20, 20);
	// 2.��ȡ��ǰ�ٶ�
//	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
//	if(motorB_dir){motorB.now =  speed2;}else{motorB.now = - speed2;}
//	Encoder_count1 = 0;
//	Encoder_count2 = 0;
//	// 3.����PID���������м���
//	pid_cal(&motorA);
//	pid_cal(&motorB);
//	// 4.PID�����ֵ ��������
//	set_left_PWM(motorA.out);
//	set_right_PWM(motorB.out);
}


void pid_cal(pid_t *pid)
{
    // ���㵱ǰ���
    pid->error[0] = pid->target - pid->now;

    // === �������� ===
    if (abs(pid->error[0]) < PID_DEAD_ZONE) {
        pid->out = 0;
        pid->iout = 0;  // ������ַ�ֹ����ƫ��
        pid->error[2] = pid->error[1];
        pid->error[1] = pid->error[0];
        return;
    }

    // ========== ����ʽ PID ==========
    if (pid->pid_mode == DELTA_PID)
    {
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];  // ����ֻ�Ǳ��ֻ�����
        pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);

        float delta = pid->pout + pid->iout + pid->dout;
        pid->out += delta;  // �������ֱ�ӵ���
    }

    // ========== λ��ʽ PID ==========
    else if (pid->pid_mode == POSITION_PID)
    {
        pid->pout = pid->p * pid->error[0];

        // == �����ֱ��ͣ�ֻ�������δ����ʱ���� ==
        if (fabs(pid->out) < MAX_DUTY) {
            pid->iout += pid->i * pid->error[0];

            // �����޷�
            if (pid->iout > MAX_I_OUT) pid->iout = MAX_I_OUT;
            else if (pid->iout < -MAX_I_OUT) pid->iout = -MAX_I_OUT;
        }

        pid->dout = pid->d * (pid->error[0] - pid->error[1]);

        pid->out = pid->pout + pid->iout + pid->dout;
    }

    // === ��¼��ʷ��� ===
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    // === ����޷� ===
    if (pid->out > MAX_DUTY) pid->out = MAX_DUTY;
    else if (pid->out < -MAX_DUTY) pid->out = -MAX_DUTY;
}
