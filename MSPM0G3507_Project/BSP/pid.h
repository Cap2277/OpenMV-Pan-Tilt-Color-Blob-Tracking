#ifndef __PID_h_
#define __PID_h_
#include "board.h"

enum
{
  POSITION_PID = 0,  // ¦Ë?
  DELTA_PID,         // ?
};

typedef struct
{
	float target;	
	float now;
	float error[3];	
  float error_int;	
	float p,i,d;
	float pout, dout, iout;
	float out;   
	
	uint32_t pid_mode;

}pid_t;

void pid_cal(pid_t *pid);
void pid_control(void);
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void motor_target_set(int spe1, int spe2);

extern pid_t motorA;
extern pid_t motorB;

#endif