/*
 * PID.h
 *
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	//Controller gains
	float Kp;
	float Ki;

	//Output limits
	float limMin;
	float limMax;

	//Sample time (seconds)
	float T;

	//Controller variables
	int integrator;
	float prevError;
	float prevMeasurement;

	//Controller output
	int out;
} PIDController;

void PIDController_Init(PIDController *pid);
void PIDController_Update(PIDController *pid, float measurement, float setpoint, int currentPWM);

#endif /* INC_PID_H_ */
