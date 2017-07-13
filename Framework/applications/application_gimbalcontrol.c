#include "application_gimbalcontrol.h"

#include "stdint.h"
#include "utilities_minmax.h"
#include "drivers_canmotor_user.h"
#include "drivers_imu_user.h"
#include "application_pidfunc.h"
#include "application_setmotor.h"
#include "utilities_debug.h"
#include "task_quaternion.h"
//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
PID_Regulator_t yawPositionPID = PID_INIT(3.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
PID_Regulator_t yawSpeedPID = PID_INIT(2.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
//云台偏置

int16_t YawZeroEncoderBias=-3450;
int16_t PitchZeroEncoderBias=133;  //需要用串口再调

void setYawWithAngle(float targetAngle){
	if(IOPool_hasNextRead(GMYAWRxIOPool, 0)){
		//TargetAngle
		MINMAX(targetAngle, YAWDOWNLIMIT, YAWUPLIMIT);
		//RealAngle
		IOPool_getNextRead(GMYAWRxIOPool, 0); 
		float realAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - YawZeroEncoderBias) * 360 / 8192.0;
		NORMALIZE_ANGLE180(realAngle);
		//RealSpeed
		IOPool_getNextRead(IMUIOPool, 0);
		//float realSpeed = -IOPool_pGetReadData(IMUIOPool, 0)->gYroZs;
		float realSpeed = gYroZ;
		
				static int countwhile1 = 0;
		if(countwhile1 >= 300){
			countwhile1 = 0;
        fw_printf("realAngle = %f \r\n", realAngle);
				fw_printf("gYroZ = %f \r\n", realSpeed);
		}else{
			countwhile1++;
		}		
	
		setMotorWithPositionSpeedPID(GMYAW, &yawPositionPID, &yawSpeedPID, targetAngle, realAngle, realSpeed);
		
	}
}

PID_Regulator_t pitchPositionPID = PID_INIT(10.0, 0.01, 3.0, 10000.0, 10000.0, 10000.0, 10000.0);//20 0 12
PID_Regulator_t pitchSpeedPID = PID_INIT(23.0, 0.0, 5.0, 10000.0, 10000.0, 10000.0, 3000.0);//40 0 20


void setPitchWithAngle(float targetAngle){
	if(IOPool_hasNextRead(GMPITCHRxIOPool, 0)){
		//TargetAngle
		MINMAX(targetAngle, PITCHDOWNLIMIT, PITCHUPLIMIT);
		//RealAngle
		IOPool_getNextRead(GMPITCHRxIOPool, 0); 
		float realAngle = -(IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - PitchZeroEncoderBias) * 360 / 8192.0;
		NORMALIZE_ANGLE180(realAngle);
		//RealSpeed
		IOPool_getNextRead(IMUIOPool, 0);
		float realSpeed = gYroX;
		
		static int countwhile2 = 0;
		if(countwhile2 >= 300){
			countwhile2 = 0;
        fw_printf("realAngle = %f \r\n", realAngle);
				fw_printf("gYrox = %f \r\n", realSpeed);
		}else{
			countwhile2++;
		}		
		
		setMotorWithPositionSpeedPID(GMPITCH, &pitchPositionPID, &pitchSpeedPID, targetAngle, realAngle, realSpeed);
	}
}
