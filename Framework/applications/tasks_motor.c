#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "utilities_debug.h"
#include "tasks_upper.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "drivers_uartrc_user.h"
#include "drivers_sonar_user.h"
#include "drivers_imu_user.h"
#include "application_pidfunc.h"
#include "application_setmotor.h"
#include "application_gimbalcontrol.h"
#include "application_chassiscontrol.h"
#include "application_auxmotorcontrol.h"
#include "application_remotecontrol.h"
#include "stdint.h"

uint32_t yawpidcnt = 0;
PID_Regulator_t testPositionPID = PID_INIT(6.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t testSpeedPID = PID_INIT(0.7, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t Pitch3PositionPID = PID_INIT(6.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t Pitch3SpeedPID = PID_INIT(2.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t BulletPositionPID = PID_INIT(1.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t BulletSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t Lift1PositionPID = PID_INIT(1.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t Lift1SpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t Lift2PositionPID = PID_INIT(1.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t Lift2SpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t Lift3PositionPID = PID_INIT(1.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t Lift3SpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t Lift4PositionPID = PID_INIT(1.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t Lift4SpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

uint8_t landing_flag = 0;
uint8_t liftcali_flag = 0;

extern float ZGyroModuleAngle;
extern float angles[3];
// 置1的时候云台才开始控制，置0不控制
uint8_t GM_RUN=0;
// 英雄的升降
int8_t flUpDown = 0, frUpDown = 0, blUpDown = 0, brUpDown = 0, allUpDown = 0;
//单位是度
float yawAngleTarget = 0.0;
float pitchAngleTarget = 0.0;

float BulletAngleTarget = 0.0;
float Lift1AngleTarget = 0.0;
float Lift2AngleTarget = 0.0;
float Lift3AngleTarget = 0.0;
float Lift4AngleTarget = 0.0;
//底盘速度结构体，度每秒
ChassisSpeed_Ref_t ChassisSpeedRef;

float forward_target = 0.0;
float left_target = 0.0;
float rotate_target = 0.0;
float yaw_angle_set = 0.0;
float yaw_angle_set_last = 0.0;

void CMGMControlTask(void const * argument){
	while(1){
		osSemaphoreWait(CMGMCanRefreshSemaphoreHandle, osWaitForever);
	 if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE || GetEmergencyFlag() == EMERGENCY)   //||Is_Serious_Error()|| dead_lock_flag == 1紧急停车，编码器校准，无控制输入时都会使底盘控制停止
	 {
//		 yawAngleTarget=0;
//		 pitchAngleTarget=0;
		 forward_target=0;
		 left_target=0;
		 rotate_target=0;
	 }
    
	 if(ChassisSpeedRef.forward_back_ref < forward_target)
	 {
		 if(ChassisSpeedRef.forward_back_ref < forward_target-100) ChassisSpeedRef.forward_back_ref += 100;
		 else ChassisSpeedRef.forward_back_ref = forward_target;
	 }
	 
	 if(ChassisSpeedRef.forward_back_ref > forward_target)
	 {
		 if(ChassisSpeedRef.forward_back_ref > forward_target + 100) ChassisSpeedRef.forward_back_ref -= 100;
		 else ChassisSpeedRef.forward_back_ref = forward_target;
	 }
	 
	 if(ChassisSpeedRef.left_right_ref < left_target)
	 {
		 if(ChassisSpeedRef.left_right_ref < left_target-100) ChassisSpeedRef.left_right_ref += 100;
		 else ChassisSpeedRef.left_right_ref = left_target;
	 }
	 
	 if(ChassisSpeedRef.left_right_ref > left_target)
	 {
		 if(ChassisSpeedRef.left_right_ref> left_target + 100) ChassisSpeedRef.left_right_ref -= 100;
		 else ChassisSpeedRef.left_right_ref = left_target;
	 }
	 
	 if(ChassisSpeedRef.rotate_ref < rotate_target)
	 {
		 if(ChassisSpeedRef.rotate_ref < rotate_target-100) ChassisSpeedRef.rotate_ref += 100;
		 else ChassisSpeedRef.rotate_ref = rotate_target;
	 }
	 
	 if(ChassisSpeedRef.rotate_ref > rotate_target)
	 {
		 if(ChassisSpeedRef.rotate_ref > rotate_target + 100) ChassisSpeedRef.rotate_ref -= 100;
		 else ChassisSpeedRef.rotate_ref = rotate_target;
	 }
	 
		setChassisWithSpeed(ChassisSpeedRef.forward_back_ref, ChassisSpeedRef.left_right_ref, ChassisSpeedRef.rotate_ref);

	 if (yawpidcnt>0 ) yawpidcnt--;
	 if(yawpidcnt==0) 
	 {
			yawPositionPID.kp =5 ;
			yawSpeedPID.kp=40;
	 }
//	  if(yawAngleTarget < yaw_angle_set)
//    {
//			if(yawAngleTarget < yaw_angle_set-1) yawAngleTarget += 1;
//			else yawAngleTarget = yaw_angle_set;
//		}			
//	 
//		if(yawAngleTarget > yaw_angle_set)
//    {
//			if(yawAngleTarget > yaw_angle_set+1) yawAngleTarget -= 1;
//			else yawAngleTarget = yaw_angle_set;
//		}			
		yawAngleTarget = yaw_angle_set;
    setYawWithAngle(yawAngleTarget);
	 
	 //if(pitchAngleTarget>PITCHLIMIT)  pitchAngleTarget = PITCHLIMIT;     //3510云台电机的限制
	 //if(pitchAngleTarget<0)  pitchAngleTarget = 0;
	 
	  //SetPitchWithAngle3510(pitchAngleTarget);
	  
	  //setPitchWithAngle(pitchAngleTarget);    //6623电机程序
	  setPitchWithAngle(pitchAngleTarget);    //6623电机程序
	}
}
float bulletSpeed = 0;
float aux1_targetSpeed=0;
float aux2_targetSpeed=0;
float aux3_targetSpeed=0;
float aux4_targetSpeed=0;
void AMControlTask(void const * argument){
	while(1){
		osSemaphoreWait(AMCanRefreshSemaphoreHandle, osWaitForever);
		if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE || GetEmergencyFlag() == EMERGENCY)   //||Is_Serious_Error()|| dead_lock_flag == 1紧急停车，编码器校准，无控制输入时都会使底盘控制停止
		 {
				
			 
		 }
		 
		 if(ChassisSpeedRef.forward_back_ref < forward_target)
		{
				if(ChassisSpeedRef.forward_back_ref < forward_target-100) ChassisSpeedRef.forward_back_ref += 100;
				else ChassisSpeedRef.forward_back_ref = forward_target;
		}
	 
		if(ChassisSpeedRef.forward_back_ref > forward_target)
		{
				if(ChassisSpeedRef.forward_back_ref > forward_target + 100) ChassisSpeedRef.forward_back_ref -= 100;
				else ChassisSpeedRef.forward_back_ref = forward_target;
		}
	 
		if(ChassisSpeedRef.left_right_ref < left_target)
		{
				if(ChassisSpeedRef.left_right_ref < left_target-100) ChassisSpeedRef.left_right_ref += 100;
				else ChassisSpeedRef.left_right_ref = left_target;
		}
	 
		if(ChassisSpeedRef.left_right_ref > left_target)
		{
				if(ChassisSpeedRef.left_right_ref> left_target + 100) ChassisSpeedRef.left_right_ref -= 100;
				else ChassisSpeedRef.left_right_ref = left_target;
		}
	 
		if(ChassisSpeedRef.rotate_ref < rotate_target)
		{
				if(ChassisSpeedRef.rotate_ref < rotate_target-100) ChassisSpeedRef.rotate_ref += 100;
				else ChassisSpeedRef.rotate_ref = rotate_target;
		}
	 
		if(ChassisSpeedRef.rotate_ref > rotate_target)
		{
				if(ChassisSpeedRef.rotate_ref > rotate_target + 100) ChassisSpeedRef.rotate_ref -= 100;
				else ChassisSpeedRef.rotate_ref = rotate_target;
		}
		 
		 float aux3Speed =ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.rotate_ref;
	   float aux4Speed =ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.rotate_ref;
		 
		 setAux3WithSpeed(aux3Speed);
	   setAux4WithSpeed(aux4Speed);
		 
		 if (Lift1AngleTarget > LIFTUPLIMIT)  Lift1AngleTarget = LIFTUPLIMIT;
		 else if (Lift1AngleTarget < LIFTDOWNLIMIT)  Lift1AngleTarget = LIFTDOWNLIMIT;
		 if (Lift2AngleTarget > LIFTUPLIMIT)  Lift2AngleTarget = LIFTUPLIMIT;
			else if (Lift2AngleTarget < LIFTDOWNLIMIT)  Lift2AngleTarget = LIFTDOWNLIMIT;
		 
		 if(landing_flag)
			 
		 {
			 SetLift1Angle(Lift1AngleTarget);
			 SetLift2Angle(Lift2AngleTarget);
		 }
		 
			 if (BulletAngleTarget < BULLETDOWNLIMIT) BulletAngleTarget = BULLETDOWNLIMIT ;
		   else if (BulletAngleTarget > BULLETUPLIMIT) BulletAngleTarget = BULLETUPLIMIT ;
		   SetBulletAngle(BulletAngleTarget);
		 
		 if (liftcali_flag)
		 {
				setAux1WithSpeed(aux1_targetSpeed);
				setAux2WithSpeed(aux2_targetSpeed);
		 }
		 
		 SetFrictionWheelSpeed();
	   ShooterLoop();
	}
}

float testRealAngle = 0.0;
float testRealAngleLast = 0.0;
float testAngleTarget = 0;
void ShooterLoop()
{
	    static uint8_t rotate90 = 1; 
			if(IOPool_hasNextRead(AMPLATERxIOPool, 0)){
			IOPool_getNextRead(AMPLATERxIOPool, 0);
			static float testRealAngle = 0.0;
			static float testRealAngleLast = 0.0;
			
			static float testAngleTarget = 0;

				if(GetShootState() == SHOOTING)
			  {
					if (rotate90 == 1) 
					{
//						if(testAngleTarget > 0)
//						{
//							testAngleTarget = 0.0;
//						}else{
//							testAngleTarget = 90.0*96;
//					  }
						if (shootemer == 0)
						{
							if(shoot_dir == 1) testAngleTarget += 90.0*96;
							else  testAngleTarget -= 90.0*96;
						}
						rotate90 = 0;
			    }
				}
				else
				{
						rotate90 = 1;
				}
					
			
			float testRealAngleCurr = (IOPool_pGetReadData(AMPLATERxIOPool, 0)->angle) * 360 / 8192.0;
			float testRotateSpeed = (IOPool_pGetReadData(AMPLATERxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(testRealAngleCurr - testRealAngleLast > 180){
				testRealAngle += testRealAngleCurr - 360 - testRealAngleLast;
			}else if(testRealAngleCurr - testRealAngleLast < -180){
				testRealAngle += testRealAngleCurr + 360 - testRealAngleLast;
			}else{
				testRealAngle += testRealAngleCurr - testRealAngleLast;
			}
			testRealAngleLast = testRealAngleCurr;
			
			//position		
			testPositionPID.target = testAngleTarget;
			testPositionPID.feedback = testRealAngle;
			testPositionPID.Calc(&testPositionPID);
			//speed
			testSpeedPID.target = testPositionPID.output;
			testSpeedPID.feedback = testRotateSpeed;
			testSpeedPID.Calc(&testSpeedPID);
			int16_t testIntensity = (int16_t)testSpeedPID.output;
			
			if(shootemer) testIntensity=0;
			
			setMotor(AM2PLATE, testIntensity);
   }
}

void SetPitchWithAngle3510(float Pitch3AngleTargetSet)
{
			if(IOPool_hasNextRead(GMPITCHRxIOPool, 0)){
			IOPool_getNextRead(GMPITCHRxIOPool, 0);
			static float Pitch3RealAngle = 0.0;
			static float Pitch3RealAngleLast = 0.0;
			
			static float Pitch3AngleTarget = 0;
				
			Pitch3AngleTarget = Pitch3AngleTargetSet;
	
			float Pitch3RealAngleCurr = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle) * 360 / 8192.0;
			float Pitch3RotateSpeed = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(Pitch3RealAngleCurr - Pitch3RealAngleLast > 180){
				Pitch3RealAngle += Pitch3RealAngleCurr - 360 - Pitch3RealAngleLast;
			}else if(Pitch3RealAngleCurr - Pitch3RealAngleLast < -180){
				Pitch3RealAngle += Pitch3RealAngleCurr + 360 - Pitch3RealAngleLast;
			}else{
				Pitch3RealAngle += Pitch3RealAngleCurr - Pitch3RealAngleLast;
			}
			Pitch3RealAngleLast = Pitch3RealAngleCurr;
			
			//position		
			Pitch3PositionPID.target = Pitch3AngleTarget;
			Pitch3PositionPID.feedback = Pitch3RealAngle;
			Pitch3PositionPID.Calc(&Pitch3PositionPID);
			//speed
			Pitch3SpeedPID.target = Pitch3PositionPID.output;
			Pitch3SpeedPID.feedback = Pitch3RotateSpeed;
			Pitch3SpeedPID.Calc(&Pitch3SpeedPID);
			int16_t Pitch3Intensity = (int16_t)Pitch3SpeedPID.output;
			
			setMotor(GMPITCH, Pitch3Intensity);
   }
}

void SetBulletAngle(float BulletAngleTargetSet)
{
			if(IOPool_hasNextRead(AMGETBULLETRxIOPool, 0)){
			IOPool_getNextRead(AMGETBULLETRxIOPool, 0);
			static float BulletRealAngle = 0.0;
			static float BulletRealAngleLast = 0.0;
			
			static float BulletAngleTarget = 0;
				
			BulletAngleTarget = BulletAngleTargetSet;
	
			float BulletRealAngleCurr = (IOPool_pGetReadData(AMGETBULLETRxIOPool, 0)->angle) * 360 / 8192.0;
			float BulletRotateSpeed = (IOPool_pGetReadData(AMGETBULLETRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(BulletRealAngleCurr - BulletRealAngleLast > 180){
				BulletRealAngle += BulletRealAngleCurr - 360 - BulletRealAngleLast;
			}else if(BulletRealAngleCurr - BulletRealAngleLast < -180){
				BulletRealAngle += BulletRealAngleCurr + 360 - BulletRealAngleLast;
			}else{
				BulletRealAngle += BulletRealAngleCurr - BulletRealAngleLast;
			}
			BulletRealAngleLast = BulletRealAngleCurr;
			
			//position		
			BulletPositionPID.target = BulletAngleTarget;
			BulletPositionPID.feedback = BulletRealAngle;
			BulletPositionPID.Calc(&BulletPositionPID);
			//speed
			BulletSpeedPID.target = BulletPositionPID.output;
			BulletSpeedPID.feedback = BulletRotateSpeed;
			BulletSpeedPID.Calc(&BulletSpeedPID);
			int16_t BulletIntensity = (int16_t)BulletSpeedPID.output;
			
			if (BulletIntensity<500 && BulletIntensity > -500) BulletIntensity = 0;
			
			setMotor(AM2GETBULLET, BulletIntensity);
   }
}

void SetLift1Angle(float Lift1AngleTargetSet)
{
			if(IOPool_hasNextRead(AMUDFLRxIOPool, 0)){
			IOPool_getNextRead(AMUDFLRxIOPool, 0);
			static float Lift1RealAngle = 0.0;
			static float Lift1RealAngleLast = 0.0;
			
			static float Lift1AngleTarget = 0;
				
			Lift1AngleTarget = Lift1AngleTargetSet;
	
			float Lift1RealAngleCurr = (IOPool_pGetReadData(AMUDFLRxIOPool, 0)->angle) * 360 / 8192.0;
			float Lift1RotateSpeed = (IOPool_pGetReadData(AMUDFLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(Lift1RealAngleCurr - Lift1RealAngleLast > 180){
				Lift1RealAngle += Lift1RealAngleCurr - 360 - Lift1RealAngleLast;
			}else if(Lift1RealAngleCurr - Lift1RealAngleLast < -180){
				Lift1RealAngle += Lift1RealAngleCurr + 360 - Lift1RealAngleLast;
			}else{
				Lift1RealAngle += Lift1RealAngleCurr - Lift1RealAngleLast;
			}
			Lift1RealAngleLast = Lift1RealAngleCurr;
			
			//position		
			Lift1PositionPID.target = Lift1AngleTarget;
			Lift1PositionPID.feedback = Lift1RealAngle;
			Lift1PositionPID.Calc(&Lift1PositionPID);
			//speed
			Lift1SpeedPID.target =Lift1PositionPID.output;
			Lift1SpeedPID.feedback = Lift1RotateSpeed;
			Lift1SpeedPID.Calc(&Lift1SpeedPID);
			int16_t Lift1Intensity = (int16_t)Lift1SpeedPID.output;
			
			if (Lift1Intensity<500 && Lift1Intensity > -500) Lift1Intensity = 0;
			
			setMotor(AM1UDFL, Lift1Intensity);
   }
}

void SetLift2Angle(float Lift2AngleTargetSet)
{
			if(IOPool_hasNextRead(AMUDFRRxIOPool, 0)){
			IOPool_getNextRead(AMUDFRRxIOPool, 0);
			static float Lift2RealAngle = 0.0;
			static float Lift2RealAngleLast = 0.0;
			
			static float Lift2AngleTarget = 0;
				
			Lift2AngleTarget = Lift2AngleTargetSet;
	
			float Lift2RealAngleCurr = (IOPool_pGetReadData(AMUDFRRxIOPool, 0)->angle) * 360 / 8192.0;
			float Lift2RotateSpeed = (IOPool_pGetReadData(AMUDFRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(Lift2RealAngleCurr - Lift2RealAngleLast > 180){
				Lift2RealAngle += Lift2RealAngleCurr - 360 - Lift2RealAngleLast;
			}else if(Lift2RealAngleCurr - Lift2RealAngleLast < -180){
				Lift2RealAngle += Lift2RealAngleCurr + 360 - Lift2RealAngleLast;
			}else{
				Lift2RealAngle += Lift2RealAngleCurr - Lift2RealAngleLast;
			}
			Lift2RealAngleLast = Lift2RealAngleCurr;
			
			//position		
			Lift2PositionPID.target = Lift2AngleTarget;
			Lift2PositionPID.feedback = Lift2RealAngle;
			Lift2PositionPID.Calc(&Lift2PositionPID);
			//speed
			Lift2SpeedPID.target =Lift2PositionPID.output;
			Lift2SpeedPID.feedback = Lift2RotateSpeed;
			Lift2SpeedPID.Calc(&Lift2SpeedPID);
			int16_t Lift2Intensity = (int16_t)Lift2SpeedPID.output;
			
			if (Lift2Intensity<500 && Lift2Intensity > -500) Lift2Intensity = 0;
			
			setMotor(AM1UDFR, Lift2Intensity);
   }
}

void SetLift3Angle(float Lift3AngleTargetSet)
{
			if(IOPool_hasNextRead(AMUDBLRxIOPool, 0)){
			IOPool_getNextRead(AMUDBLRxIOPool, 0);
			static float Lift3RealAngle = 0.0;
			static float Lift3RealAngleLast = 0.0;
			
			static float Lift3AngleTarget = 0;
				
			Lift3AngleTarget = Lift3AngleTargetSet;
	
			float Lift3RealAngleCurr = (IOPool_pGetReadData(AMUDBLRxIOPool, 0)->angle) * 360 / 8192.0;
			float Lift3RotateSpeed = (IOPool_pGetReadData(AMUDBLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(Lift3RealAngleCurr - Lift3RealAngleLast > 180){
				Lift3RealAngle += Lift3RealAngleCurr - 360 - Lift3RealAngleLast;
			}else if(Lift3RealAngleCurr - Lift3RealAngleLast < -180){
				Lift3RealAngle += Lift3RealAngleCurr + 360 - Lift3RealAngleLast;
			}else{
				Lift3RealAngle += Lift3RealAngleCurr - Lift3RealAngleLast;
			}
			Lift3RealAngleLast = Lift3RealAngleCurr;
			
			//position		
			Lift3PositionPID.target = Lift3AngleTarget;
			Lift3PositionPID.feedback = Lift3RealAngle;
			Lift3PositionPID.Calc(&Lift3PositionPID);
			//speed
			Lift3SpeedPID.target =Lift3PositionPID.output;
			Lift3SpeedPID.feedback = Lift3RotateSpeed;
			Lift3SpeedPID.Calc(&Lift3SpeedPID);
			int16_t Lift3Intensity = (int16_t)Lift3SpeedPID.output;
			
			if (Lift3Intensity<500 && Lift3Intensity > -500) Lift3Intensity = 0;
			
			setMotor(AM1UDBL, Lift3Intensity);
   }
}

void SetLift4Angle(float Lift4AngleTargetSet)
{
			if(IOPool_hasNextRead(AMUDBRRxIOPool, 0)){
			IOPool_getNextRead(AMUDBRRxIOPool, 0);
			static float Lift4RealAngle = 0.0;
			static float Lift4RealAngleLast = 0.0;
			
			static float Lift4AngleTarget = 0;
				
			Lift4AngleTarget = Lift4AngleTargetSet;
	
			float Lift4RealAngleCurr = (IOPool_pGetReadData(AMUDBRRxIOPool, 0)->angle) * 360 / 8192.0;
			float Lift4RotateSpeed = (IOPool_pGetReadData(AMUDBRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(Lift4RealAngleCurr - Lift4RealAngleLast > 180){
				Lift4RealAngle += Lift4RealAngleCurr - 360 - Lift4RealAngleLast;
			}else if(Lift4RealAngleCurr - Lift4RealAngleLast < -180){
				Lift4RealAngle += Lift4RealAngleCurr + 360 - Lift4RealAngleLast;
			}else{
				Lift4RealAngle += Lift4RealAngleCurr - Lift4RealAngleLast;
			}
			Lift4RealAngleLast = Lift4RealAngleCurr;
			
			//position		
			Lift4PositionPID.target = Lift4AngleTarget;
			Lift4PositionPID.feedback = Lift4RealAngle;
			Lift4PositionPID.Calc(&Lift4PositionPID);
			//speed
			Lift4SpeedPID.target =Lift4PositionPID.output;
			Lift4SpeedPID.feedback = Lift4RotateSpeed;
			Lift4SpeedPID.Calc(&Lift4SpeedPID);
			int16_t Lift4Intensity = (int16_t)Lift4SpeedPID.output;
			
			if (Lift4Intensity<500 && Lift4Intensity > -500) Lift4Intensity = 0;
			
			setMotor(AM1UDBR, Lift4Intensity);
   }
}

