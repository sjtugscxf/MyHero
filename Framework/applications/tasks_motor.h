#ifndef TASKS_MOTOR_H
#define TASKS_MOTOR_H
#include "stdint.h"

typedef __packed struct
{
    float forward_back_ref;
    float left_right_ref;
    float rotate_ref;
}ChassisSpeed_Ref_t;

#define PITCHLIMIT 2000.0

extern ChassisSpeed_Ref_t ChassisSpeedRef;

void CMGMControlTask(void const * argument);
void AMControlTask(void const * argument);

void ShooterLoop();
void SetPitchWithAngle3510(float Pitch3AngleTargetSet);
void SetBulletAngle(float BulletAngleTargetSet);
extern float BulletAngleTarget;

void SetLift1Angle(float Lift1AngleTargetSet);
void SetLift2Angle(float Lift2AngleTargetSet);
void SetLift3Angle(float Lift3AngleTargetSet);
void SetLift4Angle(float Lift4AngleTargetSet);
extern float Lift1AngleTarget;
extern float Lift2AngleTarget;
extern float Lift3AngleTarget;
extern float Lift4AngleTarget;

extern float bulletSpeed;
extern uint8_t landing_flag;
extern uint8_t liftcali_flag;

extern float aux1_targetSpeed;
extern float aux2_targetSpeed;
extern float aux3_targetSpeed;
extern float aux4_targetSpeed;

extern float testRealAngle;
extern float testRealAngleLast;
extern float testAngleTarget;

extern float forward_target;
extern float left_target;
extern float rotate_target;
extern float yaw_angle_set;
extern float yaw_angle_set_last;
extern uint32_t yawpidcnt;
#endif
