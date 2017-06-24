#ifndef APPLICATION_GIMBALCONTROL_H
#define APPLICATION_GIMBALCONTROL_H
#include "stdint.h"
#include "application_pidfunc.h"
extern int16_t YawZeroEncoderBias; //1075 ///1075//4906
#define YAWUPLIMIT 40
#define YAWDOWNLIMIT - 90

extern int16_t PitchZeroEncoderBias; //3180 ///1075//4906
#define PITCHUPLIMIT 25
#define PITCHDOWNLIMIT -25
void setYawWithAngle(float targetAngle);
void setPitchWithAngle(float targetAngle);

extern PID_Regulator_t yawPositionPID;
extern PID_Regulator_t yawSpeedPID;


#endif
