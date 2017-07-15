#include "tasks_motor.h"
#include "tasks_Hero.h"
#include "cmsis_os.h"
#include "utilities_debug.h"
#include "application_auxmotorcontrol.h"
#include "application_remotecontrol.h"
Hero_Order_t Hero_Order=HERO_STANDBY;
Hero_State_t Hero_State=HERO_NORMAL_STATE;

//function called
void Hero_Recover();
uint8_t Hero_Stretch(float value, uint32_t time_milis);
uint8_t Hero_Lift(float value, uint32_t time_milis);
void Hero_Prepare_Get_Bullet();
void HeroTask(void const * argument){
	while(1)
	{
		switch(Hero_Order)
		{
			case HERO_GETBULLET:
			{
				Hero_Prepare_Get_Bullet();
			}break;
			case HERO_STOP:
			{
				Hero_Recover();
			}break;
			case HERO_STANDBY:
			{
				osDelay(10);
			}break;
		}
	}
}

//ִ�������������Ħ���ֵĶ���
void Hero_Prepare_Get_Bullet()
{
	Hero_Order=HERO_STANDBY;
	if(Hero_State==HERO_NORMAL_STATE)
	{
		Hero_State=HERO_PREPARE_GET_BULLET;
		if(!Hero_Lift(aux34_limit,1000)){Hero_Order=HERO_STOP;return;}
		if(!Hero_Stretch(getBullet_limit,1000)){Hero_Order=HERO_STOP;return;}
		StartBulletFrictionWheel();
		Hero_State=HERO_GETTING_BULLET;
	}
}





//����ıջ�������������
uint8_t Hero_Stretch(float value, uint32_t time_milis)
{
	float original=getBullet_angle_target;
	//ʹ����β�ֵ
	//float tmp=(value-original)/time_milis;
	for(uint32_t i=1;i<time_milis+1;i++)
	{
		float a_b=original-value;
		float tmp=((float)i/time_milis);
		float tmp3=tmp*tmp*tmp;
		float tmp4=tmp3*tmp;
		float tmp5=tmp4*tmp;
		getBullet_angle_target=-6*a_b*tmp5+15*a_b*tmp4-10*a_b*tmp3+original;
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when strech!");
			return 0;
		}
		//fw_printfln("%f",aux_motor2_position_target);
		osDelay(1);
	}
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(6)-value) >1000 || (GetAuxMotorRealAngle(6)-value)<-1000)
	{
		if(cnt++>500)
		{
			fw_printfln("stretch not reach!target %f  real %f", value, GetAuxMotorRealAngle(6));
			Hero_Order=HERO_STOP;
			return 0;
		}
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when strech! ");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

//����ıջ����ƺ���
uint8_t Hero_Lift(float value, uint32_t time_milis)
{
	float original=aux_motor34_position_target;
	float tmp=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis+1;i++)
	{
		aux_motor34_position_target=original + i*tmp;
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(4)-value) > 2000 || (GetAuxMotorRealAngle(4)-value)<-2000)
	{
		if(cnt++>800)
		{
			Hero_Order=HERO_STOP;
			fw_printfln("lift not reach! target %f  real %f", value, GetAuxMotorRealAngle(4));
			return 0;
		}
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

//no check, only used in recovering!
void HeroForceLift(float value, uint32_t time_milis)
{
	float original=aux_motor34_position_target;
	float step=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis;i++)
	{
		aux_motor34_position_target+=step;
		osDelay(1);
	}
	aux_motor34_position_target=value;
}
//no check, only used in recovering!
void HeroForceStretch(float value, uint32_t time_milis)
{
	float original=getBullet_angle_target;
	float step=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis;i++)
	{
		getBullet_angle_target+=step;
		osDelay(1);
	}
	getBullet_angle_target=value;
}


//ִ�лָ����������������䣬�ر�Ħ����
void Hero_Recover()
{
	Hero_Order=HERO_STANDBY;
	Hero_State=HERO_RECOVERING;
	StopBulletFrictionWheel();
	HeroForceLift(20000,1000);
	HeroForceStretch(0,10000);
	HeroForceLift(0,1000);
	Hero_State=HERO_NORMAL_STATE;
}