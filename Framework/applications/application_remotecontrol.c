#include "application_remotecontrol.h"
#include "drivers_uartrc_low.h"
#include "utilities_debug.h"
#include "stdint.h"
#include "stddef.h"
#include "drivers_ramp.h"
#include "application_pidfunc.h"
#include "application_chassiscontrol.h"
#include "application_gimbalcontrol.h"
#include "cmsis_os.h"
#include "rtos_semaphore.h"
#include "utilities_tim.h"
#include "tasks_motor.h"
#include "drivers_led_user.h"
#include "gpio.h"
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

uint8_t watch_flag = 0;
uint8_t shoot_dir = 1;

uint16_t FrictionTargetSpeed = 800;
uint16_t FrictionRealSpeed = 800;
uint16_t BulletTargetSpeed = 1000;
uint16_t BulletRealSpeed = 1000;


float pitchref = 0;

//extern RampGen_t frictionRamp ;  //Ħ����б��
//extern RampGen_t LRSpeedRamp ;   //mouse�����ƶ�б��
//extern RampGen_t FBSpeedRamp  ;   //mouseǰ���ƶ�б��

extern float yawAngleTarget, pitchAngleTarget;
/////////////////////�����õ�״̬/////////////////////
//����ģʽ
InputMode_e inputmode = ATTACK_INPUT;
//����״̬
WorkState_e workState = PREPARE_STATE;  //PREPARE
//��һ�εĹ���״̬
WorkState_e lastWorkState = PREPARE_STATE;
//���ģʽ���ֶ������Զ�
Shoot_Mode_e shootMode = MANUL;
//ϵͳ״̬
Emergency_Flag emergency_Flag = NORMAL;
//Ħ����״̬
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;
//ң������ದ��
static RemoteSwitch_t switch1;   
//���״̬
static volatile Shoot_State_e shootState = NOSHOOTING;
///////////////////////////////////////////////////////

void RCProcess(RC_CtrlData_t* pRC_CtrlData){
	
			if(pRC_CtrlData==NULL)
				return;
			SetInputMode(&(pRC_CtrlData->rc));
			switch (inputmode)
			{
				case ATTACK_INPUT:
				{
					SetEmergencyFlag(NORMAL);
					RemoteControlProcessAttack(&(pRC_CtrlData->rc));
				}break;
				case KEY_MOUSE_INPUT:
				{
					MouseKeyControlProcess(&(pRC_CtrlData->mouse),&(pRC_CtrlData->key));
					SetEmergencyFlag(NORMAL);
				}break;
				case LANDING_INPUT:
				{
					SetEmergencyFlag(NORMAL);
					RemoteControlProcessLanding(&(pRC_CtrlData->rc));
				}break;
			}
}

extern uint64_t last_rc_time;
void Timer_1ms_lTask(void const * argument)
{
//	portTickType xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();
//	static int countwhile = 0;
//	static int countwhile1 = 0;
//	unsigned portBASE_TYPE StackResidue; //ջʣ��
	while(1)  {       //motor control frequency 2ms
//�������
//		SuperviseTask();    
			//fw_printf("tick_2ms\r\n");
		uint64_t t=fw_getTimeMicros();
		if(t-last_rc_time>50000)
		{
			SetEmergencyFlag(EMERGENCY);
		}
		else
		{
			SetEmergencyFlag(NORMAL);
		}
		WorkStateFSM();
	  WorkStateSwitchProcess();
//		if(countwhile >= 500){//��ʱ 1S
//		countwhile = 0;
//			fw_printfln("ZGyroModuleAngle:  %f",ZGyroModuleAngle);
//				fw_printfln("GMYAWEncoder.ecd_angle:%f",GMYawEncoder.ecd_angle );
//			fw_printfln("in CMcontrol_task");
//		StackResidue = uxTaskGetStackHighWaterMark( GMControlTaskHandle );
//		fw_printfln("GM%ld",StackResidue);
//		}else{
//			countwhile++;
//		}
//    if(countwhile1 > 1000){
//			if(GYRO_RESETED == 0)GYRO_RST();
//		}
//		else{countwhile1++;}
//		if(countwhile1 > 2000){
//			GYRO_RESETED = 2;
//		}
//		else{countwhile1++;}
//		ShooterMControlLoop();       //���������������
//		
//		vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_RATE_MS ) );
		osDelay(1);
	}
}


//ң������������ģʽ����

void RemoteControlProcessAttack(Remote_t *rc)
{
    if(GetWorkState()!=PREPARE_STATE)
    {
			SetShootMode(MANUL);		
			forward_target = -(rc->ch1 - 1024) / 66.0 * 4000;
			left_target = (rc->ch0 - 1024) / 66.0 * 4000;
					
			rotate_target =  (rc->ch2 - 1024) /66.0*4000;
					
	//		pitchref = (rc->ch3 - 1024) /66.0*2000;
			pitchAngleTarget += (rc->ch3 - 1024)/30.0; 
		  //pitchAngleTarget += (rc->ch3 - 1024)/500.0;    //����̨6623���
//			if (watch_flag == 1)  yaw_angle_set = CAMSET;
//			else	 yaw_angle_set = 0;
	//		yawAngleTarget   -= (rc->ch2 - 1024)/8800.0 * (YAWUPLIMIT-YAWDOWNLIMIT); 
			//���-Ħ���֣����̵��״̬
			RemoteShootControl(&switch1, rc->s1);
			
			liftcali_flag = 0;
		}
}

//ң�����ǵ�����ģʽ����

void RemoteControlProcessLanding(Remote_t *rc)
{
    if(GetWorkState()!=PREPARE_STATE)
    {
				if(rc->s1==1)  //��   ÿ����������
				{
					liftcali_flag = 1;
					landing_flag = 0;
					
//					Lift1AngleTarget += (rc->ch3 - 1024)/10.0; 
//					Lift2AngleTarget += (rc->ch2 - 1024)/10.0; 
//					Lift3AngleTarget += (rc->ch1 - 1024)/10.0; 
//					Lift4AngleTarget += (rc->ch0 - 1024)/10.0; 
					aux1_targetSpeed=(rc->ch3 - 1024)/66.0*8000;
					aux2_targetSpeed=(rc->ch0 - 1024)/66.0*8000;
					aux3_targetSpeed=(rc->ch1 - 1024)/66.0*8000;
					aux4_targetSpeed=(rc->ch2 - 1024)/66.0*8000;
					
					forward_target=0;
					left_target=0;
					rotate_target=0;
					bulletSpeed=0;
					pitchAngleTarget = 0;
				}
				else if(rc->s1==3)   //��   ǰ���������ƶ�
				{
					liftcali_flag = 0;
					landing_flag = 0;
					BulletAngleTarget += (rc->ch3 - 1024)*2; 
					
					if (BulletAngleTarget < BULLETDOWNLIMIT) BulletAngleTarget = BULLETDOWNLIMIT ;
					else if (BulletAngleTarget > BULLETUPLIMIT) BulletAngleTarget = BULLETUPLIMIT ;
			//		bulletSpeed=(rc->ch3 - 1024)/66.0*8000;
					
					ChassisSpeedRef.rotate_ref=  (rc->ch2 - 1024) /66.0*300;
					forward_target = -(rc->ch1 - 1024) / 66.0 * 1000;
			    left_target = (rc->ch0 - 1024) / 66.0 * 1000;
			    //yaw_angle_set   -= (rc->ch2 - 1024)/8800.0 * (YAWUPLIMIT-YAWDOWNLIMIT) * 0.05; 
					pitchAngleTarget = 0;
				}
				else       //���� �������ȡ�����Զ��ǵ�
				{
					  liftcali_flag = 0;
					  landing_flag = 1;
				//	  yaw_angle_set= -400; 
					  pitchAngleTarget = 0;
					
					
						float autemp3 = (rc->ch0 - 1024)*2; 
						Lift1AngleTarget += autemp3; 
						Lift2AngleTarget += autemp3; 
						Lift3AngleTarget += autemp3; 
						Lift4AngleTarget += autemp3; 
					
					  BulletAngleTarget += (rc->ch3 - 1024)*2; 
					
						if (BulletAngleTarget < BULLETDOWNLIMIT) BulletAngleTarget = BULLETDOWNLIMIT ;
					   else if (BulletAngleTarget > BULLETUPLIMIT) BulletAngleTarget = BULLETUPLIMIT ;
						
						if (Lift1AngleTarget > LIFTUPLIMIT)  Lift1AngleTarget = LIFTUPLIMIT;
						else if (Lift1AngleTarget < LIFTDOWNLIMIT)  Lift1AngleTarget = LIFTDOWNLIMIT;
						if (Lift2AngleTarget > LIFTUPLIMIT)  Lift2AngleTarget = LIFTUPLIMIT;
						else if (Lift2AngleTarget < LIFTDOWNLIMIT)  Lift2AngleTarget = LIFTDOWNLIMIT;
						if (Lift3AngleTarget > LIFTUPLIMIT)  Lift3AngleTarget = LIFTUPLIMIT;
						else if (Lift3AngleTarget < LIFTDOWNLIMIT)  Lift3AngleTarget = LIFTDOWNLIMIT;
						if (Lift4AngleTarget > LIFTUPLIMIT)  Lift4AngleTarget = LIFTUPLIMIT;
						else if (Lift4AngleTarget < LIFTDOWNLIMIT)  Lift4AngleTarget = LIFTDOWNLIMIT;
					
					  if(rc->ch2>1500)
						{
							TIM4->CCR1 = 1350;
							TIM4->CCR2 = 1350;
							TIM4->CCR3 = 1225;
						}
					  else if(rc->ch2< 600) 
						{
							TIM4->CCR1 = 1000;
							TIM4->CCR2 = 1000;
							TIM4->CCR3 = 1000;
						}
				}
		}
}
//����������ģʽ����
uint8_t KeyNormal = 1;
uint8_t GMlock = 1;
uint8_t shootemer = 0;

void MouseKeyControlProcess(Mouse_t *mouse, Key_t *key)
{
	  static float forward_back_speed = 0;
		static float left_right_speed = 0;
    if(GetWorkState()!=PREPARE_STATE)
    {
			
				if( ((key->v & 0x0020)>>5) && ((key->v & 0x0040)>>6) ) KeyNormal = 1; // CTRL+Q
				if( ((key->v & 0x0020)>>5) && ((key->v & 0x0100)>>8) ) KeyNormal = 0; // CTRL+R
				if(KeyNormal == 1)
				{
					if ( (GMlock == 0) &&  ((key->v & 0x0200)>>9) )  GMlock = 1;   //F
					if( ((key->v & 0x0010)>>4) && ((key->v & 0x0200)>>9) ) GMlock = 0; // SHIFT + F
					
					//speed mode: normal speed/high speed
					if( (key->v & 0x0020)>>5 )
					{
						forward_back_speed =  FASTSPEED;
						left_right_speed = FASTSPEED;
					}
					else if((key->v & 0x0010)>>4)
					{
						forward_back_speed =  SLOWSPEED;
						left_right_speed = SLOWSPEED;
					}
					else 
					{
						forward_back_speed =  NORMALSPEED;
						left_right_speed = NORMALSPEED;
					}
		

					//movement process
					if( (key->v & 0x0001)>>0 )  // key: w
					{
						forward_target = -forward_back_speed;
					}
					else if( (key->v & 0x0002)>>1 ) //key: s
					{
						forward_target = forward_back_speed;
					}
					else
					{
						forward_target = 0;
					}
					
					
					if((key->v & 0x0004)>>2)  // key: d
					{
						left_target = -left_right_speed;
					}
					else if((key->v & 0x0008)>>3) //key: a
					{
						left_target = left_right_speed;
					}
					else
					{
						left_target = 0;
					}
					
					VAL_LIMIT(mouse->x, -150, 150); 
					VAL_LIMIT(mouse->y, -150, 150); 
				
					pitchAngleTarget -= mouse->y * GMYSET;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
					
					float mouse_rotate_target = 0;
					
					if (GMlock == 1)
					{
						mouse_rotate_target = mouse->x * ROTATESPEED;
						yaw_angle_set_last = yaw_angle_set ;
						yaw_angle_set = 0;
						if((yaw_angle_set-yaw_angle_set_last)>250 || (yaw_angle_set-yaw_angle_set_last)<-250 )
						{
							yawPositionPID.kp =3 ;
							yawSpeedPID.kp=6;
							yawpidcnt = 700;
						}
					}
					else 
					{
						mouse_rotate_target = 0;
						yaw_angle_set_last = yaw_angle_set ;
						yaw_angle_set -= mouse->x * GMXSET;
					}
					
					if( (key->v & 0x0040)>>6 )  // key: Q
					{
						rotate_target = -NORMALSPEED;
					}
					else if( (key->v & 0x0080)>>7 ) //key: E
					{
						rotate_target = NORMALSPEED;
					}
					else
					{
						rotate_target = mouse_rotate_target;
					}
					
					MouseShootControl(mouse);
					
					if(((key->v & 0x0100)>>8) && ((key->v & 0x0400)>>10) ) 
					{
						shootemer = 1;   //R+G
						testRealAngle = 0.0;
						testRealAngleLast = 0.0;
						testAngleTarget = 0;
					}
					
					if(((key->v & 0x0100)>>8) && ((key->v & 0x4000)>>14) ) 
					{
						shootemer = 0;   //R+V
						testRealAngle = 0.0;
						testRealAngleLast = 0.0;
						testAngleTarget = 0;
						shoot_dir = 1- shoot_dir;
					}
					
					if ( (key->v & 0x0800)>>11)     //Z
					{
						//�л�����ͷ
						camset();
					}
				}
			else 
			{
						forward_back_speed =  SLOWSPEED;
						left_right_speed = SLOWSPEED;
				if(((key->v & 0x0020)>>5) && ((key->v & 0x0001)>>0))
				{
					  landing_flag = 1 ;
					  liftcali_flag = 0;
						Lift1AngleTarget += LIFTNUM; 
						Lift2AngleTarget += LIFTNUM; 
						Lift3AngleTarget += LIFTNUM; 
						Lift4AngleTarget += LIFTNUM; 
					
					  if (Lift1AngleTarget > LIFTUPLIMIT)  Lift1AngleTarget = LIFTUPLIMIT;
						else if (Lift1AngleTarget < LIFTDOWNLIMIT)  Lift1AngleTarget = LIFTDOWNLIMIT;
						if (Lift2AngleTarget > LIFTUPLIMIT)  Lift2AngleTarget = LIFTUPLIMIT;
						else if (Lift2AngleTarget < LIFTDOWNLIMIT)  Lift2AngleTarget = LIFTDOWNLIMIT;
						if (Lift3AngleTarget > LIFTUPLIMIT)  Lift3AngleTarget = LIFTUPLIMIT;
						else if (Lift3AngleTarget < LIFTDOWNLIMIT)  Lift3AngleTarget = LIFTDOWNLIMIT;
						if (Lift4AngleTarget > LIFTUPLIMIT)  Lift4AngleTarget = LIFTUPLIMIT;
						else if (Lift4AngleTarget < LIFTDOWNLIMIT)  Lift4AngleTarget = LIFTDOWNLIMIT;
						
				}
				else if(((key->v & 0x0002)>>1) && ((key->v & 0x0020)>>5))
				{
					  landing_flag = 1 ;
					  liftcali_flag = 0;
						Lift1AngleTarget -= LIFTNUM; 
						Lift2AngleTarget -= LIFTNUM; 
						Lift3AngleTarget -= LIFTNUM; 
						Lift4AngleTarget -= LIFTNUM; 
					
					  if (Lift1AngleTarget > LIFTUPLIMIT)  Lift1AngleTarget = LIFTUPLIMIT;
						else if (Lift1AngleTarget < LIFTDOWNLIMIT)  Lift1AngleTarget = LIFTDOWNLIMIT;
						if (Lift2AngleTarget > LIFTUPLIMIT)  Lift2AngleTarget = LIFTUPLIMIT;
						else if (Lift2AngleTarget < LIFTDOWNLIMIT)  Lift2AngleTarget = LIFTDOWNLIMIT;
						if (Lift3AngleTarget > LIFTUPLIMIT)  Lift3AngleTarget = LIFTUPLIMIT;
						else if (Lift3AngleTarget < LIFTDOWNLIMIT)  Lift3AngleTarget = LIFTDOWNLIMIT;
						if (Lift4AngleTarget > LIFTUPLIMIT)  Lift4AngleTarget = LIFTUPLIMIT;
						else if (Lift4AngleTarget < LIFTDOWNLIMIT)  Lift4AngleTarget = LIFTDOWNLIMIT;
				}
				else if(((key->v & 0x0002)>>1) && ((key->v & 0x0010)>>4))
				{
					  landing_flag = 1 ;
					  liftcali_flag = 0;
						Lift1AngleTarget = 0; 
						Lift2AngleTarget = 0; 
						Lift3AngleTarget = 0; 
						Lift4AngleTarget = 0;  
				}
				else 
				{
						//movement process
					if( (key->v & 0x0001)>>0 )  // key: w
					{
						forward_target = -forward_back_speed;
					}
					else if( (key->v & 0x0002)>>1 ) //key: s
					{
						forward_target = forward_back_speed;
					}
					else
					{
						forward_target = 0;
					}
					
					
					if((key->v & 0x0004)>>2)  // key: d
					{
						left_target = -left_right_speed;
					}
					else if((key->v & 0x0008)>>3) //key: a
					{
						left_target = left_right_speed;
					}
					else
					{
						left_target = 0;
					}
				}
				
				
					if( ((key->v & 0x0010)>>4) && ((key->v & 0x0200)>>9) ) 
					{
							yaw_angle_set = CAMSET;
							watch_flag = 1;
							yawPositionPID.kp =3 ;
							yawSpeedPID.kp=6;
							yawpidcnt = 700;
					} // SHIFT + F
					else if ( ((key->v & 0x0200)>>9) )  
          {
							yaw_angle_set = 0;
							watch_flag = 0;
							yawPositionPID.kp =3 ;
							yawSpeedPID.kp=6;
							yawpidcnt = 700;
					}   //F
				
					if ( (key->v & 0x0800)>>11)     //Z
					{
						//�л�����ͷ
						camset();
					}
					
					
					if(((key->v & 0x0020)>>5) && ((key->v & 0x4000)>>14) )
					{
						BulletAngleTarget -= GETBULLETSPEED;
						if (BulletAngleTarget < BULLETDOWNLIMIT) BulletAngleTarget = BULLETDOWNLIMIT ;
		        else if (BulletAngleTarget > BULLETUPLIMIT) BulletAngleTarget = BULLETUPLIMIT ;
					}
					else if ((key->v & 0x4000)>>14)
					{
						BulletAngleTarget += GETBULLETSPEED;
						if (BulletAngleTarget < BULLETDOWNLIMIT) BulletAngleTarget = BULLETDOWNLIMIT ;
		        else if (BulletAngleTarget > BULLETUPLIMIT) BulletAngleTarget = BULLETUPLIMIT ;
					}
						
					if(mouse->press_r== 1) BulletTargetSpeed = 1000;
					else if(mouse->press_l== 1) BulletTargetSpeed = 1375;
					
					VAL_LIMIT(mouse->x, -150, 150); 
					VAL_LIMIT(mouse->y, -150, 150); 
				
					rotate_target = mouse->x * ROTATESPEED;
			}
	}
}


// ��������ģʽ
void SetInputMode(Remote_t *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = ATTACK_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = LANDING_INPUT;
	}	
}

/*??????*/
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	/* ����״ֵ̬ */
	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	/* ȡ����ֵ����һ��ֵ */
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);


	/* ���ϵ�״ֵ̬������ */
	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* �ϲ�����ֵ */
	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	/* �����ж� */
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}

	//����ѭ��
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}
////return the state of the remote 0:no action 1:action 
//uint8_t IsRemoteBeingAction(void)
//{
//	return (abs(ChassisSpeedRef.forward_back_ref)>=10 || abs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
//}

InputMode_e GetInputMode()
{
	return inputmode;
}

/*
input: RemoteSwitch_t *sw, include the switch info
*/

void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	GetRemoteSwitchAction(sw, val);
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�ӹرյ�start turning
			{
				SetShootState(NOSHOOTING);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�������ͱ��ر�
			{
				LASER_OFF();
				SetShootState(NOSHOOTING);
				FrictionTargetSpeed = 800;
				friction_wheel_state = FRICTION_WHEEL_OFF;
			}
			else
			{
				FrictionTargetSpeed = 1150;
				friction_wheel_state = FRICTION_WHEEL_ON; 	
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			////test6.24
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�ر�Ħ����
			{
				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				FrictionTargetSpeed = 800;
				SetShootState(NOSHOOTING);
				//
			}
			else if(sw->switch_value_raw == 2)
			{
				SetShootState(SHOOTING);
			}
			else
			{
				SetShootState(NOSHOOTING);
			}					 
		} break;				
	}
}

void MouseShootControl(Mouse_t *mouse)
{
	static int16_t closeDelayCount = 0;   //�Ҽ��ر�Ħ����3s��ʱ����
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(mouse->last_press_r == 0 && mouse->press_r == 1)   //�ӹرյ�start turning
			{
				SetShootState(NOSHOOTING);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
				closeDelayCount = 0;
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>100)   //�ر�Ħ����
			{
				LASER_OFF();
				FrictionTargetSpeed = 800;
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetShootState(NOSHOOTING);
			}
			else
			{
				FrictionTargetSpeed = 1150;
				friction_wheel_state = FRICTION_WHEEL_ON; 
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			
			if(closeDelayCount>100)   //�ر�Ħ����
			{
				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;		
        FrictionTargetSpeed = 800;				
				SetShootState(NOSHOOTING);
			}			
			else if(mouse->press_l== 1)  //������������
			{
				SetShootState(SHOOTING);				
			}
			else
			{
				SetShootState(NOSHOOTING);				
			}					 
		} break;				
	}	
	mouse->last_press_r = mouse->press_r;
}


Shoot_State_e GetShootState()
{
	return shootState;
}

void SetShootState(Shoot_State_e v)
{
	shootState = v;
}

FrictionWheelState_e GetFrictionState()
{
	return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	friction_wheel_state = v;
}
void SetFrictionWheelSpeed()
{
	if (FrictionRealSpeed < FrictionTargetSpeed) FrictionRealSpeed++;
	else FrictionRealSpeed = FrictionTargetSpeed;
	TIM12->CCR1 = FrictionRealSpeed;
	TIM12->CCR2 = FrictionRealSpeed;
//	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);
//	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}

void SetBulletWheelSpeed()
{
//	if (BulletRealSpeed < BulletTargetSpeed) BulletRealSpeed++;
//	else BulletRealSpeed = BulletTargetSpeed;
//	TIM4->CCR1 = BulletRealSpeed;
//	TIM4->CCR2 = BulletRealSpeed;
//	if(BulletRealSpeed == 1000) TIM4->CCR3 = 1000;
//	else if (BulletRealSpeed>1225) TIM4->CCR3 = 1225;
//	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);
//	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}

Shoot_Mode_e GetShootMode()
{
	return shootMode;
}
void SetShootMode(Shoot_Mode_e v)
{
	shootMode = v;
}


Emergency_Flag GetEmergencyFlag()
{
	return emergency_Flag;
}

void SetEmergencyFlag(Emergency_Flag v)
{
	emergency_Flag = v;
}

WorkState_e GetWorkState()
{
	return workState;
}

void RemoteTaskInit(void)
{
//	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
//	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
//	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
//	frictionRamp.ResetCounter(&frictionRamp);
//	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
//	FBSpeedRamp.ResetCounter(&FBSpeedRamp);

//	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
//	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	forward_target = 0.0f;
	left_target = 0.0f;
	rotate_target = 0.0f;
	
	SetFrictionState(FRICTION_WHEEL_OFF);
}


/**********************************************************
*����״̬�л�״̬��
**********************************************************/
static uint32_t time_tick_2ms = 0;
void WorkStateFSM(void)
{
	lastWorkState = workState;
	time_tick_2ms ++;
	switch(workState)
	{
		case PREPARE_STATE:
		{
//			if(GetInputMode() == STOP )//|| Is_Serious_Error())
//			{
//				workState = STOP_STATE;
//			}
//			else 
				if(time_tick_2ms > PREPARE_TIME_TICK_MS)
			{
				fw_printf("Normal state\r\n");
				workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
//			if(GetInputMode() == STOP )//|| Is_Serious_Error())
//			{
//				workState = STOP_STATE;
//			}
//			else if(!IsRemoteBeingAction()  && GetShootState() != SHOOTING) //||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC
//			{
//				fw_printfln("����STANDBY");
//				workState = STANDBY_STATE;      
//			}
		}break;
		case STANDBY_STATE:  
		{
//			if(GetInputMode() == STOP )//|| Is_Serious_Error())
//			{
//				workState = STOP_STATE;
//			}
//			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
//			{
//				workState = NORMAL_STATE;
//			}
		}break;
		case STOP_STATE:   
		{
//			if(GetInputMode() != STOP )//&& !Is_Serious_Error())
//			{
//				workState = PREPARE_STATE;   
//			}
		}break;
		default:
		{
			
		}
	}	
}
void WorkStateSwitchProcess(void)
{
	//���������ģʽ�л���prapareģʽ��Ҫ��һϵ�в�����ʼ��
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		//CMControtLoopTaskInit();
		fw_printf("remote init\r\n");
		RemoteTaskInit();
	}
}

