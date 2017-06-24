#include "tasks_engineering.h"
#include "stdint.h"
#include "tasks_motor.h"
#include "cmsis_os.h"
#include "application_pidfunc.h"
#include "stdlib.h"
#include "utilities_debug.h"
uint8_t engineer_task_on=0;

extern uint32_t ADC_Value[60];
uint32_t targetValue2=1430;
//假设这个pid的输出和遥控器是同一个单位
PID_Regulator_t Engineer_ForwardBackPID = PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);
PID_Regulator_t Engineer_LeftRightPID = PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);


void EngineeringTask(void const * argument){
	static int32_t closecnt=0;
	while(1){
		if(engineer_task_on)
		{
			////////////注意！
			////ad1 右边，ad2，中间，ad3，左边
			int32_t ad1=0,ad2=0,ad3=0;
			for(uint16_t i=0;i<60;i++)
			{
				if(i%3==0)ad1+=ADC_Value[i];
				if(i%3==1)ad2+=ADC_Value[i];
				if(i%3==2)ad3+=ADC_Value[i];
			}
			ad1/=20;
			ad2/=20;
			ad3/=20;
			
			if(ad2<600)
			{
				osDelay(10);
				continue;
			}
			if(abs(ad2-targetValue2)<30 && abs(ad1-ad3)<30)
			{
				closecnt++;
			}
			else
			{
				closecnt--;
			}
			if(closecnt>1100)
				closecnt=1100;
			else if(closecnt<0)
				closecnt=0;
			if(closecnt>1000)
			{
				fw_printf("close! cnt:%d\r\n",closecnt);
				osDelay(2);
				continue;
			}
			Engineer_ForwardBackPID.target=targetValue2;
			Engineer_ForwardBackPID.feedback=ad2;
			Engineer_ForwardBackPID.Calc(&Engineer_ForwardBackPID);
			
			Engineer_LeftRightPID.target=0;
			Engineer_LeftRightPID.feedback= ad1-ad3;
			Engineer_LeftRightPID.Calc(&Engineer_LeftRightPID);
			
			ChassisSpeedRef.forward_back_ref=Engineer_ForwardBackPID.output/66.0*4000;
			ChassisSpeedRef.left_right_ref=Engineer_LeftRightPID.output/66.0*3000;
		}
		
		osDelay(2);
		
}
	
}
