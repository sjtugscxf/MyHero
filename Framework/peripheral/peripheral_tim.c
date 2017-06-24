#include "peripheral_tim.h"
#include "cmsis_os.h"
#include "tim.h"
#include "peripheral_define.h"
#include "utilities_debug.h"
#define FRICTION_TIM htim12
#define BULLET_TIM htim4

void UserTimerInit(void)
{
//	HAL_TIM_Encoder_Start(&PLATE_TIM, TIM_CHANNEL_ALL);
//	HAL_TIM_PWM_Start(&PLATE_MOTOR_TIM , TIM_CHANNEL_1);//2|?¨¬¦Ì??¨²
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&BULLET_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&BULLET_TIM, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&BULLET_TIM, TIM_CHANNEL_3);
	fw_printf("Friction Init Success!");
}
