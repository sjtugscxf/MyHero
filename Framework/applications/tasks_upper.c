#include "tasks_upper.h"
#include "drivers_uartupper_user.h"

#include "drivers_flash.h"

#include "utilities_debug.h"

#include "drivers_uartupper_user.h"
#include "tasks_motor.h"
#include "string.h"
#include "drivers_uartrc_user.h"
#include "UserProtocal.h"
#include "application_pidfunc.h"
#include "drivers_canmotor_user.h"
#include "application_remotecontrol.h"

NaiveIOPoolDefine(upperIOPool, {0});

void getCtrlUartTask(void const * argument){
	while(1){

	}
}
