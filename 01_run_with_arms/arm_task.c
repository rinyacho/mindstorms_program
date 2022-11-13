#include "ev3api.h"
#include "app.h"
#define ARM EV3_PORT_C

void arm_task(intptr_t x) {
	int y = 0;
	int z = 0;
	ev3_motor_config( ARM, MEDIUM_MOTOR );
//	while (1){
		if ( x == 0 ){
			ev3_motor_set_power( ARM,-50);
		} else if ( x == 1 ){
			ev3_motor_set_power( ARM,50);
		}
		dly_tsk(10);
		ev3_motor_set_power( ARM,0);
}
