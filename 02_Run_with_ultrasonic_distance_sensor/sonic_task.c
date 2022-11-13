#include "ev3api.h"
#include "app.h"
#include "sonic.h"
#define SONIC EV3_PORT_4

void sonic_task(intptr_t unused) {
	int z = 0;
	int y = 0;
	ev3_sensor_config(SONIC,ULTRASONIC_SENSOR);
	while (1){			// roop追加
		int16_t sonic;	// intを16bit調に修正
		sonic = ev3_ultrasonic_sensor_get_distance( SONIC );
		
		if ( sonic < 20 ){
			snd_dtq( (ID)DTQ_SONIC, SONIC_BACK );		// 5~20の距離でバックする
		} else if ( sonic >= 20 && sonic < 40 ){
			snd_dtq( (ID)DTQ_SONIC, SONIC_RUN );		// 20~40の距離で前進する
		} else {
			snd_dtq( (ID)DTQ_SONIC, SONIC_STOP );		// それ以外でストップする
		}
		dly_tsk(10);
	}
}
