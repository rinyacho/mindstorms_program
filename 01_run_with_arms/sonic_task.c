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
			if ( z == 0 ){
				arm_task( 0 );		// sonic_taskからarm_taskを呼び出す
				y = y + 1;
				if ( y >= 15 ){
					z = 1;
					y = 0;
				}
			} else if ( z == 1 ){
				arm_task( 1 );		// sonic_taskからarm_taskを呼び出す
				y = y + 1;
				if ( y >= 15 ){
					z = 0;
					y = 0;
				}
			}
		} else {
			snd_dtq( (ID)DTQ_SONIC, SONIC_STOP );		// それ以外でストップする
		}
//		snd_dtq((ID)DTQ_SONIC,(intptr_t)sonic >= 10 ? SONIC_RUN : SONIC_STOP );
		
		dly_tsk(10);	// 追加 ↑含め
	// 近い時と遠いときでメッセージを変更
	// ディレイタスク
	// while必要？
	}
}
