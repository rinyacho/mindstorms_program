#include "ev3api.h"
#include "app.h"
#include "sonic.h"
#define COLOR EV3_PORT_3

void color_task(intptr_t unused) {
	ev3_sensor_config( EV3_PORT_3, COLOR_SENSOR);
	ev3_speaker_set_volume(1);
	while (1){			// roop追加
		if ( COLOR_GREEN == ev3_color_sensor_get_color( COLOR )){
			snd_dtq( (ID)DTQ_COLOR, COLOR_LEFT );	// 左回転
			ev3_led_set_color(LED_ORANGE );			// デバッグ用
		} else if ( COLOR_YELLOW == ev3_color_sensor_get_color( COLOR )){
			snd_dtq( (ID)DTQ_COLOR, COLOR_RIGHT );	// 右回転
			ev3_led_set_color(LED_RED );			// デバッグ用
		} else {
			snd_dtq( (ID)DTQ_COLOR, COLOR_FRONT );	// 通常運転
		}
		dly_tsk(10);
		ev3_led_set_color(LED_GREEN );
	}
}
