#include "ev3api.h"
#include "app.h"
#include "sonic.h"
#define COLOR EV3_PORT_3

void color_task(intptr_t unused) {
	ev3_sensor_config( EV3_PORT_3, COLOR_SENSOR);
	ev3_speaker_set_volume(1);
	while (1){			// roop�ǉ�
		if ( COLOR_GREEN == ev3_color_sensor_get_color( COLOR )){
			snd_dtq( (ID)DTQ_COLOR, COLOR_LEFT );	// ����]
			ev3_led_set_color(LED_ORANGE );			// �f�o�b�O�p
		} else if ( COLOR_YELLOW == ev3_color_sensor_get_color( COLOR )){
			snd_dtq( (ID)DTQ_COLOR, COLOR_RIGHT );	// �E��]
			ev3_led_set_color(LED_RED );			// �f�o�b�O�p
		} else {
			snd_dtq( (ID)DTQ_COLOR, COLOR_FRONT );	// �ʏ�^�]
		}
		dly_tsk(10);
		ev3_led_set_color(LED_GREEN );
	}
}
