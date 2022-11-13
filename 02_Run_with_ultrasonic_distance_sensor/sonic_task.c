#include "ev3api.h"
#include "app.h"
#include "sonic.h"
#define SONIC EV3_PORT_4

void sonic_task(intptr_t unused) {
	int z = 0;
	int y = 0;
	ev3_sensor_config(SONIC,ULTRASONIC_SENSOR);
	while (1){			// roop�ǉ�
		int16_t sonic;	// int��16bit���ɏC��
		sonic = ev3_ultrasonic_sensor_get_distance( SONIC );
		
		if ( sonic < 20 ){
			snd_dtq( (ID)DTQ_SONIC, SONIC_BACK );		// 5~20�̋����Ńo�b�N����
		} else if ( sonic >= 20 && sonic < 40 ){
			snd_dtq( (ID)DTQ_SONIC, SONIC_RUN );		// 20~40�̋����őO�i����
		} else {
			snd_dtq( (ID)DTQ_SONIC, SONIC_STOP );		// ����ȊO�ŃX�g�b�v����
		}
		dly_tsk(10);
	}
}
