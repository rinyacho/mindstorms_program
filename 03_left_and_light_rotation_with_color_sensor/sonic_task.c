#include "ev3api.h"
#include "app.h"
#include "sonic.h"
#define SONIC EV3_PORT_4

void sonic_task(intptr_t unused) {
	ev3_sensor_config(SONIC,ULTRASONIC_SENSOR);
	while (1){			// roop�ǉ�
		int16_t sonic;	// int��16bit���ɏC��
		sonic = ev3_ultrasonic_sensor_get_distance( SONIC );
		if ( sonic < 20 ){
			snd_dtq( (ID)DTQ_SONIC, SONIC_BACK );
		} else if ( sonic >= 20 && sonic < 40 ){
			snd_dtq( (ID)DTQ_SONIC, SONIC_RUN );
		} else {
			snd_dtq( (ID)DTQ_SONIC, SONIC_STOP );
		}
//		snd_dtq((ID)DTQ_SONIC,(intptr_t)sonic >= 10 ? SONIC_RUN : SONIC_STOP );
		
		dly_tsk(10);	// �ǉ� ���܂�
	// �߂����Ɖ����Ƃ��Ń��b�Z�[�W��ύX
	// �f�B���C�^�X�N
	// while�K�v�H
	}
}
