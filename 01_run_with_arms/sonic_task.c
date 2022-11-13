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
			if ( z == 0 ){
				arm_task( 0 );		// sonic_task����arm_task���Ăяo��
				y = y + 1;
				if ( y >= 15 ){
					z = 1;
					y = 0;
				}
			} else if ( z == 1 ){
				arm_task( 1 );		// sonic_task����arm_task���Ăяo��
				y = y + 1;
				if ( y >= 15 ){
					z = 0;
					y = 0;
				}
			}
		} else {
			snd_dtq( (ID)DTQ_SONIC, SONIC_STOP );		// ����ȊO�ŃX�g�b�v����
		}
//		snd_dtq((ID)DTQ_SONIC,(intptr_t)sonic >= 10 ? SONIC_RUN : SONIC_STOP );
		
		dly_tsk(10);	// �ǉ� ���܂�
	// �߂����Ɖ����Ƃ��Ń��b�Z�[�W��ύX
	// �f�B���C�^�X�N
	// while�K�v�H
	}
}
