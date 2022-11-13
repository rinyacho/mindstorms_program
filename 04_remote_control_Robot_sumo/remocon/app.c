#include "ev3api.h"
#include "app.h"
#include "color.h"


#include <string.h>

//���� ev3rt �� bluetooth �o�R�ŃV���A���ڑ�
#include "spp_master_test_api.h"

void main_task(intptr_t unused)
{
	//�ڑ�����̏��
	uint8_t remote_addr[6] = {0xcc, 0x78, 0xab, 0xd3, 0xbc, 0xf0}; //no.19��mac�A�h���X
	const char* remote_pincode = "1234";

	ev3_led_set_color(LED_RED);

	//�V���A���ڑ��p�t�@�C�����쐬
	FILE *bt_slave = spp_master_test_open_file();

	act_tsk(SONIC_TASK);
	act_tsk(COLOR_TASK);


	int st = 0; //�����Z���T�p��flag
	int cl = 0; //�J���[�Z���T�p��flag
	while (1) {
		// Connect to slave
		while (!spp_master_test_is_connected()) {
			ev3_led_set_color(LED_RED);
			spp_master_test_connect_ev3(remote_addr, remote_pincode);
		}
		ev3_led_set_color(LED_GREEN);

		int sonic = 0; //�����Z���T�̃L���[�p
		int color = 0; //�J���[�Z���T�̃L���[�p
		
		/**************** �f�[�^�L���[���󂯎�� ****************/
		rcv_dtq(DTQ_SONIC, (intptr_t *)&sonic);
		rcv_dtq(DTQ_COL, (intptr_t *)&color);
		
		/*******************�����Z���T*********************/
		/		//���� < 20cm�̂Ƃ�
				if (sonic < 20) {
					fprintf(bt_slave, "s\n");//"s\n"�𑗐M
					ev3_speaker_play_tone(220, 100);//�f�o�b�N�p
					st = 1;
				}
				//20cm < ���� < 40cm �̂Ƃ�
				else 
				if (sonic < 40) {
						fprintf(bt_slave, "w\n");//"w\n"�𑗐M
						ev3_speaker_play_tone(440, 100);//�f�o�b�N�p
						st = 2;
					}
				// ���� >= 40�̂Ƃ�
				else {
					switch (st) {
					case 1:
						fprintf(bt_slave, "w\n");//"w\n"�𑗐M
						break;
					case 2:
						fprintf(bt_slave, "s\n");//"s\n"�𑗐M
						break;
					default:
						break;
					}
					st = 0;
				}
			
			
		/*******************�J���[�Z���T*********************/
				if (color == COL_RIGHT) {
					fprintf(bt_slave, "a\n");
					ev3_speaker_play_tone(110, 100);//�f�o�b�N�p
					cl = 1;
				}
				else
					if (color == COL_LEFT) {
						fprintf(bt_slave, "d\n");//"d\n"�𑗐M
						ev3_speaker_play_tone(660, 100);//�f�o�b�N�p
						cl = 2;
					}
					else
						if (color == COL_STALL) {
							fprintf(bt_slave, "z\n");//"z\n"�𑗐M
							cl = 3;
						}
					else {
						switch (cl) {
						case 1:
							fprintf(bt_slave, "a\n");//"a\n"�𑗐M
							break;
						case 2:
							fprintf(bt_slave, "d\n");//"d\n"�𑗐M
							break;
						case 3:
							fprintf(bt_slave, "z\n");//"z\n"�𑗐M
							break;
						default:
							break;
						}
						cl = 0;
					}
				dly_tsk(25);
		
	}
}
