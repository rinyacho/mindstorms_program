#include "ev3api.h"
#include "app.h"
#include "color.h"


#include <string.h>

//他の ev3rt に bluetooth 経由でシリアル接続
#include "spp_master_test_api.h"

void main_task(intptr_t unused)
{
	//接続相手の情報
	uint8_t remote_addr[6] = {0xcc, 0x78, 0xab, 0xd3, 0xbc, 0xf0}; //no.19のmacアドレス
	const char* remote_pincode = "1234";

	ev3_led_set_color(LED_RED);

	//シリアル接続用ファイルを作成
	FILE *bt_slave = spp_master_test_open_file();

	act_tsk(SONIC_TASK);
	act_tsk(COLOR_TASK);


	int st = 0; //距離センサ用のflag
	int cl = 0; //カラーセンサ用のflag
	while (1) {
		// Connect to slave
		while (!spp_master_test_is_connected()) {
			ev3_led_set_color(LED_RED);
			spp_master_test_connect_ev3(remote_addr, remote_pincode);
		}
		ev3_led_set_color(LED_GREEN);

		int sonic = 0; //距離センサのキュー用
		int color = 0; //カラーセンサのキュー用
		
		/**************** データキューを受け取る ****************/
		rcv_dtq(DTQ_SONIC, (intptr_t *)&sonic);
		rcv_dtq(DTQ_COL, (intptr_t *)&color);
		
		/*******************距離センサ*********************/
		/		//距離 < 20cmのとき
				if (sonic < 20) {
					fprintf(bt_slave, "s\n");//"s\n"を送信
					ev3_speaker_play_tone(220, 100);//デバック用
					st = 1;
				}
				//20cm < 距離 < 40cm のとき
				else 
				if (sonic < 40) {
						fprintf(bt_slave, "w\n");//"w\n"を送信
						ev3_speaker_play_tone(440, 100);//デバック用
						st = 2;
					}
				// 距離 >= 40のとき
				else {
					switch (st) {
					case 1:
						fprintf(bt_slave, "w\n");//"w\n"を送信
						break;
					case 2:
						fprintf(bt_slave, "s\n");//"s\n"を送信
						break;
					default:
						break;
					}
					st = 0;
				}
			
			
		/*******************カラーセンサ*********************/
				if (color == COL_RIGHT) {
					fprintf(bt_slave, "a\n");
					ev3_speaker_play_tone(110, 100);//デバック用
					cl = 1;
				}
				else
					if (color == COL_LEFT) {
						fprintf(bt_slave, "d\n");//"d\n"を送信
						ev3_speaker_play_tone(660, 100);//デバック用
						cl = 2;
					}
					else
						if (color == COL_STALL) {
							fprintf(bt_slave, "z\n");//"z\n"を送信
							cl = 3;
						}
					else {
						switch (cl) {
						case 1:
							fprintf(bt_slave, "a\n");//"a\n"を送信
							break;
						case 2:
							fprintf(bt_slave, "d\n");//"d\n"を送信
							break;
						case 3:
							fprintf(bt_slave, "z\n");//"z\n"を送信
							break;
						default:
							break;
						}
						cl = 0;
					}
				dly_tsk(25);
		
	}
}
