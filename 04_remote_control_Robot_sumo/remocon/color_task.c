#include "ev3api.h"
#include "app.h"
#include "string.h"
#include "color.h"

#define COLOR EV3_PORT_3

void color_task(intptr_t unused) {
	ev3_sensor_config(COLOR, COLOR_SENSOR);
	colorid_t the_color;
	while(1){
		the_color = ev3_color_sensor_get_color(COLOR);
		if (the_color == COLOR_YELLOW) { snd_dtq((ID)DTQ_COL, COL_RIGHT); }
		else if(the_color == COLOR_GREEN){ snd_dtq((ID)DTQ_COL, COL_LEFT); }
		else if (the_color == COLOR_BLUE) { snd_dtq((ID)DTQ_COL, COL_STALL); }

		else   { snd_dtq((ID)DTQ_COL, COL_STOP); }
		dly_tsk(30);

	}
}
