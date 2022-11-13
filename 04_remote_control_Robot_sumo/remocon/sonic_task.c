#include "ev3api.h"
#include "app.h"

#include <string.h>

void sonic_task(intptr_t unused)
{
	sensor_port_t port = EV3_PORT_4;
	ev3_sensor_config(port, ULTRASONIC_SENSOR);

	while (1) {
		int sonic = (int)ev3_ultrasonic_sensor_get_distance(port);
		snd_dtq((ID)DTQ_SONIC, (intptr_t)sonic);
		dly_tsk(20);
	}
}
