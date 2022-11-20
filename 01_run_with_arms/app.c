/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */
#define ARM EV3_PORT_C
#include "sonic.h"
#include "ev3api.h"
#include "app.h"
// �J�����I��������f�o�b�N�������A����ȊO�̏ꍇ�͂��̂܂܎g���C���[�W
#define DEBUG
// ���ꂼ��ǂ��炩���L���ɂȂ�̂����L
#ifdef DEBUG
#define _debug(x) (x)
#else
// #define DEBUG���R�����g�A�E�g����Ă��鉺�L
#define _debug(x)
#endif

/**
 * Define the connection ports of the gyro sensor and motors.
 * By default, the Gyro Boy robot uses the following ports:
 * Gyro sensor: Port 2
 * Left motor:  Port A
 * Right motor: Port D
 */
const int gyro_sensor = EV3_PORT_2, left_motor = EV3_PORT_A, right_motor = EV3_PORT_D;

/**
 * Constants for the self-balance control algorithm.�ς��Ȃ������悢��
 */
const float KSTEER=-0.25;
const float EMAOFFSET = 0.0005f, KGYROANGLE = 7.5f, KGYROSPEED = 1.15f, KPOS = 0.07f, KSPEED = 0.1f, KDRIVE = -0.02f;
const float WHEEL_DIAMETER = 5.6;
const uint32_t WAIT_TIME_MS = 5;
const uint32_t FALL_TIME_MS = 1000;
const float INIT_GYROANGLE = -0.25;
const float INIT_INTERVAL_TIME = 0.014;

/**
 * Constants for the self-balance control algorithm. (Gyroboy Version)
 */
//const float EMAOFFSET = 0.0005f, KGYROANGLE = 15.0f, KGYROSPEED = 0.8f, KPOS = 0.12f, KSPEED = 0.08f, KDRIVE = -0.01f;
//const float WHEEL_DIAMETER = 5.6;
//const uint32_t WAIT_TIME_MS = 1;
//const uint32_t FALL_TIME_MS = 1000;
//const float INIT_GYROANGLE = -0.25;
//const float INIT_INTERVAL_TIME = 0.014;

/**
 * Global variables used by the self-balance control algorithm.�{�t�@�C���̒��ł͎g�p�\
 ���̑��̃t�@�C������͎Q�ƕs�̒�`�����L
 */
static int motor_diff, motor_diff_target;
static int loop_count, motor_control_drive, motor_control_steer;
static float gyro_offset, gyro_speed, gyro_angle, interval_time;
static float motor_pos, motor_speed;
//�X�^�e�B�b�N
/**
 * Calculate the initial gyro offset for calibration.
 �[���A�W���X�g�A�ŏ��̂܂������̈ʒu���擾�i�L�����u���[�V�����j�A�ŏ��ɌX����ƑS�������Ȃ�
 */
static ER calibrate_gyro_sensor() {
    int gMn = 1000, gMx = -100, gSum = 0;
    for (int i = 0; i < 200; ++i) {
        int gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        gSum += gyro;
        if (gyro > gMx)
            gMx = gyro;
        if (gyro < gMn)
            gMn = gyro;
        tslp_tsk(4);
    }
    if(!(gMx - gMn < 2)) { // TODO: recheck the condition, '!(gMx - gMn < 2)' or '(gMx - gMn < 2)'
        gyro_offset = gSum / 200.0f;
        return E_OK;
    } else {
        return E_OBJ;
    }
}

/**
 * Calculate the average interval time of the main loop for the self-balance control algorithm.
 * Units: seconds get_tim�̓A�C�g�����̊֐��A���Ԃ�ǂݏo���A�����ϐ��ɓ����B�X�^�[�g�^�C��
 */
static void update_interval_time() {
    static SYSTIM start_time;

    if(loop_count++ == 0) { // Interval time for the first time (use 6ms as a magic number)
        //interval_time = 0.006;
        interval_time = INIT_INTERVAL_TIME;
        ER ercd = get_tim(&start_time); // �ǂݏo�����Ƃ��̃~��sec�J�E���^�̒l���Ƃ�A�J�[�l���J�n�������
        assert(ercd == E_OK);
    } else {
        SYSTIM now;
        ER ercd = get_tim(&now);
        assert(ercd == E_OK);
        interval_time = ((float)(now - start_time)) / loop_count / 1000; // �����Ōv�Z�A�C���^�[�o���^�C���ɍŏI
    }
}

/**
 * Update data of the gyro sensor.
 * gyro_offset: the offset for calibration.
 * gyro_speed: the speed of the gyro sensor after calibration.
 * gyro_angle: the angle of the robot.�W���C���f�[�^��ǂݏo��
 */
static void update_gyro_data() {
    int gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    gyro_offset = EMAOFFSET * gyro + (1 - EMAOFFSET) * gyro_offset;
    gyro_speed = gyro - gyro_offset;
    gyro_angle += gyro_speed * interval_time;
}

/**
 * Update data of the motors�|�ꂩ�����烂�[�^�[�𓮂���
 */
static void update_motor_data() {
    static int32_t prev_motor_cnt_sum, motor_cnt_deltas[4];

    if(loop_count == 1) { // Reset
        motor_pos = 0;
        prev_motor_cnt_sum = 0;
        motor_cnt_deltas[0] = motor_cnt_deltas[1] = motor_cnt_deltas[2] = motor_cnt_deltas[3] = 0;
    }

    int32_t left_cnt = ev3_motor_get_counts(left_motor);
    int32_t right_cnt = ev3_motor_get_counts(right_motor);
    int32_t motor_cnt_sum = left_cnt + right_cnt;
    motor_diff = right_cnt - left_cnt; // TODO: with diff
    int32_t motor_cnt_delta = motor_cnt_sum - prev_motor_cnt_sum;

    prev_motor_cnt_sum = motor_cnt_sum;
    motor_pos += motor_cnt_delta;
    motor_cnt_deltas[loop_count % 4] = motor_cnt_delta;
    motor_speed = (motor_cnt_deltas[0] + motor_cnt_deltas[1] + motor_cnt_deltas[2] + motor_cnt_deltas[3]) / 4.0f / interval_time;
}

/**
 * Control the power to keep balance.
 * Return false when the robot has fallen.����̂��߂̊֐�
 */
static bool_t keep_balance() {
    static SYSTIM ok_time;

    if(loop_count == 1) // Reset ok_time
        get_tim(&ok_time);

    float ratio_wheel = WHEEL_DIAMETER / 5.6;

    // Apply the drive control value to the motor position to get robot to move.
    motor_pos -= motor_control_drive * interval_time;

    // This is the main balancing equation
    int power = (int)((KGYROSPEED * gyro_speed +               // Deg/Sec from Gyro sensor
                       KGYROANGLE * gyro_angle) / ratio_wheel + // Deg from integral of gyro
                       KPOS       * motor_pos +                // From MotorRotaionCount of both motors
                       KDRIVE     * motor_control_drive +       // To improve start/stop performance
                       KSPEED     * motor_speed);              // Motor speed in Deg/Sec

    // Check fallen
    SYSTIM time;
    get_tim(&time);
    if(power > -100 && power < 100)
        ok_time = time;
    else if(time - ok_time >= FALL_TIME_MS)
        return false;

    // Steering control
    motor_diff_target += motor_control_steer * interval_time;

    int left_power, right_power;

    // TODO: support steering and motor_control_drive
    int power_steer = (int)(KSTEER * (motor_diff_target - motor_diff));
    left_power = power + power_steer;
    right_power = power - power_steer;
    if(left_power > 100)
        left_power = 100;
    if(left_power < -100)
        left_power = -100;
    if(right_power > 100)
        right_power = 100;
    if(right_power < -100)
        right_power = -100;

    ev3_motor_set_power(left_motor, (int)left_power);
    ev3_motor_set_power(right_motor, (int)right_power);

    return true;
}
// ����̂��߂̃^�X�N
void balance_task(intptr_t unused) {
    ER ercd;

    /**
     * Reset
     */
    loop_count = 0;
    motor_control_drive = 0;
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    //TODO: reset the gyro sensor
    ev3_gyro_sensor_reset(gyro_sensor);

    /**
     * Calibrate the gyro sensor and set the led to green if succeeded.
     */
    _debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor."));
    for(int i = 10; i > 0; --i) { // Max retries: 10 times.
        ercd = calibrate_gyro_sensor();
        if(ercd == E_OK) break;
        if(i != 1)
            syslog(LOG_ERROR, "Calibration failed, retry.");
        else {
            syslog(LOG_ERROR, "Max retries for calibration exceeded, exit.");
            return;
        }
    }
    _debug(syslog(LOG_INFO, "Calibration succeed, offset is %de-3.", (int)(gyro_offset * 1000)));
    gyro_angle = INIT_GYROANGLE;
    ev3_led_set_color(LED_GREEN);

    /**
     * Main loop for the self-balance control algorithm
     */
    while(1) {
        // Update the interval time
        update_interval_time();

        // Update data of the gyro sensor
        update_gyro_data();

        // Update data of the motors
        update_motor_data();

        // Keep balance �o�����X�����Ȃ��Ƃ��ɓ�����~���鉺�L
        if(!keep_balance()) {
            ev3_motor_stop(left_motor, false);
            ev3_motor_stop(right_motor, false);
            ev3_led_set_color(LED_RED); // TODO: knock out
            syslog(LOG_NOTICE, "Knock out!");
            return;
        }

        tslp_tsk(WAIT_TIME_MS); 
        // �f�B���C�^�X�N�ƈꏏ�i�߂�l�ȊO�j�A���Ԃ̎w�肪�����ꍇ�͌Ăяo������܂Œ�~�A
    }
}

static void button_clicked_handler(intptr_t button) {	// �{�^�����I�X�ƕ������o��
    switch(button) {
    case BACK_BUTTON:
        syslog(LOG_NOTICE, "Back button clicked.");
        break;
    case LEFT_BUTTON:
    	syslog(LOG_NOTICE, "Left button clicked.");
    }
}

static FILE *bt = NULL;

void idle_task(intptr_t unused) {
    while(1) {
    	fprintf(bt, "Press 'h' for usage instructions.\n"); // �t�@�C���iBluetooth�j�Ƀv�����g
    	tslp_tsk(1000);
    }
}
// �̐S
void main_task(intptr_t unused) {
    // Draw information
    lcdfont_t font = EV3_FONT_MEDIUM;	// �t���̐ݒ�
    ev3_lcd_set_font(font);	// �t�H���g�̃T�C�Y���擾
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);	// �t�H���g�̃T�C�Y���擾�A���ƍ����A���s�Y���
    char lcdstr[100];
    ev3_lcd_draw_string("App: Gyroboy", 0, 0);
    sprintf(lcdstr, "Port%c:Gyro sensor", '1' + gyro_sensor);
    ev3_lcd_draw_string(lcdstr, 0, fonth);
    sprintf(lcdstr, "Port%c:Left motor", 'A' + left_motor);
    ev3_lcd_draw_string(lcdstr, 0, fonth * 2);
    sprintf(lcdstr, "Port%c:Right motor", 'A' + right_motor);
    ev3_lcd_draw_string(lcdstr, 0, fonth * 3);

    // Register button handlers		�����݂݂����Ȃ��́A�A�b�v�A�_�E���A
    ev3_button_set_on_clicked(BACK_BUTTON, button_clicked_handler, BACK_BUTTON);
    ev3_button_set_on_clicked(ENTER_BUTTON, button_clicked_handler, ENTER_BUTTON);
    ev3_button_set_on_clicked(LEFT_BUTTON, button_clicked_handler, LEFT_BUTTON);

    // Configure sensors	���Ȃ���
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);

    // Configure motors		���Ȃ���
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    // Start task for self-balancing	�R���t�B�O�ł̓I�t�ɂȂ��Ă���̂ŁA���̃^�X�N���A�N�e�B�x�[�g�icfg�j�Q��
    act_tsk(BALANCE_TASK);

    // Open Bluetooth file		Bluetooth�ɂȂ����ĂȂ���΃G���[
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    // Start task for printing message while idle	�R���t�B�O�ł̓I�t�ɂȂ��Ă���̂ŁA���̃^�X�N���A�N�e�B�x�[�g�icfg�j�Q��
	act_tsk(IDLE_TASK);
	
	// �����܂ŏ����A�ȍ~���[�v
    while(1) {
    	int soni;
		rcv_dtq( DTQ_SONIC, &soni );
		sus_tsk(IDLE_TASK);
		if ( soni == SONIC_BACK ){
				motor_control_drive = (-500);
		} else if ( soni == SONIC_RUN ){
				motor_control_drive = (500);
		} else {
				motor_control_drive = 0;
		}
		dly_tsk(10);
		rsm_tsk(IDLE_TASK);
    }
}

// �ۑ�ł̓O���[�o���ϐ��̒l��ύX���đΉ����Ă���
// �r��U���Č��C�悭�����悤�ɁAm_moter_tsk�Ői�߂�A�\�[�X�R�[�h�̓V���v���A���C���g���[�X�̃C���[�W
// �O�i���Ă���Ԃ̎��Ԃɂ���ē�����؂�ւ��邱��
// �o�����X�^�X�N�̓W���C���R���g���[���A���C���^�X�N��Bluetooth�A�A�C�h���^�X��Bluetooth�A�C�ɂ���K�v�͂Ȃ�