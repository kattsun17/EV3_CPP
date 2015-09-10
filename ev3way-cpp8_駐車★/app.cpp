/**
 ******************************************************************************
 ** ?��t?��@?��C?��?��?��?�� : app.cpp
 **
 ** ?��T?��v : 2?��֓|?��?��?��U?��q?��?��?��C?��?��?��g?��?��?��[?��X?��?��?��{?��b?��g?��?��TOPPERS/HRP2?��pC++?��T?��?��?��v?��?��?��v?��?��?��O?��?��?��?��
 **
 ** ?��?��?��L : sample_cpp (?��?��?��C?��?��?��g?��?��?��[?��X/?��K?��?��[?��^/?��?��?��?��?��g?��Z?��?��?��T/?��?��?��?��?��[?��g?��X?��^?��[?��g)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"

using namespace ev3api;

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* Bluetooth */
static int32_t   bt_cmd = 0;      /* Bluetooth?��R?��}?��?��?��h 1:?��?��?��?��?��[?��g?��X?��^?��[?��g */
static FILE     *bt = NULL;      /* Bluetooth?��t?��@?��C?��?��?��n?��?��?��h?��?�� */

/* ?��?��?��L?��̃}?��N?��?��?��͌�/?���?��ɍ�?�����?��ĕύX?��?��?��?��?��K?��v?��?��?��?��?��?��?��܂� */
#define GYRO_OFFSET           0  /* ?��W?��?��?��C?��?��?��Z?��?��?��T?��I?��t?��Z?��b?��g?��l(?��p?��?��?��x0[deg/sec]?��?��) */
#define LIGHT_WHITE          25  /* ?��?��?��F?��̌�?��Z?��?��?��T?��l */
#define LIGHT_BLACK           1  /* ?��?��?��F?��̌�?��Z?��?��?��T?��l */
#define SONAR_ALERT_DISTANCE  5  /* ?��?��?��?��?��g?��Z?��?��?��T?��ɂ�?��?��?��?��?��Q?��?��?��?��?��m?��?��?��?��[cm] */
#define TAIL_ANGLE_STAND_UP  93  /* ?��?��?��S?��?��?��~?��?��?��̊p?��x[?��x] */
#define TAIL_ANGLE_DRIVE      0  /* ?��o?��?��?��?��?��X?��?��?��s?��?��?��̊p?��x[?��x] */
#define P_GAIN             2.5F  /* ?��?��?��S?��?��?��~?��p?��?��?��[?��^?��?��?��?��?��?��?��?��?��W?��?�� */
#define PWM_ABS_MAX          60  /* ?��?��?��S?��?��?��~?��p?��?��?��[?��^?��?��?��?��PWM?��?��?��΍ő�?��l */
//#define DEVICE_NAME     "ET0"  /* Bluetooth?��?�� hrp2/target/ev3.h BLUETOOTH_LOCAL_NAME?��Őݒ� */
//#define PASS_KEY        "1234" /* ?��p?��X?��L?��[    hrp2/target/ev3.h BLUETOOTH_PIN_CODE?��Őݒ� */
#define CMD_START         '1'    /* ?��?��?��?��?��[?��g?��X?��^?��[?��g?��R?��}?��?��?��h */

/* LCD?��t?��H?��?��?��g?��T?��C?��Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* ?��֐�?��v?��?��?��g?��^?��C?��v?��錾 */
static int32_t sonar_alert(void);
static void tail_control(int32_t angle);
static void lookup_strategy(void);
static void style_change(int32_t angle, int32_t time_ms);
static void garage_stop(void);

/* ?��I?��u?��W?��F?��N?��g?��ւ̃|?��C?��?��?��^?��?��?��` */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;

/* ?��?��?��C?��?��?��^?��X?��N */
void main_task(intptr_t unused)
{
    int8_t forward;      /* ?��O?��?��?��i?��?��?��?�� */
    int8_t turn;         /* ?��?��?��?��� */
    int8_t pwm_L, pwm_R; /* ?��?��?��E?��?��?��[?��^PWM?��o?��?�� */
    int8_t flag_lookup = 0;  /* the flag for lookup gate */
    //int8_t flag_garage = 0;  /* the flag for Garage */

    /* ?��e?��I?��u?��W?��F?��N?��g?��?���?��E?��?��?��?��?��?��?��?��?��?�� */
    touchSensor = new TouchSensor(PORT_1);
    colorSensor = new ColorSensor(PORT_3);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();

    /* LCD?��?��?��ʕ\?��?�� */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);

    /* ?��K?��?��[?��^?��[?���??��?��Z?��b?��g */
    tailMotor->reset();


    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth?��ʐM?��^?��X?��N?��̋N?��?�� */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* ?��?��?��?��?��?��?��?��?��?��?��ʒm */

    /* ?��X?��^?��[?��g?��ҋ@ */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* ?��?��?��S?��?��?��~?��p?��p?��x?��ɐ�?��?�� */

        if (bt_cmd == 1)
        {
            break; /* ?��?��?��?��?��[?��g?��X?��^?��[?��g */
        }

        if (touchSensor->isPressed())
        {
            ev3_speaker_play_tone(300, 20);
            ev3_speaker_play_tone(400, 30);
            ev3_speaker_play_tone(500, 40);
            ev3_speaker_play_tone(600, 50);
            ev3_speaker_play_tone(700, 60);
            break; /* ?��^?��b?��`?��Z?��?��?��T?��?��?��?��?��?��?��ꂽ */
        }

        clock->sleep(10);
    }

    /* ?��?��?��s?��?��?��[?��^?��[?��G?��?��?��R?��[?��_?��[?��?��?��Z?��b?��g */
    leftMotor->reset();
    rightMotor->reset();

    /* ?��W?��?��?��C?��?��?��Z?��?��?��T?��[?��?��?��Z?��b?��g */
    gyroSensor->reset();
    balance_init(); /* ?��|?��?��?��U?��qAPI?��?��?��?��?��?�� */

    ev3_led_set_color(LED_GREEN); /* ?��X?��^?��[?��g?��ʒm */

    /**
    * Main loop for the self-balance control algorithm
    */
    while(1)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;
        // if (touchSensor->isPressed()) { break; }

        tail_control(TAIL_ANGLE_DRIVE); /* ?��o?��?��?��?��?��X?��?��?��s?��p?��p?��x?��ɐ�?��?�� */





        if (sonar_alert() == 1 && flag_lookup == 0) /* ?��?��?��Q?��?��?��?��?��m */
        {
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);

            lookup_strategy();
            ev3_speaker_play_tone(300, 10);
            garage_stop();

            flag_lookup = 1;
            //flag_garage = 1;
        }



        forward = 30; /* ?��O?��i?��?��?��?�� */



        /*
        // ?��?��?��?��?��ő�?��x?��ύX
        if ( ev3_motor_get_counts(EV3_PORT_B ) < 100 ) { forward = 100;  ev3_speaker_play_tone(300, 3);  }
        else if ( ev3_motor_get_counts(EV3_PORT_B ) < 800 ) { forward = 10; ev3_speaker_play_tone(800, 3);}
        else { forward = 10;  ev3_speaker_play_tone(100, 3); }
        */

        //if ( ev3_gyro_sensor_get_angle(EV3_PORT_4) > 0 ) {  ev3_speaker_play_tone(300, 1); }
        //if ( ev3_gyro_sensor_get_angle(EV3_PORT_4) == 0 ) {  ev3_speaker_play_tone(1, 1); }
        //else if ( ev3_gyro_sensor_get_angle(EV3_PORT_4) > 0 ) {  ev3_speaker_play_tone(800, 1); }

        if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2)
        {
            turn = 30; /* ?��?��?��?��?��?��� */
        }
        else
        {
            turn = -30; /* ?��E?��?��?��?��� */
        }

        /* ?��|?��?��?��U?��q?��?��?��?��API ?��ɓn?��?��?��p?��?��?��?��?��[?��^?��?��?��擾?��?��?��?�� */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* ?��|?��?��?��U?��q?��?��?��?��API?��?��?��Ăяo?��?��?��A?��|?��?��?��?��?��s?��?��?��邽?��߂� */
        /* ?��?��?��E?��?��?��[?��^?��o?��͒l?��?��� */
        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (int8_t *)&pwm_L,
            (int8_t *)&pwm_R
        );

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

        clock->sleep(4); /* 4msec?��?��?��?��?��N?��?�� */
    }

    leftMotor->reset();
    rightMotor->reset();

    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// ?��֐�?��?�� : sonar_alert
// ?��?��?��?�� : ?��?��?��?��
// ?��Ԃ�?��l : 1(?��?��?��Q?��?��?��?��?��?��)/0(?��?��?��Q?��?��?��?��?��?��)
// ?��T?��v : ?��?��?��?��?��g?��Z?��?��?��T?��ɂ�?��?��?��?��?��Q?��?��?��?��?��m
//*****************************************************************************
static int32_t sonar_alert(void)
{
    static uint32_t counter = 0;
    static int32_t alert = 0;

    int32_t distance;

    if (++counter == 40/4) /* ?��?��40msec?��?��?��?��?��?��?��ɏ�?��Q?��?��?��?��?��m  */
    {
        /*
         * ?��?��?��?��?��g?��Z?��?��?��T?��ɂ�?��鋗�?��?��?��?��?��?��?��?��?��?��́A?��?��?��?��?��g?��̌�?��?��?��?��?��?��?��Ɉˑ�?��?��?��܂�?��B
         * NXT?��̏ꍇ?��́A40msec?��?��?��?��?��?��?��x?��?��?��o?��?��?��?��?��̍ŒZ?��?��?��?��?��?��?��?��?��ł�?��B
         * EV3?��̏ꍇ?��́A?��v?��m?��F
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* ?��?��?��Q?��?��?��?��?��?��?��m */
        }
        else
        {
            alert = 0; /* ?��?��?��Q?��?��?��?��?��?�� */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// ?��֐�?��?�� : tail_control
// ?��?��?��?�� : angle (?��?��?��[?��^?��ڕW?��p?��x[?��x])
// ?��Ԃ�?��l : ?��?��?��?��
// ?��T?��v : ?��?��?��s?��̊�?��S?��?��?��~?��p?��?��?��[?��^?��̊p?��x?��?��?��?��
//*****************************************************************************
static void tail_control(int32_t angle)
{
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* ?��?��?��ᐧ?��?�� */
    /* PWM?��o?��͖O?��a?��?��?��?�� */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);
}

//*****************************************************************************
// ?��֐�?��?�� : bt_task
// ?��?��?��?�� : unused
// ?��Ԃ�?��l : ?��Ȃ�
// ?��T?��v : Bluetooth?��ʐM?��ɂ�?��郊�?��?��[?��g?��X?��^?��[?��g?��B Tera Term?��Ȃǂ̃^?��[?��~?��i?��?��?��\?��t?��g?��?��?��?��?��A
//       ASCII?��R?��[?��h?��?��1?��?��M?��?��?��?��?��ƁA?��?��?��?��?��[?��g?��X?��^?��[?��g?��?��?��?��?��B
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* ?��?��?��M */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* ?��G?��R?��[?��o?��b?��N */
    }
}

//*****************************************************************************
// ?��֐�?��?�� : lookup_strategy
// ?��?��?��?�� : unused
// ?��Ԃ�?��l : ?��Ȃ�
// ?��T?��v : lookup gate
//
//*****************************************************************************
void lookup_strategy(void)
{
    int32_t i;
    int32_t forward = 0;
    int32_t turn = 0;
    //int32_t ddeff;
    int32_t gyro;
    int32_t volt;
    int32_t motor_ang_l, motor_ang_r;
    int8_t pwm_L, pwm_R;

    //int32_t oldang_l, oldang_r;

    i = 0;
    while (1) {
        if (i++ >= 50) { break; }

        tail_control(79);

        // ?��|?��?��?��?��?��?��
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)-30, // ?��W?��?��?��C?��?��?��I?��t?��Z?��b?��g?��l
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (int8_t *)&pwm_L,
            (int8_t *)&pwm_R
        );
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);


        clock->sleep(4);
    }

    style_change(77, 500);
    style_change(75, 100);
    style_change(73, 100);
    style_change(71, 100);
    style_change(69, 100);
    style_change(66, 100);
    style_change(63, 100);
    style_change(62, 100);
    style_change(61, 100);
    style_change(60, 100);

    // ?��?��?��?��?��ۗ�?��ĂĂ�?��?��?��?��?��?��?��i?��?��

    // ?��?��?��E?��?��?��[?��^?��̃Y?��?��?��␳?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��?��B
    //oldang_l = leftMotor->getCount();
    //oldang_r = rightMotor->getCount();

    ev3_speaker_play_tone(300, 20);
    ev3_speaker_play_tone(400, 30);
    ev3_speaker_play_tone(500, 40);
    ev3_speaker_play_tone(600, 50);
    ev3_speaker_play_tone(700, 60);

    motor_ang_l = ev3_motor_get_counts(EV3_PORT_B);
    motor_ang_r = ev3_motor_get_counts(EV3_PORT_C);
    int32_t lpower = 10;
    int32_t rpower = 10;

    i = 0;
    while (1) {
        if (i++ >= 500) { break; }

            ev3_motor_set_power(EV3_PORT_B, lpower);
            ev3_motor_set_power(EV3_PORT_C, rpower);
            //ev3_motor_steer(EV3_PORT_C, EV3_PORT_B, 10, 0);
            //ev3_motor_rotate(EV3_PORT_C, 10000, 100, false);
            //ev3_motor_rotate(EV3_PORT_B, 10000, 100, false);


            // まっすぐすすませるため
            if ( i % 10 == 0 ) {
                if ( (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l) - (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r) > 0 ) {
                    //ddeff = (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l) - (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r);
                  //  for ( k = 0; k < ddeff; k++ ) {   ev3_speaker_play_tone(100, 200); clock->sleep(4); }
                    lpower = 10;
                    rpower = 20;

                    //ev3_motor_rotate(EV3_PORT_C, ddeff, 10, true);
                } else if ( (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l) - (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r) < 0 ) {
                    //ddeff = (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r) - (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l);
                    //for ( k = 0; k < ddeff; k++ ) {   ev3_speaker_play_tone(300, 200); clock->sleep(4); }
                    //ev3_motor_rotate(EV3_PORT_B, ddeff, 10, true);
                    lpower = 20;
                    rpower = 10;
                } else {
                    lpower = 10;
                    rpower = 10;
                }

                motor_ang_l = ev3_motor_get_counts(EV3_PORT_B);
                motor_ang_r = ev3_motor_get_counts(EV3_PORT_C);
            }

            tail_control(60);

            /*
            ev3_speaker_play_tone(300, 20);
            ev3_speaker_play_tone(400, 30);
            ev3_speaker_play_tone(500, 40);
            ev3_speaker_play_tone(600, 50);
            ev3_speaker_play_tone(700, 60);
            */

            clock->sleep(4);
    }


    // 180?��x?��?��?��

    /*
    W�Ƃ�

    i = 0;
    while (1) {
        if (i++ >= 700) { break; }

            ev3_motor_set_power(EV3_PORT_B, -10);
            ev3_motor_set_power(EV3_PORT_C, 10);

            tail_control(60);
            clock->sleep(4);
    }
    */


    style_change(65, 100);
    style_change(70, 100);
    style_change(75, 100);
    style_change(80, 100);


    style_change(83, 100);
    style_change(85, 400);

    /*
    style_change(86, 400);
    style_change(87, 400);
    style_change(88, 400);
    */

    i = 0;
    while(1) {
        if (i++ >= 10) { break; }
        tail_control(85);
        ev3_led_set_color(LED_RED);
        clock->sleep(4);
    }
    ev3_led_set_color(LED_OFF);

    /*
    while (1) {
        if (i++ >= 1500) { break; }

            //ev3_motor_set_power(EV3_PORT_B, 10);
            //  ev3_motor_set_power(EV3_PORT_C, 10);
            ev3_speaker_play_tone(300, 20);
            ev3_speaker_play_tone(400, 30);
            ev3_speaker_play_tone(500, 40);
            ev3_speaker_play_tone(600, 50);
            ev3_speaker_play_tone(700, 60);
            tail_control(90);
            clock->sleep(4);
    }
    */



    // ?��y?��Ƀv?��?��?��O?��?��?��~?��?��?��O?��s?��?��?��悤

    /*
    i = 0;
    while (1) {
        if ( i++ >= 1000 )
        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);
        tail_control(0);
    }
    */

    /*
    clock->sleep(4000);
    */
}

//*****************************************************************************
// ?��֐�?��?�� : style_change
// ?��?��?��?�� :
// ?��Ԃ�?��l :
// ?��T?��v : lookup gate
//
//*****************************************************************************

static void style_change(int32_t angle, int32_t time)
{
    int32_t i = 0;
    while (1) {
        if (i++ >= time) { break; }

            ev3_motor_set_power(EV3_PORT_B, 0);
            ev3_motor_set_power(EV3_PORT_C, 0);

            tail_control(angle);

            clock->sleep(4);
    }
}

//*****************************************************************************
// ?��֐�?��?�� : garage_stop
// ?��?��?��?�� :
// ?��Ԃ�?��l :
// ?��T?��v : Garage
//
//*****************************************************************************

static void garage_stop(void)
{
    //int8_t forward;
    //int8_t turn;
    // int8_t pwm_L, pwm_R;

    int8_t i;
    int8_t j = 0;

    // 方向転換
    for ( i = 0; i < 90; i++ ) {
        ev3_motor_set_power(EV3_PORT_B, 10);
        ev3_motor_set_power(EV3_PORT_C, 5);
        ev3_speaker_play_tone(300, 20);
        tail_control(80);
        clock->sleep(4);
    }

    while(1)
    {
        j++;

        // int32_t motor_ang_l, motor_ang_r;
        // int32_t gyro, volt;

        //if (ev3_button_is_pressed(BACK_BUTTON)) break;
        // if (touchSensor->isPressed()) { break; }

        tail_control(80); /* ?��o?��?��?��?��?��X?��?��?��s?��p?��p?��x?��ɐ�?��?�� */

        //forward = 30; /* ?��O?��i?��?��?��?�� */

        if (colorSensor->getBrightness() >= ((LIGHT_WHITE + LIGHT_BLACK)/2)-10)
        {
          ev3_motor_set_power(EV3_PORT_B, 5);
          ev3_motor_set_power(EV3_PORT_C, 10);
        }
        else
        {
          ev3_motor_set_power(EV3_PORT_B, 10);
          ev3_motor_set_power(EV3_PORT_C, 5);
        }


        if ( j % 1500 == 0 ) {
            for ( i = 0; i < 90; i++ ) {
                  ev3_motor_set_power(EV3_PORT_B, 0);
                  ev3_motor_set_power(EV3_PORT_C, 0);
                  tail_control(75);
                  ev3_led_set_color(LED_RED );

                  clock->sleep(4);
              }
              j+=10;
        }


        ev3_led_set_color(LED_ORANGE );

        /*

        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (int8_t *)&pwm_L,
            (int8_t *)&pwm_R
        );

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

        */

        clock->sleep(4); /* 4msec?��?��?��?��?��N?��?�� */
    }
}
