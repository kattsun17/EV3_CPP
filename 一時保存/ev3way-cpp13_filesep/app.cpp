/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子黒線追跡プログラム
 **
 ** 注記 : sample_cpp
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
static int32_t   bt_cmd = 0;      /* Bluetoothコマンド１ : リモートスタート */
static FILE     *bt = NULL;      /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
// #define LIGHT_WHITE          25  /* 白色の光センサ値 */
// #define LIGHT_BLACK           1  /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE  5  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  94  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      0  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例定数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAME?��Őݒ� */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODE?��Őݒ� */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* 輝度PID制御のパラメタ(旋回制御) */
#define DELTA_T 0.004  // 走行制御の動作周期
#define KP 1        // Pパラメタ
#define KI 1         // Iパラメタ
#define KD 1        // Dパラメタ

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* 関数プロトタイプ宣言 */
static int32_t sonar_alert(void);

static void tail_control(int32_t angle);
static void lookup_strategy(void);
static void style_change(int32_t angle, int32_t time_ms);
static void garage_stop(void);
static void figure_strategy(void);
static void figure_strategy2(void);
static void limited_line_trace(int8_t forward, int16_t time);
static void straight_limited_line_trace(int8_t forward, int16_t time);
static void tailfree_straight_limited_line_trace(int8_t forward, int16_t time);
static void moving_style_change(int32_t current_angle, int32_t angle, int32_t time, int32_t motor);
static void calib_strategy(void);

/* pid制御関連の大域宣言 */
static int32_t diff[2];    // 目標値と測定値の差分の累積値
static int32_t integral;

/* 光センサから得た値　*/
static int32_t light_black;
static int32_t light_white;
static int32_t light_gray;

/* オブジェクトへのポインタ定義 */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;

/* メインタスク */
void main_task(intptr_t unused)
{
    int8_t forward;      /* 前後進命令 */
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    //int8_t flag_lookup = 0;  /* ルックアップゲート用関数のフラグ */
    int8_t flag_figure = 0;  /* フィギュアL用関数のフラグ */
    int32_t figure_count = 0;
    //int8_t flag_garage = 0;  /* ガレージ用関数のフラグ */
    int8_t p, i, d;

    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    colorSensor = new ColorSensor(PORT_3);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);


    /* 電圧をみる */

    char lcdstr[100];
  	sprintf(lcdstr, "%4d mV", ev3_battery_voltage_mV());
  	ev3_lcd_draw_string(lcdstr, 30, 50);

    // キャリブレーション
    calib_strategy();




    /* 尻尾モータのリセット */
    tailMotor->reset();

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

        if (bt_cmd == 1)
        {
            break; /* リモートスタート */
        }

        if (touchSensor->isPressed())
        {
            /*
            ev3_speaker_play_tone(300, 20);
            ev3_speaker_play_tone(400, 30);
            ev3_speaker_play_tone(500, 40);
            ev3_speaker_play_tone(600, 50);
            ev3_speaker_play_tone(700, 60);
            */
            break; /* タッチセンサが押された */
        }

        clock->sleep(10);
    }

    /* 走行モータエンコーダリセット */
    leftMotor->reset();
    rightMotor->reset();

    /* ジャイロセンサリセット */
    gyroSensor->reset();
    balance_init(); /* 倒立振子API初期化 */

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    /* Main loop for the self-balance control algorithm */
    while(1)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

        /* フィギュアL検知による攻略動作 */
        if ( flag_figure == 0 ) {
            if ( ev3_gyro_sensor_get_rate(EV3_PORT_4) > 100 || ev3_gyro_sensor_get_rate(EV3_PORT_4) < -100 ) {
                flag_figure = 2;
                //ev3_speaker_play_tone(600, 50);
            }
        }

        if ( flag_figure > 1 ) {
            //figure_count++;
            figure_count = 1000;
        }

        if ( figure_count > 500 ) {
            ev3_speaker_play_tone(600, 20);
            figure_strategy2();
            flag_figure = 0;
        }


        forward = 30; /* 前進命令 */

        //ev3_speaker_play_tone(300, 10);

        diff[0] = diff[1];
        diff[1] = (colorSensor->getBrightness()) - ((light_white + light_black)/2);
        integral += ( diff[1] + diff[0] ) / 2.0 * DELTA_T;
        // 比例
        p = KP * diff[1];
        // 微分
        i = KI * integral;
        // 積分
        d = KD * (diff[1] - diff[0]) / DELTA_T;

        turn = p + i + d;
        if ( p + i + d > 100.0 ) turn = 100.0;
        if ( p + i + d < -100.0 ) turn = -100.0;

        /* 倒立振子制御APIに渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る */
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

        clock->sleep(4); /* 4msec周期起動 */
    }

    leftMotor->reset();
    rightMotor->reset();

    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int32_t sonar_alert(void)
{
    static uint32_t counter = 0;
    static int32_t alert = 0;

    int32_t distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知 */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します.
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です.
         * EV3の場合は、要確認.
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle(モータ目標角度[度])
// 返り値 : 無し
// 概要 : 奏功体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(int32_t angle)
{
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* 比例制御 */
    /* PWM出力総和処理 */
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
// 関数名 : bt_task
// 引数 : unused
// 返り値 : 無し
// 概要 : Bluetooth通信によるリモートスタート。Tera Termなどのターミナルソフトから、
//        ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}

//*****************************************************************************
// 関数名 : lookup_strategy
// 引数 : なし
// 返り値 : なし
// 概要 : ルックアップゲート攻略手順
//*****************************************************************************
void lookup_strategy(void)
{
    /* 変数宣言 */
    int32_t i;
    int32_t forward = 0;
    int32_t turn = 0;
    //int32_t ddeff;
    int32_t gyro;
    int32_t volt;
    int32_t motor_ang_l, motor_ang_r;
    int8_t pwm_L, pwm_R;

    //int32_t oldang_l, oldang_r;

    /* 4msec周期50回の間に、走行体の角度を79度に変更 */
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

    /* 体勢をゆっくり後ろに倒し、60度にする */
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

    /* デバッグ */
    ev3_speaker_play_tone(300, 20);
    ev3_speaker_play_tone(400, 30);
    ev3_speaker_play_tone(500, 40);
    ev3_speaker_play_tone(600, 50);
    ev3_speaker_play_tone(700, 60);

    motor_ang_l = ev3_motor_get_counts(EV3_PORT_B);
    motor_ang_r = ev3_motor_get_counts(EV3_PORT_C);
    int32_t lpower = 10;
    int32_t rpower = 10;

    /* 4msec周期500回で、前進する(ゲート通過) */
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
ｓ            clock->sleep(4);
    }
    */


    style_change(65, 100);
    style_change(70, 100);
    style_change(75, 100);
    style_change(80, 100);
    style_change(83, 100);
    style_change(85, 400);

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
// 関数名 : style_change
// 引数 : angle(モータ目標角度[度]), time(4msecでの処理繰り返し回数)
// 返り値 : なし
// 概要 : 指定の角度に指定の時間で移行する
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
// 関数名 : garage_stop
// 引数 : なし
// 返り値 : なし
// 概要 : ガレージ攻略
//*****************************************************************************

static void garage_stop(void)
{
    //int8_t forward;
    //int8_t turn;
    //int8_t pwm_L, pwm_R;

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

        if (colorSensor->getBrightness() >= ((light_white + light_black)/2)-10)
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

//*****************************************************************************
// 関数名 : figure_strategy
// 引数 : なし
// 返り値 : なし
// 概要 : フィギュアL攻略手順
//*****************************************************************************

static void figure_strategy (void)
{
    ev3_led_set_color(LED_ORANGE);
    //ev3_speaker_play_tone(300,400);
    // clock->sleep(400000);

    /* 変数宣言 */
    int32_t i;
    int32_t forward = 50;
    int32_t turn = 0;
    //int32_t ddeff;
    int32_t gyro;
    int32_t volt;
    int32_t motor_ang_l, motor_ang_r;
    int8_t pwm_L, pwm_R;

    //int32_t oldang_l, oldang_r;

    /* 4msec周期50回の間に、走行体の角度を79度に変更 */
    i = 0;
    while (1) {
        if (i++ >= 20) { break; }

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
            (float)-40, // ?��W?��?��?��C?��?��?��I?��t?��Z?��b?��g?��l
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

    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);
        tail_control(79);
        clock->sleep(4);

    }

    ev3_speaker_play_tone(300, 20);
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 5);
        ev3_motor_set_power(EV3_PORT_B, 0);
        tail_control(79);
        clock->sleep(4);
    }
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 0);
        ev3_motor_set_power(EV3_PORT_B, 5);
        tail_control(79);
        clock->sleep(4);
    }
    //ev3_speaker_play_tone(300, 20);


    // 両輪で上げる
    /*
    i = 0;
    while (1) {
        if (i++ >= 400) { break; }
        ev3_motor_set_power(EV3_PORT_C, 20);
        ev3_motor_set_power(EV3_PORT_B, 20);
        tail_control(85);
        clock->sleep(4);

    }
    */

    // 片輪づつ上げる
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 20);
        ev3_motor_set_power(EV3_PORT_B, 0);
        tail_control(85);
        clock->sleep(4);

    }

    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 0);
        ev3_motor_set_power(EV3_PORT_B, 20);
        tail_control(85);
        clock->sleep(4);

    }

    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 5);
        ev3_motor_set_power(EV3_PORT_B, 5);
        tail_control(85);
        clock->sleep(4);

    }

    // しっぽ立てる
    /*
    style_change(86, 100);
    style_change(87, 100);
    style_change(88, 100);
    style_change(89, 100);
    style_change(90, 100);
    style_change(91, 100);
    style_change(92, 100);
    style_change(93, 100);
    style_change(94, 10000);
    */
    i = 0;
    while (1) {
        if (i++ >= 300) { break; }
        ev3_led_set_color(LED_RED);
        ev3_motor_set_power(EV3_PORT_C, 0);
        ev3_motor_set_power(EV3_PORT_B, 0);
        tail_control(85);
        ev3_led_set_color(LED_ORANGE);
        clock->sleep(4);

    }



    // タイヤを後ろに回しながら、姿勢制御
    moving_style_change(85, 110, 600, -1);

    ev3_speaker_play_tone(300, 20);

     moving_style_change(110, 40, 90, 100);

     ev3_speaker_play_tone(300, 20);


    i = 0;
    while (1) {
        if (i++ >= 100) { break; }
        ev3_motor_set_power(EV3_PORT_C, 100);
        ev3_motor_set_power(EV3_PORT_B, 100);
        tail_control(85);
        clock->sleep(4);

    }

    i = 0;
    while (1) {
        if (i++ >= 1000) { break; }
        ev3_motor_set_power(EV3_PORT_C, 0);
        ev3_motor_set_power(EV3_PORT_B, 0);
        tail_control(85);
        clock->sleep(4);

    }








    /*
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, -10);
        ev3_motor_set_power(EV3_PORT_B, -10);
        tail_control(90);
        clock->sleep(4);

    }


    i = 0;
    while (1) {
        if (i++ >= 20) { break; }
        ev3_motor_set_power(EV3_PORT_C, 100);
        ev3_motor_set_power(EV3_PORT_B, 100);
        tail_control(100);
        clock->sleep(4);

    }
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 100);
        ev3_motor_set_power(EV3_PORT_B, 100);
        tail_control(80);
        clock->sleep(4);

    }


    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 0);
        ev3_motor_set_power(EV3_PORT_B, 0);
        tail_control(90);
        clock->sleep(4);


    }
    */






    // 停止

    while (1) {

        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);
        tail_control(0);
        clock->sleep(4);

    }
}

//*****************************************************************************
// 関数名 : figure_strategy2
// 引数 :
// 返り値 :
// 概要 :
//*****************************************************************************

static void figure_strategy2(void)
{

    /* 変数宣言 */
    int32_t i;
    int32_t forward = 0;
    int32_t turn = 0;
    //int32_t ddeff;
    //int32_t gyro;
    //int32_t volt;
    //int32_t motor_ang_l, motor_ang_r;
    int8_t pwm_L, pwm_R;

    // 回転角の一時保管用係数
    int32_t motor_angle;

    // straight_limited_line_trace(0, 500);

    // のぼらせる

    motor_angle = leftMotor->getCount();

    while (1) {

        if ( leftMotor->getCount() - motor_angle > 500 ) { break; }

        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

        forward = 30; /* 前進命令 */
        turn = 0;

        /* 倒立振子制御APIに渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る */
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

        clock->sleep(4); /* 4msec周期起動 */
    }

    ev3_speaker_play_tone(300, 20);

    // しっぽ立てる
    i = 0;
    while (1) {
        if (i++ >= 100) { break; }
        ev3_motor_set_power(EV3_PORT_B, 30);
        ev3_motor_set_power(EV3_PORT_C, 30);
        tail_control(79);
        clock->sleep(4);
    }

    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);
        tail_control(79);
        clock->sleep(4);
    }


    i = 0;
    while (1) {
        if (i++ >= 400) { break; }
        ev3_motor_set_power(EV3_PORT_B, -30);
        ev3_motor_set_power(EV3_PORT_C, 30);
        tail_control(79);
        clock->sleep(4);
    }

    // ライン復帰
    i = 0;
    while (1) {

        if (i++ >= 700) { break; }

        if (colorSensor->getBrightness() >= ((light_white + light_black)/2))
        {
          ev3_motor_set_power(EV3_PORT_B, 10);
          ev3_motor_set_power(EV3_PORT_C, 0);
        }
        else
        {
          ev3_motor_set_power(EV3_PORT_B, 0);
          ev3_motor_set_power(EV3_PORT_C, 10);
        }
        tail_control(79);
        clock->sleep(4);

    }

    // バランス走行にフッキ
    i = 0;
    while (1) {
        if (i++ >= 50) { break; }
        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);
        tail_control(90);
        clock->sleep(4);
    }

    limited_line_trace(20, 10000);






    while (1) {

        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);
        tail_control(79);
        clock->sleep(4);

    }

    return;
}

//*****************************************************************************
// 関数名 : limited_line_trace
// 引数 :
// 返り値 :
// 概要 :
//*****************************************************************************

static void limited_line_trace(int8_t forward, int16_t time)
{
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    int8_t p, i, d;      /* PiD 変数 */

    int16_t count; // 反復変数

    for(count = 0; count < time; count++)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

        forward = 30; /* 前進命令 */

        // PiD control
        diff[0] = diff[1];
        diff[1] = (colorSensor->getBrightness()) - ((light_white + light_black)/2);
        integral += ( diff[1] + diff[0] ) / 2.0 * DELTA_T;
        // 比例
        p = KP * diff[1];
        // 微分
        i = KI * integral;
        // 積分
        d = KD * (diff[1] - diff[0]) / DELTA_T;

        turn = p + i + d;
        if ( p + i + d > 100.0 ) turn = 100.0;
        if ( p + i + d < -100.0 ) turn = -100.0;

        /* 倒立振子制御APIに渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る */
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

        clock->sleep(4); /* 4msec周期起動 */
    }

    return;
}

//*****************************************************************************
// 関数名 : straight_limited_line_trace
// 引数 :
// 返り値 :
// 概要 :
//*****************************************************************************

static void straight_limited_line_trace(int8_t forward, int16_t time)
{
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */


    int16_t count; // 反復変数

    for(count = 0; count < time; count++)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

        forward = 30; /* 前進命令 */
        turn = 0;

        /* 倒立振子制御APIに渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る */
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

        clock->sleep(4); /* 4msec周期起動 */
    }

    return;
}

//*****************************************************************************
// 関数名 : tailfree_straight_limited_line_trace
// 引数 :
// 返り値 :
// 概要 :
//*****************************************************************************

static void tailfree_straight_limited_line_trace(int8_t forward, int16_t time)
{
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    //int8_t p, i, d;      /* PiD 変数 */

    int16_t count; // 反復変数

    for(count = 0; count < time; count++)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        // tail_control(TAIL_ANGLE_DRIVE);

        forward = 30; /* 前進命令 */
        turn = 0;

        /* 倒立振子制御APIに渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る */
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

        clock->sleep(4); /* 4msec周期起動 */
    }

    return;
}

//*****************************************************************************
// 関数名 : moving_style_change
// 引数 : angle(モータ目標角度[度]), time(4msecでの処理繰り返し回数), motor(pwm?)
// 返り値 : なし
// 概要 : 動きながら指定の角度に指定の時間で移行する
//*****************************************************************************

static void moving_style_change(int32_t current_angle, int32_t angle, int32_t time, int32_t motor)
{
    int32_t i;
    int32_t tmp;

    for (i = 1; i < time; i++) {
        ev3_motor_set_power(EV3_PORT_B, motor);
        ev3_motor_set_power(EV3_PORT_C, motor);

        tmp = current_angle + ((angle - current_angle) * i / time);
        tail_control(tmp);

        clock->sleep(4);
    }
}

//*****************************************************************************
// 関数名 : calib_strategy
// 引数 :
// 返り値 :
// 概要 : キャリブレーションということで3つの値をとる
//*****************************************************************************

static void calib_strategy()
{
    int i;
    int tmp[100];
    int sum ;
    int flag = 0;

    // white
    while (1) {
        if (touchSensor->isPressed()) { break; }
        ev3_lcd_draw_string("calib white (^^)/", 30, 90);
        clock->sleep(4);
    }
    for ( i = 0; i < 100; i++ ) {
        tmp[i] = colorSensor->getBrightness();
        ev3_speaker_play_tone(300, 20);
        clock->sleep(4);
    }
    sum = 0;
    for ( i = 0; i < 100; i++ ) {
        sum += tmp[i];
    }
    light_white = sum / 100;

    // glay
    while (1) {
        if (touchSensor->isPressed()) { break; }
        ev3_lcd_draw_string("calib gray (^^)/", 30, 90);
        clock->sleep(4);
    }
    for ( i = 0; i < 100; i++ ) {
        tmp[i] = colorSensor->getBrightness();
        ev3_speaker_play_tone(300, 20);
        clock->sleep(4);
    }
    sum = 0;
    for ( i = 0; i < 100; i++ ) {
        sum += tmp[i];
    }
    light_gray = sum / 100;

    // black
    while (1) {
        if (touchSensor->isPressed()) { break; }
        ev3_lcd_draw_string("calib black (^^)/", 30, 90);
        clock->sleep(4);
    }
    for ( i = 0; i < 100; i++ ) {
        tmp[i] = colorSensor->getBrightness();
        ev3_speaker_play_tone(300, 20);
        clock->sleep(4);
    }
    sum = 0;
    for ( i = 0; i < 100; i++ ) {
        sum += tmp[i];
    }
    light_black = sum / 100;

    if ( light_white > light_gray ) {
        if ( light_gray > light_black ) {
            flag = 5;
        }
    }

    // キャリブレーション失敗
    i = 0;
    if ( flag < 3 ) {
        while ( 1 ) {
            if (i++ > 30) { break; }
            ev3_lcd_draw_string("calib error( ;∀;)", 30, 90);
            ev3_led_set_color(LED_RED);
            clock->sleep(200);
            ev3_led_set_color(LED_GREEN);
        }
        light_white = 25;
        light_black = 1;
        light_gray = 12;
    } else {
        ev3_lcd_draw_string("calib success!(^^)!", 30, 90);
    }


}
