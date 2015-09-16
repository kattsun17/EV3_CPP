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

// Bluetooth
static int32_t   bt_cmd = 0;      // Bluetoothコマンド１ : リモートスタート
static FILE     *bt = NULL;       // Bluetoothファイルハンドル

// 下記のマクロは個体/環境に合わせて変更する必要があります
#define GYRO_OFFSET           0   // ジャイロセンサオフセット値(角速度0[deg/sec]時)
#define SONAR_ALERT_DISTANCE  13   // 超音波センサによる障害物検知距離[cm]
#define TAIL_ANGLE_STAND_UP  100   // 完全停止時の角度[度]
#define TAIL_ANGLE_DRIVE      0   // バランス走行時の角度[度]
#define P_GAIN             2.5F   // 完全停止用モータ制御比例定数
#define PWM_ABS_MAX          60   // 完全停止用モータ制御PWM絶対最大値
//#define DEVICE_NAME     "ET0"  // Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAME?��Őݒ�
//#define PASS_KEY        "1234" // パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODE?��Őݒ�
#define CMD_START         '1'     // リモートスタートコマンド

// 輝度PID制御のパラメタ(旋回制御)
#define DELTA_T 0.004  // 走行制御の動作周期
// 本文中に kp ki kdで
/*
#define KP 0.36          // Pパラメタ
#define KI 1.2           // Iパラメタ
#define KD 0.027           // Dパラメタ
*/



// LCDフォントサイズ
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6)
#define CALIB_FONT_HEIGHT (8)

// 区間の境界の位置
#define S1 5913
#define C1 1811
#define S2 937
#define C2C3 2486
#define S3 912

// 関数プロトタイプ宣言
static int32_t sonar_alert(void); // ソナーセンサで障害物を検知する
static void tail_control(int32_t angle); // 尻尾モータの角度を固定する
static void lookup_strategy(void); // ルックアップゲートを攻略する
static void style_change(int32_t angle, int32_t time_ms); // 指定した時間をかけて、尻尾モータを動かす
static void garage_stop(void); // 駐車時の動作(現状ルックアップゲートのあと用)
static void figure_strategy(void); // フィギュアLを膠着する
static void figure_strategy2(void); // フィギュアLを攻略する(現在こちらを採用)
static void limited_line_trace(int8_t forward, int16_t time); // 指定した時間ラインtのレースする
static void straight_limited_line_trace(int8_t forward, int16_t time); // 指定した時間turn値0でライントレースする
static void tail_limited_line_trace(int8_t forward, int16_t c_angle, int16_t angle, int16_t num); // 指定した時間turn値0でライントレースする。このとき尻尾のモータをd
static void moving_style_change(int32_t current_angle, int32_t angle, int32_t time, int32_t motor); // タイヤを動かしながら尻尾モータを動かす
static void calib_strategy(void); // キャリブレーションを行う
static void stop_style_change(int16_t angle, int16_t time); // 走行状態から倒立状態へ移行

// 大域宣言

// pid制御関連の大域宣言
static int32_t diff[2];    // 目標値と測定値の差分の累積値
static int32_t integral;

static float kp = 0.36F;
static float ki = 1.2F;
static float kd = 0.027F;

// 光センサから得た値
static int32_t light_black;
static int32_t light_white;
static int32_t light_gray;

// オブジェクトへのポインタ定義
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;

// メインタスク
void main_task(intptr_t unused)
{
    int8_t forward;      // 前後進命令
    int8_t turn;         // 旋回命令
    int8_t pwm_L, pwm_R; // 左右モータPWM出力
    int8_t flag_lookup = 0;  // ルックアップゲート用関数のフラグ
    int8_t flag_figure = 0;  // フィギュアL用関数のフラグ
    int32_t figure_count = 0;
    //int8_t flag_garage = 0;  // ガレージ用関数のフラグ
    int8_t p, i, d;
    // pidゲイン値
    //int32_t kp, kii, kd;
    int32_t tail_reset = 0;

    // 各オブジェクトを生成・初期化する
    touchSensor = new TouchSensor(PORT_1);
    colorSensor = new ColorSensor(PORT_3);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();

    // LCD画面表示 "コード名"
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);


    // LCD画面表示 "電圧"
    char lcdstr[100];
  	sprintf(lcdstr, "%4d mV", ev3_battery_voltage_mV());
  	ev3_lcd_draw_string(lcdstr, 30, 50);

    // キャリブレーション
    calib_strategy();

    // 尻尾モータの遊びをなくすため、巻き込む
    while  (1) {
        ev3_motor_set_power(EV3_PORT_A, -10);
        clock->sleep(4);
        ev3_speaker_play_tone(700, 60);
        tail_reset++;
        if ( tail_reset > 300) { break; }
    }



    // 尻尾モータのリセット
    tailMotor->reset();



    // Open Bluetooth file
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    // Bluetooth通信タスクの起動
    act_tsk(BT_TASK);

    // 初期化完了通知
    ev3_led_set_color(LED_ORANGE);

    // スタート待機
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); // 完全停止用角度に制御

        if (bt_cmd == 1)
        {
            // リモートスタート
            break;
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
            // タッチセンサが押された
            break;
        }

        clock->sleep(10);
    }

    // 走行モータエンコーダリセット
    leftMotor->reset();
    rightMotor->reset();

    // ジャイロセンサリセット
    gyroSensor->reset();

    // 倒立振子API初期化
    balance_init();

    // スタート通知
    ev3_led_set_color(LED_GREEN);

    // バランス走行のループ
    while(1)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        // バックボタンを押したら停止
        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        // バランス走行用角度に制御
        tail_control(TAIL_ANGLE_DRIVE);

        /*
        // フィギュアL検知による攻略動作
        // フィギュアLフラグが0のとき
        if ( flag_figure == 0 ) {
            // ジャイロセンサの絶対値が一定より大きいとき、フィギュアLフラグを立てる
            if ( ev3_gyro_sensor_get_rate(EV3_PORT_4) > 100 || ev3_gyro_sensor_get_rate(EV3_PORT_4) < -100 ) {
                ev3_speaker_play_tone(600, 20);
                figure_strategy2();
                flag_figure = 2;
                //ev3_speaker_play_tone(600, 50);
            }
        }
        */


        // ルックアップゲート検知による攻略動作
        // ルックアップフラグが0のとき、かつソナーセンサが反応したとき
        if (sonar_alert() == 1 && flag_lookup == 0)
        {
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);
            ev3_speaker_play_tone(300, 10);
            flag_lookup = 2;
            lookup_strategy();
        }

        // 前進命令
        // forward = 100;





        // 区間切り分け ( 前進命令をコメントして使う、LコースRコースでそれぞれ使用しない方はコメント )
        // 直線 : 緑、カーブ : 赤、難所 : 橙

        forward = 30;
        kp = 0.91;
        ki = 0.3;
        kd = 0.075;
        ev3_led_set_color(LED_ORANGE);

        /*
        // S1
        if ( motor_ang_r < S1 ) {
            forward = 50;
            kp = 0.36;
            ki = 1.2;
            kd = 0.027;
            ev3_led_set_color(LED_GREEN);
            flag_figure = 2;
        }
        // C1
        else if ( motor_ang_r < S1+C1 ) {
            forward = 50;
            kp = 0.91;
            ki = 0.1;
            kd = 0.075;
            ev3_led_set_color(LED_RED);
        }

        // S2
        else if ( motor_ang_r < S1+C1+S2 ) {
            forward = 50;
            kp = 0.36;
            ki = 1.2;
            kd = 0.027;
            ev3_led_set_color(LED_GREEN);
            flag_figure = 0;
        }
        // C2 C3
        else if ( motor_ang_r < S1+C1+S2+C2C3 ) {
            forward = 50;
            kp = 0.91;
            ki = 0.1;
            kd = 0.075;
            ev3_led_set_color(LED_RED);
        }
        // S3
        else if ( motor_ang_r < S1+C1+S2+C2C3+S3 ) {
            forward = 50;
            kp = 0.36;
            ki = 1.2;
            kd = 0.027;
            ev3_led_set_color(LED_GREEN);
        }
        // 難所周辺
        else {
            forward = 30;
            kp = 0.91;
            ki = 0.1;
            kd = 0.075;
            ev3_led_set_color(LED_ORANGE);
        }
        */

        // PID制御
        diff[0] = diff[1];
        diff[1] = (colorSensor->getBrightness()) - ((light_white + light_black)/2);
        integral += ( diff[1] + diff[0] ) / 2.0 * DELTA_T;
        // 比例
        p = kp * diff[1];
        // 微分
        i = ki * integral;
        // 積分
        d = kd * (diff[1] - diff[0]) / DELTA_T;
        turn = p + i + d;
        if ( p + i + d > 100.0 ) turn = 100.0;
        if ( p + i + d < -100.0 ) turn = -100.0;

        // 倒立振子制御APIに渡すパラメータを取得する
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る
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

        // ライントレース時はセンサの仕様のため4msec周期起動
        clock->sleep(4);
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

    // 約40msec周期毎に障害物検知
    if (++counter == 40/4)
    {
         /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します.
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です.
         * EV3の場合は、要確認.
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            // 障害物を検知
            alert = 1;
        }
        else
        {
            // 障害物無し
            alert = 0;
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
    // 比例制御
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN;
    // PWM出力総和処理
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
        // 受信
        uint8_t c = fgetc(bt);
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        // エコーバック
        fputc(c, bt);
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
    // 変数宣言
    int32_t i;
    int32_t forward = 0;
    int32_t turn = 0;
    //int32_t ddeff;
    int32_t gyro;
    int32_t volt;
    int32_t motor_ang_l, motor_ang_r;
    int8_t pwm_L, pwm_R;

    //int32_t oldang_l, oldang_r;

    // しっぽを出してわずかに後退
    tail_limited_line_trace(-5, TAIL_ANGLE_DRIVE, 90, 1000);

    // 4msec周期50回の間に、走行体の角度を79度に変更

    /*

    i = 0;
    while (1) {
        if (i++ >= 50) { break; }

        tail_control(79);

        // しっぽを出してとまる
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)-30, // ジャイロセンサのターゲット値を偽装して、倒れて止まる状態を
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

    */

    stop_style_change(95, 50);

    // 体勢をゆっくり後ろに倒し、60度にする

    moving_style_change(90, 65, 1000, 0);

    /*
    style_change(77, 500);
    style_change(75, 100);
    style_change(73, 100);
    style_change(71, 100);
    style_change(69, 100);
    style_change(66, 100);
    */

    //style_change(63, 100);
    //style_change(62, 100);
    //style_change(61, 100);
    //style_change(60, 100);

    //oldang_l = leftMotor->getCount();
    //oldang_r = rightMotor->getCount();

    // デバッグ
    ev3_speaker_play_tone(300, 20);
    ev3_speaker_play_tone(400, 30);
    ev3_speaker_play_tone(500, 40);
    ev3_speaker_play_tone(600, 50);
    ev3_speaker_play_tone(700, 60);

    motor_ang_l = ev3_motor_get_counts(EV3_PORT_B);
    motor_ang_r = ev3_motor_get_counts(EV3_PORT_C);
    int32_t lpower = 10;
    int32_t rpower = 10;

    // 4msec周期500回で、前進する(ゲート通過)
    i = 0;
    while (1) {
        if (i++ >= 800) { break; }

            ev3_motor_set_power(EV3_PORT_B, lpower);
            ev3_motor_set_power(EV3_PORT_C, rpower);
            //ev3_motor_steer(EV3_PORT_C, EV3_PORT_B, 10, 0);
            //ev3_motor_rotate(EV3_PORT_C, 10000, 100, false);
            //ev3_motor_rotate(EV3_PORT_B, 10000, 100, false);

            // まっすぐすすませるため
            if ( i % 10 == 0 ) {
                if ( (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l) - (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r) > 0 ) {
                    // ddeff = (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l) - (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r);
                    // for ( k = 0; k < ddeff; k++ ) {   ev3_speaker_play_tone(100, 200); clock->sleep(4); }
                    ev3_speaker_play_tone(100, 200);
                    lpower = 10;
                    rpower = 25;

                    //ev3_motor_rotate(EV3_PORT_C, ddeff, 10, true);
                } else if ( (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l) - (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r) < 0 ) {
                    //ddeff = (ev3_motor_get_counts(EV3_PORT_C) - motor_ang_r) - (ev3_motor_get_counts(EV3_PORT_B) - motor_ang_l);
                    //for ( k = 0; k < ddeff; k++ ) {   ev3_speaker_play_tone(300, 200); clock->sleep(4); }
                    //ev3_motor_rotate(EV3_PORT_B, ddeff, 10, true);
                     ev3_speaker_play_tone(600, 200);
                    lpower = 20;
                    rpower = 20;
                } else {
                    lpower = 10;
                    rpower = 20;
                }

                motor_ang_l = ev3_motor_get_counts(EV3_PORT_B);
                motor_ang_r = ev3_motor_get_counts(EV3_PORT_C);
            }

            tail_control(65);

            /*
            ev3_speaker_play_tone(300, 20);
            ev3_speaker_play_tone(400, 30);
            ev3_speaker_play_tone(500, 40);
            ev3_speaker_play_tone(600, 50);
            ev3_speaker_play_tone(700, 60);
            */

            clock->sleep(4);
    }

    /*
    i = 0;
    while (1) {
        if (i++ >= 700) { break; }

        ev3_motor_set_power(EV3_PORT_B, -10);
        ev3_motor_set_power(EV3_PORT_C, 10);

        tail_control(60);
        clock->sleep(4);
    }
    */

    /*
    style_change(65, 100);
    style_change(70, 100);
    style_change(75, 100);
    style_change(80, 100);
    style_change(83, 100);
    style_change(85, 400);
    */

    moving_style_change(65, 85, 1000, -5);

    i = 0;
    while(1) {
        if (i++ >= 10) { break; }
        tail_control(85);
        ev3_led_set_color(LED_RED);
        clock->sleep(4);
    }
    ev3_led_set_color(LED_GREEN);

    garage_stop();

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
    // 変数宣言
    int32_t i = 0;

    while (1) {
        // timeを超えたらループを出る
        if (i++ >= time) { break; }

        // モータのパワーを0にセット
        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);

        // しっぽモータをangleに固定
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
    // 変数宣言
    // int8_t forward;
    // int8_t turn;
    // int8_t pwm_L, pwm_R;

    int32_t i;       // 反復変数
    int32_t j = 0;   // 反復変数

    // 方向転換
    /*
    for ( i = 0; i < 90; i++ ) {
        ev3_motor_set_power(EV3_PORT_B, 10);
        ev3_motor_set_power(EV3_PORT_C, 5);
        ev3_speaker_play_tone(300, 20);
        tail_control(85);
        clock->sleep(4);
    }
    */

    while(1)
    {
        j++;

        // int32_t motor_ang_l, motor_ang_r;
        // int32_t gyro, volt;

        // if (ev3_button_is_pressed(BACK_BUTTON)) break;
        // if (touchSensor->isPressed()) { break; }

        tail_control(85); // しっぽ

        if (colorSensor->getBrightness() >= ((light_white + light_black)/2)-10)
        {
            ev3_motor_set_power(EV3_PORT_B, 7);
            ev3_motor_set_power(EV3_PORT_C, 10);
        }
        else
        {
            ev3_motor_set_power(EV3_PORT_B, 10);
            ev3_motor_set_power(EV3_PORT_C, 0);
        }

        if ( j > 900 && j % 500 == 0 ) {
            for ( i = 0; i < 1000; i++ ) {
                ev3_motor_set_power(EV3_PORT_B, 0);
                ev3_motor_set_power(EV3_PORT_C, 0);
                tail_control(85);
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

        clock->sleep(4); // 4msec?��?��?��?��?��N?��?��
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
    // ev3_speaker_play_tone(300,400);
    // clock->sleep(400000);

    int32_t forward = 50;
    int32_t turn = 0;
    // int32_t ddeff;
    // 変数宣言
    int32_t i;
    int32_t gyro;
    int32_t volt;
    int32_t motor_ang_l, motor_ang_r;
    int8_t pwm_L, pwm_R;

    // int32_t oldang_l, oldang_r;

    // 4msec周期50回の間に、走行体の角度を79度に変更
    i = 0;
    while (1) {
        if (i++ >= 20) { break; }

        tail_control(79);

        // 倒立振子APIに値を渡す
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)-40, // ジャイロセンサのターゲット値を偽装
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

    // 800msecの間、しっぽモータを79、左右モータを0にセット
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_B, 0);
        ev3_motor_set_power(EV3_PORT_C, 0);
        tail_control(79);
        clock->sleep(4);

    }

    // 800msecの間、しっぽモータを79、左モータを5、右モータを0にセット
    // 後ろに傾いた状態で、ゆっくり右に旋回
    ev3_speaker_play_tone(300, 20);
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 5);
        ev3_motor_set_power(EV3_PORT_B, 0);
        tail_control(79);
        clock->sleep(4);
    }

    // 800msecの間、しっぽモータを79、左モータを0、右モータを5にセット
    // 後ろに傾いた状態で、ゆっくり左に旋回
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 0);
        ev3_motor_set_power(EV3_PORT_B, 5);
        tail_control(79);
        clock->sleep(4);
    }

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
    // 左モータを前に回す
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 20);
        ev3_motor_set_power(EV3_PORT_B, 0);
        tail_control(85);
        clock->sleep(4);
    }

    // 右モータを前に回す
    i = 0;
    while (1) {
        if (i++ >= 200) { break; }
        ev3_motor_set_power(EV3_PORT_C, 0);
        ev3_motor_set_power(EV3_PORT_B, 20);
        tail_control(85);
        clock->sleep(4);
    }

    // 800msecの間、左右モータ5でゆっくり前進
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

    // 1200secの間、しっぽモータ85で、停止
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
    // 2400msecかけて、左右モータのパワー-1で、しっぽモータを85から110へ移行
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
    // 変数宣言
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

        // バランス走行用角度に制御
        tail_control(TAIL_ANGLE_DRIVE);

        forward = 30; // 前進命令
        turn = 0;

        // 倒立振子制御APIに渡すパラメータを取得する
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る
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

        // 4msec周期起動
        clock->sleep(4);
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
    int8_t turn;         // 旋回命令
    int8_t pwm_L, pwm_R; // 左右モータPWM出力
    int8_t p, i, d;      // PiD 変数

    int16_t count;       // 反復変数

    for(count = 0; count < time; count++)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tail_control(TAIL_ANGLE_DRIVE); // バランス走行用角度に制御

        forward = 30; // 前進命令

        // PiD control
        diff[0] = diff[1];
        diff[1] = (colorSensor->getBrightness()) - ((light_white + light_black)/2);
        integral += ( diff[1] + diff[0] ) / 2.0 * DELTA_T;
        // 比例
        p = kp * diff[1];
        // 微分
        i = ki * integral;
        // 積分
        d = kd * (diff[1] - diff[0]) / DELTA_T;

        turn = p + i + d;
        if ( p + i + d > 100.0 ) turn = 100.0;
        if ( p + i + d < -100.0 ) turn = -100.0;

        // 倒立振子制御APIに渡すパラメータを取得する
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る
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

        clock->sleep(4); // 4msec周期起動
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
    int8_t turn;         // 旋回命令
    int8_t pwm_L, pwm_R; // 左右モータPWM出力


    int16_t count; // 反復変数

    for(count = 0; count < time; count++)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        // バランス走行用角度に制御
        tail_control(TAIL_ANGLE_DRIVE);

        forward = 30; // 前進命令
        turn = 0;

        // 倒立振子制御APIに渡すパラメータを取得する
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る
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

        clock->sleep(4); // 4msec周期起動
    }

    return;
}

//*****************************************************************************
// 関数名 : tail_imited_line_trace
// 引数 :
// 返り値 :
// 概要 :
//*****************************************************************************

static void tail_limited_line_trace(int8_t forward, int16_t c_angle, int16_t angle, int16_t num)
{
    int8_t turn;         // 旋回命令
    int8_t pwm_L, pwm_R; // 左右モータPWM出力
    int8_t p, i, d;      // PiD 変数
    int16_t count;       // 反復変数
    int16_t tmp;         // 一時変数、現在周期の目標角度

    for(count = 0; count < num; count++)
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;

        if (ev3_button_is_pressed(BACK_BUTTON)) break;

        tmp = c_angle + ((angle - c_angle) * count / num);
        tail_control(tmp);

        // PiD control
        diff[0] = diff[1];
        diff[1] = (colorSensor->getBrightness()) - ((light_white + light_black)/2);
        integral += ( diff[1] + diff[0] ) / 2.0 * DELTA_T;
        // 比例
        p = kp * diff[1];
        // 微分
        i = ki * integral;
        // 積分
        d = kd * (diff[1] - diff[0]) / DELTA_T;

        turn = p + i + d;
        if ( p + i + d > 100.0 ) turn = 100.0;
        if ( p + i + d < -100.0 ) turn = -100.0;

        // 倒立振子制御APIに渡すパラメータを取得する
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // 倒立振子制御APIを呼び出し、倒立走行するための左右モータ出力地を得る
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

        clock->sleep(4); // 4msec周期起動
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
            ev3_lcd_draw_string("calib error( ;__;)", 30, 90);
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



//*****************************************************************************
// 関数名 : stio_style_change
// 引数 : しっぽの角度、時間
// 返り値 :
// 概要 :
//*****************************************************************************

static void stop_style_change(int16_t angle, int16_t num)
{
    // 変数宣言
    int32_t i;
    int32_t forward = 0;
    int32_t turn = 0;
    //int32_t ddeff;
    int32_t gyro;
    int32_t volt;
    int32_t motor_ang_l, motor_ang_r;
    int8_t pwm_L, pwm_R;

    i = 0;
    while (1) {
        if (i++ >= num) { break; }

        tail_control(angle);

        // しっぽを出してとまる
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)-30, // ジャイロセンサのターゲット値を偽装して、倒れて止まる状態を
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
}
