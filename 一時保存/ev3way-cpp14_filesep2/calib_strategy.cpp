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
