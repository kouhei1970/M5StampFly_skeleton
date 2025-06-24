/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//
// 2024-08-11 StampFly 自己開発用のスケルトンプログラム制作開始

#include "main_loop.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "button.hpp"
#include "buzzer.h"
#include "stampfly.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


volatile uint8_t armButtonState = 0;
volatile uint8_t armButtonPressedAndRerleased = 0;
volatile uint8_t previousArmButtonState = 0; 

void IRAM_ATTR onTimer(void);
void init_copter(void);
void update_loop400Hz(void);
void init_mode(void);
void average_mode(void);
void flight_mode(void);
void parking_mode(void);
void loop_400Hz(void);
float limit(float value, float min, float max);
float deadband(float value, float deadband);

// Main loop
void loop_400Hz(void) {
    // 400Hzで以降のコードが実行

    update_loop400Hz();
    
    // Mode select
    if (StampFly.flag.mode == INIT_MODE) 
        init_mode();
    else if (StampFly.flag.mode == AVERAGE_MODE)
        average_mode();
    else if (StampFly.flag.mode == FLIGHT_MODE)
        flight_mode();
    else if (StampFly.flag.mode == PARKING_MODE)
        parking_mode();

    //// Telemetry
    telemetry();
    StampFly.flag.oldmode = StampFly.flag.mode;  // Memory now mode
    
    // End of Loop_400Hz function    
}

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer(void) {
    StampFly.flag.loop = 1;
    //loop_400Hz();
}

// Initialize StampFly
void init_copter(void) {
    //disableCore1WDT();
    // Initialize Mode
    StampFly.flag.mode = INIT_MODE;
    StampFly.flag.loop = 0;
    // Initialaze LED function
    led_init();
    // Initialize Serial communication
    USBSerial.begin(115200);
    delay(1500);
    USBSerial.printf("Start StampFly! Skeleton\r\n");
    motor_init();
    sensor_init();
    rc_init();

    // init button G0
    init_button();
    setup_pwm_buzzer();
    USBSerial.printf("Finish StampFly init!\r\n");
    start_tone();

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);
    timerAlarmEnable(timer);

}

//loop400Hzの更新関数
void update_loop400Hz(void) {
    uint32_t now_time;

    while (StampFly.flag.loop == 0);
    StampFly.flag.loop = 0;

    #if 0
    USBSerial.printf("%9.4f %9.4f %04d\n\r", 
        StampFly.times.elapsed_time, 
        StampFly.times.interval_time,
        StampFly.sensor.bottom_tof_range);
    #endif

    //Clock
    now_time = micros();
    StampFly.times.old_elapsed_time = StampFly.times.elapsed_time;
    StampFly.times.elapsed_time = 1e-6 * (now_time - StampFly.times.start_time);
    StampFly.times.interval_time = StampFly.times.elapsed_time - StampFly.times.old_elapsed_time;
    
    // Read Sensor Value
    sensor_read(&StampFly.sensor);
    
    // LED Drive
    led_drive();

    // Read Button Value
    armButtonState = Stick[BUTTON_ARM];
    if (armButtonState != previousArmButtonState) {
        if (armButtonState == 0) {
            armButtonPressedAndRerleased = 1;
        }
        previousArmButtonState = armButtonState;
    }

    // Send Angle Rate
    //USBSerial.printf(">RollRate:%f\n", StampFly.sensor.roll_rate);
    //USBSerial.printf(">PitchRate:%f\n", StampFly.sensor.pitch_rate);
    //USBSerial.printf(">YawRate:%f\n", StampFly.sensor.yaw_rate);
}

void init_mode(void) {
    motor_stop();
    StampFly.counter.offset = 0;
    
    //Set PID Gain
    StampFly.pid.roll.set_parameter(0.08, 1.0e1, 0.01, 0.125, 0.0025);
    StampFly.pid.pitch.set_parameter(0.08, 1.0e1, 0.01, 0.125, 0.0025);
    StampFly.pid.yaw.set_parameter(0.08, 1.0e2, 0.0, 0.125, 0.0025);



    //Mode change
    StampFly.flag.mode = AVERAGE_MODE;
    return;

}

void average_mode(void) {
    // Gyro offset Estimate 角速度のオフセットを取得
    // Set LED Color
    onboard_led1(PERPLE, 1);
    onboard_led2(PERPLE, 1);

    if (StampFly.counter.offset < AVERAGENUM) {
        sensor_calc_offset_avarage();
        StampFly.counter.offset++;
        return;
    }
    // Mode change
    StampFly.flag.mode   = PARKING_MODE;
    StampFly.times.start_time = micros();
    StampFly.times.old_elapsed_time = 0.0f;
    return;
}

void flight_mode(void) {
    //飛行するためのコードを以下に記述する
    // Set LED Color
    onboard_led1(YELLOW, 1);
    onboard_led2(YELLOW, 1);

    float throttle_delta = limit(deadband(Stick[THROTTLE], 0.065), 0.0, 0.9);

    //Command
    float roll_com = 60.0*PI/180*limit(deadband(Stick[AILERON], 0.05), -1.0, 1.0);
    float pitch_com = 60.0*PI/180*limit(deadband(Stick[ELEVATOR], 0.05), -1.0, 1.0);
    float yaw_com = 180.0*PI/180*limit(deadband(Stick[RUDDER], 0.05), -1.0, 1.0);
    //USBSerial.printf("Throttle: %f, Roll: %f, Pitch: %f, Yaw: %f\n", 
    //    throttle_delta, roll_com, pitch_com, yaw_com);


    //Error
    float roll_err = roll_com - StampFly.sensor.roll_rate;
    float pitch_err = pitch_com - StampFly.sensor.pitch_rate;
    float yaw_err = yaw_com - StampFly.sensor.yaw_rate;
    //USBSerial.printf("Roll: %9.6f, Pitch: %9.6f, Yaw: %9.6f\n", roll_err, pitch_err, yaw_err);

    //PID
    float roll_delta = 0.1f*roll_err;
    float pitch_delta = 0.1f*pitch_err;
    float yaw_delta = 0.2f*yaw_err;

    //Set Duty
    float fr_duty = limit(throttle_delta - roll_delta + pitch_delta + yaw_delta, 0.0, 0.9);
    float fl_duty = limit(throttle_delta + roll_delta + pitch_delta - yaw_delta, 0.0, 0.9); 
    float rr_duty = limit(throttle_delta - roll_delta - pitch_delta - yaw_delta, 0.0, 0.9);
    float rl_duty = limit(throttle_delta + roll_delta - pitch_delta + yaw_delta, 0.0, 0.9);

    motor_set_duty_fr(fr_duty);
    motor_set_duty_fl(fl_duty);
    motor_set_duty_rr(rr_duty);
    motor_set_duty_rl(rl_duty);

    //Mode change
    if (armButtonPressedAndRerleased)StampFly.flag.mode = PARKING_MODE;
    armButtonPressedAndRerleased = 0;
}

void parking_mode(void) {
    //着陸している時に行う処理を記述する
    // Set LED Color
    onboard_led1(GREEN, 1);
    onboard_led2(GREEN, 1);
    
    //Motor Stop
    motor_stop();

    //Reset PID
    StampFly.pid.roll.reset();
    StampFly.pid.pitch.reset();
    StampFly.pid.yaw.reset();

    //Send range sensor value by USB Serial
    //USBSerial.printf("Bottom ToF Range: %04d\n", StampFly.sensor.bottom_tof_range);

    //Mode change
    if (armButtonPressedAndRerleased)StampFly.flag.mode = FLIGHT_MODE;
    armButtonPressedAndRerleased = 0;
}

//Limit function
float limit(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

//Dead band function
float deadband(float value, float deadband) {
    if (value < deadband && value > -deadband) return 0.0;
    return value;
}