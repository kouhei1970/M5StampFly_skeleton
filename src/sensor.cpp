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

#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "stampfly.hpp"


Madgwick ahrs;
INA3221 ina3221(INA3221_ADDR40_GND);  // Set I2C address to 0x40 (A0 pin -> GND)

// Sensor data
float ax, ay, az, gx, gy, gz;
float roll_rate_offset, pitch_rate_offset, yaw_rate_offset;
float sensor[16];
uint16_t offset_counter = 0;

uint8_t scan_i2c() {
    USBSerial.println("I2C scanner. Scanning ...");
    delay(50);
    byte count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        Wire1.beginTransmission(i);        // Begin I2C transmission Address (i)
        if (Wire1.endTransmission() == 0)  // Receive 0 = success (ACK response)
        {
            USBSerial.print("Found address: ");
            USBSerial.print(i, DEC);
            USBSerial.print(" (0x");
            USBSerial.print(i, HEX);
            USBSerial.println(")");
            count++;
        }
    }
    USBSerial.print("Found ");
    USBSerial.print(count, DEC);  // numbers of devices
    USBSerial.println(" device(s).");
    return count;
}

void test_voltage(void) {
    for (uint16_t i = 0; i < 1000; i++) {
        USBSerial.printf("Voltage[%03d]:%f\n\r", i, ina3221.getVoltage(INA3221_CH2));
    }
}

void ahrs_reset(void) {
    ahrs.reset();
}

void sensor_init() {
    Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);
    if (scan_i2c() == 0) {
        USBSerial.printf("No I2C device!\r\n");
        USBSerial.printf("Can not boot AtomFly2.\r\n");
        while (1);
    }
    tof_init();
    imu_init();
    ina3221.begin(&Wire1);
    ina3221.reset();
    ahrs.begin(400.0);

    uint16_t cnt = 0;
    while (cnt < 10) {
        if (ToF_bottom_data_ready_flag) {
            ToF_bottom_data_ready_flag = 0;
            cnt++;
            USBSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());
        }
    }
    delay(10);
    USBSerial.printf("Finish sensor init!\r\n");
}

void sensor_calc_offset_avarage(void) {
    roll_rate_offset  = (offset_counter * roll_rate_offset  + gx) / (offset_counter + 1);
    pitch_rate_offset = (offset_counter * pitch_rate_offset + gy) / (offset_counter + 1);
    yaw_rate_offset   = (offset_counter * yaw_rate_offset   + gz) / (offset_counter + 1);
    offset_counter++;
}

void sensor_read(sensor_value_t* data) {
    float acc_x, acc_y, acc_z, roll_rate, pitch_rate, yaw_rate;
    float roll_angle, pitch_angle, yaw_angle;
    static uint16_t bottom_tof_range=0;
    float voltage;

    // 以下では航空工学の座標軸の取り方に従って
    // X軸：前後（前が正）左肩上がりが回転の正
    // Y軸：右左（右が正）頭上げが回転の正
    // Z軸：下上（下が正）右回りが回転の正
    // となる様に軸の変換を施しています
    // BMI270の座標軸の撮り方は
    // X軸：右左（右が正）頭上げが回転の正
    // Y軸：前後（前が正）左肩上がりが回転の正
    // Z軸：上下（上が正）左回りが回転の正

    // Get IMU raw data
    imu_update();  // IMUの値を読む前に必ず実行
    ax  =   imu_get_acc_y();
    ay  =   imu_get_acc_x();
    az  = - imu_get_acc_z();
    gx =   imu_get_gyro_y();
    gy =   imu_get_gyro_x();
    gz = - imu_get_gyro_z();

    if (StampFly.flag.mode > AVERAGE_MODE) {
        acc_x = ax;
        acc_y = ay;
        acc_z = az;
        roll_rate  = gx - roll_rate_offset;
        pitch_rate = gy - pitch_rate_offset;
        yaw_rate   = gz - yaw_rate_offset;

        ahrs.updateIMU( roll_rate  * (float)RAD_TO_DEG, 
                        pitch_rate * (float)RAD_TO_DEG,
                        yaw_rate   * (float)RAD_TO_DEG, 
                        -acc_x, 
                        -acc_y,
                        -acc_z);

        roll_angle  = ahrs.getRoll()  * (float)DEG_TO_RAD;
        pitch_angle = ahrs.getPitch() * (float)DEG_TO_RAD;
        yaw_angle   = ahrs.getYaw()   * (float)DEG_TO_RAD;
    }
    // Battery voltage check
    voltage   = ina3221.getVoltage(INA3221_CH2);

    // Get ToF bottom range
    //bottom_tof_range = 0;
    if (ToF_bottom_data_ready_flag) {
        ToF_bottom_data_ready_flag = 0;
        // 距離の値の更新
        bottom_tof_range = tof_bottom_get_range();
        //USBSerial.printf("%04d\n\r", bottom_tof_range);
    }
    // Set sensor data
    data->bottom_tof_range = bottom_tof_range;

    // set value
    data->accx = acc_x;
    data->accy = acc_y;
    data->accz = acc_z;
    data->roll_rate = roll_rate;
    data->pitch_rate = pitch_rate;
    data->yaw_rate = yaw_rate;
    data->roll_angel = roll_angle;
    data->pitch_angle = pitch_angle;
    data->yaw_angle = yaw_angle;
    data->voltage = voltage;

    return;
}

void bottom_tof_read(sensor_value_t* data) {
    uint16_t range; 
    if (ToF_bottom_data_ready_flag) {
        //dcnt = 0u;
        ToF_bottom_data_ready_flag = 0;
        // 距離の値の更新
        range = tof_bottom_get_range();
        USBSerial.printf("%04d\n\r", range);
    }
    data->bottom_tof_range = range;   
}
