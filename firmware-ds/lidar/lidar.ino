#include <Arduino.h>
#include <Wire.h>
#include <array>

#include "constants.h"
#include "pins.h"
#include "i2cRepairer.h"

#include "lidar.h"

#include "types.h"

// Add fixer for I2C channels which can break under high frequency interrupts and OLED buffer delays
I2CRepairer i2cRepairer(I2C_SDA, I2C_SCL, LIDAR_FRONT_ADDRESS);

LidarSensor front(LIDAR_FRONT_ADDRESS, TOF_3_GPO);
LidarSensor left(LIDAR_LEFT_ADDRESS, TOF_1_GPO);
LidarSensor right(LIDAR_RIGHT_ADDRESS, TOF_2_GPO);
LIDAR lidar(std::array<LidarSensor*, 3>{&front, &left, &right});

float dt = 0;

unsigned long previous_time = 0;
unsigned long current_time = 0;

unsigned long previous_imu_time = 0;
unsigned long previous_print_time = 0;
unsigned long previous_oled_time = 0;
long previous_count_left = 0;
long previous_count_right = 0;

void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("Beginning setup:");

    Serial.print("Initialising I2C...");
    i2cRepairer.begin();
    Serial.println("\b\b\b [OKAY]");

    Serial.print("Initialising Lidar...");
    lidar.init();
    Serial.println("\b\b\b [OKAY]");
    
    previous_time = micros();

    Serial.println("Setup complete!");
}

void loop() {
    current_time = micros();
    dt = (current_time - previous_time) / 1000000.0f;

    if (dt <= MIN_LOOP_DT_S) return;

    i2cRepairer.update();

    lidar.update();

    uint16_t val = lidar.getReading(LIDAR::Front);
    Serial.print("F: ");
    Serial.println(val);

     val = lidar.getReading(LIDAR::Left);
    Serial.print("L: ");
    Serial.println(val);

     val = lidar.getReading(LIDAR::Right);
    Serial.print("R: ");
    Serial.println(val);

    previous_time = current_time;
}
