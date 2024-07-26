/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * @file BasicReadings.ino
 * @author Tinyu(tinyu@m5stack.com)
 * @brief TMOS basic readings
 * @version 0.1
 * @date 2024-07-22
 *
 *
 * @Hardwares: M5Core + Unit TMOS
 * @Platform Version: Arduino M5Stack Board Manager v2.1.1
 * @Dependent Library:
 * M5_STHS34PF80 https://github.com/m5stack/M5_STHS34PF80
 */
#include <M5Stack.h>
#include "M5_STHS34PF80.h"

M5_STHS34PF80 TMOS;

uint8_t motionHysteresis = 0;
int16_t motionVal = 0, presenceVal = 0;
uint16_t motionThresholdVal = 0, precenceThresholdVal = 0;
sths34pf80_gain_mode_t gainMode;

void setup() {
    M5.begin();
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(80, 10);
    M5.Lcd.println("Unit-TMOS TEST");
    Serial.println("Unit-TMOS TEST");
    M5.Lcd.setCursor(120, 35);
    M5.Lcd.setTextColor(WHITE);

    // Initialise I2C and devices
    while (TMOS.begin(&Wire, STHS34PF80_I2C_ADDRESS, 21, 22) == false) {
        M5.Lcd.setCursor(60, 35);
        M5.Lcd.setTextColor(RED);
        M5.Lcd.println("Device init error");
        Serial.println("I2C Error - check I2C Address");
        Serial.println("Error setting up device - please check wiring.");
        delay(200);
    }

    Serial.println("Open the Serial Plotter for graphical viewing");
    TMOS.setTmosODR(STHS34PF80_TMOS_ODR_AT_2Hz);
    // TMOS.setMotionThreshold(0xFF);
    TMOS.setPresenceThreshold(0xC8);  // Default value
    TMOS.setMotionThreshold(0xC8);
    TMOS.setPresenceHysteresis(0x32);
    TMOS.setMotionHysteresis(0x32);

    ////Default mode, may cause inaccurate detection of MotionValFlag bit.
    // TMOS.setGainMode(STHS34PF80_GAIN_DEFAULT_MODE);

    // Decreasing gain mode, detection distance decreases
    TMOS.setGainMode(STHS34PF80_GAIN_WIDE_MODE);
    TMOS.setTmosSensitivity(0xff);
    TMOS.resetAlgo();

    TMOS.getGainMode(&gainMode);
    TMOS.getMotionThreshold(&motionThresholdVal);
    TMOS.getPresenceThreshold(&precenceThresholdVal);
    TMOS.getMotionHysteresis(&motionHysteresis);
    Serial.printf("precenceThresholdVal:%x, motionThresholdVal:%x, motionHysteresis:%x GainMode:%x\n",
                  precenceThresholdVal, motionThresholdVal, motionHysteresis, gainMode);
    delay(1000);
}

void loop() {
    sths34pf80_tmos_drdy_status_t dataReady;
    TMOS.getDataReady(&dataReady);

    if (dataReady.drdy == 1) {
        // Real-time detection of presence and motion infrared energy values
        sths34pf80_tmos_func_status_t status;
        TMOS.getPresenceValue(&presenceVal);
        TMOS.getMotionValue(&motionVal);
        TMOS.getStatus(&status);

        M5.Lcd.fillRect(104, 68, 80, 20, BLACK);
        M5.Lcd.setCursor(0, 70);
        M5.Lcd.printf("Presence:%d", presenceVal);

        M5.Lcd.fillRect(165, 90, 100, 20, BLACK);
        M5.Lcd.setCursor(0, 90);
        M5.Lcd.printf("PrescenceFlag:%s", status.pres_flag ? "Detected" : "None");

        M5.Lcd.fillRect(83, 108, 60, 20, BLACK);
        M5.Lcd.setCursor(0, 110);
        M5.Lcd.printf("Motion:%d", motionVal);

        M5.Lcd.fillRect(165, 130, 100, 20, BLACK);
        M5.Lcd.setCursor(0, 130);
        M5.Lcd.printf("MotionValFlag:%s", status.mot_flag ? "Detected" : "None");

        // Output only if presence/motion is detected
        if (status.pres_flag == 1) {
            TMOS.getPresenceValue(&presenceVal);
            Serial.printf("Prescence Detected!  PrescenceValue:%\n", presenceVal);
        }

        if (status.mot_flag == 1) {
            TMOS.getMotionValue(&motionVal);
            Serial.printf("Motion Detected!  MotionValue:%d\n", motionVal);
        }
    }
}