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
 * @Hardwares: M5StickCPlus + Hat TMOS
 * @Platform Version: Arduino M5Stack Board Manager v2.1.1
 * @Dependent Library:
 * M5_STHS34PF80 https://github.com/m5stack/M5_STHS34PF80
 */
#include <M5StickCPlus.h>
#include "M5_STHS34PF80.h"

#define INTERRUP_MODE  /// Enable interrupt mode

M5_STHS34PF80 TMOS;

uint8_t motionHysteresis = 0;
int16_t motionVal = 0, presenceVal = 0;
uint16_t motionThresholdVal = 0, precenceThresholdVal = 0;
sths34pf80_gain_mode_t gainMode;

#ifdef INTERRUP_MODE
bool volatile interruptFlag = false;
int intPin                  = 36;
// ISR to set the triggered interrupt
void isr1() {
    interruptFlag = true;
}
#endif

void setup() {
    M5.begin();
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(50, 10);
    M5.Lcd.println("Hat-TMOS TEST");
    Serial.println("Hat-TMOS TEST");
    M5.Lcd.setCursor(120, 35);
    M5.Lcd.setTextColor(WHITE);

    // Initialise I2C and devices
    while (TMOS.begin(&Wire, STHS34PF80_I2C_ADDRESS, 0, 26) == false) {
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

#ifdef INTERRUP_MODE
    // Set INT pin to be triggered on rising and falling edges of INT pin
    pinMode(intPin, INPUT);
    // Attach interrupt to the pin as a digital pin that triggers on a change
    attachInterrupt(digitalPinToInterrupt(intPin), isr1, CHANGE);
    // Route all interrupts from device to interrupt pin
    TMOS.setTmosRouteInterrupt(STHS34PF80_TMOS_INT_OR);
    // Enable the presence and motion interrupt source
    // (see page 17 of application note AN5867 for more information)
    TMOS.setTmosInterruptOR(STHS34PF80_TMOS_INT_ALL);
    // Set interrupt value to pulsed on the INT pin
    TMOS.setInterruptPulsed(1);
#endif

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
#ifndef INTERRUP_MODE
    // Real-time detection of presence and motion infrared energy values
    sths34pf80_tmos_drdy_status_t dataReady;
    TMOS.getDataReady(&dataReady);
    if (dataReady.drdy == 1) {
#else
    // Interrupt detection of presence and motion infrared energy values
    if (interruptFlag == true) {
        interruptFlag = false;
#endif

        sths34pf80_tmos_func_status_t status;
        TMOS.getPresenceValue(&presenceVal);
        TMOS.getMotionValue(&motionVal);
        TMOS.getStatus(&status);

        M5.Lcd.fillRect(106, 48, 80, 20, BLACK);
        M5.Lcd.setCursor(2, 50);
        M5.Lcd.printf("Presence:%d", presenceVal);

        M5.Lcd.fillRect(167, 70, 100, 20, BLACK);
        M5.Lcd.setCursor(2, 70);
        M5.Lcd.printf("PrescenceFlag:%s", status.pres_flag ? "Det" : "None");

        M5.Lcd.fillRect(85, 88, 60, 20, BLACK);
        M5.Lcd.setCursor(2, 90);
        M5.Lcd.printf("Motion:%d", motionVal);

        M5.Lcd.fillRect(167, 110, 100, 20, BLACK);
        M5.Lcd.setCursor(2, 110);
        M5.Lcd.printf("MotionValFlag:%s", status.mot_flag ? "Det" : "None");

        // Output only if presence/motion is detected
        if (status.pres_flag == 1) {
            TMOS.getPresenceValue(&presenceVal);
            Serial.printf("Prescence Detected!  PrescenceValue:%d\n", presenceVal);
        }

        if (status.mot_flag == 1) {
            TMOS.getMotionValue(&motionVal);
            Serial.printf("Motion Detected!  MotionValue:%d\n", motionVal);
        }
    }
}