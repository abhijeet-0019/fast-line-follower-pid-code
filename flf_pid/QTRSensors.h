#ifndef _QTR_SENSORS_H_
    #define _QTR_SENSORS_H_

    #include <Arduino.h>
    //#include <Serial.h>
    #include "hardware/gpio.h"
    #include "hal.h"
    #include "debug.h"
    #include "utils.h"
    #include "motor.h"

    #define MAX_TIME 4000
    #define NOISE_IN_DATA                     -5000
    #define LINE_NUMBER_ERROR_DUE_TOO_MUCH_NOISE            6000
    #define POSITION_OF_BLACK_BLOCK_DETECTED   5000
    #define NOISE_COUNT_FOR_EXIT                2000
    #define DELAY_10nF_CAPACITOR_CHARGE_uS      500   // DO NOT CHANGE Yes its verified, for given QTR Circuit we have

    const int photoDiodePins[8] = { IR_D1_PIN_NO, IR_D2_PIN_NO, IR_D3_PIN_NO, IR_D4_PIN_NO,
                                    IR_D5_PIN_NO, IR_D6_PIN_NO, IR_D7_PIN_NO, IR_D8_PIN_NO };

    void getSensorArrayValues(unsigned int result[], bool print);

    void performCalibration(bool print);

    void getSensorArrayValuesCalibrated(unsigned int sensorValues[], bool print);
    unsigned char getSensorArrayValuesBinary( bool print);


    int readLine(unsigned int s[], bool printIt); //unsigned char whiteLine = 0)
    int readLineBINARY(unsigned char s, bool printIt);

    void drawLine(int lineNumber);

#endif // _QTR_SENSORS_H_