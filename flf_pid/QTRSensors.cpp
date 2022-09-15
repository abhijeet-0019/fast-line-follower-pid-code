#include "QTRSensors.h"

bool isCalibrated = false;
unsigned int calMin[8];
const unsigned int calMax = MAX_TIME;


void getSensorArrayValues(unsigned int result[], bool print)
{
// QTR-8RC line detector (IR LINE TRACKER)
//
// REFERENCE:
//
// https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__gpio.html#gae895be2d3c5af5df460150eafe7858a4
//
// https://shop.mchobby.be/en/motor-robotic/497-detecteur-de-ligne-ir-line-tracker-3232100004979-pololu.html
//

    int index;    
    unsigned int time; //register int time;

    uint32_t quadByte;
    uint32_t quadBytePrevious;
    uint32_t changedBits;

    // int totalChanges =0;


    // On RPi Pico GPIO mArk the pins of interest by "1"
    //
    //       D1, D2, D3, D4, D5, D6, D7, D8
    //        9,  8,  7,  6,  5,  4,  3,  2
    //
    //
    //
    //       0000-0011-1111-1100
    //          0    3    F    C

    const uint32_t mask_PhotoDiodeAndCapPins = 0x03FC;

    for(index=0; index <= 7; index++)  // Initialize result array
        result[index] =  MAX_TIME;     // Assume all Black

    // STEP: Charge the capacitors of the ALL sensors, by applying a voltage to the OUT pin
    gpio_set_dir_out_masked(mask_PhotoDiodeAndCapPins); // Switch all GPIOs in "mask" as OUTPUT
    gpio_set_mask(mask_PhotoDiodeAndCapPins);           // Drive high every GPIO appearing in mask
    delayMicroseconds(DELAY_10nF_CAPACITOR_CHARGE_uS);                              // This will charge all the Capacitors

    // STEP: Count the time taken to discharge the capacitor:
    //       White discharges fast: Lower count
    //       Black discharges slow: High cout or MAX
    //       In other words:
    //         Increment time until this pin gets low again,
    //         If its taking too much time, stop incrementing time, assume it as black
    //       OLD code: NOT relavent now:
    //        while( (digitalRead(photoDiodePins[sno]) != LOW) && (time < MAX_TIME) )
    //            time++;

    gpio_set_dir_in_masked(mask_PhotoDiodeAndCapPins); // Switch all GPIOs in "mask" as INPUT
    quadByte = gpio_get_all(); // READ All input pins

    quadByte = quadByte & mask_PhotoDiodeAndCapPins; // Keep only pins of interest, Zero-out others

                              // Pin 0 and 1 are of UART
    quadByte = quadByte >> 2; // After this shift we get sensor values properly arranged from MSB to LSB
    quadBytePrevious = quadByte;

    // STEP: Switch ON IR LEDs
    digitalWrite(IR_ON_PIN_NO, HIGH);  // HIGH = ON

    time = 0;
    while (time < MAX_TIME)
    {
        time++;

        quadByte = gpio_get_all();
        quadByte = quadByte & mask_PhotoDiodeAndCapPins;

        quadByte = quadByte >> 2;

        if( quadByte == quadBytePrevious)
            continue;

        changedBits = quadByte ^ quadBytePrevious; // XOR: All changed bits becoms 1, unchanged bits becomes 0

        index =7;
        while(changedBits) // For all changed bits: Save the result
        {
            if(changedBits & 0x0001)
            {
               // STEP: Save Result for the n'th sensor:
               result[index] = time;
            }
            changedBits = changedBits >> 1;
            index--;
        }

        quadBytePrevious = quadByte;
    }

    // STEP: Switch OFF IR LEDs
    digitalWrite(IR_ON_PIN_NO, LOW);

    // IMPORTANT: 
    // Keep the cps on charging for next use
    // Making the pins as o/p will introduce less noise in uC !
    // STEP: Charge the capacitors of the ALL sensors, by applying a voltage to the OUT pin
    gpio_set_dir_out_masked(mask_PhotoDiodeAndCapPins); // Switch all GPIOs in "mask" as OUTPUT
    gpio_set_mask(mask_PhotoDiodeAndCapPins);           // Drive high every GPIO appearing in mask
//    delayMicroseconds(10);                              // This will charge all the Capacitors


    // STEP: Display Result
    if(print)
    {   
        DUMP_SA_a("\r\n");
        for(index=7; index >=0; index--)
        {
            DUMP_SA_abcd(" D", index+1, " = ", result[index]);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////
unsigned char getSensorArrayValuesBinary( bool print)
{

    int index;    
    unsigned int time; //register int time;
    unsigned char sensorByte;

    uint32_t quadByte;
    uint32_t quadBytePrevious;
    uint32_t changedBits;

    // On RPi Pico GPIO mArk the pins of interest by "1"
    //
    //       D1, D2, D3, D4, D5, D6, D7, D8
    //        9,  8,  7,  6,  5,  4,  3,  2
    //
    //
    //
    //       0000-0011-1111-1100
    //          0    3    F    C

    const uint32_t mask_PhotoDiodeAndCapPins = 0x03FC;

    // STEP: Charge the capacitors of the ALL sensors, by applying a voltage to the OUT pin
    gpio_set_dir_out_masked(mask_PhotoDiodeAndCapPins); // Switch all GPIOs in "mask" as OUTPUT
    gpio_set_mask(mask_PhotoDiodeAndCapPins);           // Drive high every GPIO appearing in mask
    delayMicroseconds(DELAY_10nF_CAPACITOR_CHARGE_uS);                              // This will charge all the Capacitors
    //delay(1);

    // STEP: Count the time taken to discharge the capacitor:
    //       White discharges fast: Lower count
    //       Black discharges slow: High cout or MAX
    //       In other words:
    //         Increment time until this pin gets low again,
    //         If its taking too much time, stop incrementing time, assume it as black
    //       OLD code: NOT relavent now:
    //        while( (digitalRead(photoDiodePins[sno]) != LOW) && (time < MAX_TIME) )
    //            time++;

    gpio_set_dir_in_masked(mask_PhotoDiodeAndCapPins); // Switch all GPIOs in "mask" as INPUT
    quadByte = gpio_get_all(); // READ All input pins

    quadByte = quadByte & mask_PhotoDiodeAndCapPins; // Keep only pins of interest, Zero-out others

                              // Pin 0 and 1 are of UART
    quadByte = quadByte >> 2; // After this shift we get sensor values properly arranged from MSB to LSB
    quadBytePrevious = quadByte;

    // STEP: Switch ON IR LEDs
    digitalWrite(IR_ON_PIN_NO, HIGH);  // HIGH = ON

    sensorByte =0;
    time = 0;
    while (time < MAX_TIME) ///// 2* MAX_TIME as it runs too fast, so take some extra time to observe the photo-diodes 
    {
        time++;

        quadByte = gpio_get_all();
        quadByte = quadByte & mask_PhotoDiodeAndCapPins;

        quadByte = quadByte >> 2;

        if( quadByte == quadBytePrevious)
            continue;

        changedBits = quadByte ^ quadBytePrevious; // XOR: All changed bits becoms 1, unchanged bits becomes 0

        index =7;
        while(changedBits) // For all changed bits: Save the result
        {
            if(changedBits & 0x0001)
            {
               // STEP: Save Result for the n'th sensor:
               sensorByte = sensorByte | ( 1 << index);
            }
            changedBits = changedBits >> 1;
            index--;
        }

        quadBytePrevious = quadByte;
    }

    // Black is the HEIGHER VALUE so Black is One
    // BLACK is ONE (White is Zero)
    sensorByte = ~sensorByte;

    // STEP: Switch OFF IR LEDs
    digitalWrite(IR_ON_PIN_NO, LOW);

    // IMPORTANT: 
    // Keep the cps on charging for next use
    // Making the pins as o/p will introduce less noise in uC !
    // STEP: Charge the capacitors of the ALL sensors, by applying a voltage to the OUT pin
    gpio_set_dir_out_masked(mask_PhotoDiodeAndCapPins); // Switch all GPIOs in "mask" as OUTPUT
    gpio_set_mask(mask_PhotoDiodeAndCapPins);           // Drive high every GPIO appearing in mask
//    delayMicroseconds(10);                              // This will charge all the Capacitors


    // STEP: Display Result
    if(print)
    {   
        DUMP_SA_aBIN("\r\nSA= ", sensorByte)
    }

return sensorByte;
}
//////////////////////////////////////////////////////////////////////////////////
#define MIN_ALLOWED_READING 50 // NOISE free minimum VALUE of reading

////////////////////////////////////////////////////////////////////////////
void performCalibration(bool print)
{
    const int noSamples     = 20;
    const int motor_speed_during_calibration   = 70;
    const int samplingDelay = 25;
          unsigned int samples[8];
    
    int i, j;
    for(j=0; j<=7; j++)
    {
        calMin[j] = MAX_TIME;
    }

    DUMP_SA_a("\r\n performCalibration()");

        DUMP_MO_a("\r\n Turn Left");
        moveMotors(motor_speed_during_calibration, -motor_speed_during_calibration, print);
        for(i=0; i<= noSamples; i++)
        {
            getSensorArrayValues(&samples[0], print);
            for(j=0; j<= 7; j++) // For all 8 Sensors in the QTR Sensor Array
            {
                if( samples[j] < calMin[j] && samples[j] > MIN_ALLOWED_READING) // if currently acquired sensor value is lower than calMin
                    calMin[j] = samples[j]; // then save it                       
            }
            delay(samplingDelay);
        }
        

        DUMP_MO_a("\r\n STOP Both Motors");
        moveMotors(0, 0, print);
        delay(1500);

        DUMP_MO_a("\r\n Turn Right");
        moveMotors(-motor_speed_during_calibration, motor_speed_during_calibration, print);
        for(i=0; i<= noSamples; i++)
        {
            getSensorArrayValues(&samples[0], print);
            for(j=0; j<= 7; j++) // For all 8 Sensors in the QTR Sensor Array
            {
                if( samples[j] < calMin[j] && samples[j] > MIN_ALLOWED_READING) // if currently acquired sensor value is lower than calMin
                    calMin[j] = samples[j]; // then save it                       
            }
            delay( (samplingDelay * 2) ); // 180 degree
        }
                
        DUMP_MO_a("\r\n STOP Both Motors");
        moveMotors(0, 0, print);
        delay(1500);

        DUMP_MO_a("\r\n Turn Left and stop at center");
        moveMotors(motor_speed_during_calibration, -motor_speed_during_calibration, print);
        for(i=0; i<= noSamples; i++)
        {
            getSensorArrayValues(&samples[0], print);
            for(j=0; j<= 7; j++) // For all 8 Sensors in the QTR Sensor Array
            {
                if( samples[j] < calMin[j] && samples[j] > MIN_ALLOWED_READING) // if currently acquired sensor value is lower than calMin
                    calMin[j] = samples[j]; // then save it                       
            }
            delay(samplingDelay);
        }

        DUMP_MO_a("\r\n STOP Both Motors");
        moveMotors(0, 0, print);
        delay(1500);

        DUMP_MO_a("\r\n Finally, the MINIMUM values, which will be used for calibration are:\r\n");
        for(j=7; j>= 0; j--) // For all 8 Sensors in the QTR Sensor Array
            {
                DUMP_SA_abcd(" M", j+1, "= ", calMin[j]); 
            }

    isCalibrated = true; // Indicate globally that the QTR Sensors are now calibrated.
    DUMP_SA_a("\r\n Calibration constants for QTR Sensors are ready.");

}
////////////////////////////////////////////////////////////////////////////
void getSensorArrayValuesCalibrated(unsigned int sensorValues[], bool print)
{
    int i;

    if(isCalibrated == false)
    {

      DUMP_SA_a("\r\n ERROR: Sensors must be calibrated before using: getSensorArrayValuesCalibrated()");
      DUMP_SA_a("\r\n        Modify your program to call: performCalibration()");

           hang("\r\n ERROR: Sensors must be calibrated before using: getSensorArrayValuesCalibrated()");
    }

    getSensorArrayValues(&sensorValues[0], print);

    uint16_t denominator;
    int16_t value;

    for(i=0; i<=7; i++)
    {
        denominator = calMax - calMin[i];


        value = 0;
        if (denominator != 0)
        {
          value = (( (int32_t) sensorValues[i]) - calMin[i]) * 1000 / denominator;
        }

        if (value < 0) { value = 0; }
        else if (value > 1000) { value = 1000; }

        sensorValues[i] = value;


    }

    // STEP: Display Result
    if(print)
    {   
        DUMP_SA_a("\r\n Normalized Values:\r\n");
        for(i=7; i >=0; i--)
        {
            DUMP_SA_abcd(" N", i+1, " = ", sensorValues[i]);
        }
    }

}
////////////////////////////////////////////////////////////////////////////////
int readLine(unsigned int s[], bool printIt) //unsigned char whiteLine = 0)
{

    //// FOR NOW, I AM NOT USING POLOLU's METHOD ##############################################################################
                                            //
                                            // REFERENCE: https://www.pololu.com/docs/0J19/3
                                            //
                                            // Support » Arduino Library for the Pololu QTR Reflectance Sensors »
                                            // 3. QTRSensors Methods & Usage Notes
                                            //
                                            // This function returns an estimated position of the line.
                                            // The estimate is made using a weighted average of the sensor indices multiplied by 1000,
                                            // so that a return value of 0 indicates that the line is directly below sensor 0
                                            // (or was last seen by sensor 0 before being lost),
                                            // a return value of 1000 indicates that the line is directly below sensor 1,
                                            // 2000 indicates that it’s below sensor 2, etc.
                                            // Intermediate values indicate that the line is between two sensors.
                                            // The formula is:
                                            //
                                            //  0*value0 + 1000*value1 + 2000*value2 + ...
                                            // --------------------------------------------
                                            //     value0  +  value1  +  value2 + ...
                                            //
                                            // This function/ method remembers where it last saw the line,
                                            // so if you ever lose the line to the left or the right, it’s
                                            // line position will continue to indicate the direction you need to go to reacquire the line.
                                            // For example,
                                            // if sensor 4 is your rightmost sensor and you end up completely off the line to the left,
                                            // this function will continue to return 4000.
                                            //
//#################################################################
//
//   FOR NOW, I AM USING AN EASY TO UNDERSTAND METHOD, OF MY OWN:-
//
//   ( This function is NOT made for efficiency purpose, 
//   but designed for easy to understanding )
//
//#################################################################

    int i;
    int lineNumber;

           int position;
    static int pastValue; // Due to "static" key word, it rembembers the value even after the function returns
    static int noise;
    
    int noOfBlacks;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//               
//                             Left                                            Right                                           
//                     
//      |      |      |      | D8   | D7   | D6   | D5   | D4   | D3   | D2   | D1   |
//      |      |      |      |      |      |      |      |      |      |      |      |
// S[]= |      |      |      |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |     
//      |      |      |      |      |      |      |      |      |      |      |      |____________________|  1200 : OFF LEFT
//      |      |      |      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |      |      |      |____________________|         1100 : D1
//      |      |      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |      |      |____________________|                1000 : D2 D1
//      |      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |      |____________________|                        900 : D3 D2 D1
//      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |____________________|                               800
//      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |____________________|                                      700
//      |      |      |      |      |      |                    |
//      |      |      |      |      |      |____________________|                                             600
//      |      |      |      |      |                    |
//      |      |      |      |      |____________________|                                                    500
//      |      |      |      |                    |
//      |      |      |      |____________________|                                                           400
//      |      |      |                    |
//      |      |      |____________________|                                                                  300
//      |      |                    |
//      |      |____________________|                                                                         200
//      |                    |
//      |____________________|                                                                                100
//
// P=          100    200    300    400    500    600    700    800    900    1000   1100    1200    
//                     
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(noise > NOISE_COUNT_FOR_EXIT)
    {
        noise=0;
        DUMP_L1_a("\r\n ERROR: TOO MANY NOISE SAMPLES");
        return POSITION_OF_BLACK_BLOCK_DETECTED; 
    }

    noOfBlacks = 0;
    for(i=0 ; i<=7; i++)
        if(s[i] == MAX_TIME)
            noOfBlacks++;
    if(printIt)
        {DUMP_SA_ab("\r\n noOfBlacks= ", noOfBlacks);}

    switch(noOfBlacks)
    {
        case 0:                   // LOST THE TRACK

                if(pastValue <= 200)
                  { //pastValue = 100;
                    return pastValue;
                  }

                if(pastValue >=  1100)
                  { //pastValue = 1200;
                    return pastValue;
                  }
                return pastValue;
                //break;
    
        case 1: 
                if(s[0] == MAX_TIME)
                    {   position  = 200;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[7] == MAX_TIME)
                    {   position  = 1100;
                        pastValue = position; noise=0;
                        return position;
                     }
                DUMP_SA_a("1N"); noise++;
                return NOISE_IN_DATA;
                //break;
    
        case 2:
                if(s[0] == MAX_TIME && s[1] == MAX_TIME)
                    {   position  = 300;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1000;
                        pastValue = position; noise=0;
                        return position;
                     }
                DUMP_SA_a("2N"); noise++;
                return NOISE_IN_DATA;
                //break;
    
        case 3:
                if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME)
                    {   position  = 400;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME)
                    {   position  = 500;
                        pastValue = position; noise=0; 
                        return position;
                     }
                if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME)
                    {   position  = 600;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME)
                    {   position  = 700;
                        pastValue = position; noise=0; 
                        return position;
                     }
                if(s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME)
                    {   position  = 800;
                        pastValue = position; noise=0; 
                        return position;
                     }
                if(s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 900;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("3N"); noise++;
                 return NOISE_IN_DATA;
                 //break;
    
        case 4:
                if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME)
                    {   position  = 450;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME)
                    {   position  = 550;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME)
                    {   position  = 650;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME)
                    {   position  = 750;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 850;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("4N"); noise++;
                 return NOISE_IN_DATA;
                 //break;
    
        case 5:
                if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME)
                    {   position  = 100; //0;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME) // NOISE
                    {   //position  = ;
                        //pastValue = position; 
                        noise=0;
                        return pastValue;
                     }
                if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME) // NOISE
                    {   //position  = ;
                        //pastValue = position; 
                        noise=0;
                        return pastValue;
                     }
                if(s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1200; //1300;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("5N"); noise++;
                 return NOISE_IN_DATA;
                 //break;

    
        case 6:
                if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME)
                    {   position  = 100; //0;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME) // NOISE
                    {   //position  = ;
                        //pastValue = position; 
                        noise=0;
                        return pastValue;
                     }
                if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1200; //1300;
                        pastValue = position; 
                        noise=0;
                        return position;
                     }
                 DUMP_SA_a("6N"); noise++;
                 return NOISE_IN_DATA;
                 //break;
    
        case 7:
                if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME)
                    {   position  = 100; //0;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1200; //1300;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("7N"); noise++;
                 return NOISE_IN_DATA;
                 //break;
    
        case 8:
                 //position  = ;
                 //pastValue = position; 
                 return POSITION_OF_BLACK_BLOCK_DETECTED; 
                 //break;
    
        default:
                 DUMP_L1_a("\r\n WARNING: readLine: default case");
                 return NOISE_IN_DATA;
    
    } // switch

/////////////////////////////////////////////////////////////////////////

    return pastValue;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
int readLineBINARY(unsigned char s, bool printIt) //unsigned char whiteLine = 0)
{
                          //
//#################################################################
//
//   FOR NOW, I AM USING AN EASY TO UNDERSTAND METHOD, OF MY OWN:-
//
//   ( This function is NOT made for efficiency purpose, 
//   but designed for easy to understanding )
//
//#################################################################

    int i;
    int lineNumber;

           int position;
    static int pastValue; // Due to "static" key word, it rembembers the value even after the function returns
    static int noise=0;
    
    int noOfBlacks;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//               
//                             Left                                            Right                                           
//                     
//      |      |      |      | D8   | D7   | D6   | D5   | D4   | D3   | D2   | D1   |
//      |      |      |      |      |      |      |      |      |      |      |      |
// S[]= |      |      |      |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |     
//      |      |      |      |      |      |      |      |      |      |      |      |____________________|  1200 : OFF LEFT
//      |      |      |      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |      |      |      |____________________|         1100 : D1
//      |      |      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |      |      |____________________|                1000 : D2 D1
//      |      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |      |____________________|                        900 : D3 D2 D1
//      |      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |      |____________________|                               800
//      |      |      |      |      |      |      |                    |
//      |      |      |      |      |      |      |____________________|                                      700
//      |      |      |      |      |      |                    |
//      |      |      |      |      |      |____________________|                                             600
//      |      |      |      |      |                    |
//      |      |      |      |      |____________________|                                                    500
//      |      |      |      |                    |
//      |      |      |      |____________________|                                                           400
//      |      |      |                    |
//      |      |      |____________________|                                                                  300
//      |      |                    |
//      |      |____________________|                                                                         200
//      |                    |
//      |____________________|                                                                                100
//
// P=          100    200    300    400    500    600    700    800    900    1000   1100    1200    
//                     
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(noise > NOISE_COUNT_FOR_EXIT)
    {
        noise=0;
        DUMP_L1_a("\r\n ERROR: TOO MANY NOISE SAMPLES");
        return LINE_NUMBER_ERROR_DUE_TOO_MUCH_NOISE; 
    }

    noOfBlacks = 0;  // Find the number of Ones in the Byte
    for(i=0x01 ; i<=0x80; i<<=1)
        if(s & i)
            noOfBlacks++;
    // Black is represented as 1 in the input sebsor byte

    if(printIt)
        {DUMP_SA_ab("\r\n noOfBlacks= ", noOfBlacks);}

    switch(noOfBlacks)
    {
        case 0:                   // LOST THE TRACK
                if(pastValue <= 200)
                  { //pastValue = 100;
                    return pastValue;
                  }

                if(pastValue >= 1100)
                  { //pastValue = 1200;
                    return pastValue;
                  }
                return pastValue;
    
        case 1: 
                if( s & 0x01 ) // 0b0000-0001 // if(s[0] == MAX_TIME)
                    {   position  = 200;
                        pastValue = position; noise=0;
                        return position;
                     }
                if(s & 0x80) // 0b1000-000 // if(s[7] == MAX_TIME)
                    {   position  = 1100;
                        pastValue = position; noise=0;
                        return position;
                     }
                DUMP_SA_a("1N"); noise++;
                return NOISE_IN_DATA;
    
        case 2:
                if((s & 0x03) == 0x03) // 0b0000-0011 // if(s[0] == MAX_TIME && s[1] == MAX_TIME)
                    {   position  = 300;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0xC0) == 0xC0) // 0b1100-0000 // if(s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1000;
                        pastValue = position; noise=0;
                        return position;
                     }
                DUMP_SA_a("2N"); noise++;
                return NOISE_IN_DATA;
    
        case 3:
                if((s & 0x07) == 0x07) // 0b0000-0111 // if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME)
                    {   position  = 400;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0x0E) == 0x0E) // 0b0000-1110 // if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME)
                    {   position  = 500;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0x1C) == 0x1C ) // 0b0001-1100 // if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME)
                    {   position  = 600;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0x38) == 0x38 ) // 0b0011-1000 // if(s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME)
                    {   position  = 700;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0x70) == 0x70 ) // 0b0111-0000 // if(s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME)
                    {   position  = 800;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0xE0) == 0xE0 ) // 0b1110-0000 // if(s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 900;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("3N"); noise++;
                 return NOISE_IN_DATA;
    
        case 4:
                if((s & 0x0F) == 0x0F ) // 0b0000-1111 // if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME)
                    {   position  = 450;
                        pastValue = position; noise=0; 
                        return position;
                     }
                if((s & 0x1E) == 0x1E ) // 0b0001-1110 // if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME)
                    {   position  = 550;
                        pastValue = position; noise=0; 
                        return position;
                     }
                if((s & 0x3C) == 0x3C ) // 0b0011-1100 // if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME)
                    {   position  = 650;
                        pastValue = position; noise=0; 
                        return position;
                     }
                if((s & 0x78) == 0x78 ) // 0b0111-1000 // if(s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME)
                    {   position  = 750;
                        pastValue = position; noise=0; 
                        return position;
                     }
                if((s & 0xF0) == 0xF0 ) // 0b1111-0000 // if(s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 850;
                        pastValue = position; noise=0; 
                        return position;
                     }
                 DUMP_SA_a("4N"); noise++;
                 return NOISE_IN_DATA;
    
        case 5:
                if((s & 0x1F) == 0x1F ) // 0b0001-1111 // if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME)
                    {   position  = 100; //0;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0x3E) == 0x3E) // 0b0011-1110// if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME) // NOISE
                    {   //position  = ;
                        //pastValue = position; 
                        noise=0;
                        return pastValue;
                     }
                if((s & 0x7C) == 0x7C ) // 0b0111-1100 // if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME) // NOISE
                    {   //position  = ;
                        //pastValue = position; 
                        noise=0;
                        return pastValue;
                     }
                if((s & 0xF8) == 0xF8 ) // 0b1111-1000 // if(s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1200; //1300;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("5N"); noise++;
                 return NOISE_IN_DATA;

    
        case 6:
                if((s & 0x3F) == 0x3F ) // 0b0011-1111 // if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME)
                    {   position  = 100; //0;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0x7E) == 0x7E ) // 0b0111-1110 // if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME) // NOISE
                    {   //position  = ;
                        //pastValue = position; 
                        noise=0;
                        return pastValue;
                     }
                if((s & 0xFC) == 0xFC ) // 0b1111-1100 // if(s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1200; //1300;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("6N"); noise++;
                 return NOISE_IN_DATA;
    
        case 7:
                if((s & 0x7F) == 0x7F ) // 0b0111-1111 // if(s[0] == MAX_TIME && s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME)
                    {   position  = 100; //0;
                        pastValue = position; noise=0;
                        return position;
                     }
                if((s & 0xFE) == 0xFE ) // 0b1111-1110 // if(s[1] == MAX_TIME && s[2] == MAX_TIME && s[3] == MAX_TIME && s[4] == MAX_TIME && s[5] == MAX_TIME && s[6] == MAX_TIME && s[7] == MAX_TIME)
                    {   position  = 1200; //1300;
                        pastValue = position; noise=0;
                        return position;
                     }
                 DUMP_SA_a("7N"); noise++;
                 return NOISE_IN_DATA;
    
        case 8:
                 return POSITION_OF_BLACK_BLOCK_DETECTED; 
    
        default:
                 DUMP_L1_a("\r\n WARNING: readLine: default case");
                 return NOISE_IN_DATA;
    
    } // switch

    DUMP_SA_a("\r\n ############ CHECK ###############");
    return pastValue;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
void drawLine(int lineNumber)
{
   int s,n, errNo;
   char errStr[20];
      
   //DUMP_L1_ab("\r\n lineNumber= ",        lineNumber);
   
   n = (lineNumber-100)/20;
   //DUMP_L1_ab("\r\n n = (lineNumber-100)/20= ", n);
   

        DUMP_L1_a("\r\n");
        DUMP_L1_a("\r\n                | LEFT                            RIGHT |");
        DUMP_L1_a("\r\n |    |    |    | D8 | D7 | D6 | D5 | D4 | D3 | D2 | D1 |    |    |    |");
        DUMP_L1_a("\r\n |    |    |    |    |    |    |    |    |    |    |    |    |    |    |");
        DUMP_L1_a("\r\n |    |    |    |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |    |    |    |");
        DUMP_L1_a("\r\n ");


        if(lineNumber == POSITION_OF_BLACK_BLOCK_DETECTED)
            {DUMP_L1_a("               #########################################");
             DUMP_L1_a("\r\n                        BLACK BLOCK DETECTED");
             
             return;
             }
        
        errNo = (1300/2) -  lineNumber;



        for(s=1; s<=n; s++)
            {  DUMP_L1_a(" "); }
        DUMP_L1_a("|______________|");


        DUMP_L1_a("\r\n ");
        for(s=1; s<=n; s++)
            { DUMP_L1_a(" ");  }

        sprintf(errStr, "|### E=%4d ###|", errNo);
        DUMP_L1_a(errStr);

}
//////////////////////////////////////////////////////////////////////////////
//int readLine(unsigned int sensorValues[]) //unsigned char whiteLine = 0)
//{
////
//// REFERENCE: https://www.pololu.com/docs/0J19/3
////
//// Support » Arduino Library for the Pololu QTR Reflectance Sensors »
//// 3. QTRSensors Methods & Usage Notes
////
//// This function returns an estimated position of the line.
//// The estimate is made using a weighted average of the sensor indices multiplied by 1000,
//// so that a return value of 0 indicates that the line is directly below sensor 0
//// (or was last seen by sensor 0 before being lost),
//// a return value of 1000 indicates that the line is directly below sensor 1,
//// 2000 indicates that it’s below sensor 2, etc.
//// Intermediate values indicate that the line is between two sensors.
//// The formula is:
////
////  0*value0 + 1000*value1 + 2000*value2 + ...
//// --------------------------------------------
////     value0  +  value1  +  value2 + ...
////
//// This function/ method remembers where it last saw the line,
//// so if you ever lose the line to the left or the right, it’s
//// line position will continue to indicate the direction you need to go to reacquire the line.
//// For example,
//// if sensor 4 is your rightmost sensor and you end up completely off the line to the left,
//// this function will continue to return 4000.
////
//
//    int index, numerator =0, denominator=0;
//    int oneThousand = 1000;
//    int lineNumber;
//    static int pastValue; // Due to "static" key word, it rembembers the value even after the function returns
//                          // Study more on "static"
//
////    DUMP_SA_a("\r\n N= ");
////    for(index=0; index <= 7; index++)  //
////    {
////      numerator   = numerator   + (index * oneThousand * sensorValues[index] );
////      DUMP_SA_ab( (index * oneThousand * sensorValues[index] ), " + ");
////    }
////    //DUMP_SA_ab("\r\n----------------");
////    DUMP_SA_ab("\r\n Total: numerator=", numerator);
////
////    DUMP_SA_a("\r\n D= ");
////    for(index=0; index <= 7; index++)  //
////    {
////        denominator = denominator +                 sensorValues[index]  ;
////         DUMP_SA_ab( sensorValues[index], " + ");
////    }
////    //DUMP_SA_ab("\r\n----------------");
////    DUMP_SA_ab("\r\n Total: denominator=", denominator);
////
////   lineNumber =  numerator / denominator;
////   DUMP_SA_a("\r\n");
///////////////////////////////////////////////////////////////////////////
//  DUMP_SA_a("\r\n N= ");
//    for(index=7; index >= 0; index--)  //
//    {
//      numerator   = numerator   + ((index+1)  * sensorValues[index] );
//      DUMP_SA_ab( ((index+1) * sensorValues[index] ), " + ");
//    }
//    //DUMP_SA_a("\r\n----------------");
//    //DUMP_SA_ab("\r\n Total: numerator=", numerator);
//
//   lineNumber =  numerator;// / denominator;
//   DUMP_SA_a("\r\n");
///////////////////////////////////////////////////////////////////////////
//  // STEP: Display Result
//   DUMP_SA_ab(" LINE= ", lineNumber);
//
//    pastValue = lineNumber;
//    return lineNumber;
//}
//
////////////////////////////////////////////////////
// Reads the sensors, provides calibrated values, and returns an estimated black line position.

////////////////////////////////////////////////////////////////////////
//
//  Calibrating your sensors can lead to substantially more reliable sensor readings
//  During this calibration phase, you will need to expose each of your reflectance
//  sensors to the lightest and darkest readings they will encounter.
//  For example, if you have made a line follower, you will want to slide it across
//  the line during the calibration phase so the each sensor can get a reading of how
//  dark the line is and how light the ground is.

// This method reads the sensors 10 times and uses the results for calibration.
// The sensor values are not returned; instead, the maximum and minimum values found
// over time are stored
//////////////////////////////////////////////////////////////////////////////////////
