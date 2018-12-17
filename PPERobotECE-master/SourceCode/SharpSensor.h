
#ifndef SHARPSENSOR_H_INCLUDED
#define SHARPSENSOR_H_INCLUDED

#include "mbed.h"

#define SHARP_UPDATE_PERIOD 0.010


/*
    TODO
        define
        doc
        Option to disable US (define / switch)
        Test laser

        Sensors class that auto update sensors (Ticker or queue or manually -> queue) => for US

*/

/*
    Pinout (left to right):
        V0
        Gnd
        Vcc: 5V

    Range is 100-800 mm or 40-300.
*/
class SharpSensor {
public:
    SharpSensor(PinName pin);

    void update(void);

    int16_t get_val(void);

    AnalogIn pin_;
    int16_t val_;
};

#endif // #ifndef SHARPSENSOR_H_INCLUDED