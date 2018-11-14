
#include "SharpSensor.h"

SharpSensor::SharpSensor(PinName pin) : pin_(pin) {
    val_ = -1;
}

void SharpSensor::update(void) {
    val_ = 75 * 1.0/pin_.read();  // TODO define - state that it is empirical
    // todo: detect errors due to too close / too far ?
}

int16_t SharpSensor::get_val(void) {
    return val_;
}