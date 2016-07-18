/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ANEM__HIL_H__
#define __AP_ANEM__HIL_H__

#include "AP_Anemometer.h"

class AP_Anemometer_HIL : public AP_Anemometer
{
private:
    float Anglecd;
    float Speed;
    volatile uint8_t _count;

public:
    bool init();
    uint8_t read();
    float get_anglecd();
    float get_speed();
    void setHIL(float angle, float speed);
};

#endif //  __AP_ANEM__HIL_H__
