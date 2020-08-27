//
// Created by llach on 26/8/20.
//

#ifndef TIAGO_TACTILE_TA11_TA11_H
#define TIAGO_TACTILE_TA11_TA11_H

#include <mutex>

#include "ros/ros.h"
#include "tiago_tactile_ta11/u6.h"

#define SLEN 16
#define RLEN 16

namespace tiago_tactile_ta11{

class TA11 {

public:
    TA11();
    ~TA11();

    std::mutex values_lock;
    std::vector<float> values;

    void read_loop();
private:
    int _slen = SLEN;
    int _rlen = RLEN;
    uint8 _sendBuff[SLEN], _recBuff[RLEN];

    HANDLE _hDevice;
    u6CalibrationInfo _caliInfo;

    int configIO();
};

}
#endif //TIAGO_TACTILE_TA11_TA11_H
