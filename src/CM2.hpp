#pragma once

#include <mutex>

#include <Mahi/Daq.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo/Control/PdController.hpp>
#include <Mahi/Robo/Mechatronics/CurrentAmplifier.hpp>
#include <Mahi/Robo/Mechatronics/DcMotor.hpp>

#include "Util/RateMonitor.hpp"
#include "Util/MedianFilter.hpp"
#include "Util/MiniPID.hpp"

class CM2 : public mahi::util::Device {

    enum Status : int {
        Disabled = 0,
        Idle     = 1,
        Tracking = 2
    };

    enum TrackingMode : int {
        Torque   = 0,
        Position = 1,
        Force    = 2,
    };

    

};