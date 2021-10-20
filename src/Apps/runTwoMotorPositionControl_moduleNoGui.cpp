#include "CMHub.hpp"
#include "CapstanModule.hpp"
#include <Mahi/Robo.hpp>


using namespace mahi::util;
using namespace mahi::robo;

int main(int argc, char const *argv[])
{
    // make hub and normal and tangential capstan modules
    CMHub hub; 

    int id_n = 0;
    hub.createDevice(id_n, 0, 0, 0, 0, Axis::AxisZ, "FT06833.cal", {0,1,2,3,4,5},0); 
    auto cm_n = hub.getDevice(id_n); 

    int id_t = 1;
    hub.createDevice(id_t, 2, 2, 1, 1, Axis::AxisX, "FT06833.cal", {0,1,2,3,4,5},0); 
    auto cm_t = hub.getDevice(id_t); 

    // start hub (runs asynchronously in another thread)
    hub.start();

    // initialize normal capstan module
   
    cm_n->zeroPosition(); //makes wherever it is when the program starts the zero position
    cm_n->setControlMode(CM::ControlMode::Position);
    cm_n->setVelocityMax(100, 1);
    cm_n->setTorqueMax(1,1);
    cm_n->setPositionRange(-5.5,5.5); //(0, 65); //[mm]
    cm_n->setPositionGains(1.0,0.1);
    cm_n->enable(); 

    // initialize tangential capstan module
    cm_t->zeroPosition(); //makes wherever it is when the program starts the zero position
    cm_t->setControlMode(CM::ControlMode::Position);
    cm_t->setVelocityMax(100, 1);
    cm_t->setTorqueMax(1,1); 
    cm_t->setPositionRange(-5.5,5.5); //(0, 65); //[mm]
    cm_t->setPositionGains(1.0,0.1);
    cm_t->enable();

    // create timer for our loop (100 Hz)
    Timer timer(100_Hz);
    mahi::util::Time t;
    while (t < 1_s) {
        t = timer.get_elapsed_time();
        double cv_n = 0.2 * std::sin(PI*0.25*t.as_seconds()); // 0.75, sine wave moving 75% of the range upward and downward
        cm_n->setControlValue(cv_n);
        cm_n->velocity_limit_exceeded();
        double cv_t = 0.2 * std::sin(PI*0.25*t.as_seconds()); // 0.75, sine wave moving 75% of the range forward and backward
        cm_t->setControlValue(cv_t);
        cm_t->velocity_limit_exceeded();
        std::cout << "norm " << cv_n << "shear" << cv_t << std::endl;
        timer.wait();
    };

    // disable everything
    cm_n->disable();
    cm_t->disable();
    hub.stop();
    return 0;
}
