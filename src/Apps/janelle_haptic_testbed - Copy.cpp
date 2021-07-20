#include "CMHub.hpp"
#include "CapstanModule.hpp"


using namespace mahi::util;

int main(int argc, char const *argv[])
{
    // make hub and normal and tangential capstan modules
    CMHub hub;

    int id_n = 0;
    hub.createDevice(id_n, 0, 0, 0, 6, 0); // which force?????? (6) want ati, not AI
    auto cm_n = hub.getDevice(id_n); 

    int id_t = 1;
    hub.createDevice(id_t, 2, 2, 1, 7, 1); // which force?????? (7) want ati, not AI
    auto cm_t = hub.getDevice(id_t); 

    // start hub (runs asynchronously in another thread)
    hub.start();

    // initialize normal capstan module
   
    cm_n->zeroPosition(); //makes wherever it is when the program starts the zero position
    cm_n->setControlMode(CM::ControlMode::Position);
    cm_n->setPositionRange(-3,3); //(0, 65); //[mm]
    cm_n->setPositionGains(10,1);
    cm_n->enable(); 

    // initialize tangential capstan module
    cm_t->zeroPosition(); //makes wherever it is when the program starts the zero position
    cm_t->setControlMode(CM::ControlMode::Position);
    cm_t->setPositionRange(-3,3); //(0, 65); //[mm]
    cm_t->setPositionGains(10,1);
    cm_t->enable();

    // create timer for our loop (100 Hz)
    Timer timer(100_Hz);
    mahi::util::Time t;
    while (t < 1_s) {
        t = timer.get_elapsed_time();
        double cv_n = 0.375 + 0.375 * std::sin(2*PI*0.25*t.as_seconds()); // 0.75, sine wave moving 75% of the range upward and downward
        //cm_n->setControlValue(cv_n);
        double cv_t = 0.375 + 0.375 * std::sin(2*PI*0.25*t.as_seconds()); // 0.75, sine wave moving 75% of the range forward and backward
        cm_t->setControlValue(cv_t);
        std::cout << "cv_t " << cv_t << " "; //as expected
        timer.wait();
    };

    // disable everything
    cm_n->disable();
    cm_t->disable();
    hub.stop();
    return 0;
}
