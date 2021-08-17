#include "CMHub.hpp"
#include "CapstanModule.hpp"
#include <Mahi/Daq.hpp>
#include <Mahi/Robo.hpp>
#include <syntacts>


using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;
using namespace tact;       //syntacts

int main(int argc, char const *argv[])
{
    // make hub and CM
    CMHub hub;
    int id = 0;
    hub.createDevice(id, 7, 7, 7, 7, 7, {0,1,0});
    auto cm = hub.getDevice(id);  
    // start hub (runs asynchronously in another thread)
    hub.start();
    // initialize cm
    cm->zeroPosition();
    cm->setControlMode(CM::ControlMode::Position);
    cm->setPositionRange(0, 100);
    cm->enable();
    //cm->set_channel(&hub.daq.AI[7]);
    //cm->set_force_calibration(0,1,0);
    // make Syntacts session
    Session session;
    session.open("MOTU Pro Audio", API::ASIO);  
    // create timer for our loop (100 Hz)
    Timer timer(100_Hz);
    mahi::util::Time t;
    while (t < 1_s) {
        t = timer.get_elapsed_time();
        double cv = 0.5 * 0.5 * std::sin(2*3.14*1*t.as_seconds());
        cm->setControlValue(cv);
        timer.wait();
    };
    // make Signal (i.e. VT cue)
    Signal s = Sine(175) * ExponentialDecay(1, 10);
    // play syntacts signal on channel 0
    session.play(0, s);
    tact::sleep(s.length());
    // disable shit
    cm->disable();
    hub.stop();
    return 0;
}
