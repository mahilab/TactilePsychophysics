#include <Mahi/Gui.hpp>
#include <Mahi/Daq.hpp>

using namespace mahi::util;
using namespace mahi::gui;
using namespace mahi::daq;

class RobotJoint {
public:

    RobotJoint(EncoderHandle enc) :
        h_enc(enc)
    { }

    void update() {
        double pos = h_enc.get_pos();
    }

    EncoderHandle h_enc;
    AIHandle      h_sen;
    AOHandle      h_com;
    DOHandle      h_enb;
    DIHandle      h_flt;
};

class Robot {
public:
    Robot() {
        joints.emplace_back(EncoderHandle(q8.encoder, 0));
    }
    void update() {
        joints[0].update();
        joints[1].update();
    }
    std::vector<RobotJoint> joints;
    Q8Usb                   q8;
};