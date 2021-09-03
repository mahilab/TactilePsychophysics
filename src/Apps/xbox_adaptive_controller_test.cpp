#include <Mahi/Util.hpp>
#include "Util/XboxController.hpp"

using namespace mahi::util;
using namespace xbox;

bool stop = false;

int main(int argc, char const *argv[])
{
    XboxController xbox;
    if (!xbox.is_connected())
        LOG(Error) << "No Xbox controller detected!"; 

    Timer timer(hertz(250));
    Time tm;
    while (!stop) {
        {                    
            if (xbox.is_button_pressed(XboxController::A))
                std::cout << "Pressed A" << std::endl;
            else if (xbox.is_button_pressed(XboxController::B))
                std::cout << "Pressed B" << std::endl;    
        }
        tm = timer.wait();
    }

    return 0;
}