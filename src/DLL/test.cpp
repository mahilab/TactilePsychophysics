
#define TASBI_API __declspec(dllexport)

#include <Mahi/Daq.hpp>
#include <Mahi/Robo/Control/PidController.hpp>
#include <thread>
#include <map>
#include <memory>
#include <mutex>

extern "C" {
    TASBI_API int test() {
        std::mutex y;
        mahi::robo::PidController x;
        return 45;
    }
}