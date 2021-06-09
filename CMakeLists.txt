cmake_minimum_required(VERSION 3.0.0)
project(HapticPsychMotorControl VERSION 0.1.0)

include(FetchContent) 
FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(mahi-gui)

include(FetchContent) 
FetchContent_Declare(mahi-daq GIT_REPOSITORY https://github.com/mahilab/mahi-daq.git) 
FetchContent_MakeAvailable(mahi-daq)


add_executable(HapticPsychMotorControl src/runSingleMotorPos.cpp)
target_link_libraries(HapticPsychMotorControl mahi::gui)
target_link_libraries(HapticPsychMotorControl mahi::daq)