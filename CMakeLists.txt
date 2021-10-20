cmake_minimum_required(VERSION 3.7)

# create project
project(CM VERSION 0.1.0 LANGUAGES CXX)

# fetch mahi libs
include(FetchContent) 

FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git) 
FetchContent_MakeAvailable(mahi-robo)

FetchContent_Declare(mahi-daq GIT_REPOSITORY https://github.com/mahilab/mahi-daq.git) 
FetchContent_MakeAvailable(mahi-daq)

FetchContent_Declare(mahi-gui GIT_REPOSITORY https://github.com/mahilab/mahi-gui.git) 
FetchContent_MakeAvailable(mahi-gui)

# FetchContent_Declare(etfe GIT_REPOSITORY https://github.com/epezent/etfe.git GIT_TAG origin/main) 
# FetchContent_MakeAvailable(etfe)

add_library(cm 
    src/CapstanModule.hpp 
    src/CapstanModule.cpp 
    src/CMHub.hpp 
    src/CMHub.cpp
    src/UserParams.hpp 
    src/UserParams.cpp
    src/PsychophysicalTesting.hpp
    src/PsychophysicalTesting.cpp
    src/Util/RateMonitor.hpp
    src/Util/MedianFilter.hpp
    src/Util/MiniPID.hpp
    src/Util/MiniPID.cpp
    src/Util/XboxController.cpp
    src/Util/XboxController.hpp
    src/Util/ATI_windowCal.cpp
    src/Util/ATI_windowCal.hpp
    src/Util/HertzianContact.cpp
    src/Util/HertzianContact.hpp
    src/Util/ForceTorqueCentroid.cpp
    src/Util/ForceTorqueCentroid.hpp
)
target_include_directories(cm PUBLIC src)
target_link_libraries(cm PUBLIC mahi::daq mahi::robo XInput)
target_compile_features(cm PUBLIC cxx_std_17)

# ANSI C DLL

# add_executable(survey src/Apps/survey.cpp misc/cm.rc)
# target_link_libraries(survey mahi::gui)

add_executable(PosControl2DOF_NoGui src/Apps/runTwoMotorPositionControl_moduleNoGui.cpp)
target_link_libraries(PosControl2DOF_NoGui mahi::daq mahi::robo mahi::util cm)

add_executable(PosControl2DOF src/Apps/runTwoMotorPositionControl.cpp)
target_link_libraries(PosControl2DOF mahi::gui mahi::daq mahi::robo)

add_executable(PosThreaded src/Apps/runTwoMotorPositionControl_threaded.cpp)
target_link_libraries(PosThreaded mahi::gui mahi::daq mahi::robo mahi::util)

add_executable(PosModule src/Apps/runTwoMotorPositionControl_module.cpp)
target_link_libraries(PosModule mahi::gui mahi::daq mahi::robo mahi::util cm)

add_executable(ForceThreaded src/Apps/runTwoMotorForceControl_threaded.cpp)
target_link_libraries(ForceThreaded mahi::gui mahi::daq mahi::robo mahi::util)

add_executable(ForceControl2DOF src/Apps/runTwoMotorForceControl.cpp)
target_link_libraries(ForceControl2DOF mahi::gui mahi::daq mahi::robo)

add_executable(ForceModule src/Apps/runTwoMotorForceControl_module.cpp)
target_link_libraries(ForceModule mahi::gui mahi::daq mahi::robo mahi::util cm)

add_executable(xboxContoller src/Apps/xbox_adaptive_controller_test.cpp)
target_link_libraries(xboxContoller mahi::util cm)

add_executable(psychGui src/Apps/PsychGuiExp.cpp src/Apps/PsychGui.hpp src/Apps/PsychGui.cpp)
target_link_libraries(psychGui mahi::util mahi::gui mahi::robo cm)

add_executable(testWindowCal src/Apps/ex_testAtiWindowCal.cpp)
target_link_libraries(testWindowCal mahi::util mahi::gui mahi::robo cm)

add_executable(contactExp src/Apps/ContactMechExp.cpp src/Apps/ContactMechGui.hpp src/Apps/ContactMechGui.cpp)
target_link_libraries(contactExp mahi::util mahi::gui mahi::robo cm)