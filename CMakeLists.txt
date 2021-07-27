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

FetchContent_Declare(etfe GIT_REPOSITORY https://github.com/epezent/etfe.git GIT_TAG origin/main) 
FetchContent_MakeAvailable(etfe)

set(SYNTACTS_BUILD_GUI OFF CACHE BOOL "" FORCE)
set(SYNTACTS_BUILD_C_DLL OFF CACHE BOOL "" FORCE)
set(SYNTACTS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SYNTACTS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(syntacts GIT_REPOSITORY https://github.com/mahilab/Syntacts.git)
FetchContent_MakeAvailable(syntacts)

add_library(cm 
    src/CapstanModule.hpp 
    src/CapstanModule.hpp 
    src/CapstanModule.cpp 
    src/CMHub.hpp 
    src/CMHub.cpp 
    src/Util/RateMonitor.hpp
    src/Util/PolyFit.hpp
    src/Util/PolyFit.cpp
    src/Util/MedianFilter.hpp
    src/Util/MiniPID.hpp
    src/Util/MiniPID.cpp
)
target_include_directories(cm PUBLIC src)
target_link_libraries(cm PUBLIC mahi::daq mahi::robo)
target_compile_features(cm PUBLIC cxx_std_17)

# ANSI C DLL
add_library(cm-c SHARED  src/DLL/dll.hpp src/DLL/dll.cpp)
target_link_libraries(cm-c cm)

add_library(test_dll SHARED src/DLL/test.cpp)
target_link_libraries(test_dll cm)

add_executable(calibrator src/Apps/calibrator.cpp misc/cm.rc)
target_link_libraries(calibrator cm mahi::gui etfe)

add_executable(identification src/Apps/identification.cpp misc/cm.rc)
target_link_libraries(identification cm mahi::gui syntacts)

add_executable(control_display src/Apps/control_display.cpp misc/cm.rc)
target_link_libraries(control_display mahi::gui)

add_executable(survey src/Apps/survey.cpp misc/cm.rc)
target_link_libraries(survey mahi::gui)

add_executable(vsb src/Apps/vsb.cpp)
target_include_directories(vsb PUBLIC src)
target_link_libraries(vsb mahi::gui mahi::daq mahi::robo)

add_library(vsb_plugin SHARED src/DLL/vsb_plugin.cpp)
target_include_directories(vsb_plugin PUBLIC src)
target_link_libraries(vsb_plugin mahi::gui mahi::daq mahi::robo)

add_executable(gaurav src/Apps/gaurav.cpp)
target_link_libraries(gaurav PUBLIC cm syntacts)

add_executable(kalman src/Apps/kalman.cpp)
target_link_libraries(kalman PUBLIC mahi::gui mahi::robo)

add_executable(PosControl2DOF src/Apps/runTwoMotorPositionControl.cpp)
target_link_libraries(PosControl2DOF mahi::gui mahi::daq mahi::robo)

add_executable(PosThreaded src/Apps/runTwoMotorPositionControl_threaded.cpp)
target_link_libraries(PosThreaded mahi::gui mahi::daq mahi::robo mahi::util)

add_executable(ForceControl2DOF src/Apps/runTwoMotorForceControl.cpp)
target_link_libraries(ForceControl2DOF mahi::gui mahi::daq mahi::robo)

add_executable(PosControl2DOF_CMtest src/Apps/janelle_haptic_testbed.cpp)
target_link_libraries(PosControl2DOF_CMtest mahi::daq mahi::robo mahi::util cm)