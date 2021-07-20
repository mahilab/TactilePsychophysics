#pragma once
#include "CapstanModule.hpp"
#include "CMHub.hpp"

/// DLL export macro
#define TASBI_API __declspec(dllexport)

// Exported DLL Functions
extern "C" {

TASBI_API int initLog();
TASBI_API int getNextLogLength();
TASBI_API int popNextLog(char* buf, int len);

//=============================================================================
// TASBI HUB API
//=============================================================================

TASBI_API int startDaq();
TASBI_API int startDaqSoft();
TASBI_API int stopDaq();
TASBI_API int queryDaq(CMHub::Query& q);

//=============================================================================
// TASBI DEVICE API
//=============================================================================

TASBI_API int createDevice(int id, int enable, int fault, int command, int force, int encoder);
TASBI_API int destroyDevice(int id);
TASBI_API int validateDeviceId(int id);

TASBI_API int setParams(int id, CM::Params& params);
TASBI_API int exportParams(int id, char* filepath, int len);
TASBI_API int importParams(int id, char* filepath, int len);
TASBI_API int getParams(int id, CM::Params& params);
TASBI_API int queryDevice(int id, CM::Query& q);

TASBI_API int enable(int id);
TASBI_API int disable(int id);

TASBI_API int zeroPosition(int id);
TASBI_API int setPositionRange(int id, double min, double max);
TASBI_API int setPositionGains(int id, double kp, double kd);

TASBI_API int setForceCalibration(int id, double a, double b, double c);
TASBI_API int setForceRange(int id, double min, double max);
TASBI_API int setForceGains(int id, double kp, double kd);

TASBI_API int setControlMode(int id, int mode);
TASBI_API int setControlValue(int id, double value);
TASBI_API int setControlValueFilter(int id, double cutoff);
TASBI_API int enableControlValueFilter(int id, bool filter);

}