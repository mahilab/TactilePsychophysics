#include "dll.hpp"
#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Util/Templates/RingBuffer.hpp>
#include <Mahi/Util/Templates/SPSCQueue.hpp>
#include <thread>
#include <map>
#include <memory>
#include <mutex>
#include <fstream>

using namespace mahi::daq;
using namespace mahi::util;

//=============================================================================
// LOGGING API
//=============================================================================

bool g_logInitialized = false;

SPSCQueue<std::pair<int,std::string>> g_logs(128);

class UnityFormatter {
public:
    static std::string header() { return std::string(); }
    static std::string format(const LogRecord& record) {
        std::ostringstream ss;
        ss << "[" << record.get_tid_() << "] ";
        ss << record.get_message() << "\n";
        return ss.str();
    }
};

template <class Formatter>
class UnityWriter : public Writer {
public:
    UnityWriter(Severity max_severity = Info) : Writer(max_severity) {}
    virtual void write(const LogRecord& record) override {
        std::string str = Formatter::format(record);
        auto log = std::pair<int, std::string>((int)record.get_severity(), str);
        g_logs.try_push(log);
    }
};

static UnityWriter<UnityFormatter> g_unityWriter;

int initLog() {
    if (!g_logInitialized) {
        get_logger<DEFAULT_LOGGER>()->add_writer(&g_unityWriter);
        LOG(Info) << "CM log initialized.";
        g_logInitialized = true;
        return 0;
    }
    return -1;
}

TASBI_API int getNextLogLength() {
    if (g_logs.front()) {
        auto log = *g_logs.front();
        return (int)log.second.length();
    }
    else
        return 0;
}

TASBI_API int popNextLog(char* buf, int len) {
    if (g_logs.front()) {
        auto log = *g_logs.front();
        int severity = log.first;
        log.second.copy(buf, len);
        g_logs.pop();
        return severity;
    }
    else
        return -1;
}


//=============================================================================
// TASBI HUB API
//=============================================================================

static CMHub g_hub;

int startDaq() {
    return g_hub.start();
}

int startDaqSoft() {
    return g_hub.start(true);
}

int stopDaq() {
    return g_hub.stop();
}

/// Queries the hub controller status
int queryDaq(CMHub::Query& q) {
    q = g_hub.getQuery();
    return 0;
}

//=============================================================================
// TASBI DEVICE API
//=============================================================================

int createDevice(int id, int enable, int fault, int command, int encoder, int force, std::vector<double> forceCal) {
    return g_hub.createDevice(id, enable, fault, command, encoder, force, forceCal);
}

int destroyDevice(int id) {
    return g_hub.destroyDevice(id);
}

int validateDeviceId(int id) {
    if (g_hub.validateDeviceId(id))
        return CMHub::NoError;
    return CMHub::InvalidID;
}

int setParams(int id, CM::Params& params) {
    if (auto d = g_hub.getDevice(id)) {
        d->setParams(params);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int exportParams(int id, char* filepath, int len) {
    if (auto d = g_hub.getDevice(id)) {
        d->exportParams(filepath);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int importParams(int id, char* filepath, int len) {
    if (auto d = g_hub.getDevice(id)) {
        d->importParams(filepath);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int getParams(int id, CM::Params& params) {
    if (auto d = g_hub.getDevice(id)) {
        params = d->getParams();
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int queryDevice(int id, CM::Query& q) {    
    if (auto d = g_hub.getDevice(id)) {
        q = d->getQuery();
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int enable(int id) {
    if (auto d = g_hub.getDevice(id)) {
        d->enable();
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int disable(int id) {
    if (auto d = g_hub.getDevice(id)) {
        d->disable();
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int zeroPosition(int id) {
    if (auto d = g_hub.getDevice(id)) {
        d->zeroPosition();
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setPositionRange(int id, double min, double max) {
    if (auto d = g_hub.getDevice(id)) {
        d->setPositionRange(min, max);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setPositionGains(int id, double kp, double kd) {
    if (auto d = g_hub.getDevice(id)) {
        d->setPositionGains(kp, kd);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setForceCalibration(int id, double a, double b, double c) {
    if (auto d = g_hub.getDevice(id)) {
        d->setForceCalibration(a,b,c);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setForceRange(int id, double min, double max) {
    if (auto d = g_hub.getDevice(id)) {
        d->setForceRange(min, max);
        return CMHub::NoError;
    }
    return 0;
}

int setForceGains(int id, double kp, double kd) {
    if (auto d = g_hub.getDevice(id)) {
        d->setForceGains(kp, 0, kd);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setControlMode(int id, int mode) {
    if (auto d = g_hub.getDevice(id)) {
        d->setControlMode((CM::ControlMode)mode);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setControlValue(int id, double value) {
    if (auto d = g_hub.getDevice(id)) {
        d->setControlValue(value);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setControlValueFilter(int id, double cutoff) {
    if (auto d = g_hub.getDevice(id)) {
        d->setControlValueFilter(cutoff);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int enableControlValueFilter(int id, bool filter) {
    if (auto d = g_hub.getDevice(id)) {
        d->enableControlValueFilter(filter);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}

int setForceFilter(int id, double cutoff) {
    if (auto d = g_hub.getDevice(id)) {
        d->setForceFilter(cutoff);
        return CMHub::NoError;
    }
    return CMHub::InvalidID;
}
