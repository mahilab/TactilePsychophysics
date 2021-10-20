#pragma once

#include <Mahi/Daq.hpp>
#include <thread>
#include <map>
#include <memory>
#include <mutex>
#include "CapstanModule.hpp"

// Written by Janelle Clark, based off code by Evan Pezent

class CMHub : public mahi::util::NonCopyable {
public:
    /// Hub Status
    enum Status : int {
        Idle = 0,
        Running = 1,
        Error = 2
    };
    /// Hub Query
    struct Query {
        Status status = Idle;
        int devices = 0;
        double time = 0;
        int tick = 0;
        int misses = 0;
        double missRate = 0;
        double waitRatio = 0;
        int lockCount = 0;
        double loopRate = 0;
    };
    /// Hub Error Codes
    enum ErrorCode : int {
        NoError = 0,
        NotRunning = -1,
        AlreadyRunning = -2,
        InvalidID = -3,
        DaqOpenFailed = -4,
        DaqEnableFailed = -5,
        UpdateFailed = -6
    };
    /// Constructor
    CMHub(int Fs = 1000);
    /// Destructor
    ~CMHub();
    /// Initializes a base CM device to this Daq with a AI force sensor
    int createDevice(int id, int enable, int fault, int command, int encoder, int force, std::vector<double> forceCal);
    /// Initializes a base CM device to this Daq with an ati force sensor
    int createDevice(int id, int enable, int fault, int command, int encoder, Axis forceAxis, const std::string& filepath, std::vector<int> ati_chan, bool windowCal);
    /// Adds an exsiting (and possibly derived) CM device to this Daq
    int addDevice(int id, std::shared_ptr<CM> cm);
    /// Destroys a CM device on this Daq
    int destroyDevice(int id);
    /// Starts the Daq thread. Hardware will not be update if soft is true (thread safe)
    int start(bool soft = false);
    /// Stops the Daq thread (thread safe)
    int stop();
    /// Returns true if a device with ID is active (thread safe)
    bool validateDeviceId(int id);
    /// Returns the device with ID, nullptr if no device with ID exists (thread safe)
    std::shared_ptr<CM> getDevice(int id);
    /// Returns a full query of the CMHub (thread safe)
    Query getQuery(bool immediate = false);
    /// Sets hub sampling rate (default = 500 Hz)
    void setSampleRate(int Fs);

public:
    mahi::daq::Q8Usb daq; ///< the DAQ that all CMs run on
private:
    void controlThreadFunction(bool soft);
    bool update();
    bool updateSoft();
    void fillQuery(Query& q);
private:
    Status m_status;
    Query m_q;
    mahi::util::Timer m_timer;
    mahi::util::ctrl_bool m_running;
    std::thread m_controlThread;
    std::mutex m_mutex;
    int m_lockCount;
    std::map<int, std::shared_ptr<CM>> m_devices;
    RateMonitor m_loopRate;
};