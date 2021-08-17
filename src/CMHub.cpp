#include "CMHub.hpp"
#include <Mahi/Util.hpp>
#include <Mahi/Robo.hpp>

using namespace mahi::daq;
using namespace mahi::util;
using namespace mahi::robo;

#define TASBI_THREAD_SAFE
#ifdef TASBI_THREAD_SAFE
#define TASBI_DAQ_LOCK std::lock_guard<std::mutex> lock(m_mutex); m_lockCount++;
#else
#define TASBI_DAQ_LOCK
#endif

CMHub::CMHub(int Fs) :
    daq(false),
    m_status(Status::Idle),
    m_timer(hertz(Fs), Timer::WaitMode::Busy),
    m_lockCount(0),
    m_running(false),
    m_loopRate(seconds(0.5))
{ 
    LOG(Info) << "CMHub created.";
}

CMHub::~CMHub() {
    if (m_running)
        stop();    
}

int CMHub::createDevice(int id, int enable, int fault, int command, int encoder, int force, std::vector<double> forceCal) {
    TASBI_DAQ_LOCK
    if (m_devices.count(id)) {
        LOG(Info) << "CM " << m_devices[id]->name() << " already initialized.";
        return ErrorCode::InvalidID;
    }
    else {
        AIForceSensor* aisensor = new AIForceSensor();
        aisensor->set_force_calibration(forceCal[0], forceCal[1], forceCal[2]);
        aisensor->set_channel(&daq.AI[force]);

        CM::Io io = {
            DOHandle(daq.DO,enable),
            DIHandle(daq.DI,fault),
            AOHandle(daq.AO,command),
            EncoderHandle(daq.encoder,encoder),
            *aisensor,
            Axis::AxisX,
            &daq.velocity[encoder],
            &daq.velocity.velocities[encoder]
        };
        m_devices[id] = std::make_shared<CM>( "cm_" + std::to_string(id), io, CM::Params());
    }
    return ErrorCode::NoError;
}

int CMHub::createDevice(int id, int enable, int fault, int command, int encoder, Axis forceAxis, const std::string& filepath, std::vector<int> ati_chan) {
    TASBI_DAQ_LOCK
    if (m_devices.count(id)) {
        LOG(Info) << "CM " << m_devices[id]->name() << " already initialized.";
        return ErrorCode::InvalidID;
    }
    else {
        AtiSensor* ati = new AtiSensor();
        ati->set_channels(&daq.AI[ati_chan[0]], &daq.AI[ati_chan[1]], &daq.AI[ati_chan[2]], &daq.AI[ati_chan[3]], &daq.AI[ati_chan[4]], &daq.AI[ati_chan[5]]);
        ati->load_calibration(filepath);

        CM::Io io = {
            DOHandle(daq.DO,enable),
            DIHandle(daq.DI,fault),
            AOHandle(daq.AO,command),
            EncoderHandle(daq.encoder,encoder),
            *ati,
            forceAxis,
            &daq.velocity[encoder],
            &daq.velocity.velocities[encoder]
        };

        m_devices[id] = std::make_shared<CM>( "cm_" + std::to_string(id), io, CM::Params());
    }
    return ErrorCode::NoError;
}


int CMHub::addDevice(int id, std::shared_ptr<CM> cm) {
    TASBI_DAQ_LOCK
    if (m_devices.count(id)) {
        LOG(Info) << "CM " << m_devices[id]->name() << " already initialized.";
        return ErrorCode::InvalidID;
    }
    else {
        m_devices.emplace(id, std::move(cm));
    }
    return ErrorCode::NoError;
}

int CMHub::destroyDevice(int id) {
    TASBI_DAQ_LOCK
    if (m_devices.count(id) == 0) { 
        LOG(mahi::util::Error) << "CM ID " << id << " invalid."; 
        return ErrorCode::InvalidID;
    }
    m_devices.erase(id);
    return ErrorCode::NoError;
}

void CMHub::setSampleRate(int Fs) {
    TASBI_DAQ_LOCK
    m_timer = Timer(hertz(Fs), Timer::WaitMode::Busy);
}

int CMHub::start(bool soft) {
    if (m_running) {
        LOG(Warning) << "CM Hub already running";
        return ErrorCode::AlreadyRunning;
    }
    if (!soft) {
        if (!daq.open()) {
            m_status = Status::Error;
            fillQuery(m_q);
            return ErrorCode::DaqOpenFailed;
        }
        auto opts = daq.get_options();
        opts.update_rate = QuanserOptions::UpdateRate::Fast;
        for (uint32 i = 0; i < 8; ++i)
            opts.encX_dir[i] = QuanserOptions::EncoderDirection::Reversed;
        daq.set_options(opts);

        if (!daq.enable()) {        
            m_status = Status::Error;
            fillQuery(m_q);
            return ErrorCode::DaqEnableFailed; 
        }   
    }
    m_status = Status::Running;
    m_running = true;
    m_controlThread = std::thread(&CMHub::controlThreadFunction, this, soft);
    return ErrorCode::NoError;
}

int CMHub::stop() {

    if (!m_running) { 
        LOG(Warning) << "CM Hub not running";  
        return ErrorCode::NotRunning; 
    }
    LOG(Info) << "Stopping CM Hub ...";
    m_running = false;
    m_status = Status::Idle;
    m_controlThread.join();
    LOG(Info) << "CM Hub stopped.";
    return ErrorCode::NoError;
}

void CMHub::controlThreadFunction(bool soft) {
    LOG(Info) << "CM Hub started";
    m_timer.restart();
    if (soft) {
        while (m_running) {
            updateSoft();
            m_timer.wait();
        }
    }
    else {
        while(m_running) {
            if (!update()) {
                LOG(mahi::util::Error) << "CM Hub failed to update.";         
                m_status = Status::Error;
                m_running = false;   
            }
            m_timer.wait();
        }
    }
    // shutdown devices
    LOG(Info) << "Disabling CM(s) ...";
    for (auto& device : m_devices) {
        if (device.second->is_enabled())
            device.second->disable();
    }
    // shutdown DAQ
    LOG(Info) << "Disabling and closing DAQ ...";
    if (!soft) {
        if (daq.is_enabled())
            daq.disable();
        if (daq.is_open())
            daq.close();
    }
    fillQuery(m_q);
}

bool CMHub::update() {
    TASBI_DAQ_LOCK
    Time t = m_timer.get_elapsed_time();
    // update inputs
    if (!daq.read_all())
        return false;
    // update devices
    for (auto& device : m_devices)
        device.second->update(t);
    // update ouputs
    if (!daq.write_all())
        return false;
    // update query info
    m_loopRate.tick();
    m_loopRate.update(t);
    fillQuery(m_q);
    m_lockCount = 0;
    return true;
}

bool CMHub::updateSoft() {
    TASBI_DAQ_LOCK
    Time t = m_timer.get_elapsed_time();
    // update devices
    for (auto& device : m_devices)
        device.second->update(t);
    // update query info
    m_loopRate.tick();
    m_loopRate.update(t);
    fillQuery(m_q);
    m_lockCount = 0;
    return true;
}

bool CMHub::validateDeviceId(int id) {
    TASBI_DAQ_LOCK
    if (m_devices.count(id))
        return true;
    return false;
}

std::shared_ptr<CM> CMHub::getDevice(int id) {
    TASBI_DAQ_LOCK
    if (m_devices.count(id)) 
        return m_devices[id];
    LOG(mahi::util::Error) << "CM ID " << id << " invalid.";
    return nullptr;
}

CMHub::Query CMHub::getQuery(bool immediate) {
    TASBI_DAQ_LOCK
    if (immediate) {
        Query q;
        fillQuery(q);
        return q;
    }
    return m_q;
}

void CMHub::fillQuery(Query& q) {
    q.devices = (int)m_devices.size();
    q.status = m_status;
    q.time = m_timer.get_elapsed_time_ideal().as_seconds();
    q.tick = (int)m_timer.get_elapsed_ticks();
    q.misses = (int)m_timer.get_misses();
    q.missRate = m_timer.get_miss_rate();
    q.waitRatio = m_timer.get_wait_ratio();
    q.lockCount = m_lockCount;
    q.loopRate = m_loopRate.rate();
}