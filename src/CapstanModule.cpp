#include <Mahi/Util/Logging/Log.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include "CapstanModule.hpp"
#include "Util/PolyFit.hpp"
#include <filesystem>
#include <fstream>

using namespace mahi::daq;
using namespace mahi::util;
namespace fs = std::filesystem;

CM::CM(const std::string &name, Io io, Params config) :
    Device(name),
    m_status(Status::Disabled),
    m_io(io),
    m_ctrlMode(ControlMode::Torque),
    m_Q(10000),
    m_positionPd(config.positionKp, config.positionKd),
    m_forcePID(0,0,0),
    m_ctrlFilter(2, 0.02, Butterworth::Lowpass),
    m_forceFiltMode(FilterMode::Median),
    m_forceFilterL(2, 0.1, Butterworth::Lowpass),
    m_forceFilterM(31),
    m_outputFilter(2,config.outputFilterCutoff,Butterworth::Lowpass),
    m_posDiff(),
    m_velocityFilter(2,0.1),
    m_ctrlValue(0.0),
    m_ctrlValueFiltered(0.0),
    m_feedRate(seconds(0.5)),
    m_customController(std::make_shared<CMController>())
//  m_melshare(name)
{
    setParams(config);
    LOG(Info) << "Created CM " << this->name() << ".";
}

CM::~CM() {
    if (is_enabled())
        disable();
    LOG(Info) << "Destroyed CM " << name() << ".";
}

//=============================================================================
// PUBLIC (THREAD SAFE)
//=============================================================================

void CM::update(const Time &t) {
    TASBI_LOCK
    // filter incomming control value
    //std::cout << m_ctrlValue << " "; // 
    m_ctrlValueFiltered  = m_ctrlFilter.update(m_ctrlValue);
    double ctrlValueUsed = m_params.filterControlValue ? m_ctrlValueFiltered : m_ctrlValue;
    // software velocity estimation
    auto vel = m_posDiff.update(getMotorPosition(), t);
    m_velocityFilter.update(vel);
    // control upate
    if (m_status == Status::Enabled)
        controlUpdate(ctrlValueUsed, t);
    // update feedrate
    m_feedRate.update(t);
    // update fixed query
    fillQuery(m_q);
    m_q.time = t.as_microseconds();
    m_Q.push_back(m_q);
    // on update
    onUpdate(t);
    // reset lockcount
    m_lockCount = 0;
};

void CM::setParams(CM::Params config) {
    TASBI_LOCK
    m_params = config;
    // these need to be udpated now
    m_io.encoderCh.set_units(m_params.degPerCount);
    m_positionPd.kp = m_params.positionKp;
    m_positionPd.kd = m_params.positionKd;
    m_forcePd.kp = m_params.forceKp;
    m_forcePd.kd = m_params.forceKd;
    m_ctrlFilter.configure(2, m_params.cvFilterCutoff);
    m_forceFilterL.configure(2, m_params.forceFilterCutoff);
    m_forceFilterM.resize(m_params.forceFilterN);
    m_outputFilter.configure(2, m_params.outputFilterCutoff);
    m_velocityFilter.configure(2, m_params.velFilterCutoff);
    m_forcePID.setPID(m_params.forceKp, m_params.forceKi, m_params.forceKd);
}

bool CM::exportParams(const std::string& filepath) {
    fs::path path(filepath);
    CM::Params params = getParams();
    Timestamp ts;
    json j;
    j["name"]                = name();
    j["date"]                = ts.yyyy_mm_dd();
    j["motorNominalTorque"]  = params.motorNominalTorque;
    j["motorStallTorque"]    = params.motorStallTorque;
    j["motorNominalCurrent"] = params.motorNominalCurrent;
    j["motorStallCurrent"]   = params.motorStallCurrent;
    j["motorTorqueConstant"] = params.motorTorqueConstant;
    j["motorNominalSpeed"]   = params.motorNominalSpeed;
    j["motorMaxSpeed"]       = params.motorMaxSpeed;
    j["gearRatio"]           = params.gearRatio;
    j["degPerCount"]         = params.degPerCount;
    j["commandGain"]         = params.commandGain;
    j["senseGain"]           = params.senseGain;
    j["has_velocity_limit_"] = params.has_velocity_limit_;
    j["has_torque_limit_"]   = params.has_torque_limit_;
    j["velocityMax"]         = params.velocityMax;
    j["torqueMax"]           = params.torqueMax;
    j["positionMin"]         = params.positionMin;
    j["positionMax"]         = params.positionMax;
    j["positionKp"]          = params.positionKp;
    j["positionKd"]          = params.positionKd;
    j["forceMin"]            = params.forceMin;
    j["forceMax"]            = params.forceMax;
    j["forceKp"]             = params.forceKp;
    j["forceKd"]             = params.forceKd;
    j["forceCalibA"]         = params.forceCalibA;
    j["forceCalibB"]         = params.forceCalibB;
    j["forceCalibC"]         = params.forceCalibC;
    j["forceFilterCutoff"]   = params.forceFilterCutoff;
    j["cvFilterCutoff"]      = params.cvFilterCutoff;
    j["filterControlValue"]  = params.filterControlValue;
    std::ofstream file(path);
    if (file.is_open()) {
        file << std::setw(10) << j;
        LOG(Info) << "Exported CM " << name() << " parameters to " << path.generic_string();
        file.close();
        return true;
    }
    LOG(Error) << "Failed to export CM " << name() << " parameters because because " << path << " could not be created or opened.";
    return false;
}

bool CM::importParams(const std::string& filepath) {
    fs::path path(filepath);
    if (fs::exists(path)) {

        try {
            std::ifstream file(path);
            json j;
            file >> j;
            Params params;
            // j["name"]                = name();
            set_name(j["name"].get<std::string>());
            params.motorNominalTorque = j["motorNominalTorque"].get<double>();
            params.motorStallTorque   = j["motorStallTorque"].get<double>();
            params.motorNominalCurrent = j["motorNominalCurrent"].get<double>();;
            params.motorStallCurrent  = j["motorStallCurrent"].get<double>();
            params.motorTorqueConstant = j["motorTorqueConstant"].get<double>();
            params.motorNominalSpeed  = j["motorNominalSpeed"].get<double>();
            params.motorMaxSpeed      = j["motorMaxSpeed"].get<double>();
            params.gearRatio          = j["gearRatio"].get<double>();
            params.degPerCount        = j["degPerCount"].get<double>();
            params.commandGain        = j["commandGain"].get<double>();
            params.senseGain          = j["senseGain"].get<double>();
            params.has_velocity_limit_= j["has_velocity_limit_"].get<bool>();
            params.has_torque_limit_  = j["has_torque_limit_"].get<bool>();
            params.velocityMax        = j["velocityMax"].get<double>();
            params.torqueMax          = j["torqueMax"].get<double>();
            params.positionMin        = j["positionMin"].get<double>();
            params.positionMax        = j["positionMax"].get<double>();
            params.positionKp         = j["positionKp"].get<double>();
            params.positionKd         = j["positionKd"].get<double>();
            params.forceMin           = j["forceMin"].get<double>();
            params.forceMax           = j["forceMax"].get<double>();
            params.forceKp            = j["forceKp"].get<double>();
            params.forceKd            = j["forceKd"].get<double>();
            params.forceCalibA        = j["forceCalibA"].get<double>();
            params.forceCalibB        = j["forceCalibB"].get<double>();
            params.forceCalibC        = j["forceCalibC"].get<double>();
            params.forceFilterCutoff  = j["forceFilterCutoff"].get<double>();
            params.cvFilterCutoff     = j["cvFilterCutoff"].get<double>();
            params.filterControlValue = j["filterControlValue"].get<bool>();
            setParams(params);
            LOG(Info) << "Imported CM " << name() << " parameters from " << path.generic_string();
        }
        catch(...) {
            LOG(Error) << "Failed to import CM " << name() << " parameters.";
        }

        return true;
    }
    LOG(Error) << "Failed to import CM " << name() << " parameters because " << path << " does not exist.";
    return false;
}

CM::Params CM::getParams() const {
    TASBI_LOCK
    return m_params;
}

CM::Query CM::getQuery(bool immediate) {
    TASBI_LOCK
    if (immediate) {
        Query q;
        fillQuery(q);
        return q;
    }
    return m_q;
}

void CM::dumpQueries(const std::string& filepath) {
    TASBI_LOCK
    Csv csv(filepath);
    if (csv.is_open()) {
        csv.write_row("time",
                      "status",
                      "counts",
                      "countsPerSecond",
                      "motorPosition",
                      "motorVelocity",
                      "motorTorqueCommand",
                      "spoolPosition",
                      "spoolVelocity",
                      "forceRaw",
                      "forceRawFiltered",
                      "force",
                      "forceFiltered",
                      "forceEst",
                      "ctrlMode",
                      "filtMode",
                      "ctrlValue",
                      "ctrlValueFiltered",
                      "ctrlValueScaled",
                      "lockCount",
                      "feedRate",
                      "dFdt");
        for (int i = 0; i < m_Q.size(); ++i) {
            Query& q = m_Q[i];
            csv.write_row(q.time,          
                          q.status,            
                          q.counts,            
                          q.countsPerSecond,   
                          q.motorPosition,     
                          q.motorVelocity,     
                          q.motorTorqueCommand,
                          q.spoolPosition,     
                          q.spoolVelocity,     
                          q.forceRaw,         
                          q.forceRawFiltered,  
                          q.force,             
                          q.forceFiltered,     
                          q.forceEst,          
                          q.ctrlMode,          
                          q.filtMode,          
                          q.ctrlValue,         
                          q.ctrlValueFiltered, 
                          q.ctrlValueScaled,   
                          q.lockCount,         
                          q.feedRate,          
                          q.dFdt);
        }
        csv.close();
    }
}

void CM::zeroPosition() {
    TASBI_LOCK
    if (m_io.encoderCh.zero())
        LOG(Info) << "Zeroed CM " << name() << " position.";
    else
        LOG(Error) << "Failed to zero CM " << name() << " position.";
}

void CM::zeroPosition(double position) {
    double x = m_io.encoderCh.get_counts() / m_io.encoderCh.get_pos();
    int    c = (position / m_params.gearRatio) * x;
    if (m_io.encoderCh.write_counts(c))
        LOG(Info) << "Zeroed CM " << name() << " position to " << position << " degrees.";
    else
        LOG(Error) << "Failed to zero CM " << name() << " position.";
}

void CM::setVelocityMax(double vel, bool has_limit_) {
    TASBI_LOCK
    m_params.velocityMax = vel;
    m_params.has_velocity_limit_ = has_limit_;
    LOG(Info) << "Set CM " << name() << " velocity maximum to " << vel << " what units?? .";
}

void CM::setTorqueMax(double tor, bool has_limit_) {
    TASBI_LOCK
    m_params.torqueMax = tor;
    m_params.has_torque_limit_ = has_limit_;
    LOG(Info) << "Set CM " << name() << " torque maximum to " << tor << " Nm .";
}

void CM::setPositionRange(double min, double max) {
    TASBI_LOCK
    m_params.positionMin = min;
    m_params.positionMax = max;
    LOG(Info) << "Set CM " << name() << " position range to [ " << min << " , " << max << " ].";
}

void CM::setPositionGains(double kp, double kd) {
    TASBI_LOCK
    m_params.positionKp = kp;
    m_params.positionKd = kd;
    m_positionPd.kp    = kp;
    m_positionPd.kd    = kd;
    LOG(Info) << "Set CM " << name() << " position gains to kp = " << kp << ", kd = " << kd << ".";
}

void CM::setForceRange(double min, double max) {
    TASBI_LOCK
    m_params.forceMin = min;
    m_params.forceMax = max;
    LOG(Info) << "Set CM " << name() << " force range to [ " << min << " , " << max << " ].";
}

void CM::setForceGains(double kp, double ki, double kd) {
    TASBI_LOCK
    m_params.forceKp = kp;
    m_params.forceKi = ki;
    m_params.forceKd = kd;
    m_forcePd.kp = kp;
    m_forcePd.kd = kd;

    m_forcePID.setPID(kp,ki,kd);
    LOG(Info) << "Set CM " << name() << " position gains to kp = " << kp << ", kd = " << kd << ".";
}


void CM::setForceCalibration(double a, double b, double c) {
    TASBI_LOCK
    m_params.forceCalibA = a;
    m_params.forceCalibB = b;
    m_params.forceCalibC = c;
}

void CM::setPositionForceCalibration(double p2fA, double p2fB, double p2fC, double f2pA, double f2pB, double f2pC) {
    TASBI_LOCK
    m_params.posToFrcCalibA = p2fA;
    m_params.posToFrcCalibB = p2fB;
    m_params.posToFrcCalibC = p2fC;
    m_params.frcToPosCalibA = f2pA;
    m_params.frcToPosCalibB = f2pB;
    m_params.frcToPosCalibC = f2pC;
}
void CM::setForceFilterMode(FilterMode mode) {
    TASBI_LOCK
    m_forceFiltMode = mode;
}


void CM::setControlMode(CM::ControlMode mode) {
    TASBI_LOCK
    m_ctrlMode  = mode;
    m_ctrlValue = 0.0;
    if (m_ctrlMode == ControlMode::Torque)
        LOG(Info) << "Set CM " << name() << " Control Mode to Torque.";
    else if (m_ctrlMode == ControlMode::Position)
        LOG(Info) << "Set CM " << name() << " Control Mode to Position.";
    else if (m_ctrlMode == ControlMode::Force)
        LOG(Info) << "Set CM " << name() << " Control Mode to Force.";
}

void CM::setControlValue(double value) {
    TASBI_LOCK
    if (m_ctrlMode == ControlMode::Torque)
        m_ctrlValue = clamp(value, -1.0, 1.0);
    else
        m_ctrlValue = clamp(value, 0.0, 1.0);
    m_feedRate.tick();
}

void CM::setForceFilter(double cutoff) {
    TASBI_LOCK
    LOG(Info) << "Set CM " << name() << "force filter cutoff ratio to " << cutoff;
    m_forceFilterL.configure(2, cutoff);
}

void CM::setControlValueFilter(double cutoff) {
    TASBI_LOCK
    LOG(Info) << "Set CM " << name() << " control value filter cutoff ratio to " << cutoff;
    m_ctrlFilter.configure(2, cutoff);
}

void CM::enableControlValueFilter(bool enable) {
    TASBI_LOCK
    m_params.filterControlValue = enable;
    LOG(Info) << (m_params.filterControlValue ? "Enabled" : "Disabled")
              << " control value filtering on CM " << name() << ".";
}

void CM::setCustomController(std::shared_ptr<CMController> controller) {
    m_customController = controller;
}

void CM::getControllerIo(std::vector<double>& u, std::vector<double>& y) {
    TASBI_LOCK
    std::size_t avail = m_Q.size();
    for (std::size_t i = 0; i < avail; ++i) {
        u[i] = m_Q[i].ctrlValueScaled;
        y[i] = m_Q[i].spoolPosition;
    }
}

void CM::getFilterIo(std::vector<double>& u, std::vector<double>& y) {
    TASBI_LOCK
    std::size_t avail = m_Q.size();
    for (std::size_t i = 0; i < avail; ++i) {
        u[i] = m_Q[i].forceRaw;
        y[i] = m_Q[i].forceRawFiltered;
    }
}

//=============================================================================
// PRIVATE (NOT THREAD SAFE)
//=============================================================================

void CM::controlUpdate(double ctrlValue, Time t) {
    if (m_ctrlMode == ControlMode::Torque) {
        double torque = scaleCtrlValue(ctrlValue, ControlMode::Torque);
        setMotorTorque(torque);
    } else if (m_ctrlMode == ControlMode::Position) {
        double position = scaleCtrlValue(ctrlValue, ControlMode::Position);
        controlSpoolPosition(position, t);
    } else if (m_ctrlMode == ControlMode::Force) {
        double force = scaleCtrlValue(ctrlValue, ControlMode::Force);
        controlForce(force, t);
    } else if (m_ctrlMode == ControlMode::Force2) {
        double force = scaleCtrlValue(ctrlValue, ControlMode::Force);
        controlForce2(force, t);
    } else if (m_ctrlMode == ControlMode::Custom) {
        m_customController->update(ctrlValue, t, *this);
    }
}

void CM::setMotorTorque(double torque) {
    if (m_params.filterOutputValue)
        torque = m_outputFilter.update(torque);
    double amps = torque / m_params.motorTorqueConstant;
    double volts = amps / m_params.commandGain;
    m_io.commandCh.set_volts(volts);
}

void CM::controlMotorPosition(double degrees, Time t) {
    double motor_torque = m_positionPd.calculate(degrees, getMotorPosition(), 0, getMotorVelocity());
    setMotorTorque(motor_torque);
}

void CM::controlSpoolPosition(double degrees, Time t) {
    controlMotorPosition(degrees / m_params.gearRatio, t);
}

void CM::controlForce(double newtons, Time t) {
    double df_ref = m_forceRefDiff.update(newtons, t);
    double f_act  = getForce();
    double v_act  = getSpoolVelocity();
    double torque = m_forcePd.calculate(newtons,f_act,0,v_act);
    // ff term
    double torque_ff = scaleCtrlValue(m_params.forceKff, ControlMode::Torque);
    torque += torque_ff * newtons;
    // prevent unwinding when controller wants to do less force the the weight of cm
    if (getSpoolPosition() < -10 && torque < 0)
        torque = 0;
    setMotorTorque(torque);
}

void CM::controlForce2(double newtons, mahi::util::Time t) {
    double f_act = getForce();
    double err   = newtons - f_act;
    double derr  = m_forceDiff.update(err, t);
    double torque = m_params.forceKp * err + m_params.forceKd * derr;
    // double torque = m_forcePID.getOutput(f_act, newtons);
    // ff term
    double torque_ff = scaleCtrlValue(m_params.forceKff, ControlMode::Torque);
    torque += torque_ff * newtons;
    if (getSpoolPosition() < -10 && torque < 0)
        torque = 0;
    setMotorTorque(torque);
}

void CM::onUpdate(Time t) {
    // do nothing by default
}

int32 CM::getEncoderCounts() {
    return m_io.encoderCh.get_counts();
}

double CM::getEncoderCountsPerSecond() {
    return *m_io.cps;
}

double CM::getMotorTorqueCommand() {
    double volts = m_io.commandCh.get_volts();
    double amps = volts * m_params.commandGain;
    return amps * m_params.motorTorqueConstant;
}

double CM::getMotorPosition() { return m_io.encoderCh.get_pos(); }

double CM::getMotorVelocity() { return m_params.useSoftwareVelocity ? m_velocityFilter.get_value() : *m_io.vel; }

double CM::getSpoolPosition() { return getMotorPosition() * m_params.gearRatio; }

double CM::getSpoolVelocity() { return getMotorVelocity() * m_params.gearRatio; }

double CM::getForceRaw(bool filtered) {
    double raw = m_io.forceCh.get_volts();
    if (!filtered)
        return raw;
    switch(m_forceFiltMode) {
        case None:    return raw;
        case Lowpass: return m_forceFilterL.update(raw);
        case Median:  return m_forceFilterM.filter(raw);
        case Cascade: return m_forceFilterL.update(m_forceFilterM.filter(raw));
        default:      return raw;
    }
}

double CM::getForce(bool filtered) {
    double force_raw = getForceRaw(filtered);
    double force_con = convertForce(force_raw);
    return force_con;
}

double CM::convertForce(double volts) {
    double force  = m_params.forceCalibA + volts * m_params.forceCalibB + volts * volts * m_params.forceCalibC;
    return force;
}

double CM::positionToForce(double position) {
    double force = m_params.posToFrcCalibA + position * m_params.posToFrcCalibB + position * position * m_params.posToFrcCalibC;
    return force;
}

double CM::forceToPosition(double force) {
    double position = m_params.frcToPosCalibA + force * m_params.frcToPosCalibB + force * force * m_params.frcToPosCalibC;
    return position;
}

bool CM::velocity_limit_exceeded() {
    double m_velocity = getMotorVelocity();
    std::cout << m_velocity << " ";
    bool exceeded = false;
    if (m_params.has_velocity_limit_ && abs(m_velocity) > m_params.velocityMax) {
        LOG(Warning) << "Capstan Module " << name() << " velocity exceeded the velocity limit " << m_params.velocityMax << " with a value of " << m_velocity;
        exceeded = true;
        on_disable();
    }
    return exceeded;
}

bool CM::torque_limit_exceeded(double tor) {
    bool exceeded = false;
    if (m_params.has_torque_limit_ && abs(tor) > m_params.torqueMax) {
        LOG(Warning) << "Capstan Module " << name() << " command torque exceeded the torque limit " << m_params.torqueMax << " with a value of " << tor;
        exceeded = true;
        on_disable();
    }
    return exceeded;
}

double CM::scaleCtrlValue(double ctrlValue, ControlMode mode) {
    if (mode == ControlMode::Torque)
        return m_params.motorStallTorque * ctrlValue;
    else if (mode == ControlMode::Position)
        return m_params.positionMin + ctrlValue * (m_params.positionMax - m_params.positionMin);
    else if (mode == ControlMode::Force)
        return m_params.forceMin + ctrlValue * (m_params.forceMax - m_params.forceMin);
    else if (mode == ControlMode::Force2)
        return m_params.forceMin + ctrlValue * (m_params.forceMax - m_params.forceMin);
    else
        return 0;
}

bool CM::on_enable() {
    TASBI_LOCK
    if (m_io.enableCh.write_high()) {
        m_status = Status::Enabled;
        return true;
    }
    return false;
}

bool CM::on_disable() {
    TASBI_LOCK
    if (m_io.enableCh.write_low()) {
        m_status = Status::Disabled;
        return true;
    }
    return false;
}

void CM::fillQuery(CM::Query &q) {
    q.status             = m_status;
    q.counts             = getEncoderCounts();
    q.countsPerSecond    = getEncoderCountsPerSecond();
    q.motorPosition      = getMotorPosition();
    q.motorVelocity      = getMotorVelocity();
    q.motorTorqueCommand = getMotorTorqueCommand();
    q.spoolPosition      = getSpoolPosition();
    q.spoolVelocity      = getSpoolVelocity();
    q.forceRaw           = getForceRaw(false);
    q.forceRawFiltered   = getForceRaw(true);
    q.force              = getForce(false);
    q.forceFiltered      = getForce(true);
    q.forceEst           = positionToForce(getSpoolPosition());
    q.ctrlMode           = m_ctrlMode;
    q.filtMode           = m_forceFiltMode;
    q.ctrlValue          = m_ctrlValue;
    q.ctrlValueFiltered  = m_ctrlValueFiltered;
    q.ctrlValueScaled    = m_params.filterControlValue ? scaleCtrlValue(m_ctrlValueFiltered, m_ctrlMode) : scaleCtrlValue(m_ctrlValue, m_ctrlMode);
    q.lockCount = m_lockCount;
    q.feedRate  = m_feedRate.rate();
    q.dFdt      = m_forceDiff.get_value();
}