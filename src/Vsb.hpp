#pragma once

#include <Mahi/Daq.hpp>
#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo/Mechatronics/CurrentAmplifier.hpp>
#include <Mahi/Robo/Mechatronics/DcMotor.hpp>
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>
#include <mutex>

using namespace mahi::daq;
using namespace mahi::robo;
using namespace mahi::gui;
using namespace mahi::util;

class Vsb : public mahi::util::Device {
public:

    enum Mode {
        Button      = 0,
        Calibrator  = 1,
        Transparent = 2
    };

    struct Params {
        double motorNominalTorque  = 0.02860;        // [Nm]
        double motorStallTorque    = 0.12900;        // [Nm]
        double motorNominalCurrent = 1.24000;        // [A]
        double motorStallCurrent   = 5.50000;        // [A]
        double motorTorqueConstant = 0.02200;        // [Nm/A]
        double commandGain         = 5.0 / 10.0;     // [A/V]
        double senseGain           = 2.0;            // [A/V]
        double degPerCount         = 360 / 500.0;    // [deg/count]
        double spoolRadius         = 0.006;          // [m]
        double metersPerDeg        = 0.05 / 476.1;   // [m/deg]
        double mass                = 0.062;          // [kg]
        double friction            = 0.18;           // [N]
        double kp                  = 100;            // [N/m]
        double kd                  = 4.9;            // [N-s/m]
        double returnForce         = 0.10;           // [N/m/s]
        double buttonPosition      = -0.05;          // [m]
        double calibratorForce     = 5;              // [N]
        double calibratorPreload   = 0.1;            // [N]
        double calibratorFrequency = 1.0;            // [Hz]
        bool   gravityComp         = true;
        bool   frictionComp        = true;
    };

    struct Query {
        int counts;
        double countsPerSecond;
        double motorPosition;
        double motorVelocity;
        double motorTorqueSense;
        double motorTorqueCommand;    
        double linearPosition;
        double linearVelocity;
        double linearForceSense;
        double linearForceCommand;
        double ampsSense;
        double ampsCommand;
        double voltsSense;
        double voltsCommand;
        double time;
        int    misses;
    };

    Vsb(const std::string& name, ChanNum fs, ChanNum ch_enable = 0, ChanNum ch_command = 0, ChanNum ch_sense = 0, ChanNum ch_encoder = 0) :
        Device(name),
        m_ch_enable(ch_enable),
        m_ch_command(ch_command),
        m_ch_sense(ch_sense),
        m_ch_encoder(ch_encoder),
        m_mode(Button),
        m_running(false),
        m_timer(hertz(fs))
    {
        // configure QPID
        m_qpid.set_options(QuanserOptions());
        m_qpid.DO.set_channels({m_ch_enable});
        m_qpid.DO.write(m_ch_enable, TTL_LOW);
        m_qpid.DO.enable_values[m_ch_enable]  = TTL_LOW;   
        m_qpid.DO.disable_values[m_ch_enable] = TTL_LOW; 
        m_qpid.DO.expire_values.write(m_ch_enable, TTL_LOW); 
        m_qpid.AO.write(m_ch_command, 0);
        m_qpid.AO.expire_values.write(m_ch_command, 0);
        m_qpid.encoder.units.set(m_ch_encoder, m_params.degPerCount);
    }

    ~Vsb() {
        stop();
    }

    void start() {
        if (!m_running) {
            if (m_thread.joinable())
                m_thread.join();
            m_thread = std::thread(&Vsb::controlThreadFunc, this);
        }
    }

    void stop() {
        if (m_running) {
            m_running = false;
            if (m_thread.joinable())
                m_thread.join();
        }
    }

    void setMode(Mode mode) {
        std::lock_guard<std::mutex> lock(m_mutex);                                                  
        m_mode = mode;
    }

    Mode getMode() {
        std::lock_guard<std::mutex> lock(m_mutex);                                                  
        return m_mode;
    }

    Query query() {
        std::lock_guard<std::mutex> lock(m_mutex);                                                  
        Query q;
        q.counts              = getEncoderCounts();
        q.countsPerSecond     = getEncoderCountsPerSecond();
        q.motorPosition       = getMotorPosition();
        q.motorVelocity       = getMotorVelocity();
        q.motorTorqueSense    = getMotorTorqueSense();
        q.motorTorqueCommand  = getMotorTorqueCommand();
        q.linearPosition      = getLinearPosition();
        q.linearVelocity      = getLinearVelocity();
        q.linearForceSense    = getLinearForceSense();
        q.linearForceCommand  = getLinearForceCommand();
        q.ampsSense           = getAmpsSense();
        q.ampsCommand         = getAmpsCommand();
        q.voltsSense          = getVoltsSense();
        q.voltsCommand        = getVoltsCommand();
        q.time                = m_t.as_seconds();
        q.misses              = m_misses;
        return q;
    }

    void setParams(Params params) {
        std::lock_guard<std::mutex> lock(m_mutex);                                                    
        m_params = params;
        m_qpid.encoder.units.set(m_ch_encoder, m_params.degPerCount);
    }

    Params getParams() {
        std::lock_guard<std::mutex> lock(m_mutex);                                                    
        return m_params;
    }

    void setMass(double kg) {
        std::lock_guard<std::mutex> lock(m_mutex);                                                    
        m_params.mass = kg;
    }

    void setGains(double kp, double kd) {
        std::lock_guard<std::mutex> lock(m_mutex);                                                    
        m_params.kp = kp;      
        m_params.kd = kd;
    };

    void setStiffness(double kp, bool criticallyDamp = false) {
        std::lock_guard<std::mutex> lock(m_mutex);
        kp = clamp(kp, 5.0, 400.0);                                                
        m_params.kp = kp;
        if (criticallyDamp)
            m_params.kd = 2 * std::sqrt(kp * m_params.mass);
    }

    void addStiffness(double kp, bool criticallyDamp = false) {
        std::lock_guard<std::mutex> lock(m_mutex); 
        kp = m_params.kp + kp; 
        kp = clamp(kp, 5.0, 400.0);                                                  
        m_params.kp = kp;
        if (criticallyDamp)
            m_params.kd = 2 * std::sqrt(kp * m_params.mass);
    }

    double getStiffness() {
        std::lock_guard<std::mutex> lock(m_mutex); 
        return m_params.kp;
    }

    void setDamping(double kd) {
        std::lock_guard<std::mutex> lock(m_mutex);                                                    
        m_params.kd = kd;
    }

    void addDamping(double kd) {
        std::lock_guard<std::mutex> lock(m_mutex);                                                    
        m_params.kd = m_params.kd + kd;
    }

    double getDamping() {
        std::lock_guard<std::mutex> lock(m_mutex); 
        return m_params.kd;
    }

    void setButtonPosition(double pos) {
        std::lock_guard<std::mutex> lock(m_mutex);     
        m_params.buttonPosition = pos;                                               
    }

    void setReturnForce(double frc) {
        std::lock_guard<std::mutex> lock(m_mutex);     
        m_params.returnForce = frc;                                               
    }


    void setCalibratorParams(double force, double preload, double frequency) {
        std::lock_guard<std::mutex> lock(m_mutex);     
        m_params.calibratorForce     = force;
        m_params.calibratorPreload   = preload;
        m_params.calibratorFrequency = frequency;
    }

    void zeroPosition() {
        std::lock_guard<std::mutex> lock(m_mutex);                                                     
        m_qpid.encoder.zero(m_ch_encoder);
    }

    double getDisplacement() {
        std::lock_guard<std::mutex> lock(m_mutex); 
        auto pos = getLinearPosition();                                                    
        return pos - m_params.buttonPosition;
    }

    void enableGravityComp(bool enabled) {
        std::lock_guard<std::mutex> lock(m_mutex); 
        m_params.gravityComp = enabled;
    }

    void enableFrictionComp(bool enabled) {
        std::lock_guard<std::mutex> lock(m_mutex); 
        m_params.frictionComp = enabled;
    }

    void setFriction(double friction) {
        std::lock_guard<std::mutex> lock(m_mutex); 
        m_params.friction = friction;
    }

protected:

    void setLinearPosition(double meters, double feedforawrdForce, Time t) {
        double x_ref  = meters;
        double x      = getLinearPosition();
        double e_x    = x_ref - x;
        double xd_ref = 0;
        double xd     = getLinearVelocity();
        double e_xd   = xd_ref - xd;
        double force = m_params.kp * e_x + m_params.kd * e_xd + feedforawrdForce;
        setLinearForce(force);
    }

    void   setMotorTorque(double torque)   { setAmps(torque / m_params.motorTorqueConstant); }
    double getMotorTorqueSense()           { return getAmpsSense() * m_params.motorTorqueConstant; }
    double getMotorTorqueCommand()         { return getAmpsCommand() * m_params.motorTorqueConstant; }

    void   setLinearForce(double newtons)  { 
        if (m_params.gravityComp)
            newtons -= m_params.mass * mahi::util::G;
        if (m_params.frictionComp)
            newtons += m_params.friction * std::tanh(10 * getLinearVelocity());
        setMotorTorque(newtons * m_params.spoolRadius); 
    }    
    
    double getLinearForceSense()           { return getMotorTorqueSense() / m_params.spoolRadius; }
    double getLinearForceCommand()         { return getMotorTorqueCommand() / m_params.spoolRadius; }

    void   setAmps(double amps)            { setVolts(amps / m_params.commandGain); }
    double getAmpsSense()                  { return getVoltsSense() * m_params.senseGain; }
    double getAmpsCommand()                { return getVoltsCommand() * m_params.commandGain; }

    void   setVolts(double volts)          { m_qpid.AO.set(m_ch_command, volts); }
    double getVoltsSense()                 { return m_qpid.AI.get(m_ch_command); }
    double getVoltsCommand()               { return m_qpid.AO.get(m_ch_command); }

    int    getEncoderCounts()              { return m_qpid.encoder.get(m_ch_encoder); }
    double getEncoderCountsPerSecond()     { return m_qpid.velocity.get(m_ch_encoder); }    

    double getMotorPosition()             { return m_qpid.encoder.positions.get(m_ch_encoder); }
    double getMotorVelocity()             { return m_qpid.velocity.velocities.get(m_ch_encoder); }
    double getLinearPosition()            { return m_params.metersPerDeg * getMotorPosition(); }
    double getLinearVelocity()            { return m_params.metersPerDeg * getMotorVelocity(); }

    bool on_enable() override {
        std::lock_guard<std::mutex> lock(m_mutex);  
        return m_running && m_qpid.DO.write(m_ch_enable, TTL_HIGH);                                               
    }

    bool on_disable() override {
        std::lock_guard<std::mutex> lock(m_mutex);                                                     
        return m_qpid.DO.write(m_ch_enable, TTL_LOW);
    }

    bool update(const Time& t) {
        if (m_mode == Button) {
            double returnForce = (m_params.kp < 20 && getLinearVelocity() < -0.01) ? m_params.returnForce : 0;
            setLinearPosition(m_params.buttonPosition, -returnForce, t);
        }
        else if (m_mode == Calibrator) {
            double p = m_params.calibratorPreload;
            double a = m_params.calibratorForce / 2;
            double f = m_params.calibratorFrequency;
            double force = p + a + a * std::sin(2*PI*f*t.as_seconds());
            setLinearForce(force);
        }
        else if (m_mode == Transparent) {
            setLinearForce(0);
        }
        return true;
    }

    void controlThreadFunc() {
        LOG(Info) << "VSB started.";
        m_running = true;
        m_timer.restart();
        m_qpid.watchdog.clear();
        m_qpid.watchdog.set_timeout(50_ms);
        m_qpid.watchdog.start();
        volatile bool ok = true;
        while (m_running && ok) {
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_t      = m_timer.get_elapsed_time();
                m_misses = (int)m_timer.get_misses();
                ok       = ok && m_qpid.read_all();
                ok       = ok && update(m_t);
                ok       = ok && m_qpid.write_all();
                ok       = ok && m_qpid.watchdog.kick();
            }
            m_timer.wait();
        }
        m_qpid.watchdog.stop();
        if (is_enabled())
            disable();
        m_running = false;
        LOG(Info) << "VSB stopped.";
    }

private:
    // DAQ
    QPid              m_qpid;
    ChanNum           m_ch_enable;
    ChanNum           m_ch_command;
    ChanNum           m_ch_sense;
    ChanNum           m_ch_encoder;
    // Parameters
    Mode              m_mode;
    Params            m_params;
    Time              m_t;
    int               m_misses;
    // Threading 
    std::thread       m_thread;
    std::mutex        m_mutex;
    std::atomic_bool  m_running;
    Timer             m_timer;
};

namespace VsbGui { 

void ShowControls(Vsb& vsb) {
    if (ImGui::Button("Start")) 
        vsb.start();
    ImGui::SameLine();
    if (ImGui::Button("Stop")) 
        vsb.stop();
    ImGui::SameLine();
    if (ImGui::Button("Enable")) {
        vsb.zeroPosition();
        vsb.enable();
    }
    ImGui::SameLine();
    if (ImGui::Button("Disable"))
        vsb.disable();
    ImGui::SameLine();
    if (ImGui::Button("Zero")) 
        vsb.zeroPosition();
    
    auto params = vsb.getParams();
    auto mode   = vsb.getMode();
    
    if (ImGui::RadioButton("Button", mode == Vsb::Button)) 
        vsb.setMode(Vsb::Button);
    ImGui::SameLine();
    if (ImGui::RadioButton("Calibrator", mode == Vsb::Calibrator)) 
        vsb.setMode(Vsb::Calibrator);    
    ImGui::SameLine();
    if (ImGui::RadioButton("Transparent", mode == Vsb::Transparent)) 
        vsb.setMode(Vsb::Transparent);    

    if (mode == Vsb::Button) {   
        static bool criticallyDamp = true;
        if (ImGui::DragDouble("Kp", &params.kp, 1, 0, 400)) 
            vsb.setStiffness(params.kp, criticallyDamp);     
        ImGui::SameLine();
        ImGui::Checkbox("Critically Damp", &criticallyDamp);   
        if (ImGui::DragDouble("Kd", &params.kd, 1, 0, 20)) 
            vsb.setDamping(params.kd);      
        if (ImGui::DragDouble("Position", &params.buttonPosition, 0.001f, -0.050, 0)) 
            vsb.setButtonPosition(params.buttonPosition);  
        if (ImGui::DragDouble("Return Force", &params.returnForce, 0.001f, 0, 1)) 
            vsb.setReturnForce(params.returnForce);        
    }
    else if (mode == Vsb::Calibrator) {
        if (ImGui::DragDouble("Force", &params.calibratorForce, 0.1f, 0, 12))
            vsb.setCalibratorParams(params.calibratorForce, params.calibratorPreload, params.calibratorFrequency);
        if (ImGui::DragDouble("Preload", &params.calibratorPreload, 0.01f, 0, 1))
            vsb.setCalibratorParams(params.calibratorForce, params.calibratorPreload, params.calibratorFrequency); 
        if (ImGui::DragDouble("Frequency", &params.calibratorFrequency, 0.01f, 0, 2))
            vsb.setCalibratorParams(params.calibratorForce, params.calibratorPreload, params.calibratorFrequency);    
    }
    if (mode == Vsb::Button || mode == Vsb::Transparent) {
        if (ImGui::DragDouble("Mass", &params.mass, 0.0001f, 0, 0.1, "%.4f"))
            vsb.setMass(params.mass);
        ImGui::SameLine();
        if (ImGui::Checkbox("##Gravity", &params.gravityComp))
            vsb.enableGravityComp(params.gravityComp);        
        if (ImGui::DragDouble("Friction", &params.friction, 0.0001f, 0, 0.2, "%.4f"))
            vsb.setFriction(params.friction);
        ImGui::SameLine();
        if (ImGui::Checkbox("##Friction", &params.frictionComp))
            vsb.enableFrictionComp(params.frictionComp); 
        if (ImGui::DragDouble("Radius", &params.spoolRadius, 0.0001f, 0.05, 0.01, "%.4f"))
            vsb.setParams(params);
        if (ImGui::DragDouble("Torque Constant", &params.motorTorqueConstant, 0.0001f, 0.02, 0.03, "%.4f"))
            vsb.setParams(params);
    }    
}

void ShowQuery(Vsb::Query& q) {
    ImGui::Value("Position", (float)q.linearPosition);   
    ImGui::Value("Velocity", (float)q.linearVelocity);
    ImGui::Value("Force", (float)q.linearForceSense);
}

} // namespace VsbGui