// MIT License
//
// AtiWindowCal - Mechatronics Engine & Library
// Copyright (c) 2021 Mechatronics and Haptic Interfaces Lab - Rice University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// Author(s): Janelle Clark (janelle.clark@rice.edu) and Nathan Dunkelberger

#pragma once

#include <Mahi/Daq.hpp>
#include <Mahi/Robo/Control/PdController.hpp>
#include <Mahi/Robo/Mechatronics/ForceSensor.hpp>
#include <Mahi/Robo/Mechatronics/AtiSensor.hpp>
#include <Mahi/Robo/Mechatronics/CurrentAmplifier.hpp>
#include <Mahi/Robo/Mechatronics/DcMotor.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Util/Coroutine.hpp>

#include <mutex>

#include "Util/RateMonitor.hpp"
#include "Util/MedianFilter.hpp"
#include "Util/MiniPID.hpp"

// Written by Janelle Clark with Nathan Dunkelberger, based off code by Evan Pezent

#define TASBI_THREAD_SAFE
#ifdef TASBI_THREAD_SAFE
#define TASBI_LOCK                                                                                 \
    std::lock_guard<std::mutex> lock(m_mutex);                                                     \
    m_lockCount++;
#else
#define TASBI_LOCK
#endif

using mahi::daq::AIHandle;
using mahi::daq::AOHandle;
using mahi::daq::DIHandle;
using mahi::daq::DOHandle;
using mahi::daq::EncoderHandle;
using mahi::robo::PdController;
using mahi::robo::ForceSensor;
using mahi::robo::AtiSensor;
using mahi::robo::Axis;
using mahi::util::Butterworth;
using mahi::util::Time;
using mahi::util::Differentiator;
using mahi::util::RingBuffer;

class CM;

struct CMController {
    virtual ~CMController()                                { }
    virtual void update(double ctrlValue, Time t, CM& cm) { }
};

class CM : public mahi::util::Device {
public:
    /// State
    enum Status : int {
        Disabled = 0,  ///< motor controller is Disabled
        Enabled  = 1   ///< motor controller is Enabled
    };

    /// Control Mode
    enum ControlMode : int {
        Torque      = 0,  ///< motor torque control
        Position    = 1,  ///< spool position control
        Force       = 2,  ///< force control
        ForceHybrid = 3,  ///< force control (hybrid with velocity)
        Custom      = 5   ///< custom controller
    };

    /// Force Filter Type
    enum FilterMode : int {
        None    = 0,
        Lowpass = 1,
        Median  = 2,
        Cascade = 3
    };

    /// CM IO Configuration
    struct Io {
        DOHandle      enableCh;   //< digital output that enables motor controller
        DIHandle      faultCh;    //< digital input that detects motor controller fault
        AOHandle      commandCh;  //< analog output that commands motor controller
        EncoderHandle encoderCh;  //< encoder input for motor
        ForceSensor&  forceCh;    //< force sensor, either analog input or ati
        Axis          forceaxis;  //< The ati axis relevant for this dof, defaults to X in case of an AI force sensor
        const double *cps;        //< pointer to encoder counts per second value
        const double *vel;        //< pointer to encoder deg/s value
    };

    /// CM Parameter Configuration
    struct Params {
        double motorNominalTorque  = 0.00634;        // [Nm]
        double motorStallTorque    = 0.0197;        // [Nm]
        double motorNominalCurrent = 0.452;        // [A]
        double motorStallCurrent   = 1.35;        // [A]
        double motorTorqueConstant = 0.0146;        // [Nm/A] make 14.6 [mNm/A]???? need to adjust kp/kd, maybe why they're multiplied by 10e3 in tasbi code for gui
        double motorNominalSpeed   = 31320.0;        // [deg/s]
        double motorMaxSpeed       = 46440.0;        // [deg/s]
        double gearRatio           = 0.332*25.4*mahi::util::PI/360.0;    // [mm/deg] from spool pitch diameter (.332") and capstan radius if applicable, converted to mm
        double degPerCount         = 360.0 / (1024.0 * 35.0); // [deg/count] for motor shaft, including gearbox if applicable
        double commandGain         = 1.35 / 10.0;   // [A/V]
        bool   posCmdSignFlip      = true;
        bool   posSenseSignFlip    = false;
        bool   forceCmdSignFlip    = true;
        bool   forceSenseSignFlip  = true;
        bool   has_velocity_limit_ = true;
        bool   has_torque_limit_   = true;
        double velocityMax         = 30.0; // [mm/s] ????
        double torqueMax           = 0.02; // [Nm]
        double positionMin         = -3.5; // [deg] ???? 
        double positionMax         = 3.0;  // [deg] ????
        double positionKp          = 0.5/(1e3); //?????
        double positionKd          = 0.01/(1e3); //????
        double forceMin            = -10.0;            // [N] ?????
        double forceMax            = 10.0;             // [N] ?????
        double forceKp             = 1500.0/(1e6);
        double forceKi             = 0;
        double forceKd             = 20.0/(1e6);
        double forceKff            = 0;
        double forceFilterCutoff   = 2000;        
        double dFdtFilterCutoff   = 0.25;        
        int    forceFilterN        = 31;  
        double cvFilterCutoff      = 0.02;           // normalized [0,1]
        bool   filterControlValue  = false;           // [true/false]
        double outputFilterCutoff  = 0.2;            //0.2
        bool   filterOutputValue   = true;          //
        double velFilterCutoff     = 0.2;
        bool   useSoftwareVelocity = false;
    };

    /// CM Query
    struct Query {
        int         time               = 0;
        int         status             = Status::Disabled;
        int         counts             = 0;
        double      countsPerSecond    = 0;
        double      motorPosition      = 0;
        double      motorVelocity      = 0;
        double      motorTorqueCommand = 0;
        double      spoolPosition      = 0;
        double      spoolVelocity      = 0;
        double      force              = 0;
        double      forceFiltered      = 0;
        int         ctrlMode           = ControlMode::Torque;
        int         filtMode           = FilterMode::Lowpass;
        double      ctrlValue          = 0;
        double      ctrlValueFiltered  = 0;
        double      ctrlValueScaled    = 0;
        int         lockCount          = 0;
        double      feedRate           = 0;
        double      dFdt               = 0;
    };

//----------------------------------------------------------------------------------
// THREAD SAFE FUNCTIONS 
//----------------------------------------------------------------------------------

    /// Constructor
    CM(const std::string &name, Io io, Params params);
    /// Destructor
    ~CM();
    /// Updates the CM device (thread safe)
    void update(const mahi::util::Time &t);
    /// Configures a CM (thread safe)
    void setParams(Params params);
    /// Exports configuration to JSON (thread safe)
    bool exportParams(const std::string& filepath);
    /// Imports configuration from JSON (thread safe)
    bool importParams(const std::string& filepath);
    /// Gets a CM configuration (thread safe)
    Params getParams() const;
    /// Queries CM for full state information (thread safe)
    Query getQuery(bool immediate = false);
    /// Get most recent 10k Queries. WILL TEMPORAIRLY STALL CONTROLLER (thread safe)
    void dumpQueries(const std::string& filepath);
    /// Set boolean to flip command current for position control if necessary
    void setPosCtrlCmdSign(bool cmdSignFlip);
    /// Set boolean to flip command current for force/torque control if necessary
    void setForceCtrlCmdSign(bool cmdSignFlip);
    /// Set boolean to flip position sensing if necessary
    void setPositionSenseSign(bool posSignFlip);
    /// Set boolean to flip force sensing if necessary
    void setForceSenseSign(bool forceSignFlip);
    /// Zero force sensor (thread safe)
    void zeroForce();
    /// Zero position to current value (thread safe)
    void zeroPosition();
    /// Zero position to exact value(thread safe)
    void zeroPosition(double position);
    /// Sets velocity limit (thread safe)
    void setVelocityMax(double vel, bool has_limit_);
    /// Sets torque limit (thread safe)
    void setTorqueMax(double tor, bool has_limit_);
    /// Sets position control range (thread safe)
    void setPositionRange(double min, double max);
    /// Sets spool position control PD gains (thread safe)
    void setPositionGains(double kp, double kd);
    /// Sets force control range (thread safe)
    void setForceRange(double min, double max);
    /// Sets spool position control PD gains (thread safe)
    void setForceGains(double kp, double ki, double kd);
    /// Sets the normalize cutoff ratio of the force filter (thead safe)
    void setForceFilter(double cutoff);
    /// Sets the normalize cutoff ratio of the force derivative filter (thead safe)
    void setdFdtFilter(double cutoff);
    /// Sets the force filter mode (thread safe)
    void setForceFilterMode(FilterMode mode);
    /// Sets the force derivative filter mode (thread safe)
    void setdFdtFilterMode(FilterMode mode);
    /// Sets the control mode used  (thread safe)
    void setControlMode(ControlMode mode);
    /// Sets the normalized value [-1 to 1] for torque or [0 to 1] for position/force (thread safe)
    void setControlValue(double value);
    /// Sets the normalized cutoff ratio of the control filter
    void setControlValueFilter(double cutoff);
    /// Enables/Disables control value filtering (thread safe)
    void enableControlValueFilter(bool enable);
    /// Sets controller to be used in ControlMode::Custom (thread safe)
    void setCustomController(std::shared_ptr<CMController> controller);
    /// Copies controller input/output history to buffers (thread safe)
    void getControllerIo(std::vector<double>& u, std::vector<double>& y);
    /// Copies filter input/output history to buffers (thread safe)
    void getFilterIo(std::vector<double>& u, std::vector<double>& y);

//----------------------------------------------------------------------------------
// UNSAFE FUNCTIONS (ONLY CALL THESE FROM WITHIN A TASBI CONTROLLER UPDATE METHOD)
//----------------------------------------------------------------------------------

    /// The control update (passed normalized control value) (DO NOT LOCK)
    virtual void controlUpdate(double ctrlValue);
    /// Implements motor position controller (DO NOT LOCK)
    virtual void controlMotorPosition(double degrees);
    /// Implements spool position controller (DO NOT LOCK)
    virtual void controlSpoolPosition(double degrees);
    /// Implements force controller (DO NOT LOCK)
    virtual void controlForce(double newtons);
    /// Implements hybrid force control with velocity instead of dF (DO NOT LOCK)
    virtual void controlForceHybrid(double newtons);
    /// Called inside of update after controlUpdate (does nothing by default) (DO NOT LOCK)
    virtual void onUpdate();

    /// Sets the current motor torque output [Nm]
    void setMotorTorque(double torque);
    /// Returns the commanded motor torque in [Nm]
    double getMotorTorqueCommand();
    /// Returns counts of motor encoder
    int getEncoderCounts();
    /// Returns the motor encoder speed
    double getEncoderCountsPerSecond();
    /// Returns the motor position in degrees
    double getMotorPosition();
    /// Return the motor velocity in degrees/second
    double getMotorVelocity();
    /// Returns the position of the spool in degrees
    double getSpoolPosition();
    /// Returns the spool velocity in [deg/s]
    double getSpoolVelocity();
    /// Returns the force sensor reading in [N]
    virtual double getForce(bool filtered = true, bool forUpdate = false);
    /// Returns the derivative of the force sensor reading in [N/s]
    virtual double getdFdt(bool filtered = true, bool forUpdate = false);
    /// Ensures the velocity does not exceed the velocity limit
    bool velocity_limit_exceeded();
    /// Ensures the torque does not exceed the torque limit
    bool torque_limit_exceeded();
    /// Ensures the torque and velocity do not exceed acceptable limits
    void limits_exceeded();
    /// Convert control reference value for to the normalized value [-1 to 1] for torque or [0 to 1] for position/force 
    virtual double scaleRefToCtrlValue(double ref);
    /// Scales the current control value into the units corresponding to the mode.
    virtual double scaleCtrlValue(double ctrlValue, ControlMode mode);
    /// Called when CM is enabled (thread safe)
    virtual bool on_enable() override;
    /// Called when CM is disabled (thread safe)
    virtual bool on_disable() override;
    /// Fills a Query with current state information
    void fillQuery(Query &q);

public:
double m_torque=0;
//Force Ringbuffer
mahi::util::RingBuffer<double> FBuff{30};
Params      m_params;    ///< parameters
Butterworth  m_outputFilter;

protected:
    // Status and Congiguration
    Status      m_status;    ///< status
    Time        m_t;
    Io          m_io;        ///< IO config
    //Params      m_params;    ///< parameters
    ControlMode m_ctrlMode;  ///< mode of control
    Query       m_q;         ///< most recent Query point
    RingBuffer<Query> m_Q;   ///< 10k Query history
    // Control
    PdController m_positionPd;         ///< position PD controller
    PdController m_forcePd;            ///< force PD controller
    MiniPID      m_forcePID;
    Differentiator m_forceDiff;
    Butterworth  m_ctrlFilter;         ///< butterworth filter that smooths control value setpoint (i.e. anti-aliases Unity 90 Hz commands)
    FilterMode   m_forceFiltMode;
    Butterworth  m_forceFilterL;        ///< filters raw voltage from integrated force sensor
    MedianFilter m_forceFilterM;
    FilterMode   m_dFdtFiltMode;
    Butterworth  m_dFdtFilterL;        ///< filters raw voltage from derivative of integrated force sensor
    MedianFilter m_dFdtFilterM;
    AverageFilter<21> m_forceFilterA;
    //Butterworth  m_outputFilter;
    Differentiator m_posDiff;
    Butterworth    m_velocityFilter;
    Differentiator m_forceRefDiff;

    double       m_ctrlValue;          ///< raw control value
    double       m_ctrlValueFiltered;  ///< filtered control value
    RateMonitor  m_feedRate;           ///< monitors ctrl value feed rate
    std::shared_ptr<CMController> m_customController;
    // Threading
    mutable std::mutex m_mutex;      ///< mutex for thready safety
    mutable int        m_lockCount;  ///< the number of times the mutex has been locked
};