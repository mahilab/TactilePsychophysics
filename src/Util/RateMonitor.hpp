#pragma once
// Written by Evan Pezent

#include <Mahi/Util/Timing/Time.hpp>

class RateMonitor {
public:

    RateMonitor(mahi::util::Time updateInterval = mahi::util::seconds(1)) : 
        m_updateInterval(updateInterval),
        m_nextUpdateTime(m_updateInterval),
        m_rate(0), m_ticks(0)
    { }

    void tick() {
        m_ticks++;
    }

    void update(const mahi::util::Time& t) {
        if (t > m_nextUpdateTime) {
            m_rate = m_ticks / m_updateInterval.as_seconds();
            m_nextUpdateTime += m_updateInterval;
            m_ticks = 0;
        }
    }

    double rate() const {
        return m_rate;
    }

private:
    mahi::util::Time m_updateInterval;
    mahi::util::Time m_nextUpdateTime;
    double m_rate;
    double m_ticks;
};