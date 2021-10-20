// Author(s): Evan Pezent (epezent@rice.edu)

#pragma once

template <typename T>
inline void ImMinMaxArray(const T* values, int count, T* min_out, T* max_out) {
    T Min = values[0]; T Max = values[0];
    for (int i = 1; i < count; ++i) {
        if (values[i] < Min) { Min = values[i]; }
        if (values[i] > Max) { Max = values[i]; }
    }
    *min_out = Min; *max_out = Max;
}

template <class Formatter>
class GuiLogWritter : public Writer {
public:
    GuiLogWritter(Severity max_severity = Debug) : Writer(max_severity), logs(500) {}

    virtual void write(const LogRecord& record) override {
        auto log =
            std::pair<Severity, std::string>(record.get_severity(), Formatter::format(record));
        logs.push_back(log);
    }
    RingBuffer<std::pair<Severity, std::string>> logs;
};

struct ScrollingData1D {
    const int        MaxSize;
    int              Offset;
    ImVector<double> Data;
    ScrollingData1D(int maxSize = 1000) : MaxSize(maxSize) {
        Offset = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(double v) {
        if (Data.size() < MaxSize)
            Data.push_back(v);
        else {
            Data[Offset] = v;
            Offset       = (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset = 0;
        }
    }
};

struct ScrollingData2D {
    const int             MaxSize;
    int                   Offset;
    ImVector<ImPlotPoint> Data;
    ScrollingData2D(int maxSize = 1000) : MaxSize(maxSize) {
        Offset = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(double x, double y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImPlotPoint(x, y));
        else {
            Data[Offset] = ImPlotPoint(x, y);
            Offset       = (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset = 0;
        }
    }
};

std::vector<double> generateShroeder(double f1, double f2, int Fs, int T) {
    auto eta = 1.0 / T;
    int  N   = (int)((f2 - f1) / eta) + 1;
    int  S   = Fs*T+1;
    std::vector<double> t(S);
    linspace(0.0,(double)T,t);
    std::vector<double> u(S,0);

    for (int k = 1; k <= N; ++k) {
        auto f_k   = f1 + (k-1)*eta;
        auto phi_k = -k*(k-1)*PI/N;
        for (int i = 0; i < S; ++i) {
            u[i] = u[i] + std::cos(2*PI*f_k*t[i] + phi_k);
        }
    }
    // demean
    auto mu = mean(u);
    for (auto& i : u)
        i = i - mu;
    // normalize
    auto u_abs = u;
    for (auto& i : u_abs)
        i = std::abs(i);
    auto mx = max_element(u_abs);
    for (auto& i : u)
        i = 0.5 + 0.5 * (i/mx);
    return u;
}

double rmse(const double* x1, const double* x2, int n) {
    double sum = 0;
    for (int i = 0; i < n; ++i) {
        double e = x2[i]-x1[i];
        sum += e*e;
    }
    return std::sqrt(sum/n);
}