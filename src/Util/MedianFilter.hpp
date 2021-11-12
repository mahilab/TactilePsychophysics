// Written by Evan Pezent

#pragma once

#include <array>

class MedianFilter {
public:
    MedianFilter(int N)  { resize(N); }
    double filter(double sample) {
        size_t W = buffer1.size();
        buffer1[W-1] = sample;
        buffer2 = buffer1;
        std::sort(buffer2.begin(), buffer2.end());
        size_t M = W/2;
        value = buffer2[M];
        for (size_t i = 1; i < W; ++i)
            buffer1[i-1] = buffer1[i];
        return value;
    }   
    void resize(int N) { buffer1.resize(N); buffer2.resize(N); } 
    double get_value(){return value;}
private:
    double value;
    std::vector<double> buffer1;
    std::vector<double> buffer2;
};

template <int T>
class AverageFilter  {
public:
    AverageFilter() { 
        for (int i = 0; i < T; ++i)
            s[i] = 0;
    }
    double filter(double in) {
        s[T-1] = in;
        double sum = s[0];
        for (int i = 1; i < T; ++i) {
            sum += s[i];
            s[i-1] = s[i];
        }
        return sum / T;
    }
private:
    double s[T];
};