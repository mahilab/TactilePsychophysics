#pragma once
#include <vector>

void polyFit(const std::vector<double>& xv, const std::vector<double>& yv, std::vector<double>& coeff, int order);
double polyEval(double x, const std::vector<double>& coeff);