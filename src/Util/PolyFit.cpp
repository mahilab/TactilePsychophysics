#include "PolyFit.hpp"
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;

void polyFit(const std::vector<double>& xv, const std::vector<double>& yv, std::vector<double>& coeff, int order) {
	Eigen::MatrixXd A(xv.size(), order + 1);
	Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
	for (size_t i = 0; i < xv.size(); i++) {
        double xx = 1;
        for (size_t j = 0; j < order + 1; j++) {
            A(i, j) = xx;
            xx *= xv[i];
        }
    }
	Eigen::VectorXd result = A.householderQr().solve(yv_mapped);
	coeff.resize(order+1);
	for (size_t i = 0; i < order+1; i++)
		coeff[i] = result[i];
}

double polyEval(double x, const std::vector<double>& coeff) {
    double xx = 1;
    double y = 0;
    for (auto& c : coeff) {
         y += xx * c;
         xx *= x;
    }
    return y;
}