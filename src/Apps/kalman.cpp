#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Robo.hpp>

using namespace mahi::gui;
using namespace mahi::util;
using namespace mahi::robo;

class KalmanFilter {
public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  /**
  * Create a blank estimator.
  */
  KalmanFilter();
  /**
  * Initialize the filter with initial states as zero.
  */
  void init();
  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0);
  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXd& y);
  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);
  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };
private:
  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;
  // System dimensions
  int m, n;
  // Initial and current time
  double t0, t;
  // Discrete time step
  double dt;
  // Is the filter initialized?
  bool initialized;
  // n-size identity
  Eigen::MatrixXd I;
  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};





class KalmanTesting : public Application {
public:

    KalmanTesting() : Application(500,500,"Kalman Testing") { 
        ImGui::DisableViewports();
    }

    void update() {
        ImGui::BeginFixed("Window",ImVec2(0,0),ImVec2(500,500),ImGuiWindowFlags_NoTitleBar);
        ImGui::DragDouble("F",&F,0.01f);
        ImGui::DragDouble("m",&m,0.01f);
        ImGui::DragDouble("b",&b,0.01f);
        ImGui::DragDouble("k",&k,0.01f);
        ImGui::DragDouble("x0",&x0,0.01f);
        ImGui::DragDouble("xd0",&xd0,0.01f);

        if (ImGui::Button("Simulate",ImVec2(-1,0)))
            simulate(0.001, 10000);
        ImPlot::SetNextPlotLimits(0,10,-1,1);
        if (ImPlot::BeginPlot("##Simulation","T [s]","X [m]", ImVec2(-1,-1))) {
            if (x_actual.size() > 0)
                ImPlot::PlotLine("Actual", &x_actual[0], (int)x_actual.size(), 0.001);
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    void simulate(double dt, int steps) {
        A(0,0) = 0;
        A(0,1) = 0;
        A(1,0) = -k/m;
        A(1,1) = -b/m;        
        B(0)   = 0;
        B(1)   = 1/m;
        X(0)   = x0;
        X(1)   = xd0;

        // Output matrix
        Eigen::Matrix<double, 1, 2> C;
        C << 1 , 0;
        // Process noise covariance
        Eigen::Matrix2d Q;
        Q << 0.031, 0, 0, 0.01;
        Q*= 0.02;
        // Measurement noise covariance
        Eigen::Matrix<double, 1, 1> R;
        R << 25;
        // Estimate error covariance
        Eigen::Matrix2d P;
        P << 0.5, 0, 0, 0.5;

        KalmanFilter kf(dt,A, C, Q, R, P);

        Eigen::Vector2d Xd;
        Eigen::Vector2d Xd_last = Eigen::Vector2d::Zero();

        x_actual.clear();
        x_actual.reserve(steps);
        x_actual.push_back(X(0));

        double xdd_last;
        double xd_last;


        for (int i = 1; i < steps; ++i) {
            Xd = A * X + B * F;
            // integrate xdd
            X(1) += dt * 0.5 * (Xd(1) + xdd_last);
            // integrate xd
            X(0) += dt * 0.5 * (X(1) + xd_last);
            // store states
            xdd_last = Xd(1);
            xd_last  = X(1);
            // save history         
            x_actual.push_back(X(0));
        }
    }

    double m = 1;    
    double b = 0.2;  
    double k = 1;
    double F = 0;
    double x0 = 1;
    double xd0 = 0;

    std::vector<double> x_actual;

    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::Vector2d X;
};

int main(int argc, char const *argv[])
{
    KalmanTesting app;
    app.run();
    return 0;
}

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {
  this->A = A;
  this->dt = dt;
  update(y);
}