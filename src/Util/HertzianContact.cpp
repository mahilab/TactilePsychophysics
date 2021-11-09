#include <Util/HertzianContact.hpp>

////////////////////////////////////////////////////////////////////
//    Make Query with measurement data
////////////////////////////////////////////////////////////////////

namespace ContactMechanics{

HertzianContact::QueryHZ HertzianContact::makeQuery_Normal(double R, double Fn, double deltaN){
    m_q.R = R*1e-3;
    m_q.Fn = Fn;
    m_q.deltaN = deltaN*1e-3;
    fillQuery_Normal();
    return m_q;
}

HertzianContact::QueryHZ HertzianContact::makeQuery_TanNoSlip(double R, double Fn, double Ft, double deltaN, double deltaT){
    m_q.R = R*1e-3;
    m_q.Fn = Fn;
    m_q.Ft = Ft;
    m_q.deltaN = deltaN*1e-3;
    m_q.deltaT = deltaT*1e-3;
    fillQuery_TanNoSlip();
    return m_q;
}

HertzianContact::QueryHZ HertzianContact::makeQuery_TanPartialSlip(double R, double Fn, double Ft, double deltaN, double deltaT, double mu){
    m_q.R = R*1e-3;
    m_q.Fn = Fn;
    m_q.Ft = Ft;
    m_q.deltaN = deltaN*1e-3;
    m_q.deltaT = deltaT*1e-3;
    m_q.mu = mu;
    fillQuery_TanPartialSlip();
    return m_q;
}

////////////////////////////////////////////////////////////////////
//    Print Query
////////////////////////////////////////////////////////////////////

void HertzianContact::printQuery(){
    std::cout << "R " << m_q.R << std::endl;                    // [mm] Radius of the indentor
    std::cout << "a " << m_q.a << std::endl;                    // [mm] radius of the contact area of the indentor
    std::cout << "planarA " << m_q.planarA << std::endl;        // [m^2] contact area calculated as if 2D
    std::cout << "sphericalA " << m_q.sphericalA << std::endl;  // [m^2] contact area calculated as if 3D
    std::cout << "Fn " << m_q.Fn << std::endl;                  // [N] Normal Load
    std::cout << "Ft " << m_q.Ft << std::endl;                  // [N] Tangential Load
    std::cout << "deltaN " << m_q.deltaN << std::endl;          // [mm] Normal Displacement, indentation depth
    std::cout << "deltaT " << m_q.deltaT << std::endl;          // [mm] Tangential Displacement
    std::cout << "combinedE " << m_q.combinedE << std::endl;    // combined Young's modulus, E*
    std::cout << "Wn " << m_q.Wn << std::endl;                  // [??? N^(7/3)/mm^(5/3)] Normal elastic strain energy
    std::cout << "couplingP " << m_q.couplingP << std::endl;    // beta - coupling parameter between normal and tangential load
    std::cout << "E " << m_q.E << std::endl;                    // Young's Modulus for elasticity of the infinite plane
    std::cout << "v " << m_q.v << std::endl;                    // nu - Poisson's ratio for the infinite plane
    std::cout << "G " << m_q.G << std::endl;                    // Shear Modulus of Rigidity of the infinite plane
    std::cout << "meanStress " << m_q.meanStress << std::endl;  // mean stress, sigma*
    std::cout << "meanStrain " << m_q.meanStrain << std::endl;  // mean strain, epsilon*
    std::cout << "Wt " << m_q.Wt << std::endl;                  // Work done by the tangential force during a complete cycle with microslip. Johnson (7.60)
    std::cout << "mu " << m_q.mu << std::endl;                  // mu - frictional coefficient
    std::cout << "complianceN " << m_q.complianceN << std::endl;// [mm/N] normal compliance (d deltaN/ d Fn)
    std::cout << "complianceT " << m_q.complianceT << std::endl;// [mm/N] tangential compliance (d deltaT/ d Ft)
}

////////////////////////////////////////////////////////////////////
//    Direct Calculations
////////////////////////////////////////////////////////////////////

// Normal Loading
double HertzianContact::getCombinedE_Normal(double R, double Fn, double deltaN){
    double combinedE = 3.0*Fn/(4.0*pow(R*1e-3,0.5)*pow(deltaN*1e-3,1.5));
    return combinedE;
}

double HertzianContact::getContactRadius_Normal(double R, double deltaN){
    double a = std::sqrt(deltaN*1e-3*R*1e-3);
    return a;
}

double HertzianContact::getPlanarA_Normal(double R, double deltaN){
    double a = getContactRadius_Normal(R,deltaN);
    double A = m_pi*pow(a,2.0);
    return A;
}

double HertzianContact::getSphericalA_Normal(double R, double deltaN){
    double a = getContactRadius_Normal(R,deltaN);
    double A = 2.0*m_pi*a*deltaN*1e-3;
    return A;
}

double HertzianContact::getMeanStress_Normal(double R, double Fn, double deltaN){
    double a = getContactRadius_Normal(R, deltaN);
    double meanStress = 0.2*Fn/(m_pi*pow(a,2.0));
    return meanStress;
}

double HertzianContact::getMeanStrain_Normal(double R, double deltaN){
    double a = getContactRadius_Normal(R, deltaN);
    double meanStrain = 0.2*a/R*1e-3;
    return meanStrain;
}

double HertzianContact::getElasticStrainEnergy_Normal(double R, double Fn, double deltaN){
    double Ec = getCombinedE_Normal(R,Fn,deltaN);
    double W = (2.0/5.0)*pow(9*pow(Ec,2.0)*pow(Fn,5.0)/(16.0/(R*1e-3)),1.0/3.0);
    return W;
}

// Tangential Loading

double HertzianContact::getCouplingParameter_Tan(double v){
    double couplingP = (1.0 - 2.0*v)/(2.0*(1.0-v));
    return couplingP;
}

std::vector<double> HertzianContact::getYoungAndPoisson_TanNoSlip(double R, double Fn, double Ft, double deltaN, double deltaT){
    double a = getContactRadius_Normal(R, deltaN);
    double C = 6.0*Fn*deltaT*1e-3/(Ft*deltaN*1e-3);
    double v = (C-2)/(C-1);
    double E = 3.0*Fn*(1-pow(v,2.0))/(4.0*a*deltaN*1e-3);
    return {E,v};
}

std::vector<double> HertzianContact::getYoungAndPoisson_TanPartialSlip(double R, double Fn, double Ft, double deltaN, double deltaT, double mu){
    double a = getContactRadius_Normal(R, deltaN);
    double C = (4.0*deltaT*1e-3/(deltaN*1e-3))/(1.0-pow(1.0 - (Ft/mu*Fn),2.0/3.0));
    double v = (C-2)/(C-1);
    double E = 3.0*Fn*(1.0-pow(v,2.0))/(4.0*a*deltaN*1e-3);
    return {E,v};
}

double HertzianContact::getShearModulus_Tan(double E, double v){
    double G = E/(2.0*(1.0+v));
    return G;
}

std::vector<double>  HertzianContact::getCompliance_Tan(double E, double v, double a){
    double G = getShearModulus_Tan(E,v);
    double Cn = (1.0-v)/(2*G*a);
    double Ct = (2.0-v)/(4*G*a);
    return {Cn,Ct};
}

double HertzianContact::getWorkofCycle_Tan(double R, double Fn, double Ft, double deltaN, double deltaT, double mu){ // valid at peak Ft
    double a = getContactRadius_Normal(R, deltaN);
    std::vector<double> V = getYoungAndPoisson_TanPartialSlip(R,Fn,Ft,deltaN,deltaT,mu); // Question - microslip same as partial slip?
    double E = V[0];
    double v = V[1];
    double G = getShearModulus_Tan(E,v);
    double W = (9*pow(mu*Fn,2.0)/(10*a))*((2-v)/G)*(1-pow(1-(Ft/(mu*Fn)),5.0/3.0) - (5.0*Ft/(6.0*mu*Fn))*(1-pow(1-(Ft/(mu*Fn)),2/3))); // Question - P0 same as Fn?
    return W;
}

////////////////////////////////////////////////////////////////////
//    Fill Query with measurement data
////////////////////////////////////////////////////////////////////

void HertzianContact::fillQuery_Normal(){
    getCombinedE_Normal();
    getContactRadius_Normal();
    getPlanarA_Normal();
    getSphericalA_Normal();
    getMeanStress_Normal();
    getMeanStrain_Normal();
    getElasticStrainEnergy_Normal();
}

void HertzianContact::fillQuery_TanNoSlip(){
    fillQuery_Normal();
    getYoungAndPoisson_TanNoSlip();
    getCouplingParameter_Tan();
    getShearModulus_Tan();
    getCompliance_Tan();
}

void HertzianContact::fillQuery_TanPartialSlip(){
    fillQuery_Normal();
    getYoungAndPoisson_TanPartialSlip();
    getCouplingParameter_Tan();
    getShearModulus_Tan();
    getCompliance_Tan();
}


////////////////////////////////////////////////////////////////////
//    Normal Indentation Functions
////////////////////////////////////////////////////////////////////

void HertzianContact::getCombinedE_Normal(){
    m_q.combinedE = 3.0*m_q.Fn/(4.0*pow(m_q.R,0.5)*pow(m_q.deltaN,1.5));
}

void HertzianContact::getContactRadius_Normal(){
    m_q.a = std::sqrt(m_q.deltaN*m_q.R);
}

void HertzianContact::getPlanarA_Normal(){
    m_q.planarA = m_pi*pow(m_q.a,2.0);
}

void HertzianContact::getSphericalA_Normal(){
    m_q.sphericalA = 2.0*m_pi*m_q.a*m_q.deltaN;
}

void HertzianContact::getMeanStress_Normal(){
    m_q.meanStress = 0.2*m_q.Fn/(m_pi*pow(m_q.a,2.0));
}

void HertzianContact::getMeanStrain_Normal(){
    m_q.meanStrain = 0.2*m_q.a/m_q.R;
}

void HertzianContact::getElasticStrainEnergy_Normal(){
    m_q.Wn = (2.0/5.0)*pow(9*pow(m_q.combinedE,2.0)*pow(m_q.Fn,5.0)/(16.0/m_q.R),1.0/3.0);
}


////////////////////////////////////////////////////////////////////
//    Tangential Loading Functions
////////////////////////////////////////////////////////////////////

void HertzianContact::getCouplingParameter_Tan(){
    m_q.couplingP = (1.0 - 2.0*m_q.v)/(2.0*(1.0-m_q.v));
}

void HertzianContact::getYoungAndPoisson_TanNoSlip(){
    double C = 6.0*m_q.Fn*m_q.deltaT/(m_q.Ft*m_q.deltaN);
    m_q.v = (C-2.0)/(C-1.0);
    m_q.E = 3.0*m_q.Fn*(1.0-pow(m_q.v,2.0))/(4.0*m_q.a*m_q.deltaN);
}

void HertzianContact::getYoungAndPoisson_TanPartialSlip(){
    double C = (4.0*m_q.deltaT/m_q.deltaN)/(1.0-pow(1.0 - m_q.Ft/m_q.mu*m_q.Fn,2.0/3.0));
    m_q.v = (C-2)/(C-1);
    m_q.E = 3.0*m_q.Fn*(1.0-pow(m_q.v,2.0))/(4.0*m_q.a*m_q.deltaN);
}

void HertzianContact::getShearModulus_Tan(){
    m_q.G = m_q.E/(2.0*(1.0+m_q.v));
}

void  HertzianContact::getCompliance_Tan(){
    m_q.complianceN = (1.0-m_q.v)/(2*m_q.G*m_q.a);
    m_q.complianceT = (2.0-m_q.v)/(4*m_q.G*m_q.a);
}

void HertzianContact::getWorkofCycle_Tan(){ // valid at peak Ft
    double W = (9*pow(m_q.mu*m_q.Fn,2.0)/(10*m_q.a))*((2-m_q.v)/m_q.G)*(1-pow(1-(m_q.Ft/(m_q.mu*m_q.Fn)),5.0/3.0) - (5.0*m_q.Ft/(6.0*m_q.mu*m_q.Fn))*(1 - pow(1-(m_q.Ft/(m_q.mu*m_q.Fn)),2/3))); // Question - P0 same as Fn?
}

} // namespace