#include "JetModel.h"
#include <cmath>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>

using namespace Eigen::numext;

JetModel::JetModel()
{
    // initialize the jet models
    m_u2TCoeff = {-4.64730485e-01,
                  -8.13171858e+00,
                  -6.19539230e+00,
                  6.61113140e-01,
                  1.67673231e+00,
                  -4.83287064e-01,
                  8.77996617e+00,
                  -1.01096376e+00,
                  -5.86442286e-01,
                  5.19093322e-01,
                  -4.23782666e-01,
                  -1.45705257e+00,
                  -7.83052261e-03};
    m_u2Tnormalization = {108.309, 65.793, 47.333, 31.483};
}

double JetModel::compute_f(double T, double Tdot)
{
    return m_u2TCoeff[0] + m_u2TCoeff[1] * T + m_u2TCoeff[2] * Tdot + m_u2TCoeff[3] * T * Tdot
           + m_u2TCoeff[4] * pow(T, 2.0) + m_u2TCoeff[5] * pow(Tdot, 2.0);
}

double JetModel::compute_df_dT(double T, double Tdot)
{
    return m_u2TCoeff[1] + m_u2TCoeff[3] * Tdot + 2 * m_u2TCoeff[4] * T;
}

double JetModel::compute_df_dTdot(double T, double Tdot)
{
    return m_u2TCoeff[2] + m_u2TCoeff[3] * T + 2 * m_u2TCoeff[5] * Tdot;
}

double JetModel::compute_dg_dT(double T, double Tdot)
{
    return m_u2TCoeff[7] + m_u2TCoeff[9] * Tdot + 2 * m_u2TCoeff[10] * T;
}

double JetModel::compute_dg_dTdot(double T, double Tdot)
{
    return m_u2TCoeff[8] + m_u2TCoeff[9] * T + 2 * m_u2TCoeff[11] * Tdot;
}

double JetModel::compute_g(double T, double Tdot)
{
    return m_u2TCoeff[6] + m_u2TCoeff[7] * T + m_u2TCoeff[8] * Tdot + m_u2TCoeff[9] * T * Tdot
           + m_u2TCoeff[10] * pow(T, 2.0) + m_u2TCoeff[11] * pow(Tdot, 2.0);
}

double JetModel::compute_v(double u)
{
    return u + m_u2TCoeff[12] * pow(u, 2.0);
}

double JetModel::standardizeThrust_u2T(double thrust)
{
    return (thrust - m_u2Tnormalization[0]) / m_u2Tnormalization[1];
}

double JetModel::standardizeThrustDot_u2T(double thrustDot)
{
    return (thrustDot) / m_u2Tnormalization[1];
}

double JetModel::standardizeThrottle_u2T(double throttle)
{
    return (throttle - m_u2Tnormalization[2]) / m_u2Tnormalization[3];
}

double JetModel::destandardizeThrust_u2T(double thrustBar)
{
    // destandardize thrust value
    return (thrustBar * m_u2Tnormalization[1] + m_u2Tnormalization[0]);
}

double JetModel::destandardizeThrustDot_u2T(double thrustDotBar)
{
    // destandardize thrust derivative value
    return thrustDotBar * m_u2Tnormalization[1];
}

double JetModel::destandardizeThrottle_u2T(double v)
{
    double u;
    // solve the equation Buu * u^2 + u - v = 0
    u = (-1 + sqrt(1 + 4 * m_u2TCoeff[12] * v)) / (2 * m_u2TCoeff[12]);
    // destandardize the throttle
    u = u * m_u2Tnormalization[3] + m_u2Tnormalization[2];
    // check if the throttle is within the limits
    if (u < 0)
    {
        u = 0;
    } else if (u > 100)
    {
        u = 100;
    }
    return u;
}

double JetModel::getThrustStandardDeviation_u2T()
{
    return m_u2Tnormalization[1];
}
