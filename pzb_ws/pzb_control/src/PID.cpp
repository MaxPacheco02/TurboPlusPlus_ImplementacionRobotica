#include "PID.h"
#include "algorithm"

PID::PID()
{
    sample_time_    = 0;
    k_p_            = 0;
    k_i_            = 0;
    k_d_            = 0;

    error_          = 0;
    prev_error_     = 0;
    chi1_d_         = 0;
    u_              = 0;

    U_MAX_ = 0;
    U_MIN_ = 0;

}

PID::PID(double sample_time, double k_p, double k_i, double k_d, double u_max, double u_min)
{
    sample_time_    = sample_time;
    k_p_            = k_p;
    k_i_            = k_i;
    k_d_            = k_d;

    error_          = 0;
    prev_error_     = 0;
    chi1_d_         = 0;
    u_              = 0;

    U_MAX_ = u_max;
    U_MIN_ = u_min;
}

void PID::updateReferences(double chi1_d)
{
    chi1_d_ = chi1_d;
}

void PID::calculateManipulation(double chi1)
{
    double error_d;
    double error_i;
    double u;

    prev_error_    = error_;
    error_         = chi1_d_ - chi1;

    error_d = (error_ - prev_error_) / sample_time_;
    error_i = ((error_ + prev_error_) / 2 * sample_time_) + error_;

    u  = k_p_ * error_ + k_i_ * error_i + k_d_ * error_d;
                                                               
    if(!isnan(u))
        u_ = u;
}

void PID::saturateManipulation(double chi1)
{
    calculateManipulation(chi1);
    u_ = std::clamp(u_, U_MIN_, U_MAX_);
}