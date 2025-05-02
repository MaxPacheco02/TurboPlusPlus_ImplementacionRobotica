#ifndef PID_h
    #define PID_h

#include <math.h>
#include <stdlib.h>

class PID
{
    public:
        double sample_time_;
        
        double error_;
        double prev_error_;

        double k_p_;
        double k_i_;
        double k_d_;

        double U_MIN_;
        double U_MAX_;
        
        double u_;
        double chi1_d_;

        // May even be usefull to create another constructor without u_max, as when PID is FBLinearized, the FBLin
        // base class already saturates the signals
        PID();
        PID(double sample_time, double k_p, double k_i, double k_d, double u_max, double u_min);
        
        void updateReferences(double chi1_d);
        void calculateManipulation(double chi1);
        
        // Saturate manipulation function is intended to be used in applications where a FBLin PID is not required,
        // as FBLin base classes already saturate the control signals
        void saturateManipulation(double chi1);
};

#endif