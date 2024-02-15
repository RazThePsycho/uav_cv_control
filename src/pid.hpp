#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Dense>

class PID {

public:
    PID(const Eigen::VectorXd & p, const Eigen::VectorXd & i, const Eigen::VectorXd & d, const double time_period)
    {
        n = p.size();

        I = Eigen::VectorXd::Zero(n);
        v0 = Eigen::VectorXd::Zero(n);

        kp = p; ki = i; kd = d;
        T = time_period; // sec
    }

    void init(const Eigen::VectorXd & v)
    {
        v0 = v;
    }

    inline Eigen::VectorXd dotPD(Eigen::VectorXd & v1)
    {
        Eigen::VectorXd out(n);

        D = (v1 - v0) / T;
        for (size_t i = 0; i < n; ++i)
            out[i] = kp[i]*v1[i] + kd[i]*D[i];
        v0 = v1;
        return out;
    }

    inline Eigen::VectorXd dotPI(Eigen::VectorXd & v1)
    {
        Eigen::VectorXd out(n);

        I += (v1 + v0) * T/2;
        for (size_t i = 0; i < n; ++i)
            out[i] = kp[i]*v1[i] + ki[i]*I[i];
        v0 = v1;
        return out;
    }

    inline Eigen::VectorXd dotPID(Eigen::VectorXd & v1)
    {
        Eigen::VectorXd out(n);

        I += (v1 + v0) * T/2;
        D = (v1 - v0) / T;
        for (size_t i = 0; i < n; ++i) {
            out[i] = kp[i]*v1[i] + ki[i]*I[i] + kd[i]*D[i];
        }
        v0 = v1;
        return out;
    }

    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;

private:
    Eigen::VectorXd I;
    Eigen::VectorXd D;
    Eigen::VectorXd v0;
    double T;
    int n;

};

