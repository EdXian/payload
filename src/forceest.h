#ifndef FORCEEST_H
#define FORCEEST_H
#include "ukf.h"
#include "iostream"





enum state{
    thetap=0,
    omegap,
    ac_x,
    ac_z,
    FF_x,
    FF_z,
    FL_x,
    FL_z,
    statesize
};

enum measurement{
    mthetap = 0,
    momegap,
    mac_x,
    mac_z,
    mFF_x,
    mFF_z,
    measurementsize
};


class forceest : public ukf
{

public:
forceest(int x, int y) : ukf(x,y){

}
Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state);

private:


};

#endif // FORCEEST_H
