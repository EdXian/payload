#ifndef FORCEEST_H
#define FORCEEST_H
#include "ukf.h"
#include "iostream"



enum state{
    pos=0,
    velocity,
    statesize
};

enum measurement{
    mpos = 0,
    mvel,
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
