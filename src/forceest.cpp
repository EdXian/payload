#include "forceest.h"

Eigen::MatrixXd forceest::dynamics(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);

    for(int i=0;i<this->x_sigmavector_size;i++){

        double p = sigma_state(pos,i) ;
        double v = sigma_state(velocity,i);
        double p_ ;
        double v_ ;


        p_ = p+v*dt;
        v_ = v;

        predict_sigma_state(pos,i) =  p_ ;
        predict_sigma_state(velocity,i) =  v_;

    }
    return predict_sigma_state;


}
