#include "forceest.h"

#define L 0.30     //object length
#define pi 3.14159 //pi
#define g  9.81    //gravity
#define r  0.15   //0.5length of object
#define a 0.04   //width of object
#define l  0.4   //  length of cable
#define M  0.6   //Mass of object
#define I  0.5*M*(a*a+L*L) //inertial



Eigen::MatrixXd forceest::dynamics(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);

    for(int i=0;i<this->x_sigmavector_size;i++){

        double theta_p = sigma_state(thetap,i) ;
        double omega_p = sigma_state(omegap,i);
        double FFx = sigma_state(FF_x,i);
        double FFz = sigma_state(FF_z,i);
        double FLx = sigma_state(FL_x,i);
        double FLz = sigma_state(FL_z,i);
        double acx = sigma_state(ac_x,i);
        double acz = sigma_state(ac_z,i);

        double theta_d = atan2(FLz , FLx);
        double theta_c = atan2(FFz , FFx);
        double FF_net = sqrt(FFx*FFx +FFz*FFz);
        double FL_net = sqrt(FLx*FLx +FLz*FLz);
        double alpha_p = r/(I)*(FF_net*sin(theta_p+theta_c)-FL_net*sin(theta_c+theta_d));

        double ap_x =acx +alpha_p*r*cos(theta_p) + omega_p*omega_p*r*sin(theta_p);
        double ap_z =acz +alpha_p*r*sin(theta_p) - omega_p*omega_p*r*cos(theta_p);

        predict_sigma_state(thetap,i) = theta_p+omega_p*dt;
        predict_sigma_state(omegap,i) =  omega_p;

        predict_sigma_state(FL_x,i) = M*ap_x - FFx;
        predict_sigma_state(FL_z,i) = M*ap_z - M*g- FFx;


    }
    return predict_sigma_state;


}
