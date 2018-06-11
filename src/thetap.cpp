#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>

sensor_msgs::Imu imu_data;
double imu_roll , imu_pitch , imu_yaw;
geometry_msgs::PoseStamped mocap_pose;
double fx ,fy ,fz;
double mocap_roll , mocap_yaw , mocap_pitch;

double theta_p;
double omega_p;
bool flag = true;
double yaw_bias ;

double x_bias , y_bias , z_bias;


double a_x_I,a_y_I,a_z_I;
double a_x_B,a_y_B,a_z_B;
Eigen::Vector3d a_I , a_B;

bool imu_flag=true;


double w,x,y,z;

Eigen::Matrix3d R_B_I ; //body frame to inertial frame matrix

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){

  a_B.setZero(3);
  imu_data = *msg;
  a_B<<imu_data.linear_acceleration.x -  x_bias,
       imu_data.linear_acceleration.y -  y_bias,
       imu_data.linear_acceleration.z ;


  omega_p = imu_data.angular_velocity.y;
  tf::Quaternion quat1(imu_data.orientation.x,imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  tf::Matrix3x3(quat1).getRPY(imu_roll, imu_pitch,  imu_yaw);
  theta_p = imu_pitch;

  //correct yaw angle bias in body frame
  if(flag){
    yaw_bias = imu_yaw;

    x_bias = imu_data.linear_acceleration.x;
    y_bias = imu_data.linear_acceleration.y;
    z_bias = imu_data.linear_acceleration.z;


    flag = false;
  }


  if( (imu_yaw - yaw_bias) < 0  ){

       imu_yaw = imu_yaw - yaw_bias +2*3.14159 ;
  }else{

    imu_yaw  = imu_yaw- yaw_bias;
  }


}



void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
   mocap_pose= *msg;
fx = mocap_pose.pose.position.x;
fy = mocap_pose.pose.position.y;
fz = mocap_pose.pose.position.z;

x = mocap_pose.pose.orientation.x;
y = mocap_pose.pose.orientation.y;
z = mocap_pose.pose.orientation.z;
w = mocap_pose.pose.orientation.w;

tf::Quaternion quat1(mocap_pose.pose.orientation.x,mocap_pose.pose.orientation.y, mocap_pose.pose.orientation.z, mocap_pose.pose.orientation.w);
tf::Matrix3x3(quat1).getRPY(mocap_roll, mocap_pitch,  mocap_yaw);

if(mocap_yaw <0){
  mocap_yaw += 2*3.14159;
}
//  std::cout<<"-----------"<<std::endl;
//  std::cout<<"roll" << imu_roll<<std::endl;
//  std::cout<<"pitch" << imu_pitch<<std::endl;
//  std::cout<<"yaw" << mocap_yaw  <<std::endl;


double cr = cos(imu_roll) , cp = cos(imu_pitch) , cy = cos(mocap_yaw);
double sr = sin(imu_roll) , sp = sin(imu_pitch) , sy = sin(mocap_yaw);

Eigen::Matrix3d Rx ,Ry,Rz ,Q;
//R_B_I.setZero(3,3);
//Rx.setZero(3,3);
//Ry.setZero(3,3);
//Rz.setZero(3,3);
Rz <<
      cy,-1*sy,0,
      sy,cy,0,
      0,0,1;
Rx << 1,0,0,
      0,cr,-sr,
      0,sr,cr;
Ry<<cp ,0, sp,
    0 ,1,0,
    -1*sp,0,cp;
R_B_I  = Rx *Ry *Rz;

Q<< w*w+x*x-y*y-z*z  , 2*x*y-2*w*z ,2*x*z+2*w*y,
    2*x*y +2*w*z           , w*w-x*x+y*y-z*z    ,2*y*z-2*w*x,
    2*x*z -2*w*y          , 2*y*z+2*w*x        ,w*w-x*x-y*y+z*z;


a_I = Q*a_B;
a_x_I = a_I[0];
a_y_I = a_I[1];
a_z_I = a_I[2];
std::cout <<"acc" <<std::endl<<a_I<<std::endl;


//std::cout<<"yaw err : " << mocap_yaw - imu_yaw<<std::endl;
//x = mocap_pose.pose.orientation.x;
//y = mocap_pose.pose.orientation.y;
//z = mocap_pose.pose.orientation.z;
//w = mocap_pose.pose.orientation.w;


//std::cout <<"*****"<<std::endl;

//std::cout << "roll" <<mocap_roll<<std::endl;

//std::cout << "yaw" <<mocap_yaw<<std::endl;

//std::cout << "pitch" <<mocap_pitch<<std::endl;

//std::cout <<"*****"<<std::endl;

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "thetap");
  ros::NodeHandle nh;
  forceest forceest1(statesize,measurementsize);
  ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data" , 10, imu_cb);
  ros::Subscriber mocap_sub = nh.subscribe("/vrpn_client_node/RigidBody2/pose" , 10 , mocap_cb);
  ros::Rate loop_rate(30);

  Eigen::MatrixXd mnoise;
  mnoise.setZero(2,2);
  mnoise   = 3e-3*Eigen::MatrixXd::Identity(measurementsize , measurementsize);

  forceest1.set_measurement_noise(mnoise);

  Eigen::MatrixXd pnoise;
  pnoise.setZero(2,2);
  pnoise   = 3e-3*Eigen::MatrixXd::Identity(statesize , statesize);
  forceest1.set_process_noise(pnoise);



  Eigen::MatrixXd measurement_matrix;
  measurement_matrix.setZero(measurementsize,measurementsize);
  measurement_matrix << 1,0,
                        0,1;
  forceest1.set_measurement_matrix(measurement_matrix);

  forceest1.dt = 0.02;


  while(ros::ok()){

    forceest1.predict();
    Eigen::VectorXd measure;
    measure.setZero(2);
    measure << theta_p , omega_p;
    forceest1.correct(measure);
//    std::cout<<"-----------"<<std::endl;


//    std::cout << forceest1.x[0] - theta_p <<std::endl;
//    std::cout << forceest1.x[1] - omega_p<<std::endl;
    loop_rate.sleep();
    ros::spinOnce();

  }


  return 0;
}
