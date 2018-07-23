#include "ros/ros.h"
#include "geometry_msgs/Point.h"


//#define interval

double upper_bound = 1.5;
double lower_bound = -1.5;
double zero = 0;
int current_state=0;
int last_state=0;
int controlloer_state;
int score;
double time_threshold = 0.1;
enum zone{
    positive_zone = 5,
    zero_zone = 0 ,
    negative_zone = -5
};


enum control_state{
  positive_engaged = 5,
  disengaged = 0,
  negative_engaged = -5
};
geometry_msgs::Point leader_force ;

void force_cb(const geometry_msgs::Point::ConstPtr& msg){
leader_force  = *msg;
}


geometry_msgs::Point output;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/leader_force", 3,force_cb);
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("controller_state",3);
  ros::Rate loop_rate(50);

  double time =ros::Time::now().toSec();
  double lastime =time;
  double last_update_time = 0;
  while(ros::ok()){




    last_state = current_state;
    if(leader_force.x > upper_bound){
      current_state = positive_zone;
    }else if(leader_force.x < lower_bound){
      current_state = negative_zone ;
    }else{
      current_state = zero_zone;
    }

#ifdef interval
     lastime = time;
    time = ros::Time::now().toSec();
    double dt = time - last_update_time;
    if((current_state == positive_zone) && (last_state ==  zero_zone)   ){
      controlloer_state = positive_engaged;
      last_update_time = time;

    }
    if((controlloer_state == positive_zone) && (leader_force.x<zero) && (dt>time_threshold )){
      controlloer_state = disengaged;
      last_update_time = time;
    }
    if((current_state == negative_zone) && (last_state ==  zero_zone)){
      controlloer_state = negative_engaged;
      last_update_time = time;
    }
    if((controlloer_state == negative_zone) && (leader_force.x>zero)&& (dt>time_threshold )){
      controlloer_state = disengaged;
      last_update_time = time;
    }
#else
    lastime = time;
   time = ros::Time::now().toSec();
   double dt = time - last_update_time;
   if((current_state == positive_zone) && (last_state ==  zero_zone)   ){
     controlloer_state = positive_engaged;
     last_update_time = time;

   }
   if((controlloer_state == positive_engaged) && (leader_force.x<zero) ){
     controlloer_state = disengaged;
     last_update_time = time;
   }
   if((current_state == negative_zone) && (last_state ==  zero_zone)){
     controlloer_state = negative_engaged;
     last_update_time = time;
   }
   if((controlloer_state == negative_engaged) && (leader_force.x>zero)){
     controlloer_state = disengaged;
     last_update_time = time;
   }
#endif

    output.x = controlloer_state;
    output.y = upper_bound;
    output.z = lower_bound;

    pub.publish(output);


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
