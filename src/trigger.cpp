#include "ros/ros.h"
#include "geometry_msgs/Point.h"




double upper_bound = 1.2;
double lower_bound = -1.2;
double zero = 0;
int current_state=0;
int last_state=0;
int controlloer_state;
int score;
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

  while(ros::ok()){




    last_state = current_state;
    if(leader_force.x > upper_bound){
      current_state = positive_engaged ;
    }else if(leader_force.x < lower_bound){
      current_state = negative_engaged ;
    }else{
      current_state = disengaged;
    }

#ifdef interval
    if((current_state == positive_engaged) && (last_state ==  disengaged)){
      controlloer_state = 5;


    }
    if((controlloer_state == 5) && (leader_force.x<zero)){
      controlloer_state = 0;

    }
    if((current_state == negative_engaged) && (last_state ==  disengaged)){
      controlloer_state = -5;

    }
    if((controlloer_state == -5) && (leader_force.x>zero)){
      controlloer_state = 0;

    }
#else
    if((current_state == positive_engaged) && (last_state ==  disengaged)){
      controlloer_state = 5;


    }
    if((controlloer_state == 5) && (leader_force.x<zero)){
      controlloer_state = 0;

    }
    if((current_state == negative_engaged) && (last_state ==  disengaged)){
      controlloer_state = -5;

    }
    if((controlloer_state == -5) && (leader_force.x>zero)){
      controlloer_state = 0;

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
