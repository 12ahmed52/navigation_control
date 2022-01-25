#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <string>
#include <iostream>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "kinmatic_model/model_state.h"
#include "std_msgs/UInt16MultiArray.h"

using namespace std;

#define v  2    //r/s


kinmatic_model::model_state nav_msg_out;


class pure_pursuit
{
    private:
        
    public:
        float ld, state_x, state_y, state_theta, x_front, y_front, l, nearist_calc, nearist_calc1,
        ld_calc, arr_x[10000], arr_y[10000], ld_calc_real,  ld_x, 
        ld_y, alpha, steering_angle,  error;
        int start_iteration, end_iteration, save_i;

        pure_pursuit(/* args */);
        ~pure_pursuit();
        void calc();
    };

    pure_pursuit::pure_pursuit(/* args */)
    {
        ld = 5;
        state_x=0;
        state_y=0;
        state_theta=0;
        x_front=0;
        y_front=0;
        l = 1;
        nearist_calc=0;
        nearist_calc1=0;
        ld_calc=0;
        arr_x[10000];
        arr_y[10000];
        ld_calc_real=0;
        end_iteration=100;
        start_iteration=0;
        ld_x=0;
        ld_y=0;
        error=0;
        steering_angle=0.0;
        alpha=0;
        save_i=0;
    }

    pure_pursuit::~pure_pursuit()
    {   
    }

    void pure_pursuit::calc()
    {
        //x_front=l*cos(state_theta);
        //y_front=l*sin(state_theta);
        if(state_theta==0){state_theta=0.00000000000000000001;};
        float m = atan(state_theta);
        float b= -1/m;
        float c = -state_x-(b*state_y);
        float min_distance=100000;
        float nearist_x=0;
        float nearist_y=0;
        float point_position_check=0;

        for(int i=start_iteration;i<end_iteration;i++){
        nearist_calc= sqrt(pow(arr_x[i]-state_x,2)+pow(arr_y[i]-state_y,2));
        nearist_calc1= sqrt(pow(arr_x[i+1]-state_x,2)+pow(arr_y[i+1]-state_y,2));
        if(nearist_calc1<nearist_calc && nearist_calc1<=min_distance){
        min_distance=nearist_calc1;
        save_i=i;
        nearist_x=arr_x[i+1];
        nearist_y=arr_y[i+1];
        }
        if(nearist_calc<nearist_calc1 && nearist_calc<=min_distance){
        min_distance=nearist_calc;
        save_i=i;
        nearist_x=arr_x[i];
        nearist_y=arr_y[i];
        }
        start_iteration=save_i;
        end_iteration=start_iteration+100;

        }

        ld =(   0.5*min_distance    +   0.5*v   );
            if (min_distance<3)
            {ld =2*v;}

        for(int j=0;j<100;j++){
        ld_calc= sqrt(pow(nearist_x-arr_x[save_i+j+1],2)+pow(nearist_y-arr_y[save_i+j+1],2));
        if(ld_calc>ld && ld_calc<ld+2){
        ld_x=arr_x[save_i+j+1];
        ld_y=arr_y[save_i+j+1];
        j=100;
        }
        }
        ld_calc_real=sqrt(pow(ld_x-state_x,2)+pow(ld_y-state_y,2));
        error=abs(ld_x+b*ld_y+c)/(sqrt(1+pow(b,2)));
        alpha=asin(error/ld_calc_real);
        steering_angle=atan((2*l*sin(alpha))/ld_calc_real);
        point_position_check=(ld_x-nearist_x)*(state_y-nearist_y)-(ld_y-nearist_y)*(state_x-nearist_x);
        if(point_position_check>0){
        steering_angle=-steering_angle;

        }
    }


pure_pursuit pure_pursuit_obj;

class sub_pub_node
{
public:

  sub_pub_node()
  {
	path_to_nav_sub = handel.subscribe("/path", 10,&sub_pub_node::path_callback,this );
    model_to_nav_sub = handel.subscribe("/model_k_state", 10,&sub_pub_node::model_k_state_callback,this );

	nav_out_pub = handel.advertise<kinmatic_model::model_state>("/nav_output", 10);

      ros::Rate loop_rate(1);
      

  
  }

  void publish()
  {
    nav_out_pub.publish(nav_msg_out);
    //model_out_pub.publish(model_msg_output);

  }

    void path_callback (const std_msgs::UInt16MultiArray::ConstPtr& msg)
    {

    }
    void model_k_state_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        pure_pursuit_obj.state_x=msg->x;
        pure_pursuit_obj.state_y=msg->y;
        pure_pursuit_obj.state_theta=msg->theta;

        pure_pursuit_obj.calc();

        nav_msg_out.steering_wheel=pure_pursuit_obj.steering_angle;
        nav_msg_out.cd_velocity= v;

        publish();
    }

private:
  ros::NodeHandle handel; 
  ros::Publisher nav_out_pub;

  ros::Subscriber model_to_nav_sub;
  ros::Subscriber path_to_nav_sub;

};





int main(int argc, char **argv)
{

    /******* test path genrate **********/
    for (int i = 0; i < 10000; i++)
    {
        pure_pursuit_obj.arr_x[i]=i;
        pure_pursuit_obj.arr_y[i]=20;

    }
    ROS_INFO("default test path updated");
    /***********************************/
    
  ros::init(argc, argv, "pure_pursuit");

  sub_pub_node sub_pub_object;

  //ros::spin();
  
  ros::Rate loop_rate(100);


  //sub_pub_object.publish();
    ros::spin();

    /*
    while (ros::ok())
    {
            sub_pub_object.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }
    */


  return 0;
}